#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""ci/run.py: the local CI. Runs one tier of the verification matrix (ci/matrix.toml) as a job graph.

    ci/run.py --tier smoke                 run on the working tree (marked dirty if it has uncommitted changes)
    ci/run.py --tier smoke --commit HEAD   run on a commit, checked out in build/ci/worktree (the hook does this)
    ci/run.py --status [N]                 the last N runs
    ci/run.py --tier smoke --only 'sim-p8' only the jobs matching a regex (and the jobs they depend on)

Every job is a make target (see the Makefile). A config's jobs wait for its pkg job, the only one that writes the
config's defs, gen_pkg and package; the guard waits for p8s6's and p8s8's.
A run writes ci/runs/<UTC time>-<commit>/ with one log per job, summary.md and summary.json, and appends one line
to ci/runs/history.jsonl. Runs are serialized by a lock. Exit status 0 iff every job passed.
"""
import argparse
import datetime as dt
import fcntl
import json
import os
import re
import subprocess
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import matrix  # noqa: E402

REPO = Path(__file__).resolve().parent.parent
RUNS = REPO / "ci" / "runs"
WORKTREE = REPO / "build" / "ci" / "worktree"
LOCK = REPO / "build" / "ci.lock"

# Per job kind: CPU weight (threads), timeout in s, the pass-line patterns (each must match `count` times; a pattern
# with a group counts checks), and extra make arguments.
SIM_PASSED = re.compile(r"^PASSED: (\d+) checks, 0 errors", re.M)
STREAM_SEEDS = (1, 2, 3)
STREAM_SIM = dict(weight=4, timeout=3600, passed=[SIM_PASSED], count=len(STREAM_SEEDS),
                  args=[f"SEEDS={' '.join(str(s) for s in STREAM_SEEDS)}"])
KINDS = {
    "pkg":    dict(weight=1, timeout=600,  passed=[]),
    "lint":   dict(weight=1, timeout=600,  passed=[]),
    "sim":    dict(weight=4, timeout=7200, passed=[SIM_PASSED]),
    "inverse": dict(weight=4, timeout=1800, passed=[re.compile(r"^PASS: ", re.M)]),
    "stream-ref": dict(weight=1, timeout=1800, passed=[re.compile(r"^PASS: ", re.M)]),
    "lint-dec-stream": dict(weight=1, timeout=600, passed=[]),
    "lint-enc-stream": dict(weight=1, timeout=600, passed=[]),
    "sim-dec-stream": STREAM_SIM,
    "sim-enc-stream": STREAM_SIM,
    "sim-stream-rt": STREAM_SIM,
    # the generator's reasons, then the pass line
    "stream-refuse": dict(weight=1, timeout=600, passed=[
        re.compile(r"^gen_pkg: refusing .* for the stream units:\n  - \S", re.M),
        re.compile(r"^PASS: gen_pkg --stream refuses ", re.M)]),
    "refuse": dict(weight=1, timeout=600,  passed=[re.compile(r"^PASS: ", re.M)]),
    "guard":  dict(weight=4, timeout=1800, passed=[re.compile(r"^PASS: ", re.M)]),
}


@dataclass
class Job:
    name: str          # also the make target
    kind: str
    config: str        # "" for jobs that are not per config
    deps: list = field(default_factory=list)
    state: str = "WAIT"  # WAIT, RUN, PASS, FAIL, SKIP
    seconds: float = 0.0
    detail: str = ""
    proc: object = None
    start: float = 0.0
    log: Path = None


def plan(m: matrix.Matrix, tier: matrix.Tier) -> list:
    jobs = []
    for c in m.configs:
        pkg = f"pkg-{c.name}"
        after_pkg = [pkg] if "pkg" in tier.jobs else []
        for kind in tier.jobs:
            if kind == "pkg":
                jobs.append(Job(pkg, "pkg", c.name))
            elif kind in ("lint", "sim", "stream-ref"):  # stream-ref compares the package's SCALE_TABLE with the model
                jobs.append(Job(f"{kind}-{c.name}", kind, c.name, after_pkg))
            elif kind == "inverse":  # the C++ reference alone: no package, but the config's defs, which pkg writes
                jobs.append(Job(f"{kind}-{c.name}", kind, c.name, after_pkg))
            elif kind in matrix.STREAM_UNIT_KINDS and c.stream == "supported":
                jobs.append(Job(f"{kind}-{c.name}", kind, c.name, after_pkg))
        # a build the stream units refuse: one job checks the refusal instead of the unit jobs
        if c.stream == "refused" and any(k in matrix.STREAM_UNIT_KINDS for k in tier.jobs):
            jobs.append(Job(f"stream-refuse-{c.name}", "stream-refuse", c.name, after_pkg))
    if "refuse" in tier.jobs:
        jobs += [Job(f"refuse-{c.name}", "refuse", c.name) for c in m.refused]
    if "guard" in tier.jobs:
        jobs.append(Job("guard", "guard", "", ["pkg-p8s6", "pkg-p8s8"] if "pkg" in tier.jobs else []))
    names = {j.name for j in jobs}
    for j in jobs:
        j.deps = [d for d in j.deps if d in names]
    return jobs


# Variables git sets for hooks would point the CI worktree's commands at the main checkout's index. The runner
# drops them.
CLEAN_ENV = {k: v for k, v in os.environ.items()
             if k not in ("GIT_DIR", "GIT_INDEX_FILE", "GIT_WORK_TREE", "GIT_PREFIX", "GIT_COMMON_DIR")}


def git(*args, cwd=REPO) -> str:
    r = subprocess.run(["git", *args], cwd=cwd, env=CLEAN_ENV, capture_output=True, text=True)
    if r.returncode != 0:
        raise RuntimeError(f"git {' '.join(args)}: {r.stderr.strip()}")
    return r.stdout.strip()


def checkout(rev: str) -> tuple:
    """The commit in build/ci/worktree; returns (tree, sha)."""
    sha = git("rev-parse", "--verify", f"{rev}^{{commit}}")
    if not (WORKTREE / ".git").exists():
        WORKTREE.parent.mkdir(parents=True, exist_ok=True)
        git("worktree", "prune")
        git("worktree", "add", "--detach", str(WORKTREE), sha)
    else:
        git("checkout", "--detach", "--force", sha, cwd=WORKTREE)
        git("clean", "-fdq", cwd=WORKTREE)  # untracked files; ignored build products stay for incremental builds
    return WORKTREE, sha


def tool_versions() -> dict:
    def first_line(cmd):
        try:
            return subprocess.run(cmd, capture_output=True, text=True, timeout=30).stdout.splitlines()[0]
        except (OSError, IndexError, subprocess.TimeoutExpired):
            return "unavailable"
    verilator = os.environ.get("VERILATOR", "verilator")
    return {"verilator": first_line([verilator, "--version"]), "cxx": first_line(["g++", "--version"]),
            "python": sys.version.split()[0]}


def config_ids(tree: Path, m: matrix.Matrix) -> dict:
    ids = {}
    for c in m.configs:
        pkg = tree / "build" / c.name / "gen" / "pans_pkg.sv"
        hit = re.search(r"CONFIG_ID = 64'h([0-9a-f]+)", pkg.read_text()) if pkg.exists() else None
        ids[c.name] = hit.group(1) if hit else ""
    return ids


def run_jobs(jobs: list, tree: Path, logs: Path, capacity: int, quiet: bool) -> None:
    by_name = {j.name: j for j in jobs}
    used = 0
    while any(j.state in ("WAIT", "RUN") for j in jobs):
        for j in jobs:  # start what is ready
            if j.state != "WAIT":
                continue
            states = [by_name[d].state for d in j.deps]
            if any(s in ("FAIL", "SKIP") for s in states):
                j.state, j.detail = "SKIP", "dependency failed"
                continue
            weight = KINDS[j.kind]["weight"]
            if all(s == "PASS" for s in states) and (used + weight <= capacity or used == 0):
                j.log = logs / f"{j.name}.log"
                j.start = time.monotonic()
                j.proc = subprocess.Popen(["make", "-s", j.name, f"JOBS={weight}", *KINDS[j.kind].get("args", [])],
                                          cwd=tree, env=CLEAN_ENV, stdout=j.log.open("w"), stderr=subprocess.STDOUT)
                j.state = "RUN"
                used += weight
        for j in jobs:  # collect what finished
            if j.state != "RUN":
                continue
            j.seconds = time.monotonic() - j.start
            code = j.proc.poll()
            if code is None and j.seconds > KINDS[j.kind]["timeout"]:
                j.proc.kill()
                code = j.proc.wait()
                j.detail = f"timeout after {KINDS[j.kind]['timeout']} s"
            if code is None:
                continue
            used -= KINDS[j.kind]["weight"]
            text = j.log.read_text(errors="replace")
            kind = KINDS[j.kind]
            hits = [p.findall(text) for p in kind["passed"]]
            passed = all(len(h) >= kind.get("count", 1) for h in hits)
            j.state = "PASS" if code == 0 and passed else "FAIL"
            if j.state == "PASS" and SIM_PASSED in kind["passed"]:
                runs = hits[kind["passed"].index(SIM_PASSED)]
                j.detail = f"{sum(int(n) for n in runs):,} checks" + (f" in {len(runs)} seeds" if len(runs) > 1 else "")
            elif j.state == "FAIL" and not j.detail:
                j.detail = f"exit {code}" if code else "no pass line in the log"
            if not quiet:
                print(f"{j.state:4}  {j.name:24} {j.seconds:7.1f} s  {j.detail}", flush=True)
        time.sleep(0.2)


def write_summary(run_dir: Path, record: dict, jobs: list, repro_prefix: str) -> None:
    lines = [f"# CI {record['tier']}: {record['result']}", "",
             f"- commit `{record['commit']}`{' (dirty working tree)' if record['dirty'] else ''}",
             f"- started {record['started']}, {record['seconds']:.0f} s",
             f"- tools: {record['tools']['verilator']}; {record['tools']['cxx']}; Python {record['tools']['python']}",
             "", "| Job | Result | Time | Detail |", "|---|---|---|---|"]
    lines += [f"| {j.name} | {j.state} | {j.seconds:.1f} s | {j.detail} |" for j in jobs]
    failed = [j for j in jobs if j.state == "FAIL"]
    for j in failed:
        tail = j.log.read_text(errors="replace").splitlines()[-25:]
        lines += ["", f"## {j.name}: {j.detail}", "", f"Reproduce: `{repro_prefix}make {j.name}`", "", "```",
                  *tail, "```"]
    lines += ["", "Config IDs: " + ", ".join(f"{k} {v}" for k, v in record["config_ids"].items())]
    (run_dir / "summary.md").write_text("\n".join(lines) + "\n")
    (run_dir / "summary.json").write_text(json.dumps(record, indent=2) + "\n")


def status(n: int) -> int:
    history = RUNS / "history.jsonl"
    if not history.exists():
        print("no runs yet")
        return 0
    for line in history.read_text().splitlines()[-n:]:
        r = json.loads(line)
        print(f"{r['started']}  {r['tier']:6} {r['commit'][:10]}{'+' if r['dirty'] else ' '} {r['result']:4} "
              f"{r['passed']}/{r['jobs']} jobs  {r['seconds']:5.0f} s  {r['dir']}")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--tier", default="smoke")
    ap.add_argument("--commit", help="run on this commit in build/ci/worktree instead of the working tree")
    ap.add_argument("--jobs", type=int, default=os.cpu_count(), help="CPU capacity shared by the jobs")
    ap.add_argument("--only", help="run only the jobs whose name matches this regex, and their dependencies")
    ap.add_argument("--quiet", action="store_true")
    ap.add_argument("--status", type=int, nargs="?", const=10, help="show the last N runs")
    args = ap.parse_args()
    if args.status is not None:
        return status(args.status)

    LOCK.parent.mkdir(parents=True, exist_ok=True)
    with LOCK.open("w") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        if args.commit:
            tree, sha = checkout(args.commit)
            dirty, repro = False, f"git checkout {sha[:10]} && "
        else:
            tree, repro = REPO, ""
            try:
                sha, dirty = git("rev-parse", "HEAD"), bool(git("status", "--porcelain"))
            except (RuntimeError, OSError):  # a tree without git, such as an export: no commit to name
                sha, dirty = "no-git", False
        m = matrix.load(tree / "ci" / "matrix.toml")
        if args.tier not in m.tiers:
            print(f"unknown tier {args.tier}; tiers: {', '.join(m.tiers)}", file=sys.stderr)
            return 2
        started = dt.datetime.now(dt.timezone.utc)
        run_dir = RUNS / f"{started:%Y%m%dT%H%M%SZ}-{sha[:10]}{'-dirty' if dirty else ''}"
        (run_dir / "logs").mkdir(parents=True)
        subprocess.run(["make", "-s", "build/matrix.mk"], cwd=tree, env=CLEAN_ENV, check=True)  # before parallel makes include it
        jobs = plan(m, m.tiers[args.tier])
        if args.only:
            keep = {j.name for j in jobs if re.search(args.only, j.name)}
            by_name = {j.name: j for j in jobs}
            stack = list(keep)
            while stack:
                for d in by_name[stack.pop()].deps:
                    if d not in keep:
                        keep.add(d)
                        stack.append(d)
            jobs = [j for j in jobs if j.name in keep]
        if not args.quiet:
            print(f"CI {args.tier}: {len(jobs)} jobs on {sha[:10]}{' (dirty)' if dirty else ''} -> {run_dir}")
        t0 = time.monotonic()
        run_jobs(jobs, tree, run_dir / "logs", args.jobs, args.quiet)
        passed = sum(j.state == "PASS" for j in jobs)
        record = {"tier": args.tier + (f" --only {args.only}" if args.only else ""), "commit": sha, "dirty": dirty, "started": f"{started:%Y-%m-%d %H:%M:%SZ}",
                  "seconds": time.monotonic() - t0, "result": "PASS" if passed == len(jobs) else "FAIL",
                  "jobs": len(jobs), "passed": passed, "dir": str(run_dir.relative_to(REPO)),
                  "tools": tool_versions(), "config_ids": config_ids(tree, m),
                  "job_results": [{"name": j.name, "state": j.state, "seconds": round(j.seconds, 1),
                                   "detail": j.detail} for j in jobs]}
        write_summary(run_dir, record, jobs, repro)
        with (RUNS / "history.jsonl").open("a") as h:
            h.write(json.dumps({k: v for k, v in record.items() if k != "job_results"}) + "\n")
        if not args.quiet:
            print(f"{record['result']}: {passed}/{len(jobs)} jobs passed in {record['seconds']:.0f} s; "
                  f"{run_dir.relative_to(REPO)}/summary.md")
        return 0 if record["result"] == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main())
