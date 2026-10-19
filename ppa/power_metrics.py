#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""ppa/power_metrics.py: collects power results into one JSON record.

    python3 ppa/power_metrics.py build/<cfg>/power <period_ns> [<stimulus>]                    the cores (power-<cfg>)
    python3 ppa/power_metrics.py --stream <dec|enc> <power directory> <period_ns> [<stimulus>]  a stream unit

JSON on stdout, a summary on stderr. The inputs are the OpenSTA reports and the gate-level simulation logs. The
stimulus is make's POWER_STIMULUS (laplace by default); the escapes are those the simulation counted.
Cores: the energy per symbol is the total power times the clock period, one transaction per cycle.
Stream units: the energy per symbol is power x period x cycles / symbols over the simulated window (STREAM.md §4.1),
for the design, the unit, the unit with the clock tree, and each register group.
"""
import json
import re
import sys
from pathlib import Path

ACTIVITY = "zero-delay gate-level simulation of the post-route netlist (no glitches)"
STREAM_UNITS = {"dec": "pans_dec_stream", "enc": "pans_enc_stream"}
STREAM_LOG = re.compile(r"(\d+) jobs, (\d+) symbols, (\d+) cycles, (\d+) checks, (\d+) mismatches(?:, (\d+) escapes)?")
CORE_LOG = re.compile(r"(\d+) transactions, (\d+) mismatches(?:, (\d+) escapes)?")
IDLE_LOG = re.compile(r": producer idle (\d+)%")
GROUP_LINE = r"^{} (\w+) (\d+) (\S+) (\S+) (\S+) (\S+)$"


def core_metrics(d: Path, core: str, period_ns: float) -> dict:
    report = (d / f"{core}.power.rpt").read_text()
    annotated = re.search(r"Annotated (\d+) pin activities", report)
    total = re.search(r"^Total\s+(\S+)\s+(\S+)\s+(\S+)\s+(\S+)", report, re.M)
    check = CORE_LOG.search((d / f"{core}.sim.log").read_text())
    corner = CORE_LOG.search((d / f"{core}.corner.log").read_text())
    internal, switching, leakage, power = (float(x) for x in total.groups())
    escapes = {"escapes": int(check.group(3))} if check.group(3) is not None else {}
    return {"power_mw": round(power * 1e3, 4), "internal_mw": round(internal * 1e3, 4),
            "switching_mw": round(switching * 1e3, 4), "leakage_mw": round(leakage * 1e3, 4),
            "energy_pj_per_symbol": round(power * period_ns * 1e3, 3),
            "annotated_pins": int(annotated.group(1)) if annotated else 0,
            "transactions": int(check.group(1)), "mismatches": int(check.group(2)), **escapes,
            "corner_transactions": int(corner.group(1)), "corner_mismatches": int(corner.group(2))}


def cores(d: Path, period: float, stimulus: str) -> int:
    record = {"config": d.parent.name, "period_ns": period,
              "activity": f"{ACTIVITY}, workload ppa/power_stimulus.cpp", "stimulus": stimulus,
              "cores": {c: core_metrics(d, c, period) for c in ("enc", "dec")}}
    json.dump(record, sys.stdout, indent=2)
    print(file=sys.stdout)
    print(f"{record['config']}: power at {period:g} ns, simulated activity, stimulus {stimulus}", file=sys.stderr)
    for core, m in record["cores"].items():
        print(f"  {core}: {m['power_mw']:.2f} mW = {m['energy_pj_per_symbol']:.1f} pJ/symbol "
              f"(leakage {m['leakage_mw']:.2f} mW)  netlist check {m['mismatches']} mismatches in "
              f"{m['transactions']} ({m.get('escapes', '?')} escapes), corners {m['corner_mismatches']} in "
              f"{m['corner_transactions']}", file=sys.stderr)
    return 0


def stream_log(path: Path) -> dict:
    m = STREAM_LOG.search(path.read_text())
    if not m:
        raise ValueError(f"{path}: no result line")
    keys = ("jobs", "symbols", "cycles", "checks", "mismatches", "escapes")
    return {k: int(x) for k, x in zip(keys, m.groups()) if x is not None}


def group_lines(report: str, kind: str) -> dict:
    groups = {}
    for m in re.finditer(GROUP_LINE.format(kind), report, re.M):
        internal, switching, leakage, power = (float(x) for x in m.groups()[2:])
        groups[m.group(1)] = {"cells": int(m.group(2)), "power_mw": round(power * 1e3, 4),
                              "internal_mw": round(internal * 1e3, 4), "switching_mw": round(switching * 1e3, 4),
                              "leakage_mw": round(leakage * 1e3, 4)}
    return groups


def workload(d: Path, delay: int, period_ns: float) -> dict:
    sim = stream_log(d / f"sim_d{delay}.log")
    idle = IDLE_LOG.search((d / f"sim_d{delay}.log").read_text())
    report = (d / f"power_d{delay}.rpt").read_text()
    annotated = re.search(r"Annotated (\d+) pin activities", report)
    total = [float(x) for x in re.search(r"^Total\s+(\S+)\s+(\S+)\s+(\S+)\s+(\S+)", report, re.M).groups()]
    groups = group_lines(report, "power")
    if set(groups) != {"unit", "clock", "wrapper"}:
        raise ValueError(f"{d}/power_d{delay}.rpt: no power split")
    split = group_lines(report, "split")  # the unit's register groups (absent in reports from before them)
    if split and abs(sum(g["power_mw"] for g in split.values()) - groups["unit"]["power_mw"]) > 1e-3:
        raise ValueError(f"{d}/power_d{delay}.rpt: the register groups do not add up to the unit")
    per_symbol = period_ns * 1e3 * sim["cycles"] / sim["symbols"]  # pJ per symbol per W

    def pj(mw: float) -> float:
        return round(mw * 1e-3 * per_symbol, 3)

    for g in split.values():
        g["energy_pj_per_symbol"] = pj(g["power_mw"])
    unit_mw, clock_mw = groups["unit"]["power_mw"], groups["clock"]["power_mw"]
    return {"ema_delay": delay, "producer_idle_pct": int(idle.group(1)) if idle else 0, **sim,
            "symbols_per_cycle": round(sim["symbols"] / sim["cycles"], 4),
            "annotated_pins": int(annotated.group(1)) if annotated else 0,
            "power_mw": round(total[3] * 1e3, 4), "leakage_mw": round(total[2] * 1e3, 4), "groups": groups,
            "unit_groups": split,
            "energy_pj_per_symbol": {"total": round(total[3] * per_symbol, 3), "unit": pj(unit_mw),
                                     "unit_and_clock_tree": pj(unit_mw + clock_mw)}}


def stream(unit: str, d: Path, period: float, stimulus: str) -> int:
    check = stream_log(d / "check.log")
    delays = sorted(int(p.name[len("sim_d"):-len(".log")]) for p in d.glob("sim_d*.log"))
    jobs = "EMA jobs from the stream model" if stimulus == "laplace" else f"{stimulus} LIST jobs"
    record = {"config": d.parents[2].name, "unit": STREAM_UNITS[unit], "params": d.parent.name,
              "period_ns": period,
              "activity": f"{ACTIVITY}, {jobs} (ppa/power_tb_stream.sv)", "stimulus": stimulus,
              "check": check, "workloads": [workload(d, k, period) for k in delays]}
    json.dump(record, sys.stdout, indent=2)
    print(file=sys.stdout)
    print(f"{record['config']} {record['unit']} {record['params']}: power at {period:g} ns, simulated activity "
          f"({stimulus}); "
          f"netlist check {check['mismatches']} mismatches in {check['checks']} checks ({check['symbols']} symbols, "
          f"{check['jobs']} jobs)", file=sys.stderr)
    for w in record["workloads"]:
        e = w["energy_pj_per_symbol"]
        idle = f", producer idle {w['producer_idle_pct']}%" if w["producer_idle_pct"] else ""
        print(f"  EMA delay {w['ema_delay']}{idle}: {w['symbols']} symbols in {w['cycles']} cycles "
              f"({w['symbols_per_cycle']:.3f}/cycle), {w['power_mw']:.2f} mW total (unit {w['groups']['unit']['power_mw']:.2f}, "
              f"clock tree {w['groups']['clock']['power_mw']:.2f}, wrapper {w['groups']['wrapper']['power_mw']:.2f}): "
              f"{e['total']:.2f} pJ/symbol total, unit {e['unit']:.2f}, unit + clock tree {e['unit_and_clock_tree']:.2f}; "
              f"{w['mismatches']} mismatches in {w['checks']} checks, {w.get('escapes', '?')} escapes", file=sys.stderr)
        if w["unit_groups"]:
            print("    unit by register group, mW (cells): " + ", ".join(
                f"{k} {g['power_mw']:.3f} ({g['cells']})" for k, g in w["unit_groups"].items() if g["cells"]),
                file=sys.stderr)
    return 0


def main(argv: list) -> int:
    if len(argv) in (3, 4) and argv[1] != "--stream":
        return cores(Path(argv[1]), float(argv[2]), argv[3] if len(argv) == 4 else "laplace")
    if len(argv) in (5, 6) and argv[1] == "--stream" and argv[2] in STREAM_UNITS:
        return stream(argv[2], Path(argv[3]), float(argv[4]), argv[5] if len(argv) == 6 else "laplace")
    print(__doc__, file=sys.stderr)
    return 2


if __name__ == "__main__":
    sys.exit(main(sys.argv))
