#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""ppa/metrics.py: collects the generic-synthesis results of one config into one JSON record.

    python3 ppa/metrics.py build/<cfg>/ppa     JSON on stdout, a summary table on stderr

Reads what ppa/synth_generic.sh writes per name: gates, depth, word-level operators and, with SYNTH_HIER=1, gates
per module. enc and dec are the cores, fn_<function> the functions, dec_stream and enc_stream_d<DIV_BITS> the units.
"""
import json
import re
import sys
from pathlib import Path

# stat -width lines read "N  $alu_64". Multiply-accumulate cells carry no width and count under width 0.
# stat per module gives "=== <module>$<instance path> ===", then "  <n> cells". "design hierarchy" is the total.
HIER_MODULE = re.compile(r"^=== (.+) ===$")
HIER_CELLS = re.compile(r"^\s+(\d+)\s+cells$")
WORD_OPS = re.compile(r"^\s*(\d+)\s+\$(alu|macc_v2|macc|mul|mux|pmux|shl|shr|sshl|sshr|shift|shiftx)(?:_(\d+))?\s*$")


def unit_metrics(d: Path, name: str) -> dict:
    stat = json.loads((d / f"{name}.stat.json").read_text())
    module = next(iter(stat["modules"].values()))
    depth = re.search(r"\(length=(\d+)\)", (d / f"{name}.ltp.txt").read_text())
    word_ops = {}
    for line in (d / f"{name}.coarse.txt").read_text().splitlines():
        m = WORD_OPS.match(line)
        if m:
            op, width = m.group(2).replace("macc_v2", "macc"), int(m.group(3) or 0)
            word_ops.setdefault(op, {})[width] = word_ops.get(op, {}).get(width, 0) + int(m.group(1))
    record = {
        "gates": module["num_cells"],
        "depth": int(depth.group(1)) if depth else None,
        "word_ops": {op: dict(sorted(w.items())) for op, w in sorted(word_ops.items())},
    }
    hier = d / f"{name}.hier.txt"
    if hier.exists():
        modules, current = {}, None
        for line in hier.read_text().splitlines():
            m = HIER_MODULE.match(line)
            if m:
                current = "total" if m.group(1) == "design hierarchy" else m.group(1).split("$")[0]
            m = HIER_CELLS.match(line)
            if m and current is not None:
                modules[current] = int(m.group(1))
                current = None
        record["modules"] = modules
    return record


def config_of(d: Path) -> str:
    """The config a result directory belongs to: the nearest ancestor holding the generated package."""
    return next((p.name for p in d.resolve().parents if (p / "gen" / "pans_pkg.sv").exists()), d.parent.name)


def main() -> int:
    d = Path(sys.argv[1])
    names = sorted(p.name[: -len(".stat.json")] for p in d.glob("*.stat.json"))
    record = {"config": config_of(d),
              "flow": "yosys generic, abc -g AND,NAND,OR,NOR,XOR,XNOR,MUX, ppa/abc_medium.script",
              "cores": {n: unit_metrics(d, n) for n in names if not n.startswith("fn_")},
              "functions": {n[3:]: unit_metrics(d, n) for n in names if n.startswith("fn_")}}
    json.dump(record, sys.stdout, indent=2)
    print(file=sys.stdout)
    print(f"{record['config']}: generic gates / depth (gates on the longest path)", file=sys.stderr)
    for kind in ("cores", "functions"):
        for name, m in record[kind].items():
            widest = max((w for ops in m["word_ops"].values() for w in ops), default=0)
            print(f"  {name:16} {m['gates']:>7,} gates  depth {m['depth']:>4}  widest operator {widest:>3} bits",
                  file=sys.stderr)
            for module, gates in m.get("modules", {}).items():
                print(f"    {module:30} {gates:>7,} gates (hierarchy kept)", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
