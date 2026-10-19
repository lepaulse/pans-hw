#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""ppa/asic_metrics.py: collects ASIC (ORFS, Nangate45 or ASAP7) results into one JSON record.

    python3 ppa/asic_metrics.py build/<cfg>/asic                       the cores (make asic-<cfg>)
    python3 ppa/asic_metrics.py --stream <dec|enc> <run directory>     a stream unit (make asic-stream-<cfg>)

JSON on stdout, a summary on stderr. The figures are ORFS's final metrics after place and route at the config's
target period: area without tap and filler cells, its sequential part, repair buffers, worst slack, TNS and Fmax.
A stream unit adds the area by group, the worst slack into each stage, and its parameters (STREAM.md §4.1).
The platform is the PLATFORM of the run's config.mk. Times are converted to ns from the libraries' unit.
"""
import json
import re
import sys
from pathlib import Path

ORFS = "ORFS openroad/orfs:26Q4-137-g9c431405a"
# per platform: its name in the summaries, the record's flow, and the time unit of its libraries (ORFS's reports)
PLATFORMS = {
    "nangate45": {"name": "Nangate45", "flow": f"{ORFS}, nangate45, ADDER_MAP_FILE off", "time_unit": "ns"},
    "asap7": {"name": "ASAP7", "flow": f"{ORFS}, asap7 RVT TT NLDM (0.7 V, 25 C), ADDER_MAP_FILE off",
              "time_unit": "ps"},
}
TOPS = {"enc": "pans_enc_core_timed", "dec": "pans_dec_core_timed"}
# stream units: (the timing wrapper, the unit's module)
STREAM_TOPS = {"dec": ("pans_dec_stream_timed", "pans_dec_stream"), "enc": ("pans_enc_stream_timed", "pans_enc_stream")}


def platform_of(work: Path) -> str:
    """The PLATFORM of an ORFS run (its WORK_HOME), from its config.mk."""
    m = re.search(r"^export PLATFORM\s*=\s*(\S+)$", (work / "config.mk").read_text(), re.M)
    if not m or m.group(1) not in PLATFORMS:
        raise ValueError(f"{work}/config.mk: no known PLATFORM")
    return m.group(1)


def to_ns(value, platform: str):
    """A time of ORFS's reports in ns."""
    return value / 1000 if PLATFORMS[platform]["time_unit"] == "ps" else value


def run_metrics(work: Path, top: str) -> dict:
    """The totals of one ORFS run (its WORK_HOME) of the design `top`."""
    platform = platform_of(work)
    m = json.loads((work / "logs" / platform / top / "base" / "6_report.json").read_text())
    area = m["finish__design__instance__area"] - m.get("finish__design__instance__area__class:tap_cell", 0.0)
    return {
        "area_um2": round(area, 1),
        "sequential_area_um2": round(m.get("finish__design__instance__area__class:sequential_cell", 0.0), 1),
        "repair_buffer_area_um2": round(m.get("finish__design__instance__area__class:timing_repair_buffer", 0.0), 1),
        "worst_slack_ns": round(to_ns(m["finish__timing__setup__ws"], platform), 3),
        "tns_ns": round(to_ns(m["finish__timing__setup__tns"], platform), 3),
        "fmax_mhz": round(m["finish__timing__fmax"] / 1e6, 1),
    }


def cores(d: Path) -> int:
    period = float((d / "period_ns").read_text())
    platform = PLATFORMS[platform_of(d / "enc")]
    record = {"config": d.parent.name, "flow": platform["flow"], "target_period_ns": period,
              "cores": {c: run_metrics(d / c, TOPS[c]) for c in TOPS}}
    json.dump(record, sys.stdout, indent=2)
    print(file=sys.stdout)
    print(f"{record['config']}: {platform['name']} post-route at a {period:g} ns target", file=sys.stderr)
    for core, m in record["cores"].items():
        print(f"  {core}: {m['area_um2']:>9,.0f} um^2 (repair buffers {m['repair_buffer_area_um2']:,.0f})  "
              f"worst slack {m['worst_slack_ns']:+.2f} ns  fmax {m['fmax_mhz']:.0f} MHz", file=sys.stderr)
    return 0


def stream(unit: str, d: Path) -> int:
    top, module = STREAM_TOPS[unit]
    period = float((d / "period_ns").read_text())
    platform = platform_of(d)
    config_mk = (d / "config.mk").read_text()
    params = re.search(r"^export VERILOG_TOP_PARAMS\s*=\s*(.*)$", config_mk, re.M).group(1).split()
    groups = {}
    for line in (d / "area.rpt").read_text().splitlines():
        f = line.split()
        if len(f) == 6 and f[0] == "area":
            groups[f[1]] = {
                "cells": int(f[2]), "area_um2": round(float(f[3]), 1), "flops": int(f[4]),
                "sequential_area_um2": round(float(f[5]), 1)}
    stages = {}
    stage_file = d / "stages.rpt"
    for line in (stage_file.read_text().splitlines() if stage_file.exists() else []):
        f = line.split()
        if len(f) == 6 and f[0] == "stage" and f[3] != "-":
            stages[f[1]] = {"endpoints": int(f[2]), "slack_ns": round(to_ns(float(f[3]), platform), 3), "from": f[4],
                            "to": f[5]}
    total = run_metrics(d, top)
    split_sum = sum(g["area_um2"] for g in groups.values())
    if set(groups) != {"unit", "clock", "wrapper"} or abs(split_sum - total["area_um2"]) > 0.5:
        print(f"{d}/area.rpt: the groups ({split_sum:.1f} um^2) do not add up to the design ({total['area_um2']})",
              file=sys.stderr)
        return 1
    # the d/<unit>_stream/<params>/asic[_<platform>] layout: the config is three levels up
    record = {"config": d.parents[2].name, "unit": module, "flow": PLATFORMS[platform]["flow"],
              "target_period_ns": period,
              "params": dict(zip(params[0::2], (int(v) for v in params[1::2]))),
              "total": total, "groups": groups, "stages": stages}
    json.dump(record, sys.stdout, indent=2)
    print(file=sys.stdout)
    u, c, w = groups["unit"], groups["clock"], groups["wrapper"]
    print(f"{record['config']} {module} {' '.join(f'{k}={v}' for k, v in record['params'].items())}: "
          f"{PLATFORMS[platform]['name']} post-route at a {period:g} ns target, inside its timing wrapper",
          file=sys.stderr)
    print(f"  unit {u['area_um2']:,.0f} um^2 ({u['flops']} flops, {u['sequential_area_um2']:,.0f} um^2), clock tree "
          f"{c['area_um2']:,.0f}, wrapper {w['area_um2']:,.0f} ({w['flops']} flops): "
          f"total {total['area_um2']:,.0f} um^2 (repair buffers {total['repair_buffer_area_um2']:,.0f})  "
          f"worst slack {total['worst_slack_ns']:+.3f} ns  fmax {total['fmax_mhz']:.0f} MHz", file=sys.stderr)
    if stages:
        print("  worst slack into each stage, ns: " + ", ".join(f"{k} {v['slack_ns']:+.3f}" for k, v in stages.items()),
              file=sys.stderr)
    return 0


def main(argv: list) -> int:
    if len(argv) == 2:
        return cores(Path(argv[1]))
    if len(argv) == 4 and argv[1] == "--stream" and argv[2] in STREAM_TOPS:
        return stream(argv[2], Path(argv[3]))
    print(__doc__, file=sys.stderr)
    return 2


if __name__ == "__main__":
    sys.exit(main(sys.argv))
