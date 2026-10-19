#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# synth_generic.sh: generic gate-level synthesis of one top (yosys + slang, SYNTHESIS defined, pinned ABC script).
#   [SYNTH_HIER=1] ppa/synth_generic.sh <out_dir> <name> <top> <source files and read_slang options...>
# Writes <name>.coarse.txt (word-level operators), <name>.stat.json (gates) and <name>.ltp.txt (depth), flattened.
# SYNTH_HIER=1 adds <name>.hier.txt, the gates per module with the hierarchy kept. Gate counts compare only here.
set -euo pipefail
out=$1 name=$2 top=$3
shift 3
here=$(cd "$(dirname "$0")" && pwd)
yosys=${YOSYS:-yosys}
mkdir -p "$out"
"$yosys" -q -m slang -l "$out/$name.log" -p "
  read_slang -D SYNTHESIS -I $here/../rtl $* --top $top;
  synth -flatten -top $top -run :fine;
  tee -q -o $out/$name.coarse.txt stat -width;
  synth -run fine:check -noabc;
  abc -g AND,NAND,OR,NOR,XOR,XNOR,MUX -script $here/abc_medium.script;
  opt_clean;
  tee -q -o $out/$name.stat.json stat -json;
  tee -q -o $out/$name.ltp.txt ltp -noff"
if [ "${SYNTH_HIER:-0}" = 1 ]; then
  "$yosys" -q -m slang -l "$out/$name.hier.log" -p "
    read_slang -D SYNTHESIS -I $here/../rtl $* --top $top --keep-hierarchy;
    synth -top $top -run :fine;
    synth -run fine:check -noabc;
    abc -g AND,NAND,OR,NOR,XOR,XNOR,MUX -script $here/abc_medium.script;
    opt_clean;
    tee -q -o $out/$name.hier.txt stat"
fi
