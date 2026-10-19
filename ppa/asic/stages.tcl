# SPDX-License-Identifier: Apache-2.0
# The worst register-to-register delay into each pipeline register group of a routed core.
# STAGE_REGS is a list of name patterns, one per stage's destination registers, e.g. "q1_* q2_* out_*".
foreach lib $::env(LIBERTY) { read_liberty $lib }
read_db $::env(RESULTS)/6_final.odb
read_sdc $::env(RESULTS)/6_final.sdc
read_spef $::env(RESULTS)/6_final.spef
foreach pat $::env(STAGE_REGS) {
  set pins [get_pins -quiet -hierarchical "u_core.${pat}/D"]
  if {[llength $pins] == 0} { puts "stage $pat: no registers"; continue }
  puts "stage $pat"
  report_checks -to $pins -path_delay max -group_path_count 1 -digits 3
}
