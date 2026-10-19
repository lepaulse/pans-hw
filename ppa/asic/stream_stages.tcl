# SPDX-License-Identifier: Apache-2.0
# The worst setup path into each pipeline stage of a routed stream unit (make asic-stream-<cfg>).
# Environment: UNIT dec or enc, LIBERTY (the platform's liberty files), RESULTS. The groups are
# ppa/asic/stream_regs.tcl's (STREAM.md §4.1).
# A group's endpoints are its registers' D pins, a clock gate latch's enable included.
# The encoder's renorm stage is every path into qd_* from elsewhere, its divider the paths from qd_* to qd_*.
# One line per group, slack in the libraries' time unit (Nangate45 ns, ASAP7 ps) at the routed parasitics, then the
# design's worst slack:
#   stage <group> <endpoints> <slack> <startpoint pin> <endpoint pin>      (no endpoints: stage <group> 0 - - -)
#   worst <slack>
source [file join [file dirname [info script]] stream_regs.tcl]
foreach lib $::env(LIBERTY) { read_liberty $lib }
read_db $::env(RESULTS)/6_final.odb
read_sdc $::env(RESULTS)/6_final.sdc
read_spef $::env(RESULTS)/6_final.spef
set unit $::env(UNIT)

set groups [stream_stage_groups $unit]
foreach g [concat $groups pipe] { set ends($g) {} }
set cg_out {}
set pipe_regs {}
set other_regs {}
foreach inst [get_cells *] {
  set name [get_full_name $inst]
  if {![string match "u_dut/*" $name]} { continue }
  set master [get_property $inst ref_name]
  set g [stream_stage_group $unit [string range $name 6 end] $master]
  if {$g eq ""} { continue }
  if {$g eq "pipe"} { lappend pipe_regs $inst } else { lappend other_regs $inst }
  foreach p [get_pins -of_objects $inst] {
    set lp [get_property $p lib_pin_name]
    if {$lp eq "D"} { lappend ends($g) $p }
    if {$g eq "cg_enable" && $lp eq "Q"} { lappend cg_out $p }
  }
}

proc stage_line {label n pes} {
  if {$n == 0 || [llength $pes] == 0} { return "$label $n - - -" }
  set pe [lindex $pes 0]
  set start [get_full_name [[[$pe path] start_path] pin]]
  return [format "%s %d %s %s %s" $label $n [sta::format_time [$pe slack] 3] $start [get_full_name [$pe pin]]]
}

set starts_other [concat $other_regs [all_inputs]]
foreach g $groups {
  set pes {}
  if {$g eq "cg_check"} {
    set n [llength $cg_out]
    if {$n > 0} { set pes [find_timing_paths -through $cg_out -path_delay max -group_path_count 1] }
  } elseif {$g eq "renorm" || $g eq "divider"} {
    set n [llength $ends(pipe)]
    if {$g eq "renorm"} { set from $starts_other } else { set from $pipe_regs }
    if {$n > 0} { set pes [find_timing_paths -from $from -to $ends(pipe) -path_delay max -group_path_count 1] }
  } else {
    set n [llength $ends($g)]
    if {$n > 0} { set pes [find_timing_paths -to $ends($g) -path_delay max -group_path_count 1] }
  }
  puts "stage [stage_line $g $n $pes]"
}
puts [format "worst %s" [sta::format_time [sta::worst_slack_cmd max] 3]]
