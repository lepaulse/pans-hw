# SPDX-License-Identifier: Apache-2.0
# Power of a routed stream unit from its gate-level activity (make power-stream-<cfg>).
# It prints power.tcl's report, then the power by group, and with UNIT set the unit's cells by register group.
# Standard cells only, tap, filler and decap cells left out of the counts, power in W. One line per group:
#   power <group> <cells> <internal> <switching> <leakage> <total>        unit, clock, wrapper: the design
#   split <group> <cells> <internal> <switching> <leakage> <total>        the unit's groups: they add up to unit
source [file join [file dirname [info script]] power.tcl]
source [file join [file dirname [info script]] stream_groups.tcl]
source [file join [file dirname [info script]] stream_regs.tcl]
array set scene_keys {}
set scene [sta::parse_scene scene_keys]
set unit [expr {[info exists ::env(UNIT)] ? $::env(UNIT) : ""}]
set split_groups [expr {$unit eq "" ? {} : [stream_power_groups $unit]}]
foreach g [concat $stream_groups $split_groups] {
  set cells($g) 0
  set sum($g) {0.0 0.0 0.0 0.0}
}
foreach inst [get_cells *] {
  set name [get_full_name $inst]
  set g [stream_group $name]
  set master [get_property $inst ref_name]
  set counted [expr {![string match "FILLCELL*" $master] && ![string match "TAPCELL*" $master]
                      && ![string match "FILLER*" $master] && ![string match "DECAP*" $master]}]
  set p [sta::instance_power $inst $scene]
  if {$counted} { incr cells($g) }
  set sum($g) [lmap a $sum($g) b $p {expr {$a + $b}}]
  if {$unit ne "" && $g eq "unit"} {
    set s [stream_power_group $unit [string range $name 6 end] $master]
    if {$counted} { incr cells($s) }
    set sum($s) [lmap a $sum($s) b $p {expr {$a + $b}}]
  }
}
foreach g $stream_groups {
  puts [format "power %s %d %.6e %.6e %.6e %.6e" $g $cells($g) {*}$sum($g)]
}
foreach g $split_groups {
  puts [format "split %s %d %.6e %.6e %.6e %.6e" $g $cells($g) {*}$sum($g)]
}
