# SPDX-License-Identifier: Apache-2.0
# Area of a routed stream unit by group of ppa/asic/stream_groups.tcl (make asic-stream-<cfg>).
# Standard cells only, tap and filler cells left out. One line per group:
#   area <group> <cells> <area um^2> <sequential cells> <sequential area um^2>
source [file join [file dirname [info script]] stream_groups.tcl]
read_db $::env(RESULTS)/6_final.odb
set dbu [[ord::get_db_tech] getDbUnitsPerMicron]
foreach g $stream_groups {
  set cells($g) 0
  set area($g) 0.0
  set seq_cells($g) 0
  set seq_area($g) 0.0
}
foreach inst [[ord::get_db_block] getInsts] {
  set master [$inst getMaster]
  if {[$master getType] ne "CORE"} { continue }
  set a [expr {double([$master getWidth]) * [$master getHeight] / ($dbu * $dbu)}]
  set g [stream_group [$inst getName]]
  incr cells($g)
  set area($g) [expr {$area($g) + $a}]
  if {[$master isSequential]} {
    incr seq_cells($g)
    set seq_area($g) [expr {$seq_area($g) + $a}]
  }
}
foreach g $stream_groups {
  puts [format "area %s %d %.3f %d %.3f" $g $cells($g) $area($g) $seq_cells($g) $seq_area($g)]
}
