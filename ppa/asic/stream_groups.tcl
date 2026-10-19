# SPDX-License-Identifier: Apache-2.0
# The groups of a routed stream unit by instance name: unit (u_dut/), clock (clkbuf*, clkload*) and wrapper.
# The unit's module is kept through synthesis, so its cells and the buffers on its nets carry the prefix u_dut/.
proc stream_group {name} {
  if {[string match "u_dut/*" $name]} { return unit }
  if {[string match "clkbuf*" $name] || [string match "clkload*" $name]} { return clock }
  return wrapper
}
set stream_groups {unit clock wrapper}
