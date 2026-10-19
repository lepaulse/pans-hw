# SPDX-License-Identifier: Apache-2.0
# Power of a core's post-route netlist from its gate-level activity (make power-<cfg>).
# It reads the routed parasitics and the target clock, and annotates every net from the waveform.
# LIBERTY lists the platform's liberty files.
foreach lib $::env(LIBERTY) { read_liberty $lib }
read_db $::env(RESULTS)/6_final.odb
read_sdc $::env(RESULTS)/6_final.sdc
read_spef $::env(RESULTS)/6_final.spef
read_vcd -scope $::env(VCD_SCOPE) $::env(VCD)
report_power -digits 6
