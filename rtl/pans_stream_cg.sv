// SPDX-License-Identifier: Apache-2.0
// pans_stream_cg: the clock gate of the stream units, a latch on en ANDed with clk.
// No model counterpart. Gated flops hold exactly as the enable muxes of CLOCK_GATE = 0 do.
// Specification: STREAM.md §2.3.
// en must come from registers, directly or through a few gates.
// A mapping with an integrated clock-gating cell substitutes it for this module.
module pans_stream_cg (
  input  logic clk,
  input  logic en,
  output logic gclk
);
  logic en_l;
  always_latch if (!clk) en_l = en;
  assign gclk = clk & en_l;
endmodule
