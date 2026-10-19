// SPDX-License-Identifier: Apache-2.0
// pans_functions_wrapped.sv: one module per function of pans_pkg the cores call, shift_of aside.
// Not part of the design. Each module only calls its function, for the per-function generic gate counts.

module fn_slope_of import pans_pkg::*; (input scale_t scale, output slope_t k);
  assign k = slope_of(scale);
endmodule

module fn_a_of_pm import pans_pkg::*; (
  input shift_t s, input logic [PM_WIDTH-1:0] pm, output align_t al
);
  assign al = a_of_pm(s, pm);
endmodule

module fn_cdf_from_a import pans_pkg::*; (
  input logic [A_WIDTH-1:0] a0, input logic upper, input logic sat, output logic [PRECISION-1:0] c
);
  assign c = cdf_from_a(a0, upper, sat);
endmodule

module fn_a_threshold import pans_pkg::*; (input logic [PRECISION-1:0] slot, input logic upper, output thr_t thr);
  assign thr = a_threshold(slot, upper);
endmodule

module fn_dividend import pans_pkg::*; (
  input logic [A_WIDTH-1:0] ap, input logic signed [V_SHIFT_WIDTH-1:0] v_shift, input logic zero,
  output dividend_t r
);
  assign r = dividend(ap, v_shift, zero);
endmodule

module fn_q_divide import pans_pkg::*; (
  input logic [MAG_WIDTH-1:0] rp, input logic [DIV_MAGIC_WIDTH-1:0] magic, output logic [Q_WIDTH-1:0] q
);
  assign q = q_divide(rp, magic);
endmodule

module fn_exponent_magnitudes import pans_pkg::*; (
  input logic [MAG_WIDTH-1:0] rp, input logic [K_WIDTH-1:0] q_lo, input logic [K_WIDTH-1:0] mant, input logic big,
  input logic lower, output exponent_magnitudes_t p
);
  assign p = exponent_magnitudes(rp, q_lo, mant, big, lower);
endmodule

module fn_clamp_of import pans_pkg::*; (
  input logic [MAG_WIDTH-1:0] rp, input logic [K_WIDTH-1:0] mant, input logic lower, output logic c
);
  assign c = clamp_of(rp, mant, lower);
endmodule
