// SPDX-License-Identifier: Apache-2.0
// pans_dec_core: the decoder core, (scale, slot) to (symbol, start, freq, escape).
// Mirrors decode_hw of ref/pans1_ref.h bit for bit, the exact inverse. Arithmetic: rtl/pans_pkg_functions.svh.
// Specification: SPEC.md §1 to §3.
// Valid-only with a fixed latency of 4 cycles. No backpressure.
// Data registers load every cycle, so outputs are meaningful only with out_valid.
// Reset is synchronous and active low. It clears the valid bits only.
module pans_dec_core
  import pans_pkg::*;
(
  input  logic                           clk,
  input  logic                           rst_n,
  // Request
  input  logic                           in_valid,
  input  scale_t                         in_scale,
  input  logic [PRECISION-1:0]           in_slot,
  // Response
  output logic                           out_valid,
  output logic signed [SYMBOL_WIDTH-1:0] out_symbol,
  output logic [PRECISION-1:0]           out_start,
  output logic [PRECISION-1:0]           out_freq,
  output logic                           out_escape
);

  // ---- Valid
  logic q1_valid, q2_valid, q3_valid;

  always_ff @(posedge clk) begin  // synchronous, active-low reset
    if (!rst_n) begin
      q1_valid  <= 1'b0;
      q2_valid  <= 1'b0;
      q3_valid  <= 1'b0;
      out_valid <= 1'b0;
    end else begin
      q1_valid  <= in_valid;
      q2_valid  <= q1_valid;
      q3_valid  <= q2_valid;
      out_valid <= q3_valid;
    end
  end

  // ---- Stage 1: the threshold and the dividend
  // Forms the slot's half, A' and the dividend R'.
  slope_t             s1_slope;
  logic               s1_upper;  // slot >= HALF
  thr_t               s1_thr;
  dividend_t          s1_dividend;

  always_comb begin
    s1_slope    = slope_of(in_scale);
    s1_upper    = in_slot[PRECISION-1];
    s1_thr      = a_threshold(in_slot, s1_upper);
    s1_dividend = dividend(s1_thr.ap, s1_slope.v_shift, s1_slope.special || s1_thr.none);
  end

  logic                       q1_escape;
  logic                       q1_special;
  logic                       q1_upper;
  logic [K_WIDTH-1:0]         q1_mant;
  logic [DIV_MAGIC_WIDTH-1:0] q1_magic;  // the division's constant for this mantissa
  shift_t                     q1_shift;
  dividend_t                  q1_dividend;

  always_ff @(posedge clk) begin
    q1_escape   <= in_slot >= PRECISION'(CODED_SLOTS);
    q1_special  <= s1_slope.special;
    q1_upper    <= s1_upper;
    q1_mant     <= s1_slope.mant;
    q1_magic    <= DIV_MAGIC[in_scale.man];
    q1_shift    <= shift_of(s1_slope.v_shift);
    q1_dividend <= s1_dividend;
  end

  // ---- Stage 2: the division and the clamp
  // Forms |x|, clamped to the half's largest |x|.
  logic [Q_WIDTH-1:0]      s2_q;
  logic                    s2_big;     // the clamp, q above the half's largest |x|
  logic                    s2_wide;    // the half's largest |x| is 2^(SW-1)
  logic [SYMBOL_WIDTH-1:0] s2_abs_x;   // |x|

  always_comb begin
    s2_q      = q_divide(q1_dividend.rp, q1_magic);
    s2_big    = q1_dividend.ovf || (!q1_special && clamp_of(q1_dividend.rp, q1_mant, !q1_upper));
    s2_wide   = LOWER_WIDE ? !q1_upper : 1'b0;
    s2_abs_x  = s2_big ? {s2_wide, {(SYMBOL_WIDTH - 1){!s2_wide}}} : s2_q;
  end

  logic                    q2_escape;
  logic                    q2_special;
  logic                    q2_upper;
  shift_t                  q2_shift;
  logic [MAG_WIDTH-1:0]    q2_rp;
  logic [K_WIDTH-1:0]      q2_q_lo;    // the division's low bits, for its remainder
  logic [K_WIDTH-1:0]      q2_mant;
  logic                    q2_big;
  logic [SYMBOL_WIDTH-1:0] q2_abs_x;

  always_ff @(posedge clk) begin
    q2_escape  <= q1_escape;
    q2_special <= q1_special;
    q2_upper   <= q1_upper;
    q2_shift   <= q1_shift;
    q2_rp      <= q1_dividend.rp;
    q2_q_lo    <= s2_q[K_WIDTH-1:0];
    q2_mant    <= q1_mant;
    q2_big     <= s2_big;
    q2_abs_x   <= s2_abs_x;
  end

  // ---- Stage 3: the exponent magnitudes
  // Forms |P(x)| and |P(x + 1)| with their halves, aligned to a, and the end-symbol tests.
  exponent_magnitudes_t s3_pm;
  logic                 s3_nz;      // |x| != 0
  logic                 s3_side_x;  // x >= 1
  logic                 s3_side_n;  // x + 1 >= 1
  logic [PM_WIDTH-1:0]  s3_pm_x;    // |P(x)|
  logic [PM_WIDTH-1:0]  s3_pm_n;    // |P(x + 1)|
  align_t               s3_align_x;
  align_t               s3_align_n;
  logic                 s3_min;     // C(x) forced to 0 at x = SYMBOL_MIN
  logic                 s3_max;     // C(x + 1) forced to M' at x = SYMBOL_MAX

  always_comb begin
    s3_pm      = exponent_magnitudes(q2_rp, q2_q_lo, q2_mant, q2_big, !q2_upper);
    s3_nz      = q2_abs_x != '0;
    s3_side_x  = q2_upper && s3_nz;
    s3_side_n  = q2_upper || !s3_nz;
    s3_pm_x    = s3_side_x ? s3_pm.lo : s3_pm.hi;
    s3_pm_n    = s3_side_n ? s3_pm.hi : s3_pm.lo;
    s3_align_x = a_of_pm(q2_shift, s3_pm_x);
    s3_align_n = a_of_pm(q2_shift, s3_pm_n);
    s3_min     = !q2_upper && q2_abs_x == SYMBOL_WIDTH'(-SYMBOL_MIN);
    s3_max     = q2_upper && q2_abs_x == SYMBOL_WIDTH'(SYMBOL_MAX);
  end

  logic                    q3_escape;
  logic                    q3_upper;
  logic [SYMBOL_WIDTH-1:0] q3_abs_x;
  logic                    q3_side_x;
  logic                    q3_side_n;
  logic [A_WIDTH-1:0]      q3_a_x;
  logic [A_WIDTH-1:0]      q3_a_n;
  logic                    q3_sat_x;
  logic                    q3_sat_n;

  always_ff @(posedge clk) begin
    q3_escape <= q2_escape;
    q3_upper  <= q2_upper;
    q3_abs_x  <= q2_abs_x;
    q3_side_x <= s3_side_x;
    q3_side_n <= s3_side_n;
    q3_a_x    <= s3_align_x.a0;
    q3_a_n    <= s3_align_n.a0;
    q3_sat_x  <= q2_special || s3_min || s3_align_x.sat;
    q3_sat_n  <= q2_special || s3_max || s3_align_n.sat;
  end

  // ---- Stage 4: C(x), C(x + 1) and the interval
  logic [PRECISION-1:0] s4_cdf_x;
  logic [PRECISION-1:0] s4_cdf_n;
  logic [PRECISION-1:0] s4_freq;  // C is non-decreasing in x, so this never wraps

  always_comb begin
    s4_cdf_x = cdf_from_a(q3_a_x, q3_side_x, q3_sat_x);
    s4_cdf_n = cdf_from_a(q3_a_n, q3_side_n, q3_sat_n);
    s4_freq  = s4_cdf_n - s4_cdf_x;
  end

  always_ff @(posedge clk) begin
    out_escape <= q3_escape;
    out_symbol <= q3_escape ? '0 : q3_upper ? signed'(q3_abs_x) : -signed'(q3_abs_x);
    out_start  <= q3_escape ? PRECISION'(CODED_SLOTS) : s4_cdf_x;
    out_freq   <= q3_escape ? PRECISION'(ESCAPE_SLOTS) : s4_freq;
  end

  // ---- Checks (simulation and formal only)
`ifndef SYNTHESIS
  a_escape_slot: assert property (@(posedge clk) disable iff (!rst_n)
    out_valid && out_escape |-> int'(out_start) == CODED_SLOTS && int'(out_freq) == ESCAPE_SLOTS)
    else $error("pans_dec_core: escape not reported as (M', E)");
  a_monotone: assert property (@(posedge clk) disable iff (!rst_n)
    q3_valid |-> s4_cdf_n >= s4_cdf_x)
    else $error("pans_dec_core: C(x + 1) < C(x)");
  // Every slot below M' lies in the interval decoded for it.
  logic [PRECISION-1:0] chk_slot_q [1:4];
  always_ff @(posedge clk) begin
    chk_slot_q[1] <= in_slot;
    for (int i = 2; i <= 4; i++) chk_slot_q[i] <= chk_slot_q[i - 1];
  end
  a_slot_in_interval: assert property (@(posedge clk) disable iff (!rst_n)
    out_valid && !out_escape |-> out_freq >= 1 && out_start <= chk_slot_q[4]
                                 && int'(chk_slot_q[4]) < int'(out_start) + int'(out_freq))
    else $error("pans_dec_core: slot %0d outside its decoded interval [%0d, +%0d)", chk_slot_q[4], out_start,
                out_freq);
`endif

  // ---- Elaboration checks
  if (ESCAPE_SLOTS < 1) begin : g_check_escape_slots
    $error("pans_dec_core: ESCAPE_SLOTS must be >= 1");
  end
  if (SYMBOL_WIDTH > PRECISION) begin : g_check_symbol_width
    $error("pans_dec_core: SYMBOL_WIDTH must be <= PRECISION");
  end
  // What the arithmetic relies on. The package generator always meets it.
  if (PRECISION < V_FRAC_BITS || longint'(TAIL_MAX) > HALF - longint'(ESCAPE_SLOTS)) begin : g_check_tail
    $error("pans_dec_core: V_FRAC_BITS > PRECISION or T(0) > 2^(P-1) - E");
  end
  if (A_WIDTH != V_FRAC_BITS + M_WIDTH || MAG_WIDTH != SYMBOL_WIDTH + K_WIDTH) begin : g_check_dec_widths
    $error("pans_dec_core: A' or R' width does not match its construction");
  end
  // The contract version this core implements (SPEC.md §4).
  if (CONTRACT_VERSION != 5) begin : g_check_contract
    $error("pans_dec_core: implements contract version 5 (SPEC.md §4)");
  end
  if (SYMBOL_MAX != 2 ** (SYMBOL_WIDTH - 1) - 1 || SYMBOL_MIN != -(2 ** (SYMBOL_WIDTH - 1)) + SYMMETRIC_ALPHABET)
  begin : g_check_alphabet
    $error("pans_dec_core: the alphabet is neither every SYMBOL_WIDTH-bit value nor -SYMBOL_MAX .. SYMBOL_MAX");
  end

endmodule
