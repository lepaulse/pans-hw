// SPDX-License-Identifier: Apache-2.0
// pans_enc_core: the encoder core, (scale, symbol) to (start, freq, escape).
// Mirrors encode of ref/pans1_ref.h bit for bit. Arithmetic: rtl/pans_pkg_functions.svh.
// Specification: SPEC.md §1 and §3.
// Valid-only with a fixed latency of 2 cycles. No backpressure.
// Data registers load every cycle, so outputs are meaningful only with out_valid.
// Reset is synchronous and active low. It clears the valid bits only.
module pans_enc_core
  import pans_pkg::*;
(
  input  logic                           clk,
  input  logic                           rst_n,
  // Request
  input  logic                           in_valid,
  input  scale_t                         in_scale,
  input  logic signed [SYMBOL_WIDTH-1:0] in_symbol,
  // Response
  output logic                           out_valid,
  output logic [PRECISION-1:0]           out_start,
  output logic [PRECISION-1:0]           out_freq,
  output logic                           out_escape
);

  // ---- Valid
  logic q1_valid;

  always_ff @(posedge clk) begin  // synchronous, active-low reset
    if (!rst_n) begin
      q1_valid  <= 1'b0;
      out_valid <= 1'b0;
    end else begin
      q1_valid  <= in_valid;
      out_valid <= q1_valid;
    end
  end

  // ---- Stage 1: the exponent products
  // Forms |P(x + 1)| by one multiply, |P(x)| by one add, and the end-symbol tests.
  localparam int XT_WIDTH = SYMBOL_WIDTH - 1;

  slope_t                     s1_slope;
  logic                       s1_positive;      // x >= 1, C(x) in the upper half
  logic                       s1_non_negative;  // x >= 0, C(x + 1) in the upper half
  logic [XT_WIDTH-1:0]        s1_xt;            // x for x >= 0, ~x = -x - 1 for x < 0
  logic [PM_WIDTH-1:0]        s1_pm_next;       // |P(x + 1)|
  logic signed [K_WIDTH:0]    s1_sk;            // s k_m, s = -1 (x >= 1), 0 (x = 0), +1 (x <= -1)
  logic [PM_WIDTH-1:0]        s1_pm_symbol;     // |P(x)| = |P(x + 1)| + 2 s k_m
  logic                       s1_min;           // C(x) forced to 0 at x = SYMBOL_MIN
  logic                       s1_max;           // C(x + 1) forced to its limit at x = SYMBOL_MAX

  always_comb begin
    s1_slope        = slope_of(in_scale);
    s1_positive     = in_symbol >= 1;
    s1_non_negative = in_symbol >= 0;
    s1_xt           = in_symbol[XT_WIDTH-1:0] ^ {XT_WIDTH{in_symbol[SYMBOL_WIDTH-1]}};
    s1_pm_next      = PM_WIDTH'({s1_xt, 1'b1}) * PM_WIDTH'(s1_slope.mant);
    s1_sk           = s1_positive ? -signed'({1'b0, s1_slope.mant}) : s1_non_negative ? '0 : signed'({1'b0, s1_slope.mant});
    s1_pm_symbol    = s1_pm_next + unsigned'(PM_WIDTH'(s1_sk) <<< 1);  // PM_WIDTH'(signed) sign-extends
    s1_min          = in_symbol == SYMBOL_WIDTH'(SYMBOL_MIN);
    s1_max          = in_symbol == SYMBOL_WIDTH'(SYMBOL_MAX);
    if (!LOWER_WIDE) begin  // symmetric alphabet: -2^(SW-1) gets [0, 0) and escapes
      s1_min = s1_min || in_symbol == SYMBOL_WIDTH'(SYMBOL_MIN - 1);
      s1_max = s1_max || in_symbol == SYMBOL_WIDTH'(SYMBOL_MIN - 1);
    end
  end

  logic                q1_sat_symbol;  // C(x) saturated by a special scale or s1_min
  logic                q1_sat_next;    // C(x + 1) saturated by a special scale or s1_max
  shift_t              q1_shift;
  logic                q1_positive;
  logic                q1_non_negative;
  logic [PM_WIDTH-1:0] q1_pm_symbol;
  logic [PM_WIDTH-1:0] q1_pm_next;

  always_ff @(posedge clk) begin
    q1_sat_symbol   <= s1_slope.special || s1_min;
    q1_sat_next     <= s1_slope.special || s1_max;
    q1_shift        <= shift_of(s1_slope.v_shift);
    q1_positive     <= s1_positive;
    q1_non_negative <= s1_non_negative;
    q1_pm_symbol    <= s1_pm_symbol;
    q1_pm_next      <= s1_pm_next;
  end

  // ---- Stage 2: the interval
  // Aligns both products to a and forms C(x), C(x + 1) and the interval.
  align_t               s2_align_symbol;
  align_t               s2_align_next;
  logic [PRECISION-1:0] s2_cdf_symbol;
  logic [PRECISION-1:0] s2_cdf_next;
  logic [PRECISION-1:0] s2_freq;  // C is non-decreasing in x, so this never wraps
  logic                 s2_coded;

  always_comb begin
    s2_align_symbol = a_of_pm(q1_shift, q1_pm_symbol);
    s2_align_next   = a_of_pm(q1_shift, q1_pm_next);
    s2_cdf_symbol   = cdf_from_a(s2_align_symbol.a0, q1_positive, q1_sat_symbol || s2_align_symbol.sat);
    s2_cdf_next     = cdf_from_a(s2_align_next.a0, q1_non_negative, q1_sat_next || s2_align_next.sat);
    s2_freq         = s2_cdf_next - s2_cdf_symbol;
    s2_coded        = s2_freq != '0;
  end

  always_ff @(posedge clk) begin
    out_start  <= s2_coded ? s2_cdf_symbol : PRECISION'(CODED_SLOTS);
    out_freq   <= s2_coded ? s2_freq       : PRECISION'(ESCAPE_SLOTS);
    out_escape <= !s2_coded;
  end

  // ---- Checks (simulation and formal only)
`ifndef SYNTHESIS
  a_coded_interval: assert property (@(posedge clk) disable iff (!rst_n)
    out_valid && !out_escape |-> out_freq >= 1 && int'(out_start) + int'(out_freq) <= CODED_SLOTS)
    else $error("pans_enc_core: coded symbol with an empty interval or one past M'");
  a_escape_slot: assert property (@(posedge clk) disable iff (!rst_n)
    out_valid && out_escape |-> int'(out_start) == CODED_SLOTS && int'(out_freq) == ESCAPE_SLOTS)
    else $error("pans_enc_core: escape not reported as (M', E)");
  a_monotone: assert property (@(posedge clk) disable iff (!rst_n)
    q1_valid |-> s2_cdf_next >= s2_cdf_symbol)
    else $error("pans_enc_core: C(symbol + 1) < C(symbol)");
`endif

  // ---- Elaboration checks
  if (ESCAPE_SLOTS < 1) begin : g_check_escape_slots
    $error("pans_enc_core: ESCAPE_SLOTS must be >= 1");
  end
  if (SYMBOL_WIDTH > PRECISION) begin : g_check_symbol_width
    $error("pans_enc_core: SYMBOL_WIDTH must be <= PRECISION");
  end
  // What the arithmetic relies on. The package generator always meets it.
  if (PRECISION < V_FRAC_BITS || longint'(TAIL_MAX) > HALF - longint'(ESCAPE_SLOTS)) begin : g_check_tail
    $error("pans_enc_core: V_FRAC_BITS > PRECISION or T(0) > 2^(P-1) - E");
  end
  // The contract version this core implements (SPEC.md §4).
  if (CONTRACT_VERSION != 5) begin : g_check_contract
    $error("pans_enc_core: implements contract version 5 (SPEC.md §4)");
  end
  if (SYMBOL_MAX != 2 ** (SYMBOL_WIDTH - 1) - 1 || SYMBOL_MIN != -(2 ** (SYMBOL_WIDTH - 1)) + SYMMETRIC_ALPHABET)
  begin : g_check_alphabet
    $error("pans_enc_core: the alphabet is neither every SYMBOL_WIDTH-bit value nor -SYMBOL_MAX .. SYMBOL_MAX");
  end

endmodule
