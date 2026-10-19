// SPDX-License-Identifier: Apache-2.0
// pans_pkg_functions.svh: the cores' arithmetic, included by the generated package pans_pkg.
// Mirrors ref/pans1_ref.h bit for bit. Each function names its model counterpart.
// Specification: SPEC.md §1 and §2.
// Every width is derived from the range of values its signal must hold.
// Every function has a single exit and assigns every local on every path.

  typedef struct packed {
    logic [SCALE_EXP_WIDTH-1:0] exp;  // the slope's exponent, 0 and all ones special
    logic [SCALE_MAN_WIDTH-1:0] man;
  } scale_t;

  // ---- Width helpers (constant functions) -----------------------------------------------------------------------

  // Bits of a two's-complement signal holding every value in [lo, hi].
  function automatic int signed_width(input longint lo, input longint hi);
    int w;
    w = 1;
    while (lo < -(longint'(1) <<< (w - 1)) || hi > (longint'(1) <<< (w - 1)) - 1) w++;
    return w;
  endfunction

  // Bits of an unsigned signal holding every value in [0, hi].
  function automatic int unsigned_width(input longint hi);
    int w;
    w = 1;
    while ((longint'(1) <<< w) <= hi) w++;
    return w;
  endfunction

  function automatic int max_int(input int a, input int b);
    return (a > b) ? a : b;
  endfunction

  // ---- The slope k = k_m 2^exp2 ---------------------------------------------------------------------------------
  // slope_t carries the alignment shift V_SHIFT in place of exp2.

  localparam int     K_WIDTH        = SCALE_MAN_WIDTH + 1;
  localparam longint K_MIN          = longint'(1) <<< SCALE_MAN_WIDTH;
  localparam longint K_MAX          = (longint'(1) <<< K_WIDTH) - 1;
  localparam int     V_SHIFT_OFFSET = V_FRAC_BITS - 1 - SCALE_MAN_WIDTH - SCALE_BIAS;  // V_SHIFT = offset + exp
  localparam int     V_SHIFT_MIN    = V_SHIFT_OFFSET;
  localparam int     V_SHIFT_WIDTH  = signed_width(longint'(V_SHIFT_OFFSET),
                                                   longint'(V_SHIFT_OFFSET) + (2 ** SCALE_EXP_WIDTH - 1));

  typedef struct packed {
    logic                            special;  // exponent 0 or all ones, k_m = 0
    logic [K_WIDTH-1:0]              mant;     // k_m
    logic signed [V_SHIFT_WIDTH-1:0] v_shift;  // exp2 - 1 + V_FRAC_BITS
  } slope_t;

  // The slope of a scale code (model: slope_of).
  function automatic slope_t slope_of(input scale_t scale);
    slope_t k;
    k.special = (scale.exp == '0) || (scale.exp == SCALE_EXP_WIDTH'(SCALE_EXP_INF));
    k.mant    = k.special ? '0 : {1'b1, scale.man};
    k.v_shift = V_SHIFT_WIDTH'(V_SHIFT_OFFSET + int'(scale.exp));
    return k;
  endfunction

  // ---- The alphabet [SYMBOL_MIN, SYMBOL_MAX] (SPEC.md §1) -------------------------------------------------------
  localparam bit LOWER_WIDE = -longint'(SYMBOL_MIN) > longint'(SYMBOL_MAX);  // -SYMBOL_MIN is one beyond SYMBOL_MAX

  // ---- The exponent product P(x) = (2x - 1) k_m -----------------------------------------------------------------
  // ODD_MIN and ODD_MAX bound 2x - 1 over x in [SYMBOL_MIN - 1, SYMBOL_MAX + 1].

  localparam longint ODD_MIN = 2 * (longint'(SYMBOL_MIN) - 1) - 1;
  localparam longint ODD_MAX = 2 * (longint'(SYMBOL_MAX) + 2) - 1;

  // ---- C = T(a) (lower half, x <= 0) or M' - T(a) (upper half, x >= 1) ------------------------------------------
  // C saturates to 0 or M' at a >= V_LIM.

  localparam longint V_LIM     = longint'(V_SAT) <<< V_FRAC_BITS;
  localparam longint HALF      = longint'(1) <<< (PRECISION - 1);  // the half boundary, C(0) <= HALF <= C(1)

  function automatic int saturating_shift();
    int s;
    s = 0;
    while ((K_MIN <<< s) < V_LIM) s++;
    return s;
  endfunction

  localparam int V_SHIFT_SAT  = saturating_shift();  // the shift that saturates every normal |P|
  localparam int A_WIDTH      = unsigned_width(V_LIM - 1);     // a
  localparam int M_WIDTH      = A_WIDTH - V_FRAC_BITS;         // a_hi in [0, P - 1], all ones gives T = 0

  // T(a) = ((2^(vf+1) - 1 - a_lo) << (P - 2 - vf)) >> a_hi (model: tail).
  localparam int T_WIDTH  = PRECISION - 1;
  localparam int TS_LEFT  = max_int(PRECISION - 2 - V_FRAC_BITS, 0);
  localparam int TS_RIGHT = max_int(V_FRAC_BITS + 2 - PRECISION, 0);
  localparam int XS_WIDTH = V_FRAC_BITS + 1 + TS_LEFT;

  // C from a and the half, sat forcing the half's limit (model: cdf_from_a).
  function automatic logic [PRECISION-1:0] cdf_from_a(input logic [A_WIDTH-1:0] a0, input logic upper, input logic sat);
    logic [V_FRAC_BITS:0]  x;   // {1, ~a_lo} = 2^(vf+1) - 1 - a_lo
    logic [XS_WIDTH-1:0]   xs;
    logic [T_WIDTH-1:0]    xt;  // x scaled to slots
    logic [M_WIDTH-1:0]    m;
    logic [T_WIDTH-1:0]    t;
    logic [PRECISION-1:0]  c;
    x  = {1'b1, ~a0[V_FRAC_BITS-1:0]};
    xs = XS_WIDTH'(x) << TS_LEFT;
    xt = T_WIDTH'(xs >> TS_RIGHT);
    m  = sat ? '1 : a0[A_WIDTH-1:V_FRAC_BITS];
    t  = xt >> m;
    c  = upper ? (~PRECISION'(t)) - PRECISION'(ESCAPE_SLOTS - 1) : PRECISION'(t);
    return c;
  endfunction

  // ---- Alignment: a = floor(|P| 2^V_SHIFT) ----------------------------------------------------------------------
  // One right shifter of |P| << L_SHIFT by u = L_SHIFT - V_SHIFT.
  localparam longint PM_MAX   = ((-ODD_MIN > ODD_MAX) ? -ODD_MIN : ODD_MAX) * K_MAX;  // the largest |P|
  localparam int     PM_WIDTH = unsigned_width(PM_MAX);
  localparam int     L_SHIFT  = V_SHIFT_SAT - 1;              // the largest left shift that does not saturate every |P|
  localparam int     U_MAX    = L_SHIFT - V_SHIFT_MIN;        // the largest right shift u
  localparam int     U_WIDTH  = unsigned_width(longint'(U_MAX));
  localparam int     W_WIDTH  = PM_WIDTH + L_SHIFT;

  typedef struct packed {
    logic               sat;  // V_SHIFT >= V_SHIFT_SAT, so a >= V_LIM
    logic [U_WIDTH-1:0] u;    // L_SHIFT - V_SHIFT, meaningful only when !sat
  } shift_t;

  // The alignment shift of a scale (model: part of a_of_pm).
  function automatic shift_t shift_of(input logic signed [V_SHIFT_WIDTH-1:0] v_shift);
    shift_t s;
    s.sat = int'(v_shift) >= V_SHIFT_SAT;
    s.u   = U_WIDTH'(L_SHIFT - int'(v_shift));
    return s;
  endfunction

  typedef struct packed {
    logic [A_WIDTH-1:0] a0;   // the low A_WIDTH bits of a, meaningful only when !sat
    logic               sat;  // a >= V_LIM, so C saturates
  } align_t;

  // a from |P| and the shift (model: a_of_pm).
  function automatic align_t a_of_pm(input shift_t s, input logic [PM_WIDTH-1:0] pm);
    logic [W_WIDTH-1:0] w;        // |P| << L_SHIFT
    logic               ovf;      // a bit of w >> u at or above A_WIDTH
    align_t             r;
    w       = W_WIDTH'(pm) << L_SHIFT;
    r.a0    = A_WIDTH'(w >> s.u);  // only the low A_WIDTH bits of the shift
    ovf     = 1'b0;
    for (int j = A_WIDTH; j < W_WIDTH; j++) if (w[j] && int'(s.u) <= j - A_WIDTH) ovf = 1'b1;
    r.sat   = s.sat || ovf;
    return r;
  endfunction

  // ---- The decoder's exact inverse (SPEC.md §2) -----------------------------------------------------------------
  // The threshold a+ = min{a : T(a) <= z'}, z' = slot or ~slot - E.
  localparam int NORM_WIDTH = unsigned_width(longint'(PRECISION) + 1);  // P - msb in [1, P + 1]

  typedef struct packed {
    logic [A_WIDTH-1:0] ap;    // A' = a+ - 1, meaningful only when !none
    logic               none;  // a+ = 0, so the symbol is 0
  } thr_t;

  // The threshold and A' of a slot (model: a_plus_hw).
  function automatic thr_t a_threshold(input logic [PRECISION-1:0] slot, input logic upper);
    logic [PRECISION-1:0]   zp;      // z', top bit 0 in both halves
    logic [M_WIDTH-1:0]     n;       // n*
    logic [NORM_WIDTH-1:0]  norm;    // the left shift that puts the leading one at bit P
    logic [PRECISION-1:0]   w;       // the P bits below the leading one of z' << norm, ones below z'
    logic [V_FRAC_BITS-1:0] wf;      // w[P-1 : P-vf]
    logic [A_WIDTH-1:0]     a_plus;
    thr_t                   r;
    zp   = upper ? (~slot) - PRECISION'(ESCAPE_SLOTS) : slot;  // M' - 1 - slot = 2^P - 1 - slot - E
    n    = M_WIDTH'(PRECISION - 1);
    norm = NORM_WIDTH'(PRECISION + 1);
    for (int k = 0; k < PRECISION - 1; k++) begin
      if (zp[k]) begin
        n    = M_WIDTH'(PRECISION - 2 - k);
        norm = NORM_WIDTH'(PRECISION - k);
      end
    end
    w    = PRECISION'((PRECISION + 1)'(zp) << norm);
    for (int j = 0; j < PRECISION; j++) if (j < int'(norm)) w[j] = 1'b1;
    wf   = V_FRAC_BITS'(w >> (PRECISION - V_FRAC_BITS));
    a_plus = {n, ~wf};  // A_WIDTH = M_WIDTH + V_FRAC_BITS
    r.none = a_plus == '0;
    r.ap   = a_plus - A_WIDTH'(1);
    return r;
  endfunction

  // ---- The dividend R' = ceil(a+ 2^-V_SHIFT) - 1 ----------------------------------------------------------------
  // One right shifter on A' << R_LEFT_MAX, ones filling a left shift.
  localparam int MAG_WIDTH  = unsigned_width(DIV_N_MAX);   // = SYMBOL_WIDTH + K_WIDTH
  localparam int R_LEFT_MAX = max_int(0, -(V_SHIFT_MIN + 1));  // the largest left shift of a normal scale (exp >= 1)
  localparam int V_SHIFT_MAX = V_SHIFT_OFFSET + 2 ** SCALE_EXP_WIDTH - 1;
  localparam int RU_WIDTH   = unsigned_width(longint'(V_SHIFT_MAX) + longint'(R_LEFT_MAX));
  localparam int RW_WIDTH   = A_WIDTH + R_LEFT_MAX;

  typedef struct packed {
    logic [MAG_WIDTH-1:0] rp;   // the low MAG_WIDTH bits of R'
    logic                 ovf;  // a bit of R' at or above MAG_WIDTH, so the symbol clamps
  } dividend_t;

  // R' from A' and the scale, 0 when zero is set (model: rp in decode_hw).
  function automatic dividend_t dividend(input logic [A_WIDTH-1:0] ap, input logic signed [V_SHIFT_WIDTH-1:0] v_shift,
                                         input logic zero);
    logic [RU_WIDTH-1:0] u;  // right shift of A' << R_LEFT_MAX
    logic [RW_WIDTH-1:0] w;
    dividend_t           r;
    u     = RU_WIDTH'(int'(v_shift) + R_LEFT_MAX);
    w     = RW_WIDTH'(ap) << R_LEFT_MAX;
    r.rp  = MAG_WIDTH'(w >> u);
    r.ovf = 1'b0;
    for (int j = MAG_WIDTH; j < RW_WIDTH; j++) if (w[j] && int'(u) <= j - MAG_WIDTH) r.ovf = 1'b1;
    for (int j = 0; j < R_LEFT_MAX && j < MAG_WIDTH; j++) if (j + int'(u) < R_LEFT_MAX) r.rp[j] = 1'b1;
    if (zero) begin
      r.rp  = '0;
      r.ovf = 1'b0;
    end
    return r;
  endfunction

  // ---- The division q = floor((R' + k_m) / (2 k_m)) -------------------------------------------------------------
  // A multiply by DIV_MAGIC[man] without the DIV_DROP lowest columns, exact for R' <= DIV_N_MAX.
  localparam int DIVP_WIDTH = MAG_WIDTH + DIV_MAGIC_WIDTH + 1;
  localparam int Q_WIDTH    = SYMBOL_WIDTH;

  // q from R' and DIV_MAGIC[man] (model: div_q).
  function automatic logic [Q_WIDTH-1:0] q_divide(input logic [MAG_WIDTH-1:0] rp, input logic [DIV_MAGIC_WIDTH-1:0] magic);
    logic [DIVP_WIDTH-1:0] acc;
    logic [DIVP_WIDTH-1:0] term;
    acc = DIVP_WIDTH'(DIV_COMP) + DIVP_WIDTH'(longint'(1) <<< (DIV_SHIFT - 1));
    for (int j = 0; j < DIV_MAGIC_WIDTH; j++) begin
      term = '0;
      for (int i = 0; i < MAG_WIDTH; i++) if (i + j >= DIV_DROP) term[i + j] = rp[i] && magic[j];
      acc = acc + term;
    end
    return Q_WIDTH'(acc >> DIV_SHIFT);
  endfunction

  // ---- The clamp, decided from R' beside the multiplier ---------------------------------------------------------
  // q > |x|max iff R' >= (2^SW + 1) k_m in a wide lower half, else iff R' >= (2^SW - 1) k_m.

  // Whether q exceeds the half's largest |x|, for k_m != 0 (model: big in decode_hw).
  function automatic logic clamp_of(input logic [MAG_WIDTH-1:0] rp, input logic [K_WIDTH-1:0] mant, input logic lower);
    logic [K_WIDTH-1:0] r_hi;
    logic [K_WIDTH-1:0] r_ll;      // r_lo's low K_WIDTH bits
    logic               top_ones;  // r_lo[SW-1:K_WIDTH] all ones
    logic               top_any;
    logic               c;
    r_hi     = rp[MAG_WIDTH-1:SYMBOL_WIDTH];
    r_ll     = rp[K_WIDTH-1:0];
    top_ones = 1'b1;
    top_any  = 1'b0;
    for (int j = K_WIDTH; j < SYMBOL_WIDTH; j++) begin
      top_ones = top_ones && rp[j];
      top_any  = top_any || rp[j];
    end
    if (LOWER_WIDE ? lower : 1'b0) c = (r_hi > mant) || (r_hi == mant && (top_any || r_ll >= mant));
    else                     c = (r_hi >= mant) || (r_hi == mant - K_WIDTH'(1) && top_ones && r_ll >= -mant);
    return c;
  endfunction

  // ---- The exponent magnitudes from the division ----------------------------------------------------------------
  // (2 |x| -+ 1) k_m = R' - rem and R' + (2 k_m - rem), rem = (R' + k_m) mod 2 k_m (SPEC.md §2).
  typedef struct packed {
    logic [PM_WIDTH-1:0] lo;  // (2 |x| - 1) k_m, unused for |x| = 0
    logic [PM_WIDTH-1:0] hi;  // (2 |x| + 1) k_m
  } exponent_magnitudes_t;

  // The exponent magnitudes from R' and the division's low bits (model: pm_lo, pm_hi in decode_hw).
  function automatic exponent_magnitudes_t exponent_magnitudes(input logic [MAG_WIDTH-1:0] rp,
                                                               input logic [K_WIDTH-1:0] q_lo,
                                                               input logic [K_WIDTH-1:0] mant, input logic big,
                                                               input logic lower);
    logic [K_WIDTH:0]     rem;
    logic [K_WIDTH:0]     rem_c;   // 2 k_m - rem in (0, 2 k_m]
    logic [PM_WIDTH-1:0]  big_lo;  // the clamp's lo, 0 for the special scales
    exponent_magnitudes_t p;
    rem    = (K_WIDTH + 1)'(rp[K_WIDTH:0]) + (K_WIDTH + 1)'(mant) - {K_WIDTH'(mant * q_lo), 1'b0};
    rem_c  = {mant, 1'b0} - rem;
    big_lo = (PM_WIDTH'(mant) << SYMBOL_WIDTH)
           - ((LOWER_WIDE ? lower : 1'b0) ? PM_WIDTH'(mant) : PM_WIDTH'(mant) + (PM_WIDTH'(mant) << 1));
    p.lo   = big ? big_lo : PM_WIDTH'(rp) - PM_WIDTH'(rem);
    p.hi   = big ? big_lo + (PM_WIDTH'(mant) << 1) : PM_WIDTH'(rp) + PM_WIDTH'(rem_c);
    return p;
  endfunction
