// SPDX-License-Identifier: Apache-2.0
// pans_enc_stream_functions.svh: the encoder stream unit's arithmetic, included in the body of pans_enc_stream.
// Mirrors ref/pans_stream_ref.h bit for bit. Each function names its model counterpart.
// Specification: STREAM.md §1.4 and §2.2.
// Every function has a single exit and assigns every local on every path.

  typedef logic [STATE_WIDTH-1:0] state_t;  // a lane's state x
  typedef logic [WORD_WIDTH-1:0]  word_t;   // a word, also the quotient x / f
  typedef logic [PRECISION-1:0]   slot_t;   // a start c, a frequency f, a remainder x mod f

  localparam int ESCAPE_LOG2 = $clog2(ESCAPE_SLOTS);  // E = 2^ESCAPE_LOG2

  // ---- Renormalization of one push ------------------------------------------------------------------------------
  // After it x < f 2^W, so x / f has W bits.
  typedef struct packed {
    logic   emit;  // a word leaves
    word_t  word;  // the word, x mod 2^W
    state_t x;     // the state after the emit, below f 2^W
  } renorm_t;

  // The renormalization of a push (c, f) (model: the emit in stream_push).
  function automatic renorm_t push_renorm(input state_t x, input slot_t f);
    renorm_t r;
    r.emit = x[STATE_WIDTH-1:WORD_WIDTH] >= f;
    r.word = x[WORD_WIDTH-1:0];
    r.x    = r.emit ? state_t'(x >> WORD_WIDTH) : x;
    return r;
  endfunction

  // ---- The escape's raw push ------------------------------------------------------------------------------------
  // The emit test is an OR of x's top S bits, and x 2^S + v is a concatenation.

  // The renormalization of the raw push (model: the emit in stream_push_raw).
  function automatic renorm_t push_renorm_raw(input state_t x);
    renorm_t r;
    r.emit = x[STATE_WIDTH-1 -: SYMBOL_WIDTH] != '0;
    r.word = x[WORD_WIDTH-1:0];
    r.x    = r.emit ? state_t'(x >> WORD_WIDTH) : x;
    return r;
  endfunction

  // x 2^S + v for x below 2^(P + W - S) (model: stream_push_raw).
  function automatic state_t push_raw(input logic [STATE_WIDTH-SYMBOL_WIDTH-1:0] x, input logic [SYMBOL_WIDTH-1:0] v);
    state_t y;
    y = {x, v};
    return y;
  endfunction

  // ---- The divider: x / f and x mod f, exact restoring, in steps of DIV_STEP_BITS quotient bits -----------------
  // A step compares t with every multiple m f in parallel and takes the largest that fits.
  localparam int DIV_T_WIDTH = PRECISION + DIV_STEP_BITS;  // t and the multiples m f
  localparam int DIV_MULTS   = 2 ** DIV_STEP_BITS;         // digits 0 ... 2^DIV_STEP_BITS - 1

  typedef struct packed {
    slot_t rem;  // the partial remainder, below f
    word_t dq;   // unused dividend bits on top, quotient digits below
  } divider_t;

  // One restoring step of DIV_STEP_BITS quotient bits (model: part of x / f in stream_push).
  function automatic divider_t div_step(input divider_t d, input slot_t f);
    logic [DIV_T_WIDTH-1:0]                t;
    logic [DIV_MULTS-1:0][DIV_T_WIDTH-1:0] diff;  // t - m f
    logic [DIV_MULTS:0]                    ge;    // t >= m f, with ge[0] = 1 and ge[DIV_MULTS] = 0
    logic [DIV_STEP_BITS-1:0]              digit;
    slot_t                                 rem;
    divider_t                              r;
    t     = {d.rem, d.dq[WORD_WIDTH-1 -: DIV_STEP_BITS]};
    diff  = '0;
    ge    = '0;
    ge[0] = 1'b1;
    diff[0] = t;
    for (int m = 1; m < DIV_MULTS; m++) begin
      ge[m]   = t >= DIV_T_WIDTH'(m) * DIV_T_WIDTH'(f);
      diff[m] = t - DIV_T_WIDTH'(m) * DIV_T_WIDTH'(f);
    end
    digit = '0;
    rem   = '0;
    for (int m = 0; m < DIV_MULTS; m++) begin
      if (ge[m] && !ge[m + 1]) begin
        digit = digit | DIV_STEP_BITS'(m);
        rem   = rem | slot_t'(diff[m]);
      end
    end
    r.rem = rem;
    r.dq  = (d.dq << DIV_STEP_BITS) | word_t'(digit);
    return r;
  endfunction

  // One divider stage of `bits` quotient bits, a constant multiple of DIV_STEP_BITS.
  function automatic divider_t div_stage(input divider_t d, input slot_t f, input int bits);
    divider_t r;
    r = d;
    for (int s = 0; s < bits / DIV_STEP_BITS; s++) r = div_step(r, f);
    return r;
  endfunction

  // ---- Assemble x = q M + c + r ---------------------------------------------------------------------------------
  // c + r < M, so x is the concatenation {q, c + r}.

  // x = q M + c + r (model: the last line of stream_push).
  function automatic state_t assemble(input word_t q, input slot_t r, input slot_t c);
    state_t x;
    x = {q, slot_t'(c + r)};
    return x;
  endfunction

  // The last step fused with the assemble, equal to div_step then assemble.
  function automatic state_t div_assemble(input divider_t d, input slot_t f, input slot_t c);
    logic [DIV_T_WIDTH-1:0]           t;
    logic [DIV_MULTS-1:0][DIV_T_WIDTH-1:0] diff;  // t - m f
    logic [DIV_MULTS-1:0][PRECISION-1:0]   sum;   // c + (t - m f), low P bits
    logic [DIV_MULTS:0]               ge;
    logic [DIV_STEP_BITS-1:0]         digit;
    slot_t                            low;
    state_t                           x;
    t     = {d.rem, d.dq[WORD_WIDTH-1 -: DIV_STEP_BITS]};
    diff  = '0;
    sum   = '0;
    ge    = '0;
    ge[0] = 1'b1;
    diff[0] = t;
    for (int m = 1; m < DIV_MULTS; m++) begin
      ge[m]   = t >= DIV_T_WIDTH'(m) * DIV_T_WIDTH'(f);
      diff[m] = t - DIV_T_WIDTH'(m) * DIV_T_WIDTH'(f);
    end
    for (int m = 0; m < DIV_MULTS; m++) sum[m] = c + slot_t'(diff[m]);
    digit = '0;
    low   = '0;
    for (int m = 0; m < DIV_MULTS; m++) begin
      if (ge[m] && !ge[m + 1]) begin
        digit = digit | DIV_STEP_BITS'(m);
        low   = low | sum[m];
      end
    end
    x = {word_t'((d.dq << DIV_STEP_BITS) | word_t'(digit)), low};
    return x;
  endfunction

  // The last divider stage of `bits` quotient bits, its last step fused with the assemble.
  function automatic state_t div_stage_last(input divider_t d, input slot_t f, input int bits, input slot_t c);
    divider_t r;
    state_t   x;
    r = div_stage(d, f, bits - DIV_STEP_BITS);
    x = div_assemble(r, f, c);
    return x;
  endfunction

  // ---- The first push's lane (n - 1) mod N_LANES ----------------------------------------------------------------
  // Sums 2^b mod N over n's set bits b in a balanced tree of modular adds.
  function automatic int pow2_mod_lanes(input int b);  // 2^b mod N_LANES, a constant
    int v;
    v = 1 % N_LANES;
    for (int k = 0; k < b; k++) v = (2 * v) % N_LANES;
    return v;
  endfunction

  // (a + b) mod N_LANES for a, b < N_LANES.
  function automatic logic [LANE_WIDTH:0] lane_mod_add(input logic [LANE_WIDTH:0] a, input logic [LANE_WIDTH:0] b);
    logic [LANE_WIDTH:0] s;
    logic [LANE_WIDTH:0] m;
    s = a + b;  // a, b < N, so s < 2N <= 2^(LANE_WIDTH + 1)
    m = (s >= (LANE_WIDTH + 1)'(N_LANES)) ? s - (LANE_WIDTH + 1)'(N_LANES) : s;
    return m;
  endfunction

  // The first push's lane (n - 1) mod N_LANES (model: lane_pos(N, n - 1)).
  function automatic logic [LANE_WIDTH-1:0] first_lane(input logic [COUNT_WIDTH-1:0] n);
    logic [LANE_WIDTH:0]   t [COUNT_WIDTH];
    logic [LANE_WIDTH-1:0] lane;  // below N after the reduction
    for (int b = 0; b < COUNT_WIDTH; b++) t[b] = n[b] ? (LANE_WIDTH + 1)'(pow2_mod_lanes(b)) : '0;
    for (int s = 1; s < COUNT_WIDTH; s = 2 * s)
      for (int b = 0; b + s < COUNT_WIDTH; b += 2 * s) t[b] = lane_mod_add(t[b], t[b + s]);
    lane = LANE_WIDTH'(lane_mod_add(t[0], (LANE_WIDTH + 1)'(N_LANES - 1)));
    return lane;
  endfunction
