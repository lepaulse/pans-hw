// SPDX-License-Identifier: Apache-2.0
// pans_dec_stream_functions.svh: the decoder stream unit's arithmetic, included in the body of pans_dec_stream.
// Mirrors ref/pans_stream_ref.h bit for bit. Each function names its model counterpart.
// Specification: STREAM.md §1 and §2.1.
// Every function has a single exit and assigns every local on every path.

  // ---- The array's bytes ----------------------------------------------------------------------------------------
  localparam int DB_WIDTH = unsigned_width(longint'(DATA_BYTES));  // bits that hold 0..DATA_BYTES

  // The bytes of a beat that lie within the array (model: the bound in StreamReader::read_le).
  function automatic logic [DATA_BYTES-1:0] bytes_present(input logic [COUNT_WIDTH-1:0] bytes_left);
    logic                  big;  // bytes_left >= 2^DB_WIDTH, every byte present
    logic [DB_WIDTH-1:0]   low;
    logic [DATA_BYTES-1:0] p;
    big = bytes_left[COUNT_WIDTH-1:DB_WIDTH] != '0;
    low = bytes_left[DB_WIDTH-1:0];
    for (int k = 0; k < DATA_BYTES; k++) p[k] = big || low > DB_WIDTH'(k);
    return p;
  endfunction

  // A beat with the bytes beyond the array zeroed (model: StreamReader::read_le).
  function automatic logic [8*DATA_BYTES-1:0] beat_mask(input logic [8*DATA_BYTES-1:0] data,
                                                        input logic [DATA_BYTES-1:0] present);
    logic [8*DATA_BYTES-1:0] b;
    for (int k = 0; k < DATA_BYTES; k++) b[8*k +: 8] = present[k] ? data[8*k +: 8] : 8'h00;
    return b;
  endfunction

  // ---- One-hot selects ------------------------------------------------------------------------------------------
  // Reads off the issue path are AND-OR over a one-hot of their index (STREAM.md §2.1).

  // The one-hot select of lane j (no model counterpart).
  function automatic logic [N_LANES-1:0] lane_onehot(input logic [LANE_WIDTH-1:0] j);
    logic [N_LANES-1:0] oh;
    for (int k = 0; k < N_LANES; k++) oh[k] = int'(j) == k;
    return oh;
  endfunction

  // ---- The header: one lane's state from the array --------------------------------------------------------------
  localparam int HDR_STATE_BITS = 8 * STATE_BYTES;

  typedef struct packed {
    logic [STATE_WIDTH-1:0] x;
    logic                   err;  // err_header
  } hdr_state_t;

  // A header state and its error flag (model: the header in decode_stream).
  function automatic hdr_state_t header_state(input logic [HDR_STATE_BITS-1:0] v);
    hdr_state_t h;
    h.x   = v[STATE_WIDTH-1:0];
    h.err = v[STATE_WIDTH-1:PRECISION] == '0;  // v mod 2^STATE_WIDTH < L
    for (int j = STATE_WIDTH; j < HDR_STATE_BITS; j++) h.err = h.err || v[j];
    return h;
  endfunction

  // ---- The pop x = ((x >> P) f + (s - c0)) mod 2^STATE_WIDTH, multiply stage ------------------------------------

  // (x >> P) f on STATE_WIDTH bits (model: the product in stream_pop).
  function automatic logic [STATE_WIDTH-1:0] pop_product(input logic [WORD_WIDTH-1:0] x_hi,
                                                         input logic [PRECISION-1:0] freq);
    logic [STATE_WIDTH-1:0] a;
    logic [STATE_WIDTH-1:0] b;
    logic [STATE_WIDTH-1:0] p;
    a = STATE_WIDTH'(x_hi);
    b = STATE_WIDTH'(freq);
    p = a * b;
    return p;
  endfunction

  // s - c0 on P + 1 bits, never negative (model: the offset in stream_pop).
  function automatic logic [PRECISION:0] pop_offset(input logic [PRECISION-1:0] slot, input logic [PRECISION-1:0] start);
    logic [PRECISION:0] d;
    d = {1'b0, slot} - {1'b0, start};
    return d;
  endfunction

  // ---- The pop, add-and-renormalize stage -----------------------------------------------------------------------

  // (x >> P) f + (s - c0) mod 2^STATE_WIDTH (model: the sum in stream_pop).
  function automatic logic [STATE_WIDTH-1:0] pop_add(input logic [STATE_WIDTH-1:0] prod, input logic [PRECISION:0] d);
    logic [STATE_WIDTH-1:0] dx;
    logic [STATE_WIDTH-1:0] x;
    dx = STATE_WIDTH'(signed'(d));
    x  = prod + dx;
    return x;
  endfunction

  // The refill if (x < L) x = (x << W) | next_word() (model: stream_pop, stream_pop_raw).
  typedef struct packed {
    logic [STATE_WIDTH-1:0] x;
    logic                   refill;  // one word read
  } refill_t;

  function automatic refill_t refill(input logic [STATE_WIDTH-1:0] x, input logic [WORD_WIDTH-1:0] word);
    refill_t r;
    r.refill = x[STATE_WIDTH-1:PRECISION] == '0;
    r.x      = r.refill ? {x[PRECISION-1:0], word} : x;
    return r;
  endfunction

  // The escape's raw pop x >> S (model: stream_pop_raw).
  function automatic logic [STATE_WIDTH-1:0] pop_raw(input logic [STATE_WIDTH-1:0] x);
    logic [STATE_WIDTH-1:0] y;
    y = x >> SYMBOL_WIDTH;
    return y;
  endfunction

  // ---- The escape's second word, known at issue from x (STREAM.md §2.1) -----------------------------------------
  // An escape reads two words iff (x >> P) E + (s mod E) < 2^(P + S - W).
  localparam int ESC_SHIFT       = $clog2(ESCAPE_SLOTS);                            // log2 E
  localparam int SECOND_WORD_LSB = max_int(PRECISION + SYMBOL_WIDTH - WORD_WIDTH, 0);  // P + S - W, at least 0

  // Whether an escape from x reads a second word (model: the escape in stream_decode_next).
  function automatic logic escape_second_word(input logic [STATE_WIDTH-1:0] x);
    logic second;
    second = 1'b1;
    for (int b = SECOND_WORD_LSB; b < WORD_WIDTH + ESC_SHIFT; b++)
      second = second && !((b >= ESC_SHIFT) ? x[PRECISION + b - ESC_SHIFT] : x[b]);
    return second;
  endfunction

  // ---- The escape's raw value v = x mod 2^S after the first pop -------------------------------------------------

  // v sign-extended from S bits (model: symbol_of_raw).
  function automatic logic signed [SYMBOL_WIDTH-1:0] raw_symbol(input logic [SYMBOL_WIDTH-1:0] v);
    logic signed [SYMBOL_WIDTH-1:0] s;
    s = signed'(v);
    return s;
  endfunction
