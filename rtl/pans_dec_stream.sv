// SPDX-License-Identifier: Apache-2.0
// pans_dec_stream: the decoder stream unit, N_LANES interleaved ANS lanes over one pans_dec_core.
// Mirrors decode_stream of ref/pans_stream_ref.h bit for bit, corrupt arrays included.
// Specification: STREAM.md §2.1. Arithmetic: rtl/pans_dec_stream_functions.svh.
// All channels are valid/ready. STREAM.md §2.1 gives their rules.
// A decoding step takes six cycles from issue to the write of the lane's state.
// Reset is synchronous and active low. It clears control state only.
module pans_dec_stream
  import pans_pkg::*;
#(
  parameter  int N_LANES     = 8,   // interleaved lanes
  parameter  int DATA_BYTES  = 4,   // data_in beat bytes, a multiple of STATE_BYTES
  parameter  int FIFO_WORDS  = 8,   // word FIFO entries, >= 4 and > DATA_BYTES / (W/8)
  parameter  int OUT_DEPTH   = 7,   // output FIFO entries, >= 7 for one symbol per cycle
  parameter  int COUNT_WIDTH = 32,  // width of the count ports and the job counters
  parameter  bit CLOCK_GATE  = 1,   // Clock gates on the rarely written registers. 0 uses enable muxes.
  localparam int LANE_WIDTH  = (N_LANES > 1) ? $clog2(N_LANES) : 1,
  localparam int POS_WIDTH   = unsigned_width(((longint'(1) <<< COUNT_WIDTH) - 2) / longint'(N_LANES))  // t < n / N
) (
  input  logic                           clk,
  input  logic                           rst_n,
  // job_in: one job, taken when idle
  input  logic                           job_in_valid,
  output logic                           job_in_ready,
  input  logic [COUNT_WIDTH-1:0]         job_in_n_sym,
  input  logic [COUNT_WIDTH-1:0]         job_in_n_bytes,
  // data_in: the byte array, DATA_BYTES bytes a beat, little-endian
  input  logic                           data_in_valid,
  output logic                           data_in_ready,
  input  logic [8*DATA_BYTES-1:0]        data_in_data,
  // scale_in: the scale input b of a lane's next symbol
  input  logic                           scale_in_valid,
  output logic                           scale_in_ready,
  input  logic [LANE_WIDTH-1:0]          scale_in_lane,
  input  logic [B_WIDTH-1:0]             scale_in_b,
  // sym_out: the symbols in decode order, global index t * N_LANES + lane
  output logic                           sym_out_valid,
  input  logic                           sym_out_ready,
  output logic [LANE_WIDTH-1:0]          sym_out_lane,
  output logic [POS_WIDTH-1:0]           sym_out_t,
  output logic signed [SYMBOL_WIDTH-1:0] sym_out_symbol,
  output logic                           sym_out_escape,
  // done_out: the end check and the job's counts
  output logic                           done_out_valid,
  input  logic                           done_out_ready,
  output logic                           done_out_ok,
  output logic [COUNT_WIDTH-1:0]         done_out_n_escapes,
  output logic [COUNT_WIDTH-1:0]         done_out_words_consumed,
  output logic                           done_out_err_words,
  output logic                           done_out_err_header
);

`include "pans_dec_stream_functions.svh"

  // ---- Geometry (STREAM.md §1.5) --------------------------------------------------------------------------------
  // A state is two words, so the header is 2 N_LANES words.
  localparam int WORD_BYTES     = WORD_WIDTH / 8;
  // max with 1 lets a bad DATA_BYTES reach the elaboration checks
  localparam int BEAT_WORDS     = max_int(DATA_BYTES / WORD_BYTES, 1);
  localparam int BEAT_STATES    = max_int(DATA_BYTES / STATE_BYTES, 1);
  localparam int HDR_BYTES      = N_LANES * STATE_BYTES;
  localparam int HDR_WORDS      = 2 * N_LANES;
  localparam int HDR_BEATS      = (N_LANES + BEAT_STATES - 1) / BEAT_STATES;
  localparam int HDR_LAST_LANES = N_LANES - (HDR_BEATS - 1) * BEAT_STATES;  // states in the last header beat
  localparam int HDR_LAST_WORDS = HDR_WORDS - (HDR_BEATS - 1) * BEAT_WORDS;  // header words in the last header beat
  localparam int HB_WIDTH       = unsigned_width(longint'(HDR_BEATS) - 1);
  localparam int BW_WIDTH       = unsigned_width(longint'(BEAT_WORDS));

  // ---- Pipeline -------------------------------------------------------------------------------------------------
  localparam int POP_LOOP   = 6;  // cycles from issue to the write of the lane's state
  localparam int STATE_L    = NUM_SLOTS;  // L = M, the state's lower bound
  localparam int RES_WIDTH  = unsigned_width(longint'(2 * POP_LOOP));          // words reserved, at most 2 per step
  localparam int WF_PTR     = (FIFO_WORDS > 1) ? $clog2(FIFO_WORDS) : 1;
  localparam int WF_CNT     = unsigned_width(longint'(FIFO_WORDS));
  localparam int WC_WIDTH   = max_int(WF_CNT, RES_WIDTH) + 1;                   // count and reservation compare
  localparam int OF_PTR     = (OUT_DEPTH > 1) ? $clog2(OUT_DEPTH) : 1;
  localparam int OF_CNT     = unsigned_width(longint'(OUT_DEPTH));

  typedef enum logic [2:0] {ST_IDLE, ST_LOAD, ST_RUN, ST_FIN, ST_DONE} phase_e;

  // An output entry. The output regenerates (lane, t) from a counter pair.
  typedef struct packed {
    logic signed [SYMBOL_WIDTH-1:0] symbol;
    logic                           escape;
  } sym_t;

  function automatic logic [WF_PTR-1:0] wf_ptr_add(input logic [WF_PTR-1:0] p, input int k);
    int s;
    s = int'(p) + k;
    if (s >= FIFO_WORDS) s = s - FIFO_WORDS;
    return WF_PTR'(s);
  endfunction

  // ---- State ----------------------------------------------------------------------------------------------------
  phase_e                  phase_q;
  logic [STATE_WIDTH-1:0]  x_q [N_LANES];          // the lane states
  scale_t                  scale_q [N_LANES];
  logic [N_LANES-1:0]      scale_present_q;        // the lane holds a scale
  logic [N_LANES-1:0]      scale_more_q;           // the lane still needs a scale
  logic [N_LANES-1:0]      lane_busy_q;            // a step of the lane in flight, for N_LANES < POP_LOOP
  logic [N_LANES-1:0]      second_word_q;          // an escape from the lane's state reads two words

  logic [COUNT_WIDTH-1:0]  sym_left_q;             // symbols not issued yet
  logic [COUNT_WIDTH-1:0]  in_bytes_left_q;        // array bytes not yet taken
  logic                    in_more_q;              // in_bytes_left != 0, registered
  logic                    sym_done_q;             // sym_left == 0, registered
  logic                    bytes_exact_q;          // n_bytes is the header plus whole words
  logic                    word_dropped_q;         // the drain dropped a whole word
  logic [HB_WIDTH-1:0]     hdr_beat_q;             // header beats loaded
  logic [LANE_WIDTH-1:0]   rr_lane_q;              // the next step's lane
  logic [RES_WIDTH-1:0]    reserved_q;             // FIFO words reserved by the steps in flight
  logic [OF_CNT-1:0]       credit_q;               // free output entries minus steps in flight
  logic [LANE_WIDTH-1:0]   out_lane_q;             // lane and position of the next symbol out
  logic [POS_WIDTH-1:0]    out_t_q;

  logic [WORD_WIDTH-1:0]   wf_mem_q [FIFO_WORDS];
  logic [WF_PTR-1:0]       wf_rd_q;
  logic [WF_PTR-1:0]       wf_wr_q;
  logic [WF_CNT-1:0]       wf_count_q;

  sym_t                    of_mem_q [OUT_DEPTH];
  logic [OF_PTR-1:0]       of_rd_q;
  logic [OF_PTR-1:0]       of_wr_q;
  logic [OF_CNT-1:0]       of_count_q;

  logic [COUNT_WIDTH-1:0]  words_consumed_q;
  logic [COUNT_WIDTH-1:0]  n_escapes_q;
  logic                    err_words_q;
  logic                    err_header_q;
  logic                    ok_q;

  // ---- Job ------------------------------------------------------------------------------------------------------
  logic job_take;
  logic job_hdr_fits;  // n_bytes covers the header

  assign job_in_ready = phase_q == ST_IDLE;
  assign job_take     = job_in_valid && job_in_ready;
  assign job_hdr_fits = job_in_n_bytes >= COUNT_WIDTH'(HDR_BYTES);

  // ---- Input: the header, then the word FIFO's writes -----------------------------------------------------------
  // After the header every beat starts on a word boundary.
  logic                    in_more;      // array bytes left to take
  logic                    in_hdr;       // the header phase
  logic                    in_drain;     // the drain of the array's unused beats
  logic                    in_room;      // the FIFO can take a whole beat of words
  logic                    in_beat;      // a beat taken
  logic                    in_hdr_step;  // a header beat loaded, or skipped past the array
  logic [DATA_BYTES-1:0]   in_present;   // byte k of the beat lies within the array
  logic                    in_word_left; // a whole word of the array is still to come
  logic [8*DATA_BYTES-1:0] in_bytes;     // the beat, bytes beyond the array zeroed
  hdr_state_t              in_state [BEAT_STATES];
  logic                    in_hdr_err;
  logic [WORD_WIDTH-1:0]   in_word [BEAT_WORDS];
  logic [BW_WIDTH-1:0]     in_wr_avail;  // stream word slots in this beat
  logic [BW_WIDTH-1:0]     in_wr_n;      // words written into the FIFO
  logic [WORD_WIDTH-1:0]   in_wr_word [BEAT_WORDS];
  logic                    pipe_busy;    // a step in stages 1 to 6

  assign in_more       = in_more_q;
  assign in_hdr        = phase_q == ST_LOAD;
  assign in_drain      = phase_q == ST_RUN && sym_done_q && !pipe_busy;
  assign in_room       = wf_count_q <= WF_CNT'(FIFO_WORDS - BEAT_WORDS);
  assign data_in_ready = in_more && (in_hdr || (phase_q == ST_RUN && (in_drain || in_room)));
  assign in_beat       = data_in_valid && data_in_ready;
  assign in_hdr_step   = in_hdr && (in_beat || !in_more);
  assign in_present    = bytes_present(in_bytes_left_q);
  assign in_word_left  = in_present[WORD_BYTES-1];

  always_comb begin
    in_bytes   = beat_mask(data_in_data, in_present);
    in_hdr_err = 1'b0;
    for (int s = 0; s < BEAT_STATES; s++) begin
      in_state[s] = header_state(in_bytes[8*STATE_BYTES*s +: 8*STATE_BYTES]);
      // beat slot s holds a lane's state unless it lies past the last lane
      if (int'(hdr_beat_q) != HDR_BEATS - 1 || s < HDR_LAST_LANES) in_hdr_err = in_hdr_err || in_state[s].err;
    end
    for (int k = 0; k < BEAT_WORDS; k++) in_word[k] = data_in_data[8*WORD_BYTES*k +: WORD_WIDTH];
    // Stream words start at word HDR_LAST_WORDS of the last header beat. Only whole words are written.
    if (in_hdr) in_wr_avail = (int'(hdr_beat_q) == HDR_BEATS - 1) ? BW_WIDTH'(BEAT_WORDS - HDR_LAST_WORDS) : '0;
    else        in_wr_avail = BW_WIDTH'(BEAT_WORDS);
    in_wr_n = '0;
    for (int k = 0; k < BEAT_WORDS; k++) begin
      // the modulo only keeps an excluded index in range
      if (!in_hdr)                              in_wr_word[k] = in_word[k];
      else if (k + HDR_LAST_WORDS < BEAT_WORDS) in_wr_word[k] = in_word[(k + HDR_LAST_WORDS) % BEAT_WORDS];
      else                                      in_wr_word[k] = '0;
      if (in_beat && !in_drain && k < int'(in_wr_avail) && in_present[(k + (in_hdr ? HDR_LAST_WORDS : 0) + 1) * WORD_BYTES - 1])
        in_wr_n = BW_WIDTH'(k + 1);
    end
  end

  // ---- Stage 0: issue -------------------------------------------------------------------------------------------
  logic [PRECISION-1:0]   s0_slot;
  scale_t                 s0_scale;
  logic                   s0_escape;    // the core's escape, predicted from slot >= M'
  logic [RES_WIDTH-1:0]   s0_need;
  logic                   s0_words_ok;
  logic                   s0_issue;
  logic                   s0_last_lane;

  // The issue's reads are mux trees, not AND-OR (STREAM.md §2.1).
  always_comb begin
    s0_slot      = x_q[rr_lane_q][PRECISION-1:0];
    s0_scale     = scale_q[rr_lane_q];
    s0_escape    = s0_slot >= PRECISION'(CODED_SLOTS);
    s0_need      = (s0_escape && second_word_q[rr_lane_q]) ? RES_WIDTH'(2) : RES_WIDTH'(1);
    s0_words_ok  = !in_word_left || WC_WIDTH'(wf_count_q) >= WC_WIDTH'(reserved_q) + WC_WIDTH'(s0_need);
    s0_issue     = phase_q == ST_RUN && !sym_done_q && scale_present_q[rr_lane_q] && !lane_busy_q[rr_lane_q]
                   && s0_words_ok && credit_q != '0;
    s0_last_lane = int'(rr_lane_q) == N_LANES - 1;
  end

  // ---- Stages 1-4: the core and the tag pipeline ----------------------------------------------------------------
  // Stage 4 reads the lane's state into q4_x.
  logic                           q1_valid, q2_valid, q3_valid, q4_valid;
  logic [LANE_WIDTH-1:0]          q1_lane, q2_lane, q3_lane, q4_lane;
  logic [N_LANES-1:0]             s4_oh;        // one-hot of q3_lane
  logic [STATE_WIDTH-1:0]         s4_x;
  logic [STATE_WIDTH-1:0]         q4_x;

  always_comb begin
    s4_oh = lane_onehot(q3_lane);
    s4_x  = '0;
    for (int j = 0; j < N_LANES; j++) s4_x |= x_q[j] & {STATE_WIDTH{s4_oh[j]}};
  end
  logic                           c_valid;
  logic signed [SYMBOL_WIDTH-1:0] c_symbol;
  logic [PRECISION-1:0]           c_start;
  logic [PRECISION-1:0]           c_freq;
  logic                           c_escape;

  pans_dec_core u_core (
    .clk        (clk),
    .rst_n      (rst_n),
    .in_valid   (s0_issue),
    .in_scale   (s0_scale),
    .in_slot    (s0_slot),
    .out_valid  (c_valid),
    .out_symbol (c_symbol),
    .out_start  (c_start),
    .out_freq   (c_freq),
    .out_escape (c_escape)
  );

  always_ff @(posedge clk) begin
    q1_lane <= rr_lane_q;
    q2_lane <= q1_lane;
    q3_lane <= q2_lane;
    q4_lane <= q3_lane;
    q4_x    <= s4_x;
  end

  // ---- Stage 5: multiply ----------------------------------------------------------------------------------------
  // Forms (x >> P) f and s - c.
  logic [WORD_WIDTH-1:0]  s5_x_hi;
  logic [STATE_WIDTH-1:0] s5_prod;
  logic [PRECISION:0]     s5_d;

  always_comb begin
    s5_x_hi = q4_x[STATE_WIDTH-1:PRECISION];
    s5_prod = pop_product(s5_x_hi, c_freq);
    s5_d    = pop_offset(q4_x[PRECISION-1:0], c_start);
  end

  logic                           q5_valid;
  logic [LANE_WIDTH-1:0]          q5_lane;
  logic [STATE_WIDTH-1:0]         q5_prod;
  logic [PRECISION:0]             q5_d;
  logic                           q5_escape;
  logic                           q5_second_word;  // an escape of this step reads two words
  logic signed [SYMBOL_WIDTH-1:0] q5_symbol;

  always_ff @(posedge clk) begin
    q5_lane        <= q4_lane;
    q5_prod        <= s5_prod;
    q5_d           <= s5_d;
    q5_escape      <= c_escape;
    q5_second_word <= second_word_q[q4_lane];
    q5_symbol      <= c_symbol;
  end

  // ---- Stage 6: add and renormalize -----------------------------------------------------------------------------
  // Forms the new state, refills up to two words, writes the lane.
  logic [WORD_WIDTH-1:0]          s6_w0;        // the FIFO's head words, 0 where absent
  logic [WORD_WIDTH-1:0]          s6_w1;
  logic [STATE_WIDTH-1:0]         s6_x1;        // (x >> P) f + (s - c)
  refill_t                        s6_r1;
  logic [SYMBOL_WIDTH-1:0]        s6_v;         // an escape's raw value
  refill_t                        s6_r2;
  logic                           s6_refill2;
  logic [STATE_WIDTH-1:0]         s6_x;         // the lane's state after the step
  logic [1:0]                     s6_pops;      // words the step reads
  logic [1:0]                     s6_pops_fifo; // words the step pops from the FIFO
  logic                           s6_cnt0;      // the FIFO holds no word
  logic                           s6_cnt1;      // the FIFO holds one word
  logic                           s6_over;      // a read past the array, which reads 0 and sets err_words
  logic signed [SYMBOL_WIDTH-1:0] s6_symbol;
  // FIFO pointer and count candidates from the registers. The late pops only select.
  logic [WF_PTR-1:0]              wf_rd_p1;     // rd + 1, rd + 2
  logic [WF_PTR-1:0]              wf_rd_p2;
  logic [WF_CNT-1:0]              wf_cnt_w;     // count + the words written this cycle

  always_comb begin
    s6_cnt0      = wf_count_q == '0;
    s6_cnt1      = wf_count_q == WF_CNT'(1);
    wf_rd_p1     = wf_ptr_add(wf_rd_q, 1);
    wf_rd_p2     = wf_ptr_add(wf_rd_q, 2);
    // the head words, AND-OR over the entries
    s6_w0        = '0;
    s6_w1        = '0;
    for (int e = 0; e < FIFO_WORDS; e++) begin
      s6_w0 |= wf_mem_q[e] & {WORD_WIDTH{!s6_cnt0 && int'(wf_rd_q) == e}};
      s6_w1 |= wf_mem_q[e] & {WORD_WIDTH{!s6_cnt0 && !s6_cnt1 && int'(wf_rd_p1) == e}};
    end
    s6_x1        = pop_add(q5_prod, q5_d);
    s6_r1        = refill(s6_x1, s6_w0);
    s6_v         = s6_r1.x[SYMBOL_WIDTH-1:0];
    s6_r2        = refill(pop_raw(s6_r1.x), s6_r1.refill ? s6_w1 : s6_w0);
    s6_refill2   = q5_escape && s6_r2.refill;
    s6_x         = q5_escape ? s6_r2.x : s6_r1.x;
    s6_pops      = 2'(s6_r1.refill) + 2'(s6_refill2);
    // more pops than the FIFO holds
    s6_over      = ((s6_r1.refill || s6_refill2) && s6_cnt0) || (s6_r1.refill && s6_refill2 && s6_cnt1);
    s6_pops_fifo = s6_over ? 2'(wf_count_q) : s6_pops;
    s6_symbol    = q5_escape ? raw_symbol(s6_v) : q5_symbol;
    wf_cnt_w     = wf_count_q + WF_CNT'(in_wr_n);
  end

  // ---- Output FIFO and credit -----------------------------------------------------------------------------------
  logic of_take;
  logic out_last_lane;

  assign out_last_lane = int'(out_lane_q) == N_LANES - 1;

  sym_t of_head;  // the entry at the read pointer (AND-OR)

  always_comb begin
    of_head = '0;
    for (int e = 0; e < OUT_DEPTH; e++) of_head |= of_mem_q[e] & {$bits(sym_t){int'(of_rd_q) == e}};
  end

  assign sym_out_valid  = of_count_q != '0;
  assign sym_out_lane   = out_lane_q;
  assign sym_out_t      = out_t_q;
  assign sym_out_symbol = of_head.symbol;
  assign sym_out_escape = of_head.escape;
  assign of_take        = sym_out_valid && sym_out_ready;

  // Output FIFO entries, one clock gate each under CLOCK_GATE.
  for (genvar e = 0; e < OUT_DEPTH; e++) begin : g_out
    logic we;
    sym_t entry;
    assign we          = q5_valid && int'(of_wr_q) == e;
    assign of_mem_q[e] = entry;
    if (CLOCK_GATE) begin : g_cg
      logic gclk;
      pans_stream_cg u_cg (.clk(clk), .en(we), .gclk(gclk));
      always_ff @(posedge gclk) entry <= '{symbol: s6_symbol, escape: q5_escape};
    end else begin : g_en
      always_ff @(posedge clk) if (we) entry <= '{symbol: s6_symbol, escape: q5_escape};
    end
  end

  // ---- Scales ---------------------------------------------------------------------------------------------------
  // b becomes the core's scale code on its way into the lane's register (SPEC.md §1).
  logic   scale_take;
  scale_t scale_in_code;

  pans_scale_of_b u_scale_of_b (.b(scale_in_b), .scale(scale_in_code));

  assign scale_in_ready = int'(scale_in_lane) < N_LANES && !scale_present_q[scale_in_lane]
                          && scale_more_q[scale_in_lane];
  assign scale_take     = scale_in_valid && scale_in_ready;

  // Lane scale registers, one clock gate each under CLOCK_GATE.
  for (genvar j = 0; j < N_LANES; j++) begin : g_scale
    logic   we;
    scale_t sc;
    assign we         = scale_take && int'(scale_in_lane) == j;
    assign scale_q[j] = sc;
    if (CLOCK_GATE) begin : g_cg
      logic gclk;
      pans_stream_cg u_cg (.clk(clk), .en(we), .gclk(gclk));
      always_ff @(posedge gclk) sc <= scale_in_code;
    end else begin : g_en
      always_ff @(posedge clk) if (we) sc <= scale_in_code;
    end
  end

  // ---- Control --------------------------------------------------------------------------------------------------
  assign pipe_busy = q1_valid || q2_valid || q3_valid || q4_valid || q5_valid;

  logic [RES_WIDTH-1:0] q5_need;
  assign q5_need = (q5_escape && q5_second_word) ? RES_WIDTH'(2) : RES_WIDTH'(1);

  // The step's pop count, registered. words_consumed adds it one cycle after stage 6.
  logic       q6_valid;
  logic [1:0] q6_pops;

  always_ff @(posedge clk) begin
    q6_pops <= s6_pops_fifo;
  end

  always_ff @(posedge clk) begin  // synchronous, active-low reset
    if (!rst_n) begin
      phase_q         <= ST_IDLE;
      q1_valid        <= 1'b0;
      q2_valid        <= 1'b0;
      q3_valid        <= 1'b0;
      q4_valid        <= 1'b0;
      q5_valid        <= 1'b0;
      q6_valid        <= 1'b0;
      scale_present_q <= '0;
      scale_more_q    <= '0;
      lane_busy_q     <= '0;
      reserved_q      <= '0;
      credit_q        <= OF_CNT'(OUT_DEPTH);
      wf_rd_q         <= '0;
      wf_wr_q         <= '0;
      wf_count_q      <= '0;
      of_rd_q         <= '0;
      of_wr_q         <= '0;
      of_count_q      <= '0;
      in_more_q       <= 1'b0;
      sym_done_q      <= 1'b0;
    end else begin
      // Registered flags for in_bytes_left != 0 and sym_left == 0.
      if (job_take)                                            in_more_q <= job_in_n_bytes != '0;
      else if (in_beat && !(in_bytes_left_q > COUNT_WIDTH'(DATA_BYTES))) in_more_q <= 1'b0;
      if (job_take)                                            sym_done_q <= job_in_n_sym == '0;
      else if (s0_issue && sym_left_q == COUNT_WIDTH'(1))       sym_done_q <= 1'b1;

      q1_valid <= s0_issue;
      q2_valid <= q1_valid;
      q3_valid <= q2_valid;
      q4_valid <= q3_valid;
      q5_valid <= q4_valid;
      q6_valid <= q5_valid;

      // Phase
      case (phase_q)
        ST_IDLE: if (job_take) phase_q <= ST_LOAD;
        ST_LOAD: if (in_hdr_step && int'(hdr_beat_q) == HDR_BEATS - 1) phase_q <= ST_RUN;
        ST_RUN:  if (in_drain && of_count_q == '0 && !in_more) phase_q <= ST_FIN;
        ST_FIN:  phase_q <= ST_DONE;
        ST_DONE: if (done_out_ready) phase_q <= ST_IDLE;
        default: phase_q <= ST_IDLE;
      endcase

      // Word FIFO. A new job flushes it.
      if (job_take) begin
        wf_rd_q    <= '0;
        wf_wr_q    <= '0;
        wf_count_q <= '0;
      end else begin
        wf_wr_q <= wf_ptr_add(wf_wr_q, int'(in_wr_n));
        // A read past the array empties the FIFO.
        if (q5_valid && s6_over) begin
          wf_rd_q    <= wf_wr_q;
          wf_count_q <= WF_CNT'(in_wr_n);
        end else if (q5_valid && s6_pops == 2'd2) begin
          wf_rd_q    <= wf_rd_p2;
          wf_count_q <= wf_cnt_w - WF_CNT'(2);
        end else if (q5_valid && s6_pops == 2'd1) begin
          wf_rd_q    <= wf_rd_p1;
          wf_count_q <= wf_cnt_w - WF_CNT'(1);
        end else begin
          wf_count_q <= wf_cnt_w;
        end
      end

      // Reservations, credit, output FIFO
      reserved_q <= reserved_q + (s0_issue ? s0_need : '0) - (q5_valid ? q5_need : '0);
      credit_q   <= credit_q - OF_CNT'(s0_issue) + OF_CNT'(of_take);
      of_wr_q    <= q5_valid ? ((int'(of_wr_q) == OUT_DEPTH - 1) ? '0 : of_wr_q + OF_PTR'(1)) : of_wr_q;
      of_rd_q    <= of_take ? ((int'(of_rd_q) == OUT_DEPTH - 1) ? '0 : of_rd_q + OF_PTR'(1)) : of_rd_q;
      of_count_q <= of_count_q + OF_CNT'(q5_valid) - OF_CNT'(of_take);

      // Scale slots, filled by scale_in and freed by stage 6. The two never meet on one lane.
      for (int j = 0; j < N_LANES; j++) begin
        if (scale_take && int'(scale_in_lane) == j)   scale_present_q[j] <= 1'b1;
        else if (q5_valid && int'(q5_lane) == j)      scale_present_q[j] <= 1'b0;
        // the lane needs a scale for its next position
        if (job_take)                                 scale_more_q[j] <= job_in_n_sym > COUNT_WIDTH'(j);
        else if (s0_issue && int'(rr_lane_q) == j)    scale_more_q[j] <= sym_left_q > COUNT_WIDTH'(N_LANES);
        if (N_LANES >= POP_LOOP)                      lane_busy_q[j] <= 1'b0;
        else if (s0_issue && int'(rr_lane_q) == j)    lane_busy_q[j] <= 1'b1;
        else if (q5_valid && int'(q5_lane) == j)      lane_busy_q[j] <= 1'b0;
      end
    end
  end

  // Job registers, loaded at a job's start. No reset.
  always_ff @(posedge clk) begin
    if (job_take) begin
      sym_left_q       <= job_in_n_sym;
      // n_bytes - header is whole words iff n_bytes is
      bytes_exact_q    <= job_hdr_fits && (job_in_n_bytes & COUNT_WIDTH'(WORD_BYTES - 1)) == '0;
      word_dropped_q   <= 1'b0;
      hdr_beat_q       <= '0;
      rr_lane_q        <= '0;
      err_words_q      <= !job_hdr_fits;  // the header reads past the array
      err_header_q     <= 1'b0;
    end else begin
      if (in_beat && in_drain && in_word_left) word_dropped_q <= 1'b1;
      if (in_hdr_step) begin
        hdr_beat_q   <= hdr_beat_q + HB_WIDTH'(1);
        err_header_q <= err_header_q || in_hdr_err;
      end
      if (s0_issue) begin
        sym_left_q <= sym_left_q - COUNT_WIDTH'(1);
        rr_lane_q  <= s0_last_lane ? '0 : rr_lane_q + LANE_WIDTH'(1);
      end
      if (q5_valid) err_words_q <= err_words_q || s6_over;
    end
  end

  // Event counters, one clock gate each under CLOCK_GATE.
  logic cnt_esc_en, cnt_wc_en, cnt_out_en, cnt_in_en;

  assign cnt_esc_en = job_take || (q5_valid && q5_escape);
  assign cnt_wc_en  = job_take || q6_valid;
  assign cnt_out_en = job_take || of_take;
  assign cnt_in_en  = !rst_n || job_take || in_beat;

  if (CLOCK_GATE) begin : g_cnt_cg
    logic gclk_esc, gclk_wc, gclk_out, gclk_in;
    pans_stream_cg u_cg_esc (.clk(clk), .en(cnt_esc_en), .gclk(gclk_esc));
    pans_stream_cg u_cg_wc  (.clk(clk), .en(cnt_wc_en),  .gclk(gclk_wc));
    pans_stream_cg u_cg_out (.clk(clk), .en(cnt_out_en), .gclk(gclk_out));
    pans_stream_cg u_cg_in  (.clk(clk), .en(cnt_in_en),  .gclk(gclk_in));
    always_ff @(posedge gclk_esc) n_escapes_q <= job_take ? '0 : n_escapes_q + COUNT_WIDTH'(1);
    always_ff @(posedge gclk_wc)  words_consumed_q <= job_take ? '0 : words_consumed_q + COUNT_WIDTH'(q6_pops);
    always_ff @(posedge gclk_out) begin  // the output's (lane, t) in decode order
      out_lane_q <= (job_take || out_last_lane) ? '0 : out_lane_q + LANE_WIDTH'(1);
      out_t_q    <= job_take ? '0 : out_last_lane ? out_t_q + POS_WIDTH'(1) : out_t_q;
    end
    always_ff @(posedge gclk_in) begin
      if (!rst_n)        in_bytes_left_q <= '0;
      else if (job_take) in_bytes_left_q <= job_in_n_bytes;
      else               in_bytes_left_q <= (in_bytes_left_q > COUNT_WIDTH'(DATA_BYTES))
                                            ? in_bytes_left_q - COUNT_WIDTH'(DATA_BYTES) : '0;
    end
  end else begin : g_cnt_en
    always_ff @(posedge clk) begin
      if (cnt_esc_en) n_escapes_q <= job_take ? '0 : n_escapes_q + COUNT_WIDTH'(1);
      if (cnt_wc_en)  words_consumed_q <= job_take ? '0 : words_consumed_q + COUNT_WIDTH'(q6_pops);
      if (cnt_out_en) begin
        out_lane_q <= (job_take || out_last_lane) ? '0 : out_lane_q + LANE_WIDTH'(1);
        out_t_q    <= job_take ? '0 : out_last_lane ? out_t_q + POS_WIDTH'(1) : out_t_q;
      end
      if (cnt_in_en) begin  // reset, a job or a beat
        if (!rst_n)        in_bytes_left_q <= '0;
        else if (job_take) in_bytes_left_q <= job_in_n_bytes;
        else               in_bytes_left_q <= (in_bytes_left_q > COUNT_WIDTH'(DATA_BYTES))
                                              ? in_bytes_left_q - COUNT_WIDTH'(DATA_BYTES) : '0;
      end
    end
  end

  // Lane states with their at-L and second-word flags, written from the header or by stage 6.
  logic [STATE_WIDTH-1:0] x_wr [BEAT_STATES];
  logic [BEAT_STATES-1:0] x_wr_at_L;
  logic [BEAT_STATES-1:0] x_wr_second_word;
  logic [N_LANES-1:0]     at_L_q;

  always_comb begin
    for (int s = 0; s < BEAT_STATES; s++) begin
      x_wr[s]             = in_hdr ? in_state[s].x : s6_x;
      x_wr_at_L[s]        = x_wr[s] == STATE_WIDTH'(STATE_L);
      x_wr_second_word[s] = escape_second_word(x_wr[s]);
    end
  end

  for (genvar j = 0; j < N_LANES; j++) begin : g_lane
    logic                   we;
    logic [STATE_WIDTH-1:0] x;
    logic                   at_L;
    logic                   second_word;
    assign we               = (in_hdr_step && int'(hdr_beat_q) == j / BEAT_STATES) || (q5_valid && int'(q5_lane) == j);
    assign x_q[j]           = x;
    assign at_L_q[j]        = at_L;
    assign second_word_q[j] = second_word;
    if (CLOCK_GATE) begin : g_cg
      logic gclk;
      pans_stream_cg u_cg (.clk(clk), .en(we), .gclk(gclk));
      always_ff @(posedge gclk) begin
        x           <= x_wr[j % BEAT_STATES];
        at_L        <= x_wr_at_L[j % BEAT_STATES];
        second_word <= x_wr_second_word[j % BEAT_STATES];
      end
    end else begin : g_en
      always_ff @(posedge clk) if (we) begin
        x           <= x_wr[j % BEAT_STATES];
        at_L        <= x_wr_at_L[j % BEAT_STATES];
        second_word <= x_wr_second_word[j % BEAT_STATES];
      end
    end
  end

  // Word FIFO entries, one clock gate each under CLOCK_GATE.
  for (genvar e = 0; e < FIFO_WORDS; e++) begin : g_word
    logic                  we;
    logic [WORD_WIDTH-1:0] wd;
    logic [WORD_WIDTH-1:0] word;
    always_comb begin
      we = 1'b0;
      wd = '0;
      for (int k = 0; k < BEAT_WORDS; k++) begin
        if (k < int'(in_wr_n) && wf_ptr_add(wf_wr_q, k) == WF_PTR'(e)) begin
          we = 1'b1;
          wd = in_wr_word[k];
        end
      end
    end
    assign wf_mem_q[e] = word;
    if (CLOCK_GATE) begin : g_cg
      logic gclk;
      pans_stream_cg u_cg (.clk(clk), .en(we), .gclk(gclk));
      always_ff @(posedge gclk) word <= wd;
    end else begin : g_en
      always_ff @(posedge clk) if (we) word <= wd;
    end
  end

  // ---- Done: the end check (STREAM.md §1.6) ---------------------------------------------------------------------
  // Every byte is consumed iff no byte is left over and no word is left or dropped.
  logic fin_at_L;   // every lane back at L
  logic fin_bytes;  // every byte consumed

  always_comb begin
    fin_at_L  = &at_L_q;
    fin_bytes = bytes_exact_q && wf_count_q == '0 && !word_dropped_q;
  end

  always_ff @(posedge clk) begin
    if (phase_q == ST_FIN)
      ok_q <= fin_at_L && fin_bytes && !err_words_q && !err_header_q;
  end

  assign done_out_valid          = phase_q == ST_DONE;
  assign done_out_ok             = ok_q;
  assign done_out_n_escapes      = n_escapes_q;
  assign done_out_words_consumed = words_consumed_q;
  assign done_out_err_words      = err_words_q;
  assign done_out_err_header     = err_header_q;

  // ---- Checks (simulation and formal only) ----------------------------------------------------------------------
`ifndef SYNTHESIS
  // Shadow counts of the steps issued, popped and output in this job.
  longint sva_issued, sva_popped, sva_out;
  int     sva_in_flight;
  logic   sva_err_any;

  always_ff @(posedge clk) begin
    if (!rst_n || job_take) begin
      sva_issued <= 0;
      sva_popped <= 0;
      sva_out    <= 0;
    end else begin
      sva_issued <= sva_issued + longint'(s0_issue);
      sva_popped <= sva_popped + longint'(q5_valid);
      sva_out    <= sva_out + longint'(of_take);
    end
  end

  assign sva_in_flight = int'(q1_valid) + int'(q2_valid) + int'(q3_valid) + int'(q4_valid) + int'(q5_valid);
  assign sva_err_any   = err_words_q || err_header_q;

  // Strict decode order at the issue, the pop and the output.
  a_issue_order: assert property (@(posedge clk) disable iff (!rst_n)
    s0_issue |-> longint'(rr_lane_q) == sva_issued % longint'(N_LANES))
    else $error("pans_dec_stream: issued lane %0d, the round robin's next step is global index %0d",
                rr_lane_q, sva_issued);
  a_pop_order: assert property (@(posedge clk) disable iff (!rst_n)
    q5_valid |-> longint'(q5_lane) == sva_popped % longint'(N_LANES))
    else $error("pans_dec_stream: stage 6 of lane %0d out of decode order (global index %0d)", q5_lane, sva_popped);
  a_out_order: assert property (@(posedge clk) disable iff (!rst_n)
    of_take |-> longint'(out_lane_q) == sva_out % longint'(N_LANES) && longint'(out_t_q) == sva_out / longint'(N_LANES))
    else $error("pans_dec_stream: output (%0d, %0d) is not global index %0d", out_lane_q, out_t_q, sva_out);
  // A lane with a step in flight never issues.
  a_lane_in_flight: assert property (@(posedge clk) disable iff (!rst_n)
    s0_issue |-> !(q1_valid && q1_lane == rr_lane_q) && !(q2_valid && q2_lane == rr_lane_q)
                 && !(q3_valid && q3_lane == rr_lane_q) && !(q4_valid && q4_lane == rr_lane_q)
                 && !(q5_valid && q5_lane == rr_lane_q))
    else $error("pans_dec_stream: lane %0d issued with its step in flight", rr_lane_q);
  // x >= L before and after each step of a clean stream.
  a_state_issue: assert property (@(posedge clk) disable iff (!rst_n)
    s0_issue && !sva_err_any |-> x_q[rr_lane_q] >= STATE_WIDTH'(STATE_L))
    else $error("pans_dec_stream: lane %0d state %0h below L on a clean stream", rr_lane_q, x_q[rr_lane_q]);
  a_state_pop: assert property (@(posedge clk) disable iff (!rst_n)
    q5_valid && !(sva_err_any || s6_over) |-> s6_x >= STATE_WIDTH'(STATE_L))
    else $error("pans_dec_stream: lane %0d state %0h below L after a clean step", q5_lane, s6_x);
  // A step reads no more words than it reserved.
  a_words_per_step: assert property (@(posedge clk) disable iff (!rst_n)
    q5_valid |-> RES_WIDTH'(s6_pops) <= q5_need)
    else $error("pans_dec_stream: a step read %0d words, reserved %0d (escape %0b)", s6_pops, q5_need, q5_escape);
  // The FIFO pops only words it holds. It runs dry only past the array's end.
  a_fifo_pop: assert property (@(posedge clk) disable iff (!rst_n)
    q5_valid |-> WF_CNT'(s6_pops_fifo) <= wf_count_q)
    else $error("pans_dec_stream: FIFO pops %0d of %0d words", s6_pops_fifo, wf_count_q);
  a_fifo_dry: assert property (@(posedge clk) disable iff (!rst_n)
    q5_valid && s6_over |-> !in_word_left)
    else $error("pans_dec_stream: the FIFO ran dry with %0d bytes of the array still to come", in_bytes_left_q);
  a_fifo_room: assert property (@(posedge clk) disable iff (!rst_n)
    int'(wf_count_q) + int'(in_wr_n) - (q5_valid ? int'(s6_pops_fifo) : 0) <= FIFO_WORDS)
    else $error("pans_dec_stream: word FIFO overflow");
  // Credit, output entries and steps in flight add up to OUT_DEPTH.
  a_credit: assert property (@(posedge clk) disable iff (!rst_n)
    int'(credit_q) + int'(of_count_q) + sva_in_flight == OUT_DEPTH)
    else $error("pans_dec_stream: credit %0d + output entries %0d + in flight %0d != OUT_DEPTH",
                credit_q, of_count_q, sva_in_flight);
  // A coded slot lies in the core's interval [c, c + f) on any input (SPEC.md §1).
  a_interval: assert property (@(posedge clk) disable iff (!rst_n)
    q4_valid && !c_escape
      |-> c_freq != '0 && int'(q4_x[PRECISION-1:0]) >= int'(c_start)
          && int'(q4_x[PRECISION-1:0]) < int'(c_start) + int'(c_freq))
    else $error("pans_dec_stream: slot %0d outside the core's interval [%0d, %0d)", q4_x[PRECISION-1:0], c_start,
                int'(c_start) + int'(c_freq));
  // The issue's escape prediction is the core's escape. The tag pipeline runs in step with the core.
  logic sva_esc_q1, sva_esc_q2, sva_esc_q3, sva_esc_q4;

  always_ff @(posedge clk) begin
    sva_esc_q1 <= s0_escape;
    sva_esc_q2 <= sva_esc_q1;
    sva_esc_q3 <= sva_esc_q2;
    sva_esc_q4 <= sva_esc_q3;
  end

  a_escape_predicted: assert property (@(posedge clk) disable iff (!rst_n)
    q4_valid |-> c_escape == sva_esc_q4)
    else $error("pans_dec_stream: the core's escape %0b differs from the issue's prediction (slot >= M')", c_escape);
  a_tag_pipe: assert property (@(posedge clk) disable iff (!rst_n) q4_valid == c_valid)
    else $error("pans_dec_stream: the tag pipeline and the core disagree on valid");
  // A job starts with no scale held.
  a_scales_used: assert property (@(posedge clk) disable iff (!rst_n)
    job_take |-> scale_present_q == '0)
    else $error("pans_dec_stream: a scale left over from the previous job");
  // The at-L flags match the lane states at the end check.
  a_at_L: assert property (@(posedge clk) disable iff (!rst_n)
    phase_q == ST_FIN |-> fin_at_L == (x_q.and() with (item == STATE_WIDTH'(STATE_L))))
    else $error("pans_dec_stream: at_L bits %b disagree with the lane states", at_L_q);
`endif

  // ---- Elaboration checks ---------------------------------------------------------------------------------------
  if (N_LANES < 1) begin : g_check_lanes
    $error("pans_dec_stream: N_LANES must be >= 1");
  end
  if (WORD_WIDTH % 8 != 0 || WORD_WIDTH < PRECISION || STATE_WIDTH != PRECISION + WORD_WIDTH
      || STATE_BYTES != 2 * WORD_BYTES || (WORD_BYTES & (WORD_BYTES - 1)) != 0) begin : g_check_widths
    $error("pans_dec_stream: the stream widths of pans_pkg do not match STREAM.md §1.1 (W/8 a power of two)");
  end
  if (DATA_BYTES < STATE_BYTES || DATA_BYTES % STATE_BYTES != 0) begin : g_check_beat
    $error("pans_dec_stream: DATA_BYTES must be a multiple of STATE_BYTES (whole states per beat)");
  end
  if (FIFO_WORDS < 4 || FIFO_WORDS < BEAT_WORDS + 1) begin : g_check_fifo
    $error("pans_dec_stream: FIFO_WORDS must be >= 4 and > DATA_BYTES / (W/8) (a beat fits beside one word)");
  end
  if (OUT_DEPTH < 1) begin : g_check_out
    $error("pans_dec_stream: OUT_DEPTH must be >= 1");
  end
  if (SYMBOL_WIDTH > PRECISION) begin : g_check_symbol
    $error("pans_dec_stream: SYMBOL_WIDTH must be <= PRECISION (SPEC.md §4; the raw pop needs S <= W)");
  end
  // The contract version this unit implements (SPEC.md §4).
  if (CONTRACT_VERSION != 5) begin : g_check_contract
    $error("pans_dec_stream: implements contract version 5 (SPEC.md §4)");
  end
  if (COUNT_WIDTH > 32 || COUNT_WIDTH <= DB_WIDTH || longint'(HDR_BYTES) >= (longint'(1) <<< COUNT_WIDTH)) begin : g_check_count
    $error("pans_dec_stream: COUNT_WIDTH must be <= 32 and hold the header and a beat's bytes");
  end

endmodule
