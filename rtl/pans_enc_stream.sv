// SPDX-License-Identifier: Apache-2.0
// pans_enc_stream: the encoder stream unit, N_LANES interleaved ANS lanes over one pans_enc_core.
// Mirrors encode_stream of ref/pans_stream_ref.h bit for bit.
// Specification: STREAM.md §2.2. Arithmetic: rtl/pans_enc_stream_functions.svh.
// All channels are valid/ready. No output depends combinationally on an input.
// Symbols arrive in push order. Words leave in emission order, then the final lane states.
// Reset is synchronous and active low. It clears control state only.
module pans_enc_stream
  import pans_pkg::*;
#(
  parameter  int N_LANES       = 8,  // interleaved lanes
  parameter  int DIV_BITS      = 2,  // quotient bits per divider stage
  parameter  int DIV_STEP_BITS = 1,  // quotient bits per restoring step, a divisor of DIV_BITS and W
  parameter  int DIV_FIRST_BITS = DIV_STEP_BITS,  // quotient bits taken in the renorm stage
  parameter  int OUT_DEPTH     = 8,  // output FIFO words, an even number
  parameter  int COUNT_WIDTH   = 32, // width of n_sym, n_escapes and their counters
  parameter  bit CLOCK_GATE    = 1,  // Clock gates on the lane states and the escape count. 0 uses enable muxes.
  localparam int LANE_WIDTH    = (N_LANES > 1) ? $clog2(N_LANES) : 1
) (
  input  logic                           clk,
  input  logic                           rst_n,
  // job_in: one job, taken when idle
  input  logic                           job_in_valid,
  output logic                           job_in_ready,
  input  logic [COUNT_WIDTH-1:0]         job_in_n_sym,
  // sym_in: the symbols in push order
  input  logic                           sym_in_valid,
  output logic                           sym_in_ready,
  input  logic [LANE_WIDTH-1:0]          sym_in_lane,
  input  logic signed [SYMBOL_WIDTH-1:0] sym_in_symbol,
  input  logic [B_WIDTH-1:0]             sym_in_b,
  // word_out: the words in emission order
  output logic                           word_out_valid,
  input  logic                           word_out_ready,
  output logic [WORD_WIDTH-1:0]          word_out_word,
  // done_out: the final lane states, lane 0 first, with the job's counts
  output logic                           done_out_valid,
  input  logic                           done_out_ready,
  output logic [LANE_WIDTH-1:0]          done_out_lane,
  output logic                           done_out_last,
  output logic [STATE_WIDTH-1:0]         done_out_state,
  output logic [COUNT_WIDTH:0]           done_out_n_words,
  output logic [COUNT_WIDTH-1:0]         done_out_n_escapes,
  output logic                           done_out_err_order
);

`include "pans_enc_stream_functions.svh"

  localparam int DIV_REST_BITS  = WORD_WIDTH - DIV_FIRST_BITS;  // quotient bits left to the divider stages
  localparam int DIV_STAGES     = (DIV_REST_BITS + DIV_BITS - 1) / DIV_BITS;
  localparam int DIV_LAST_BITS  = DIV_REST_BITS - (DIV_STAGES - 1) * DIV_BITS;  // the last stage's quotient bits
  localparam int FIFO_PTR_WIDTH = (OUT_DEPTH > 1) ? $clog2(OUT_DEPTH) : 1;
  localparam int FIFO_CNT_WIDTH = $clog2(OUT_DEPTH + 1);
  localparam int STATE_L        = NUM_SLOTS;  // L = M, every lane's start state

  typedef enum logic [1:0] {
    ST_IDLE,   // waiting for a job
    ST_SETUP,  // one cycle for the first push's lane
    ST_RUN,    // taking the symbols, then draining
    ST_DONE    // the done_out beats
  } job_state_t;

  // ---- Job control
  job_state_t             st;
  logic [COUNT_WIDTH-1:0] job_rem;      // symbols not yet taken from sym_in
  logic [LANE_WIDTH-1:0]  exp_lane;     // the next push's lane
  logic [LANE_WIDTH-1:0]  done_lane;    // the lane of the current done_out beat
  logic [LANE_WIDTH-1:0]  done_lane_next;
  logic                   run_done;     // the job's last cycle in ST_RUN
  logic [COUNT_WIDTH:0]   cnt_words;
  logic [COUNT_WIDTH-1:0] cnt_escapes;
  logic                   err_order;
  logic                   job_fire;
  logic                   done_fire;
  logic                   pipe_empty;   // no push in flight

  // ---- Lanes
  state_t                 lane_x [N_LANES];
  logic [N_LANES-1:0]     busy;         // a push of the lane in flight, not yet released
  logic [N_LANES-1:0]     fresh;        // the lane is unwritten in this job and reads as L
  logic [LANE_WIDTH-1:0]  rd_lane;      // the read port's lane, q2_lane
  state_t                 rd_x;

  // ---- Output FIFO
  // Two banks of even and odd entries, so each bank has one write port.
  localparam int FIFO_BANK      = OUT_DEPTH / 2;
  localparam int FIFO_BANK_ADDR = (FIFO_BANK > 1) ? $clog2(FIFO_BANK) : 1;

  word_t                     fifo_bank0 [FIFO_BANK];  // entries 0, 2, 4, ...
  word_t                     fifo_bank1 [FIFO_BANK];  // entries 1, 3, 5, ...
  logic [FIFO_PTR_WIDTH-1:0] fifo_rp;
  logic [FIFO_PTR_WIDTH-1:0] fifo_wp;
  logic [FIFO_CNT_WIDTH-1:0] fifo_count;
  logic                      fifo_pop;

  function automatic logic [FIFO_PTR_WIDTH-1:0] fifo_next(input logic [FIFO_PTR_WIDTH-1:0] p);
    logic [FIFO_PTR_WIDTH-1:0] n;
    n = (int'(p) == OUT_DEPTH - 1) ? '0 : p + FIFO_PTR_WIDTH'(1);
    return n;
  endfunction

  function automatic logic [FIFO_BANK_ADDR-1:0] fifo_bank_addr(input logic [FIFO_PTR_WIDTH-1:0] p);
    logic [FIFO_BANK_ADDR-1:0] a;
    a = FIFO_BANK_ADDR'(p >> 1);
    return a;
  endfunction

  // ---- Tag pipeline beside the core (stages 1-2)
  logic                    q1_valid;
  logic [LANE_WIDTH-1:0]   q1_lane;
  logic [SYMBOL_WIDTH-1:0] q1_symbol;
  logic                    q2_valid;
  logic [LANE_WIDTH-1:0]   q2_lane;
  logic [SYMBOL_WIDTH-1:0] q2_symbol;

  // ---- Stage 0: the input register
  // sym_in's b becomes the core's scale code here.
  logic                           q0_valid;
  logic [LANE_WIDTH-1:0]          q0_lane;    // the beat's lane field, checked at the accept
  logic signed [SYMBOL_WIDTH-1:0] q0_symbol;
  scale_t                         q0_scale;
  scale_t                         s0_scale;   // sym_in_b's code
  logic                           s0_take;    // a sym_in beat into the input register

  pans_scale_of_b u_scale_of_b (.b(sym_in_b), .scale(s0_scale));

  // ---- Stage 1: the accept
  // The input register's beat enters the core when its lane is free and the FIFO has room.
  logic s1_credit;
  logic s1_accept;

  assign s1_credit    = int'(fifo_count) + 2 * (int'(q1_valid) + int'(q2_valid)) + 2 <= OUT_DEPTH;
  assign s1_accept    = q0_valid && !busy[exp_lane] && s1_credit;
  assign sym_in_ready = st == ST_RUN && job_rem != '0 && (!q0_valid || s1_accept);
  assign s0_take      = sym_in_valid && sym_in_ready;

  always_ff @(posedge clk) begin
    if (!rst_n) q0_valid <= 1'b0;
    else        q0_valid <= s0_take || (q0_valid && !s1_accept);
  end

  // The data loads whenever the register can take a beat, independent of sym_in_valid.
  always_ff @(posedge clk) begin
    if (!q0_valid || s1_accept) begin
      q0_lane   <= sym_in_lane;
      q0_symbol <= sym_in_symbol;
      q0_scale  <= s0_scale;
    end
  end

  // ---- Stages 1-2: the core and the tag pipeline
  logic                           core_valid;
  slot_t                          core_start;
  slot_t                          core_freq;
  logic                           core_escape;

  pans_enc_core u_core (
    .clk        (clk),
    .rst_n      (rst_n),
    .in_valid   (s1_accept),
    .in_scale   (q0_scale),
    .in_symbol  (q0_symbol),
    .out_valid  (core_valid),
    .out_start  (core_start),
    .out_freq   (core_freq),
    .out_escape (core_escape)
  );

  always_ff @(posedge clk) begin
    q1_lane   <= exp_lane;
    q1_symbol <= q0_symbol;
    // q2_lane addresses the read port. It holds done_out's lane once the job is through.
    q2_lane   <= (st == ST_DONE || run_done) ? done_lane_next : q1_lane;
    q2_symbol <= q1_symbol;
  end

  // ---- Stage 3: renormalize
  // Pushes a coded symbol, or an escape's raw value and then (M', E).
  state_t                  s3_x;         // x[lane] before the symbol
  logic [SYMBOL_WIDTH-1:0] s3_raw;
  renorm_t                 s3_coded;     // the coded push's renorm
  renorm_t                 s3_raw_push;  // the escape's raw push, bound 2^(P + W - S)
  state_t                  s3_x_raw;     // its result x' 2^S + raw
  renorm_t    s3_esc_push;    // the escape push (f = E)
  state_t     s3_x_esc;       // the escape's final state
  logic [1:0] s3_n_words;
  word_t      s3_word_a;      // the first word out
  word_t      s3_word_b;      // the second word out, escapes only
  state_t     s3_dividend;    // the dividend, below s3_f 2^W
  slot_t      s3_f;
  slot_t      s3_c;
  logic [1:0] s3_push;        // words into the FIFO this cycle

  always_comb begin
    s3_x        = rd_x;
    s3_raw      = unsigned'(q2_symbol);
    s3_coded    = push_renorm(s3_x, core_freq);
    s3_raw_push = push_renorm_raw(s3_x);
    s3_x_raw    = push_raw(s3_raw_push.x[STATE_WIDTH-SYMBOL_WIDTH-1:0], s3_raw);
    s3_esc_push = push_renorm(s3_x_raw, slot_t'(ESCAPE_SLOTS));
    s3_x_esc    = assemble(word_t'(s3_esc_push.x >> ESCAPE_LOG2), slot_t'(s3_esc_push.x) & slot_t'(ESCAPE_SLOTS - 1),
                           slot_t'(CODED_SLOTS));
    if (core_escape) begin
      s3_n_words  = {1'b0, s3_raw_push.emit} + {1'b0, s3_esc_push.emit};
      s3_word_a   = s3_raw_push.emit ? s3_raw_push.word : s3_esc_push.word;
      s3_word_b   = s3_esc_push.word;
      s3_dividend = state_t'(s3_x_esc >> PRECISION);  // the finished state, divided by 1 below
      s3_f        = slot_t'(1);
      s3_c        = s3_x_esc[PRECISION-1:0];
    end else begin
      s3_n_words  = {1'b0, s3_coded.emit};
      s3_word_a   = s3_coded.word;
      s3_word_b   = s3_coded.word;
      s3_dividend = s3_coded.x;
      s3_f        = core_freq;
      s3_c        = core_start;
    end
    s3_push = q2_valid ? s3_n_words : 2'd0;
  end

  // ---- Stages 3 .. 2 + DIV_STAGES: the registers
  // qd_*[0] is the renorm stage's register, qd_*[k] divider stage k's.
  logic [DIV_STAGES-1:0] qd_valid;
  logic [LANE_WIDTH-1:0] qd_lane [DIV_STAGES];
  divider_t              qd_div  [DIV_STAGES];
  slot_t                 qd_f    [DIV_STAGES];
  slot_t                 qd_c    [DIV_STAGES];

  always_ff @(posedge clk) begin
    qd_lane[0] <= q2_lane;
    qd_div[0]  <= div_stage(divider_t'(s3_dividend), s3_f, DIV_FIRST_BITS);  // the dividend at DIV_FIRST_BITS = 0
    qd_f[0]    <= s3_f;
    qd_c[0]    <= s3_c;
  end

  // ---- Stages 4 .. 2 + DIV_STAGES: the registered divider stages
  for (genvar k = 1; k < DIV_STAGES; k++) begin : g_div
    divider_t sd_div;

    always_comb sd_div = div_stage(qd_div[k-1], qd_f[k-1], DIV_BITS);

    always_ff @(posedge clk) begin
      qd_lane[k] <= qd_lane[k-1];
      qd_div[k]  <= sd_div;
      qd_f[k]    <= qd_f[k-1];
      qd_c[k]    <= qd_c[k-1];
    end
  end

  // ---- Stage 3 + DIV_STAGES: the last divider stage
  // Its last step is fused with the assemble x = q M + c + r, which writes the lane.
  state_t                sa_x;
  logic                  sa_write;
  logic [LANE_WIDTH-1:0] sa_lane;

  always_comb begin
    sa_x     = div_stage_last(qd_div[DIV_STAGES-1], qd_f[DIV_STAGES-1], DIV_LAST_BITS, qd_c[DIV_STAGES-1]);
    sa_write = qd_valid[DIV_STAGES-1];
    sa_lane  = qd_lane[DIV_STAGES-1];
  end

  // Lane states, one write port and one read port. A fresh lane reads as L.
  for (genvar j = 0; j < N_LANES; j++) begin : g_lane
    logic   we;
    state_t x_q;
    assign we        = sa_write && int'(sa_lane) == j;
    assign lane_x[j] = x_q;
    if (CLOCK_GATE) begin : g_cg
      logic gclk;
      pans_stream_cg u_cg (.clk(clk), .en(we), .gclk(gclk));
      always_ff @(posedge gclk) x_q <= sa_x;
    end else begin : g_en
      always_ff @(posedge clk) if (we) x_q <= sa_x;
    end
  end

  always_ff @(posedge clk) begin
    if (!rst_n || job_fire) fresh <= '1;
    else for (int j = 0; j < N_LANES; j++) begin
      if (sa_write && int'(sa_lane) == j) fresh[j] <= 1'b0;
    end
  end

  always_comb begin
    rd_lane = q2_lane;  // done_lane in ST_DONE
    rd_x    = fresh[rd_lane] ? state_t'(STATE_L) : lane_x[rd_lane];
  end

  // ---- Pipeline slots
  // Slot 0 is q1, slot 1 is q2, slot 2 + k is qd[k].
  localparam int PIPE_SLOTS     = DIV_STAGES + 2;
  localparam int ISSUE_DISTANCE = DIV_STAGES + 1;       // cycles between two accepts of one lane, read by the testbench
  localparam int RELEASE_SLOT   = ISSUE_DISTANCE - 2;  // the slot that clears busy

  logic [PIPE_SLOTS-1:0] pipe_valid;
  logic [LANE_WIDTH-1:0] pipe_lane [PIPE_SLOTS];

  always_comb begin
    pipe_valid[0] = q1_valid;
    pipe_valid[1] = q2_valid;
    pipe_lane[0]  = q1_lane;
    pipe_lane[1]  = q2_lane;
    for (int k = 0; k < DIV_STAGES; k++) begin
      pipe_valid[2 + k] = qd_valid[k];
      pipe_lane[2 + k]  = qd_lane[k];
    end
  end

  // ---- Valid and busy bits
  // busy[j] holds from the accept until the push reaches RELEASE_SLOT.
  always_ff @(posedge clk) begin
    if (!rst_n) begin
      q1_valid <= 1'b0;
      q2_valid <= 1'b0;
      qd_valid <= '0;
      busy     <= '0;
    end else begin
      q1_valid <= s1_accept;
      q2_valid <= q1_valid;
      qd_valid[0] <= q2_valid;
      for (int k = 1; k < DIV_STAGES; k++) qd_valid[k] <= qd_valid[k-1];
      for (int j = 0; j < N_LANES; j++) begin
        if (s1_accept && int'(exp_lane) == j) busy[j] <= 1'b1;
        else if (pipe_valid[RELEASE_SLOT] && int'(pipe_lane[RELEASE_SLOT]) == j) busy[j] <= 1'b0;
      end
    end
  end

  // ---- Output FIFO: up to two words in, one out
  logic [FIFO_PTR_WIDTH-1:0] fifo_wp1;
  word_t                     fifo_d0;   // bank 0's write data
  word_t                     fifo_d1;
  logic                      fifo_we0;
  logic                      fifo_we1;
  logic [FIFO_BANK_ADDR-1:0] fifo_a0;
  logic [FIFO_BANK_ADDR-1:0] fifo_a1;

  assign fifo_wp1       = fifo_next(fifo_wp);
  assign word_out_valid = fifo_count != '0;
  assign word_out_word  = fifo_rp[0] ? fifo_bank1[fifo_bank_addr(fifo_rp)] : fifo_bank0[fifo_bank_addr(fifo_rp)];
  assign fifo_pop       = word_out_valid && word_out_ready;

  always_comb begin
    fifo_d0  = fifo_wp[0] ? s3_word_b : s3_word_a;
    fifo_d1  = fifo_wp[0] ? s3_word_a : s3_word_b;
    fifo_we0 = fifo_wp[0] ? s3_push == 2'd2 : s3_push != 2'd0;
    fifo_we1 = fifo_wp[0] ? s3_push != 2'd0 : s3_push == 2'd2;
    fifo_a0  = fifo_wp[0] ? fifo_bank_addr(fifo_wp1) : fifo_bank_addr(fifo_wp);
    fifo_a1  = fifo_wp[0] ? fifo_bank_addr(fifo_wp) : fifo_bank_addr(fifo_wp1);
  end

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      fifo_rp    <= '0;
      fifo_wp    <= '0;
      fifo_count <= '0;
    end else begin
      fifo_rp    <= fifo_pop ? fifo_next(fifo_rp) : fifo_rp;
      fifo_wp    <= (s3_push == 2'd2) ? fifo_next(fifo_wp1) : (s3_push == 2'd1) ? fifo_wp1 : fifo_wp;
      fifo_count <= fifo_count + FIFO_CNT_WIDTH'(s3_push) - FIFO_CNT_WIDTH'(fifo_pop);
    end
  end

  // Enable flops, not clock gates (STREAM.md §2.2).
  always_ff @(posedge clk) begin
    for (int e = 0; e < FIFO_BANK; e++) begin
      if (fifo_we0 && int'(fifo_a0) == e) fifo_bank0[e] <= fifo_d0;
      if (fifo_we1 && int'(fifo_a1) == e) fifo_bank1[e] <= fifo_d1;
    end
  end

  // ---- Job control
  assign job_in_ready = st == ST_IDLE;
  assign job_fire     = job_in_valid && job_in_ready;
  assign pipe_empty   = pipe_valid == '0;
  assign done_fire    = done_out_valid && done_out_ready;
  assign run_done     = st == ST_RUN && job_rem == '0 && !q0_valid && pipe_empty && fifo_count == '0;
  assign done_lane_next = done_fire ? done_lane + LANE_WIDTH'(1) : done_lane;  // 0 until the first done beat

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      st <= ST_IDLE;
    end else begin
      unique case (st)
        ST_IDLE:  if (job_fire) st <= ST_SETUP;
        ST_SETUP: st <= ST_RUN;
        ST_RUN:   if (run_done) st <= ST_DONE;
        ST_DONE:  if (done_fire && done_out_last) st <= ST_IDLE;
        default:  st <= ST_IDLE;
      endcase
    end
  end

  always_ff @(posedge clk) begin
    if (job_fire) job_rem <= job_in_n_sym;
    else if (s0_take) job_rem <= job_rem - COUNT_WIDTH'(1);
    if (st == ST_SETUP) exp_lane <= first_lane(job_rem);
    else if (s1_accept) exp_lane <= (exp_lane == '0) ? LANE_WIDTH'(N_LANES - 1) : exp_lane - LANE_WIDTH'(1);
    if (job_fire) done_lane <= '0;
    else done_lane <= done_lane_next;
    if (job_fire) cnt_words <= '0;  // counted as the words leave the FIFO
    else if (fifo_pop) cnt_words <= cnt_words + (COUNT_WIDTH + 1)'(1);
    if (job_fire) err_order <= 1'b0;
    else if (s1_accept && q0_lane != exp_lane) err_order <= 1'b1;
  end

  // The escape count, behind a clock gate under CLOCK_GATE.
  logic cnt_esc_en;
  assign cnt_esc_en = job_fire || (q2_valid && core_escape);
  if (CLOCK_GATE) begin : g_esc_cg
    logic gclk;
    pans_stream_cg u_cg (.clk(clk), .en(cnt_esc_en), .gclk(gclk));
    always_ff @(posedge gclk) cnt_escapes <= job_fire ? '0 : cnt_escapes + COUNT_WIDTH'(1);
  end else begin : g_esc_en
    always_ff @(posedge clk) if (cnt_esc_en) cnt_escapes <= job_fire ? '0 : cnt_escapes + COUNT_WIDTH'(1);
  end

  // ---- Done
  assign done_out_valid     = st == ST_DONE;
  assign done_out_lane      = done_lane;
  assign done_out_last      = int'(done_lane) == N_LANES - 1;
  assign done_out_state     = rd_x;
  assign done_out_n_words   = cnt_words;
  assign done_out_n_escapes = cnt_escapes;
  assign done_out_err_order = err_order;

  // ---- Checks (simulation and formal only)
`ifndef SYNTHESIS
  localparam longint STATE_L_SVA = longint'(STATE_L);  // L

  // The tag pipeline runs in step with the core.
  a_core_aligned: assert property (@(posedge clk) disable iff (!rst_n) core_valid == q2_valid)
    else $error("pans_enc_stream: the core's out_valid is not the tag pipeline's");
  // x >= L on every lane, at every read and at every write.
  for (genvar j = 0; j < N_LANES; j++) begin : g_lane_checks
    a_lane_state: assert property (@(posedge clk) disable iff (!rst_n)
      st != ST_IDLE && !fresh[j] |-> longint'(lane_x[j]) >= STATE_L_SVA)
      else $error("pans_enc_stream: lane %0d state %0h below L", j, lane_x[j]);
  end
  a_state_read: assert property (@(posedge clk) disable iff (!rst_n) q2_valid |-> longint'(s3_x) >= STATE_L_SVA)
    else $error("pans_enc_stream: a push read the state %0h of lane %0d, below L", s3_x, q2_lane);
  a_state_write: assert property (@(posedge clk) disable iff (!rst_n) sa_write |-> longint'(sa_x) >= STATE_L_SVA)
    else $error("pans_enc_stream: lane %0d written with %0h, below L", sa_lane, sa_x);
  // At most one word per push, two per escape.
  a_one_word_per_push: assert property (@(posedge clk) disable iff (!rst_n)
    q2_valid |-> (core_escape ? s3_raw_push.x[STATE_WIDTH-1 -: SYMBOL_WIDTH] == '0 &&
                                s3_esc_push.x[STATE_WIDTH-1:WORD_WIDTH] < slot_t'(ESCAPE_SLOTS)
                              : s3_coded.x[STATE_WIDTH-1:WORD_WIDTH] < core_freq) &&
                 s3_n_words <= (core_escape ? 2'd2 : 2'd1))
    else $error("pans_enc_stream: a push left its state at or above f 2^W (lane %0d, escape %0b)", q2_lane, core_escape);
  // Every partial remainder is below f.
  for (genvar k = 0; k < DIV_STAGES; k++) begin : g_div_checks
    a_remainder: assert property (@(posedge clk) disable iff (!rst_n)
      qd_valid[k] |-> qd_f[k] != '0 && qd_div[k].rem < qd_f[k])
      else $error("pans_enc_stream: divider stage %0d: remainder %0d, f %0d", k, qd_div[k].rem, qd_f[k]);
  end
  // The fused last step equals the unfused one.
  divider_t chk_last;
  always_comb chk_last = div_stage(qd_div[DIV_STAGES-1], qd_f[DIV_STAGES-1], DIV_LAST_BITS);
  a_remainder_last: assert property (@(posedge clk) disable iff (!rst_n)
    sa_write |-> chk_last.rem < qd_f[DIV_STAGES-1])
    else $error("pans_enc_stream: the last remainder %0d is not below f %0d", chk_last.rem, qd_f[DIV_STAGES-1]);
  a_assemble_low: assert property (@(posedge clk) disable iff (!rst_n)
    sa_write |-> int'(qd_c[DIV_STAGES-1]) + int'(chk_last.rem) < NUM_SLOTS)
    else $error("pans_enc_stream: c + r = %0d + %0d reaches M", qd_c[DIV_STAGES-1], chk_last.rem);
  a_assemble_fused: assert property (@(posedge clk) disable iff (!rst_n)
    sa_write |-> sa_x == assemble(chk_last.dq, chk_last.rem, qd_c[DIV_STAGES-1]))
    else $error("pans_enc_stream: the fused last step gives %0h, the assemble %0h", sa_x,
                assemble(chk_last.dq, chk_last.rem, qd_c[DIV_STAGES-1]));
  // A busy lane takes no symbol. A push reads its lane only after every older push of the lane wrote it.
  logic chk_older_same_lane;
  always_comb begin
    chk_older_same_lane = 1'b0;
    for (int s = 2; s < PIPE_SLOTS; s++)
      if (pipe_valid[s] && pipe_lane[s] == q2_lane) chk_older_same_lane = 1'b1;
  end
  a_busy_accept: assert property (@(posedge clk) disable iff (!rst_n) s1_accept |-> !busy[exp_lane])
    else $error("pans_enc_stream: lane %0d took a symbol with a push in flight", exp_lane);
  a_lane_in_order: assert property (@(posedge clk) disable iff (!rst_n) q2_valid |-> !chk_older_same_lane)
    else $error("pans_enc_stream: lane %0d read its state with an older push still in the divider", q2_lane);
  a_release_busy: assert property (@(posedge clk) disable iff (!rst_n)
    pipe_valid[RELEASE_SLOT] |-> busy[pipe_lane[RELEASE_SLOT]])
    else $error("pans_enc_stream: lane %0d released without being busy", pipe_lane[RELEASE_SLOT]);
  a_lane_range: assert property (@(posedge clk) disable iff (!rst_n)
    st == ST_RUN |-> int'(exp_lane) < N_LANES)
    else $error("pans_enc_stream: lane counter %0d out of range", exp_lane);
  // The FIFO never overflows, never pops empty, and no word enters without a valid push.
  a_fifo_overflow: assert property (@(posedge clk) disable iff (!rst_n)
    int'(fifo_count) + int'(s3_push) - int'(fifo_pop) <= OUT_DEPTH)
    else $error("pans_enc_stream: FIFO overflow: %0d words + %0d in", fifo_count, s3_push);
  a_fifo_underflow: assert property (@(posedge clk) disable iff (!rst_n) fifo_pop |-> fifo_count != '0)
    else $error("pans_enc_stream: FIFO popped empty");
  a_word_from_push: assert property (@(posedge clk) disable iff (!rst_n) s3_push != 2'd0 |-> q2_valid)
    else $error("pans_enc_stream: a word entered the FIFO without a valid push");
  // done_out only after the job has drained.
  a_done_drained: assert property (@(posedge clk) disable iff (!rst_n)
    done_out_valid |-> job_rem == '0 && !q0_valid && pipe_empty && fifo_count == '0)
    else $error("pans_enc_stream: done_out before the job drained");
  // The input register holds a beat only in ST_RUN.
  a_input_in_job: assert property (@(posedge clk) disable iff (!rst_n) q0_valid |-> st == ST_RUN)
    else $error("pans_enc_stream: the input register holds a beat outside ST_RUN");
`endif

  // ---- Elaboration checks
  if (N_LANES < 1) begin : g_check_lanes
    $error("pans_enc_stream: N_LANES must be >= 1");
  end
  if (DIV_BITS < 1 || DIV_BITS > WORD_WIDTH) begin : g_check_div_bits
    $error("pans_enc_stream: DIV_BITS must be in [1, WORD_WIDTH]");
  end
  if (DIV_STEP_BITS < 1 || DIV_BITS % DIV_STEP_BITS != 0 || WORD_WIDTH % DIV_STEP_BITS != 0) begin : g_check_step_bits
    $error("pans_enc_stream: DIV_STEP_BITS must divide DIV_BITS and WORD_WIDTH");
  end
  if (DIV_FIRST_BITS < 0 || DIV_FIRST_BITS >= WORD_WIDTH || DIV_FIRST_BITS % DIV_STEP_BITS != 0) begin : g_check_first_bits
    $error("pans_enc_stream: DIV_FIRST_BITS must be a multiple of DIV_STEP_BITS below WORD_WIDTH");
  end
  if (OUT_DEPTH < 2 || OUT_DEPTH % 2 != 0) begin : g_check_out_depth
    $error("pans_enc_stream: OUT_DEPTH must be even and >= 2 (an escape's two words, one per FIFO bank)");
  end
  if ((ESCAPE_SLOTS & (ESCAPE_SLOTS - 1)) != 0) begin : g_check_escape_slots
    $error("pans_enc_stream: ESCAPE_SLOTS must be a power of two (the escape push is a shift)");
  end
  if (WORD_WIDTH < PRECISION || STATE_WIDTH != PRECISION + WORD_WIDTH) begin : g_check_widths
    $error("pans_enc_stream: WORD_WIDTH must be >= PRECISION and STATE_WIDTH = PRECISION + WORD_WIDTH");
  end
  if (SYMBOL_WIDTH > PRECISION) begin : g_check_symbol_width
    $error("pans_enc_stream: SYMBOL_WIDTH must be <= PRECISION (SPEC.md §4; the raw push needs S <= W)");
  end
  // The contract version this unit implements (SPEC.md §4).
  if (CONTRACT_VERSION != 5) begin : g_check_contract
    $error("pans_enc_stream: implements contract version 5 (SPEC.md §4)");
  end

endmodule
