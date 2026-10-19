// SPDX-License-Identifier: Apache-2.0
// tb_pans_enc_stream: pans_enc_stream against the stream model through its DPI (make sim-enc-stream-<cfg>).
// The model is ref/pans_stream_dpi.cpp with ref/pans1_dpi.cpp. Specification: STREAM.md §3.
// Every word, final state and count is checked, and every push at the core, the renorm stage and the state write.
// The numbered sections of the test are its parts. Every coverage corner must occur.
// Handshakes are decided at the falling edge. A channel without a transfer for WATCHDOG cycles fails the run.
//
// Plusargs: +SEED=<n> (default 1); +P_IDLE=<%> +P_STALL=<%> +P_DSTALL=<%> run the random jobs with that one
// profile instead of the list; +VERBOSE prints every job.
module tb_pans_enc_stream;

  import pans_pkg::*;

  parameter int N_LANES       = 8;
  parameter int DIV_BITS      = 2;
  parameter int DIV_STEP_BITS = 1;
  parameter int DIV_FIRST_BITS = DIV_STEP_BITS;
  parameter int OUT_DEPTH     = 8;
  parameter int COUNT_WIDTH   = 32;
  parameter bit CLOCK_GATE    = 1;

  localparam int LANE_WIDTH  = (N_LANES > 1) ? $clog2(N_LANES) : 1;
  localparam longint WATCHDOG = 10000;
  localparam int KIND_CONST = 0, KIND_EMA = 1, KIND_LIST = 2;
  // The least S-bit input: SYMBOL_MIN, or below a symmetric alphabet -2^(SW-1), which the encoder escapes.
  localparam int SYMBOL_IN_MIN = -(2 ** (SYMBOL_WIDTH - 1));
  // The raw push's bound B = 2^(P + W - S). B - 1 is a state iff B > L.
  localparam longint RAW_BOUND      = longint'(1) <<< (PRECISION + WORD_WIDTH - SYMBOL_WIDTH);
  localparam bit     BELOW_POSSIBLE = RAW_BOUND > longint'(NUM_SLOTS);
  localparam bit     TWO_WORDS_POSSIBLE = WORD_WIDTH + $clog2(ESCAPE_SLOTS) < PRECISION + SYMBOL_WIDTH;

  import "DPI-C" function longint pans_ref_config_id();
  import "DPI-C" function void pans_ref_b_widths(output int b_frac, output int b_width, output int b_k);
  import "DPI-C" function void pans_ref_scale_of_b(input longint b, output int exp_field, output int man_field);
  import "DPI-C" function void pans_ref_encode(input int exp_field, input int man_field, input int symbol,
                                               output int start, output int freq, output int escape);
  import "DPI-C" function void pans_stream_widths(output int word_width, output int state_width,
                                                  output int state_bytes);
  import "DPI-C" function int pans_stream_job_new(input int seed, input int n_lanes, input longint n, input int kind,
                                                  input int delay);
  import "DPI-C" function int pans_stream_job_begin(input int n_lanes, input longint n, input int kind,
                                                    input int delay);
  import "DPI-C" function void pans_stream_job_set(input int job, input longint i, input int symbol, input longint b);
  import "DPI-C" function int pans_stream_job_build(input int job);
  import "DPI-C" function int pans_stream_job_bound(input int seed, input int n_lanes, output longint index0,
                                                    output longint index1);
  import "DPI-C" function void pans_stream_job_free(input int job);
  import "DPI-C" function void pans_stream_job_push(input int job, input longint k, output int lane, output int symbol,
                                                    output longint b);
  import "DPI-C" function void pans_stream_job_enc_step(input int job, input longint k, output int lane,
                                                        output longint t, output int symbol, output int start,
                                                        output int freq, output int escape, output int words,
                                                        output longint state, output longint b,
                                                        output int scale_exp, output int scale_man);
  import "DPI-C" function longint pans_stream_job_n(input int job);
  import "DPI-C" function longint pans_stream_job_n_words(input int job);
  import "DPI-C" function longint pans_stream_job_word(input int job, input longint k);
  import "DPI-C" function longint pans_stream_job_state(input int job, input int lane);
  import "DPI-C" function longint pans_stream_job_n_escapes(input int job);

  logic clk   = 1'b0;
  logic rst_n = 1'b0;
  always #5 clk = ~clk;

  // ---- The unit
  logic                           job_in_valid = 1'b0;
  logic                           job_in_ready;
  logic [COUNT_WIDTH-1:0]         job_in_n_sym = '0;
  logic                           sym_in_valid = 1'b0;
  logic                           sym_in_ready;
  logic [LANE_WIDTH-1:0]          sym_in_lane = '0;
  logic signed [SYMBOL_WIDTH-1:0] sym_in_symbol = '0;
  logic [B_WIDTH-1:0]             sym_in_b = '0;
  logic                           word_out_valid;
  logic                           word_out_ready = 1'b0;
  logic [WORD_WIDTH-1:0]          word_out_word;
  logic                           done_out_valid;
  logic                           done_out_ready = 1'b0;
  logic [LANE_WIDTH-1:0]          done_out_lane;
  logic                           done_out_last;
  logic [STATE_WIDTH-1:0]         done_out_state;
  logic [COUNT_WIDTH:0]           done_out_n_words;
  logic [COUNT_WIDTH-1:0]         done_out_n_escapes;
  logic                           done_out_err_order;

  pans_enc_stream #(
    .N_LANES       (N_LANES),
    .DIV_BITS      (DIV_BITS),
    .DIV_STEP_BITS (DIV_STEP_BITS),
    .DIV_FIRST_BITS (DIV_FIRST_BITS),
    .OUT_DEPTH     (OUT_DEPTH),
    .COUNT_WIDTH   (COUNT_WIDTH),
    .CLOCK_GATE    (CLOCK_GATE)
  ) dut (
    .clk                (clk),
    .rst_n              (rst_n),
    .job_in_valid       (job_in_valid),
    .job_in_ready       (job_in_ready),
    .job_in_n_sym       (job_in_n_sym),
    .sym_in_valid       (sym_in_valid),
    .sym_in_ready       (sym_in_ready),
    .sym_in_lane        (sym_in_lane),
    .sym_in_symbol      (sym_in_symbol),
    .sym_in_b           (sym_in_b),
    .word_out_valid     (word_out_valid),
    .word_out_ready     (word_out_ready),
    .word_out_word      (word_out_word),
    .done_out_valid     (done_out_valid),
    .done_out_ready     (done_out_ready),
    .done_out_lane      (done_out_lane),
    .done_out_last      (done_out_last),
    .done_out_state     (done_out_state),
    .done_out_n_words   (done_out_n_words),
    .done_out_n_escapes (done_out_n_escapes),
    .done_out_err_order (done_out_err_order)
  );

  // ---- Checking
  int unsigned checks = 0;
  int unsigned errors = 0;
  longint      cycle  = 0;
  bit          verbose;

  function automatic void check(input bit ok, input string what);
    checks++;
    if (!ok) begin
      errors++;
      if (errors <= 20) $display("ERROR %s", what);
    end
  endfunction

  // ---- Random numbers: splitmix64, the same sequence per seed in every simulator
  typedef longint unsigned u64_t;

  u64_t rng_state;

  function automatic u64_t rng_next();
    u64_t z;
    rng_state = rng_state + 64'h9e3779b97f4a7c15;
    z = rng_state;
    z = (z ^ (z >> 30)) * 64'hbf58476d1ce4e5b9;
    z = (z ^ (z >> 27)) * 64'h94d049bb133111eb;
    return z ^ (z >> 31);
  endfunction

  function automatic int rng_below(input int n);  // [0, n), n >= 1
    return int'(rng_next() % u64_t'(n));
  endfunction

  function automatic int rng_seed();
    return int'(rng_next() & 64'h7fffffff);
  endfunction

  // ---- The scale lookup on its own, against the model
  // Every b below 2^16, and above at each index's ends and random b.
  logic [B_WIDTH-1:0] lut_b = '0;
  scale_t             lut_scale;

  pans_scale_of_b u_lut (.b(lut_b), .scale(lut_scale));

  task automatic check_lookup();
    longint bs[$];
    longint n_dense;
    int     e, m;
    n_dense = (B_WIDTH <= 16) ? (longint'(1) <<< B_WIDTH) : (longint'(1) <<< 16);
    for (longint b = 0; b < n_dense; b++) bs.push_back(b);
    for (int l = 16; l < B_WIDTH; l++)
      for (int k = 0; k < (1 << B_K); k++) begin
        bs.push_back(((longint'(1) <<< B_K) + longint'(k)) <<< (l - B_K));
        bs.push_back((((longint'(1) <<< B_K) + longint'(k) + 1) <<< (l - B_K)) - 1);
        bs.push_back((((longint'(1) <<< B_K) + longint'(k)) <<< (l - B_K))
                     + longint'(rng_next() % u64_t'(longint'(1) <<< (l - B_K))));
      end
    foreach (bs[q]) begin
      lut_b = B_WIDTH'(bs[q]);
      #1;
      pans_ref_scale_of_b(bs[q], e, m);
      check(int'(lut_scale.exp) == e && int'(lut_scale.man) == m,
            $sformatf("pans_scale_of_b: b %0d gives {%0d, %0d}, the model {%0d, %0d}", bs[q], lut_scale.exp,
                      lut_scale.man, e, m));
    end
    $display("scale lookup: pans_scale_of_b equals the model's scale_of_b at %0d values of b (B_WIDTH %0d, B_K %0d)",
             bs.size(), B_WIDTH, B_K);
  endtask

  // ---- Per push, white box, sampled at the falling edge
  // Each push is checked at the accept, at the renorm stage and at its state write. One counter per stage.
  int     mon_h = -1;
  longint mon_accept = 0, mon_renorm = 0, mon_write = 0;

  always @(negedge clk) begin
    if (rst_n && mon_h >= 0) begin
      int     lane, sym, start, freq, escape, words, se, sm;
      longint t, state, b;
      if (dut.s1_accept) begin
        pans_stream_job_enc_step(mon_h, mon_accept, lane, t, sym, start, freq, escape, words, state, b, se, sm);
        check(int'(dut.exp_lane) == lane && int'(dut.q0_symbol) == sym && int'(dut.q0_scale.exp) == se
              && int'(dut.q0_scale.man) == sm,
              $sformatf("push %0d (lane %0d, t %0d, symbol %0d, b %0d) into the core: lane %0d symbol %0d scale {%0d, %0d}; the model's scale {%0d, %0d}",
                        mon_accept, lane, t, sym, b, dut.exp_lane, dut.q0_symbol, dut.q0_scale.exp,
                        dut.q0_scale.man, se, sm));
        mon_accept++;
      end
      if (dut.q2_valid) begin
        pans_stream_job_enc_step(mon_h, mon_renorm, lane, t, sym, start, freq, escape, words, state, b, se, sm);
        check(int'(dut.q2_lane) == lane && int'(dut.core_start) == start && int'(dut.core_freq) == freq
              && int'(dut.core_escape) == escape && int'(dut.s3_n_words) == words,
              $sformatf("push %0d (lane %0d, t %0d, symbol %0d) at the renorm stage: lane %0d interval (%0d, %0d) escape %0b, %0d words; the model's lane %0d (%0d, %0d) escape %0d, %0d words",
                        mon_renorm, lane, t, sym, dut.q2_lane, dut.core_start, dut.core_freq, dut.core_escape,
                        dut.s3_n_words, lane, start, freq, escape, words));
        mon_renorm++;
      end
      if (dut.sa_write) begin
        pans_stream_job_enc_step(mon_h, mon_write, lane, t, sym, start, freq, escape, words, state, b, se, sm);
        check(int'(dut.sa_lane) == lane && longint'(dut.sa_x) == state,
              $sformatf("push %0d (lane %0d, t %0d, symbol %0d) state write: lane %0d state %0h; the model's lane %0d state %0h",
                        mon_write, lane, t, sym, dut.sa_lane, dut.sa_x, lane, state));
        mon_write++;
      end
    end
  end

  // ---- Coverage, from the unit's internals (sampled once per cycle, at the falling edge)
  longint cov_symbols = 0, cov_escapes = 0, cov_two_words = 0, cov_f_one = 0, cov_f_max = 0;
  longint cov_busy_stall = 0, cov_credit_stall = 0, cov_fifo_high = 0, cov_fifo_full = 0, cov_err_order = 0;
  longint cov_outside = 0, cov_outside_coded = 0;  // inputs below a symmetric alphabet, and those coded
  longint cov_raw_at = 0, cov_raw_below = 0;        // escapes whose raw push reads B, B - 1

  always @(negedge clk) begin
    if (rst_n) begin
      cycle++;
      if (dut.q2_valid) begin
        cov_symbols++;
        if (dut.core_escape) cov_escapes++;
        if (dut.s3_push == 2'd2) cov_two_words++;
        if (!dut.core_escape && int'(dut.core_freq) == 1) cov_f_one++;
        if (!dut.core_escape && int'(dut.core_freq) == CODED_SLOTS) cov_f_max++;
        if (int'(signed'(dut.q2_symbol)) < SYMBOL_MIN) begin
          cov_outside++;
          if (!dut.core_escape) cov_outside_coded++;
        end
        if (dut.core_escape && longint'(dut.s3_x) == RAW_BOUND) cov_raw_at++;
        if (dut.core_escape && longint'(dut.s3_x) == RAW_BOUND - 1) cov_raw_below++;
      end
      if (int'(dut.fifo_count) >= OUT_DEPTH - 1) cov_fifo_high++;
      if (int'(dut.fifo_count) == OUT_DEPTH) cov_fifo_full++;
    end
  end

  // ---- One job through the unit
  // Runs job h with the given idle and stall percentages. bad_beat >= 0 corrupts that beat's lane field.
  // Returns the symbols accepted per cycle, 0 if n < 2.
  task automatic run_job(input int h, input string label, input int p_idle, input int p_stall, input int p_dstall,
                         input longint bad_beat, output real tput);
    longint n, n_words, n_escapes, k, w, first_cycle, last_cycle, quiet, b;
    int     d, gap, lane, sym;
    bit     sym_taken, word_seen, done_seen;
    logic [WORD_WIDTH-1:0] word_held;
    n         = pans_stream_job_n(h);
    n_words   = pans_stream_job_n_words(h);
    n_escapes = pans_stream_job_n_escapes(h);
    if (verbose)
      $display("job %s: n %0d, %0d words, %0d escapes, idle %0d%% stall %0d%% done stall %0d%%%s", label, n, n_words,
               n_escapes, p_idle, p_stall, p_dstall, bad_beat >= 0 ? ", a wrong lane" : "");
    tput = 0.0;
    mon_h      = h;
    mon_accept = 0;
    mon_renorm = 0;
    mon_write  = 0;
    // job_in: held until taken
    @(negedge clk);
    job_in_valid = 1'b1;
    job_in_n_sym = COUNT_WIDTH'(n);
    quiet = 0;
    while (!job_in_ready) begin
      @(negedge clk);
      quiet++;
      if (quiet > WATCHDOG) $fatal(1, "%s: job_in not taken in %0d cycles", label, WATCHDOG);
    end
    @(negedge clk);  // taken at the rising edge in between
    job_in_valid = 1'b0;
    k = 0;
    w = 0;
    d = 0;
    first_cycle = -1;
    last_cycle  = -1;
    gap = 0;
    while (rng_below(100) < p_idle) gap++;
    sym_taken = 1'b0;
    word_seen = 1'b0;
    done_seen = 1'b0;
    word_held = '0;
    quiet = 0;
    while (d < N_LANES) begin
      // sym_in: the next beat after its idle cycles, held until taken
      if (!sym_in_valid && k < n) begin
        if (gap > 0) begin
          gap--;
        end else begin
          pans_stream_job_push(h, k, lane, sym, b);
          sym_in_valid  = 1'b1;
          sym_in_lane   = LANE_WIDTH'(k == bad_beat ? (lane + 1) % (1 << LANE_WIDTH) : lane);
          sym_in_symbol = SYMBOL_WIDTH'(sym);
          sym_in_b      = B_WIDTH'(b);
        end
      end
      word_out_ready = rng_below(100) >= p_stall;
      done_out_ready = rng_below(100) >= p_dstall;
      // A word offered and not taken stays offered, unchanged (valid/ready).
      if (word_seen) check(word_out_valid && word_out_word == word_held,
                           $sformatf("%s: word_out dropped or changed a word before it was taken", label));
      word_seen = word_out_valid && !word_out_ready;
      word_held = word_out_word;
      if (done_seen) check(done_out_valid, $sformatf("%s: done_out dropped a beat before it was taken", label));
      done_seen = done_out_valid && !done_out_ready;
      // The transfers of the next rising edge, and the cause of a refused beat.
      if (sym_in_valid && !sym_in_ready && dut.q0_valid) begin
        if (dut.busy[dut.exp_lane]) cov_busy_stall++;
        if (!dut.s1_credit) cov_credit_stall++;
      end
      if (sym_in_valid && sym_in_ready) begin
        if (first_cycle < 0) first_cycle = cycle;
        last_cycle = cycle;
        k++;
        sym_taken = 1'b1;
        quiet = 0;
      end
      if (word_out_valid && word_out_ready) begin
        if (w < n_words)
          check(longint'(word_out_word) == pans_stream_job_word(h, w),
                $sformatf("%s: word %0d is %0h, the model's %0h", label, w, word_out_word, pans_stream_job_word(h, w)));
        else
          check(1'b0, $sformatf("%s: word %0d (%0h) beyond the model's %0d", label, w, word_out_word, n_words));
        w++;
        quiet = 0;
      end
      if (done_out_valid && done_out_ready) begin
        check(d > 0 || (k == n && w == n_words),
              $sformatf("%s: done_out after %0d of %0d beats and %0d of %0d words", label, k, n, w, n_words));
        check(int'(done_out_lane) == d && done_out_last == (d == N_LANES - 1),
              $sformatf("%s: done beat %0d has lane %0d, last %0b", label, d, done_out_lane, done_out_last));
        check(longint'(done_out_state) == pans_stream_job_state(h, d),
              $sformatf("%s: lane %0d final state %0h, the model's %0h", label, d, done_out_state,
                        pans_stream_job_state(h, d)));
        check(longint'(done_out_n_words) == n_words && longint'(done_out_n_escapes) == n_escapes,
              $sformatf("%s: done counts %0d words, %0d escapes; the model's %0d, %0d", label, done_out_n_words,
                        done_out_n_escapes, n_words, n_escapes));
        check(done_out_err_order == (bad_beat >= 0 && bad_beat < n),
              $sformatf("%s: err_order %0b", label, done_out_err_order));
        if (done_out_err_order) cov_err_order++;
        d++;
        quiet = 0;
      end
      @(negedge clk);
      quiet++;
      if (quiet > WATCHDOG)
        $fatal(1, "%s: no transfer in %0d cycles (%0d of %0d beats, %0d of %0d words, %0d done beats)", label,
               WATCHDOG, k, n, w, n_words, d);
      if (sym_taken) begin
        sym_in_valid = 1'b0;
        sym_taken    = 1'b0;
        while (rng_below(100) < p_idle) gap++;
      end
    end
    word_out_ready = 1'b0;
    done_out_ready = 1'b0;
    check(k == n && w == n_words, $sformatf("%s: %0d of %0d beats, %0d of %0d words", label, k, n, w, n_words));
    check(mon_accept == n && mon_renorm == n && mon_write == n,
          $sformatf("%s: %0d pushes into the core, %0d through the renorm stage, %0d state writes, of %0d", label,
                    mon_accept, mon_renorm, mon_write, n));
    mon_h = -1;
    if (n >= 2 && last_cycle > first_cycle) tput = real'(n) / real'(last_cycle - first_cycle + 1);
  endtask

  task automatic run_new(input string label, input longint n, input int kind, input int p_idle, input int p_stall,
                         input int p_dstall);
    int  h, delay;
    real tput;
    delay = kind == KIND_EMA ? rng_below(3) : 0;
    h = pans_stream_job_new(rng_seed(), N_LANES, n, kind, delay);
    if (h < 0) $fatal(1, "%s: pans_stream_job_new refused", label);
    run_job(h, label, p_idle, p_stall, p_dstall, -1, tput);
    pans_stream_job_free(h);
  endtask

  // ---- Directed jobs
  // (b, symbol >= 0) pairs coded with f = 1, from the core's reference. -x has f = 1 too, MAX only if -MAX does.
  localparam int SCAN_MAX = 4096, F_ONE_MAX = 64;
  longint f_one_b [$];
  int     f_one_sym [$];

  task automatic find_f_one();
    int     start, freq, escape, n_start, n_freq, n_escape, e, m;
    longint b;
    for (int l = B_K; l < B_WIDTH && f_one_sym.size() < F_ONE_MAX; l++)
      for (int k = 0; k < (1 << B_K) && f_one_sym.size() < F_ONE_MAX; k++) begin
        b = ((longint'(1) <<< B_K) + longint'(k)) <<< (l - B_K);
        pans_ref_scale_of_b(b, e, m);
        for (int x = 0; x <= SYMBOL_MAX && x < SCAN_MAX; x++) begin
          pans_ref_encode(e, m, x, start, freq, escape);
          if (escape != 0) break;
          n_freq = 1;
          n_escape = 0;
          if (x == SYMBOL_MAX) pans_ref_encode(e, m, -x, n_start, n_freq, n_escape);
          if (freq == 1 && n_freq == 1 && n_escape == 0) begin
            f_one_b.push_back(b);
            f_one_sym.push_back(x);
          end
        end
      end
  endtask

  function automatic int nonzero_symbol();
    int x;
    x = SYMBOL_IN_MIN + rng_below(SYMBOL_MAX - SYMBOL_IN_MIN + 1);
    if (x == 0) x = SYMBOL_MAX;
    return x;
  endfunction

  localparam longint D_N = 6 * N_LANES + 67;  // a directed job's symbols: unequal lanes, long enough to fill the FIFO
  localparam int D_STORM_ALL = 0, D_STORM_ONE = 1, D_F_ONE = 2, D_F_MAX = 3, D_ONE_SYMBOL = 4, D_EMPTY = 5;

  // A directed LIST job: an escape storm, f = 1 only, or f = M' only.
  function automatic int directed_job(input int what, input longint n);
    int     h, lane_storm, sym, p;
    longint b;
    h = pans_stream_job_begin(N_LANES, n, KIND_LIST, 0);
    if (h < 0) $fatal(1, "pans_stream_job_begin refused");
    lane_storm = N_LANES > 1 ? 1 : 0;
    for (longint i = 0; i < n; i++) begin
      sym = 0;
      b   = 0;
      case (what)
        D_STORM_ALL: sym = (i % 3 == 0) ? SYMBOL_MIN : (i % 3 == 1) ? SYMBOL_MAX : nonzero_symbol();
        D_STORM_ONE: sym = (i % longint'(N_LANES) == longint'(lane_storm)) ? nonzero_symbol() : 0;
        D_F_ONE: begin
          p   = rng_below(f_one_sym.size());
          sym = rng_below(2) == 0 ? f_one_sym[p] : -f_one_sym[p];
          b   = f_one_b[p];
        end
        D_F_MAX: b = longint'(rng_below(2));
        default: begin  // D_ONE_SYMBOL, D_EMPTY: a random symbol at a random b
          sym = SYMBOL_IN_MIN + rng_below(SYMBOL_MAX - SYMBOL_IN_MIN + 1);
          b   = longint'(rng_next() % u64_t'(longint'(1) <<< B_WIDTH));
        end
      endcase
      pans_stream_job_set(h, i, sym, b);
    end
    if (pans_stream_job_build(h) != 0) $fatal(1, "pans_stream_job_build refused a directed job");
    return h;
  endfunction

  task automatic run_directed(input string label, input int what, input longint n, input int p_idle,
                              input int p_stall, input int p_dstall);
    int  h;
    real tput;
    h = directed_job(what, n);
    run_job(h, label, p_idle, p_stall, p_dstall, -1, tput);
    pans_stream_job_free(h);
  endtask

  // ---- A reset in the middle of a job, then a whole job that must come out right
  task automatic reset_mid_job(input int kind);
    int     h, lane, sym;
    longint b;
    real    tput;
    h = pans_stream_job_new(rng_seed(), N_LANES, 3 * N_LANES + 5, kind, 0);
    @(negedge clk);
    job_in_valid = 1'b1;
    job_in_n_sym = COUNT_WIDTH'(pans_stream_job_n(h));
    while (!job_in_ready) @(negedge clk);
    @(negedge clk);
    job_in_valid = 1'b0;
    word_out_ready = 1'b1;
    for (longint k = 0; k < 2 * N_LANES + 1;) begin
      pans_stream_job_push(h, k, lane, sym, b);
      sym_in_valid  = 1'b1;
      sym_in_lane   = LANE_WIDTH'(lane);
      sym_in_symbol = SYMBOL_WIDTH'(sym);
      sym_in_b      = B_WIDTH'(b);
      if (sym_in_ready) k++;
      @(negedge clk);
    end
    sym_in_valid = 1'b0;
    repeat (3) @(negedge clk);
    rst_n = 1'b0;
    repeat (2) @(negedge clk);
    rst_n = 1'b1;
    word_out_ready = 1'b0;
    check(job_in_ready && !word_out_valid && !done_out_valid && !dut.q0_valid,
          "after a reset: not idle, or a word, a done beat or an input beat left");
    pans_stream_job_free(h);
    h = pans_stream_job_new(rng_seed(), N_LANES, 5 * N_LANES + 3, kind, 0);
    run_job(h, "after a reset mid-job", 10, 20, 10, -1, tput);
    pans_stream_job_free(h);
  endtask

  // ---- The first push's lane over the whole count range
  // Powers of two, their neighbours, 2^COUNT_WIDTH - 1 and random counts, each job abandoned by a reset.
  task automatic first_lane_counts();
    longint ns [$];
    longint max_n;
    max_n = (64'sd1 <<< COUNT_WIDTH) - 64'sd1;
    for (int b = 0; b < COUNT_WIDTH; b++) begin
      ns.push_back(64'sd1 <<< b);
      ns.push_back((64'sd1 <<< b) + 64'sd1);
      if (b > 1) ns.push_back((64'sd1 <<< b) - 64'sd1);
    end
    ns.push_back(max_n);
    repeat (200) ns.push_back(1 + longint'(rng_next() % u64_t'(max_n)));
    foreach (ns[q]) begin
      @(negedge clk);
      job_in_valid = 1'b1;
      job_in_n_sym = COUNT_WIDTH'(ns[q]);
      while (!job_in_ready) @(negedge clk);
      @(negedge clk);  // taken; the unit is in its setup cycle
      job_in_valid = 1'b0;
      @(negedge clk);  // the lane counter is loaded
      check(longint'(dut.exp_lane) == (ns[q] - 1) % longint'(N_LANES),
            $sformatf("first lane for n = %0d: %0d, expected %0d", ns[q], dut.exp_lane, (ns[q] - 1) % longint'(N_LANES)));
      rst_n = 1'b0;
      repeat (2) @(negedge clk);
      rst_n = 1'b1;
    end
  endtask

  // ---- Test
  initial begin
    int     seed, word_width, state_width, state_bytes, bf, bw, bk;
    int     p_idle [$], p_stall [$], p_dstall [$];
    int     pi, ps, pd, h;
    longint ns [$];
    real    tput;
    string  kind_name [3];

    kind_name = '{"CONST", "EMA", "LIST"};
    verbose = $test$plusargs("VERBOSE");
    if (!$value$plusargs("SEED=%d", seed)) seed = 1;
    rng_state = u64_t'(seed) * 64'h2545f4914f6cdd1d + 64'd1;

    // The C++ model must be the build this package was generated from.
    if (64'(pans_ref_config_id()) != CONFIG_ID)
      $fatal(1, "config mismatch: RTL package CONFIG_ID %016h, C++ reference %016h", CONFIG_ID, 64'(pans_ref_config_id()));
    pans_stream_widths(word_width, state_width, state_bytes);
    if (word_width != WORD_WIDTH || state_width != STATE_WIDTH || state_bytes != STATE_BYTES)
      $fatal(1, "stream width mismatch: RTL package W %0d, state %0d bits in %0d bytes; C++ model %0d, %0d, %0d",
             WORD_WIDTH, STATE_WIDTH, STATE_BYTES, word_width, state_width, state_bytes);
    pans_ref_b_widths(bf, bw, bk);
    if (bf != B_FRAC || bw != B_WIDTH || bk != B_K)
      $fatal(1, "config mismatch: RTL package B_FRAC %0d B_WIDTH %0d B_K %0d; C++ model %0d, %0d, %0d", B_FRAC,
             B_WIDTH, B_K, bf, bw, bk);

    $display("pans_enc_stream: PRECISION=%0d SYMBOL_WIDTH=%0d ESCAPE_SLOTS=%0d W=%0d STATE_WIDTH=%0d CONFIG_ID=%016h; N_LANES=%0d DIV_BITS=%0d DIV_STEP_BITS=%0d DIV_FIRST_BITS=%0d OUT_DEPTH=%0d COUNT_WIDTH=%0d CLOCK_GATE=%0d; seed %0d",
             PRECISION, SYMBOL_WIDTH, ESCAPE_SLOTS, WORD_WIDTH, STATE_WIDTH, CONFIG_ID, N_LANES, DIV_BITS,
             DIV_STEP_BITS, DIV_FIRST_BITS, OUT_DEPTH, COUNT_WIDTH, CLOCK_GATE, seed);

    check_lookup();

    repeat (3) @(negedge clk);
    rst_n = 1'b1;

    // 1. Throughput: no idle cycles, no stalls.
    for (int kind = 0; kind < 3; kind++) begin
      h = pans_stream_job_new(rng_seed(), N_LANES, 4000, kind, 0);
      run_job(h, $sformatf("throughput %s", kind_name[kind]), 0, 0, 0, -1, tput);
      $display("throughput: %.3f symbols/cycle (%s, n = 4000, no stalls; issue distance per lane %0d cycles, bound min(1, N/distance) = %.3f)",
               tput, kind_name[kind], dut.ISSUE_DISTANCE,
               (N_LANES >= dut.ISSUE_DISTANCE) ? 1.0 : real'(N_LANES) / real'(dut.ISSUE_DISTANCE));
      pans_stream_job_free(h);
    end

    // 2. Random jobs.
    if ($value$plusargs("P_IDLE=%d", pi) | $value$plusargs("P_STALL=%d", ps) | $value$plusargs("P_DSTALL=%d", pd)) begin
      if (!$value$plusargs("P_IDLE=%d", pi)) pi = 0;
      if (!$value$plusargs("P_STALL=%d", ps)) ps = 0;
      if (!$value$plusargs("P_DSTALL=%d", pd)) pd = 0;
      p_idle.push_back(pi);
      p_stall.push_back(ps);
      p_dstall.push_back(pd);
    end else begin
      p_idle   = '{0, 5, 30, 0, 60};
      p_stall  = '{0, 10, 30, 70, 0};
      p_dstall = '{0, 10, 30, 0, 50};
    end
    foreach (p_idle[p]) begin
      for (int kind = 0; kind < 3; kind++) begin
        ns.delete();
        ns.push_back(0);
        ns.push_back(1);
        ns.push_back(longint'(N_LANES) - 64'sd1);
        ns.push_back(longint'(N_LANES));
        ns.push_back(longint'(N_LANES) + 64'sd1);
        ns.push_back(longint'(rng_below(4000)) + 64'sd1);
        foreach (ns[q])
          run_new($sformatf("random %s n %0d profile %0d", kind_name[kind], ns[q], p), ns[q], kind, p_idle[p],
                  p_stall[p], p_dstall[p]);
      end
    end

    // 3. Directed jobs.
    find_f_one();
    check(f_one_sym.size() > 0, "no (b, symbol) with f = 1 found for the directed job");
    // No stalls; moderate stalls; word_out almost blocked (the FIFO fills at any input rate).
    for (int s = 0; s < 3; s++) begin
      pi = s == 1 ? 20 : 0;
      ps = s == 0 ? 0 : s == 1 ? 40 : 95;
      pd = s == 1 ? 20 : 0;
      run_directed("escape storm, every lane", D_STORM_ALL, D_N, pi, ps, pd);
      run_directed("escape storm, one lane", D_STORM_ONE, D_N, pi, ps, pd);
      if (f_one_sym.size() > 0) run_directed("f = 1 only", D_F_ONE, D_N, pi, ps, pd);
      run_directed("f = M' only", D_F_MAX, D_N, pi, ps, pd);
      run_directed("one symbol, the other lanes empty", D_ONE_SYMBOL, 1, pi, ps, pd);
      run_directed("n = 0", D_EMPTY, 0, pi, ps, pd);
    end

    // 4. A wrong lane field: err_order, the words and states still the model's.
    for (int kind = 0; kind < 3; kind++) begin
      h = pans_stream_job_new(rng_seed(), N_LANES, 3 * N_LANES + 2, kind, 0);
      run_job(h, $sformatf("wrong lane order %s", kind_name[kind]), 10, 10, 10, longint'(rng_below(3 * N_LANES + 2)),
              tput);
      pans_stream_job_free(h);
    end

    // 5. A reset in the middle of a job.
    reset_mid_job(KIND_LIST);

    // 6. The first push's lane for counts up to 2^32 - 1.
    first_lane_counts();

    // 7. The raw push's bound: escapes that read exactly B and B - 1, on seeds of their own.
    for (int s = 0; s < 2; s++) begin
      longint i0, i1;
      h = pans_stream_job_bound(seed * 1000 + 700 + s, N_LANES, i0, i1);
      check(h >= 0 && i0 >= 0 && (i1 >= 0 || !BELOW_POSSIBLE || N_LANES < 2),
            $sformatf("pans_stream_job_bound: handle %0d, escapes at %0d and %0d", h, i0, i1));
      if (h >= 0) begin
        run_job(h, "raw push bound", s == 0 ? 0 : 20, s == 0 ? 0 : 30, s == 0 ? 0 : 20, -1, tput);
        pans_stream_job_free(h);
      end
    end

    $display("coverage: %0d symbols, %0d escapes, %0d moving two words, %0d with f = 1, %0d with f = M'; cycles with a busy lane %0d, without FIFO credit %0d, with the FIFO at OUT_DEPTH - 1 or more %0d, full %0d; %0d err_order beats",
             cov_symbols, cov_escapes, cov_two_words, cov_f_one, cov_f_max, cov_busy_stall, cov_credit_stall,
             cov_fifo_high, cov_fifo_full, cov_err_order);
    check(cov_escapes > 0 && cov_f_one > 0 && cov_f_max > 0 && cov_err_order > 0,
          "coverage: escapes, f = 1, f = M' and err_order must all occur");
    check(cov_two_words > 0 || !TWO_WORDS_POSSIBLE,
          "coverage: an escape moving two words (W + log2 E < P + S) must occur");
    $display("coverage: escapes whose raw push read B = 2^(P + W - S) %0d, B - 1 %0d%s", cov_raw_at, cov_raw_below,
             BELOW_POSSIBLE ? string'("") : string'(" (B - 1 < L: S = W, every raw push emits)"));
    check(cov_raw_at > 0 && (cov_raw_below > 0 || !BELOW_POSSIBLE || N_LANES < 2),
          "coverage: escapes reading the raw push's bound and (if a state) one below it must occur");
    if (SYMBOL_IN_MIN < SYMBOL_MIN) begin
      $display("coverage: %0d inputs %0d outside the symmetric alphabet, %0d of them coded", cov_outside, SYMBOL_IN_MIN,
               cov_outside_coded);
      check(cov_outside > 0 && cov_outside_coded == 0,
            "coverage: inputs outside the symmetric alphabet must occur, each escaped");
    end
    // Busy-lane stalls need N_LANES < ISSUE_DISTANCE and OUT_DEPTH >= 6.
    check(cov_busy_stall > 0 || N_LANES >= dut.ISSUE_DISTANCE || OUT_DEPTH < 6,
          "coverage: a beat refused for a busy lane must occur (N_LANES < the issue distance)");
    // A full FIFO needs a two-word escape at OUT_DEPTH - 2 words. It is counted, not required.
    check(cov_credit_stall > 0 && cov_fifo_high > 0,
          "coverage: FIFO-credit stalls and a FIFO at OUT_DEPTH - 1 words or more must both occur");

    if (errors != 0) $fatal(1, "FAILED: %0d errors in %0d checks", errors, checks);
    $display("PASSED: %0d checks, 0 errors", checks);
    $finish;
  end

endmodule
