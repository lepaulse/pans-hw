// SPDX-License-Identifier: Apache-2.0
// power_tb_stream.sv: a stream unit's post-route netlist against the stream model (make power-stream-<cfg>).
// It drives the timing wrapper at the target period with DPI jobs and checks every output beat.
// Specification: STREAM.md §4.1.
//   +CHECK   a gate-level correctness check: corner jobs with random stalls on every channel
//   default  the power workload: +JOBS=<j> jobs of +N=<n> symbols, dumped to +VCD=<file>. +STIMULUS= picks
//            them: laplace (the default) the model's EMA jobs at +DELAY=<d>; static the mid-alphabet symbol at the
//            mid-range b for every symbol; random symbols uniform over the alphabet with b uniform over its B_WIDTH
//            bits (static and random are LIST jobs)
// +P_IDLE=<%> (default 0) holds the producer back on that share of the cycles, the stimulus otherwise the same.
// The last line is "<top>: <jobs> jobs, <symbols> symbols, <cycles> cycles, <checks> checks, <mismatches> mismatches,
// <escapes> escapes".
// With +P_IDLE above 0, "<top>: producer idle <%>" comes before it.
// Inputs change on the falling edge. Transfers are sampled 1 ps later.

module power_tb_dec_stream #(
  parameter int PERIOD_PS   = 10000,  // an integer: OpenSTA's VCD reader mis-parses some real-valued parameters
  parameter int N_LANES     = 8,      // the netlist's unit parameters, built into it
  parameter int DATA_BYTES  = 4,
  parameter int FIFO_WORDS  = 8,
  parameter int OUT_DEPTH   = 7,
  parameter int COUNT_WIDTH = 32,
  parameter bit CLOCK_GATE  = 1
);
  import pans_pkg::*;

  import "DPI-C" function longint pans_ref_config_id();
  import "DPI-C" function void pans_stream_widths(output int word_width, output int state_width,
                                                  output int state_bytes);
  import "DPI-C" function int pans_stream_job_new(input int seed, input int n_lanes, input longint n, input int kind,
                                                  input int delay);
  import "DPI-C" function int pans_stream_job_begin(input int n_lanes, input longint n, input int kind,
                                                    input int delay);
  import "DPI-C" function void pans_stream_job_set(input int job, input longint i, input int symbol, input longint b);
  import "DPI-C" function int pans_stream_job_build(input int job);
  import "DPI-C" function void pans_stream_job_corrupt(input int job, input int how, input longint pos,
                                                       input int value);
  import "DPI-C" function longint pans_stream_job_n(input int job);
  import "DPI-C" function longint pans_stream_job_n_bytes(input int job);
  import "DPI-C" function int pans_stream_job_byte(input int job, input longint k);
  import "DPI-C" function void pans_stream_job_step(input int job, input longint i, output int lane,
                                                    output longint t, output int slot, output int start,
                                                    output int freq, output int escape, output int raw,
                                                    output int symbol, output longint b, output int scale_exp,
                                                    output int scale_man, output int words, output longint state,
                                                    output int errors);
  import "DPI-C" function void pans_stream_job_done(input int job, output int ok, output longint n_escapes,
                                                    output longint words_consumed, output int err_words,
                                                    output int err_header);
  import "DPI-C" function int pans_stream_scale(input int job, input int lane, input longint t, output longint b);
  import "DPI-C" function int pans_stream_observe(input int job, input int lane, input longint t, input int symbol);

  localparam int     LANE_WIDTH  = (N_LANES > 1) ? $clog2(N_LANES) : 1;
  localparam int     POS_WIDTH   = unsigned_width(((longint'(1) <<< COUNT_WIDTH) - 2) / longint'(N_LANES));
  localparam longint NL          = longint'(N_LANES);
  localparam longint DBL         = longint'(DATA_BYTES);
  localparam longint WATCHDOG    = 10000;
  localparam int     KIND_CONST = 0, KIND_EMA = 1, KIND_LIST = 2;

  logic clk   = 1'b0;
  logic rst_n = 1'b0;
  always #(PERIOD_PS / 2 * 1ps) clk = ~clk;

  logic                           job_in_valid   = 1'b0;
  logic                           job_in_ready;
  logic [COUNT_WIDTH-1:0]         job_in_n_sym   = '0;
  logic [COUNT_WIDTH-1:0]         job_in_n_bytes = '0;
  logic                           data_in_valid  = 1'b0;
  logic                           data_in_ready;
  logic [8*DATA_BYTES-1:0]        data_in_data   = '0;
  logic                           scale_in_valid = 1'b0;
  logic                           scale_in_ready;
  logic [LANE_WIDTH-1:0]          scale_in_lane  = '0;
  logic [B_WIDTH-1:0]             scale_in_b     = '0;
  logic                           sym_out_valid;
  logic                           sym_out_ready  = 1'b1;
  logic [LANE_WIDTH-1:0]          sym_out_lane;
  logic [POS_WIDTH-1:0]           sym_out_t;
  logic signed [SYMBOL_WIDTH-1:0] sym_out_symbol;
  logic                           sym_out_escape;
  logic                           done_out_valid;
  logic                           done_out_ready = 1'b1;
  logic                           done_out_ok;
  logic [COUNT_WIDTH-1:0]         done_out_n_escapes;
  logic [COUNT_WIDTH-1:0]         done_out_words_consumed;
  logic                           done_out_err_words;
  logic                           done_out_err_header;

  pans_dec_stream_timed dut (
    .clk, .rst_n, .job_in_valid, .job_in_ready, .job_in_n_sym, .job_in_n_bytes, .data_in_valid, .data_in_ready,
    .data_in_data, .scale_in_valid, .scale_in_ready, .scale_in_lane, .scale_in_b, .sym_out_valid,
    .sym_out_ready, .sym_out_lane, .sym_out_t, .sym_out_symbol, .sym_out_escape, .done_out_valid, .done_out_ready,
    .done_out_ok, .done_out_n_escapes, .done_out_words_consumed, .done_out_err_words, .done_out_err_header
  );

  // splitmix64 for the stalls of +CHECK
  longint unsigned rng_s = 64'h70616e7364656321;
  function automatic longint unsigned rng_next();
    longint unsigned z;
    rng_s = rng_s + 64'h9e3779b97f4a7c15;
    z = rng_s;
    z = (z ^ (z >> 30)) * 64'hbf58476d1ce4e5b9;
    z = (z ^ (z >> 27)) * 64'h94d049bb133111eb;
    return z ^ (z >> 31);
  endfunction
  function automatic bit chance(input int pct);
    return (rng_next() % 100) < longint'(pct);
  endfunction

  longint checks = 0, mismatches = 0;
  function automatic void check(input bit ok, input string what);
    checks++;
    if (!ok) begin
      mismatches++;
      if (mismatches <= 10) $display("MISMATCH %s", what);
    end
  endfunction

  int     jb_h[$];
  longint jb_n[$], jb_nb[$], jb_beats[$];
  function automatic void add_job(input int h);
    if (h < 0) $fatal(1, "power_tb_dec_stream: the model refused a job");
    jb_h.push_back(h);
    jb_n.push_back(pans_stream_job_n(h));
    jb_nb.push_back(pans_stream_job_n_bytes(h));
    jb_beats.push_back((pans_stream_job_n_bytes(h) + DBL - 1) / DBL);
  endfunction

  // A workload job of +STIMULUS (the header); static and random draw from rng_next before the run
  function automatic int workload_job(input int j, input string stimulus, input longint n, input int delay);
    int h;
    if (stimulus == "laplace") return pans_stream_job_new(16'h7061 + j, N_LANES, n, KIND_EMA, delay);
    if (stimulus != "static" && stimulus != "random")
      $fatal(1, "power_tb_dec_stream: +STIMULUS=%s: laplace, static or random", stimulus);
    h = pans_stream_job_begin(N_LANES, n, KIND_LIST, 0);
    for (longint i = 0; i < n; i++)
      if (stimulus == "static") pans_stream_job_set(h, i, (SYMBOL_MIN + SYMBOL_MAX) / 2, longint'(1) << (B_WIDTH - 1));
      else pans_stream_job_set(h, i, SYMBOL_MIN + int'(rng_next() % longint'(SYMBOL_MAX - SYMBOL_MIN + 1)),
                               longint'(rng_next() % (longint'(1) << B_WIDTH)));
    return pans_stream_job_build(h) == 0 ? h : -1;
  endfunction

  initial begin
    bit     check_mode;
    int     n_jobs, delay, ww, sw, sb, pct, lat_max, p_idle;
    longint n_sym, symbols, cyc, start, last_progress;
    // drivers: job jd, data job dd beat db, scale job sf index sfi; checkers: symbol job oj index oi, done job dn
    int     jd, dd, sf, oj, dn;
    longint db, sfi, oi;
    bit     job_fired, data_fired, scale_fired, progress;
    int     sq_job[$], sq_lane[$];
    longint sq_b[$];
    longint sq_at[$];
    longint escapes;
    string  vcd_file, stimulus;

    check_mode = $test$plusargs("CHECK");
    if (!$value$plusargs("JOBS=%d", n_jobs)) n_jobs = 2;
    if (!$value$plusargs("N=%d", n_sym)) n_sym = 4096;
    if (!$value$plusargs("DELAY=%d", delay)) delay = 0;
    if (!$value$plusargs("STIMULUS=%s", stimulus)) stimulus = "laplace";
    if (!$value$plusargs("P_IDLE=%d", p_idle)) p_idle = 0;
    if (64'(pans_ref_config_id()) != CONFIG_ID)
      $fatal(1, "config mismatch: RTL package CONFIG_ID %016h, C++ reference %016h", CONFIG_ID,
             64'(pans_ref_config_id()));
    pans_stream_widths(ww, sw, sb);
    if (ww != WORD_WIDTH || sw != STATE_WIDTH || sb != STATE_BYTES) $fatal(1, "config mismatch: stream widths");

    if (check_mode) begin
      int h;
      add_job(pans_stream_job_new(101, N_LANES, 3000, KIND_LIST, 0));
      add_job(pans_stream_job_new(102, N_LANES, 1000, KIND_CONST, 0));
      add_job(pans_stream_job_new(103, N_LANES, 1000, KIND_EMA, 1));
      add_job(pans_stream_job_new(104, N_LANES, 777, KIND_EMA, 2));
      add_job(pans_stream_job_new(105, N_LANES, 0, KIND_LIST, 0));
      add_job(pans_stream_job_new(106, N_LANES, NL + 1, KIND_LIST, 0));
      h = pans_stream_job_new(107, N_LANES, 1500, KIND_LIST, 0);  // a flipped byte in the words
      pans_stream_job_corrupt(h, 0, pans_stream_job_n_bytes(h) / 2, 8'h5a);
      add_job(h);
      h = pans_stream_job_new(108, N_LANES, 800, KIND_CONST, 0);  // truncated
      pans_stream_job_corrupt(h, 1, pans_stream_job_n_bytes(h) - 3, 0);
      add_job(h);
      h = pans_stream_job_new(109, N_LANES, 600, KIND_EMA, 0);    // a byte appended
      pans_stream_job_corrupt(h, 2, 0, 8'ha5);
      add_job(h);
      // an escape storm: every symbol nonzero at b = 0 (the narrowest code), so every symbol escapes
      h = pans_stream_job_begin(N_LANES, 5 * NL + 3, KIND_LIST, 0);
      for (longint i = 0; i < 5 * NL + 3; i++)
        pans_stream_job_set(h, i, (i % 2 == 0) ? SYMBOL_MAX - int'(i % 5) : SYMBOL_MIN + int'(i % 7), 0);
      if (pans_stream_job_build(h) != 0) $fatal(1, "power_tb_dec_stream: the storm job did not build");
      add_job(h);
    end else begin
      for (int j = 0; j < n_jobs; j++) add_job(workload_job(j, stimulus, n_sym, delay));
    end
    pct     = check_mode ? 15 : 0;
    lat_max = check_mode ? 3 : 1;
    if (check_mode) p_idle = pct;  // +CHECK keeps its own stalls
    else if (p_idle != 0) $display("power_tb_dec_stream: producer idle %0d%%", p_idle);

    jd = 0; dd = 0; sf = 0; oj = 0; dn = 0;
    db = 0; sfi = 0; oi = 0;
    job_fired = 0; data_fired = 0; scale_fired = 0;
    symbols = 0;
    escapes = 0;
    repeat (4) @(negedge clk);
    rst_n = 1'b1;
    if ($value$plusargs("VCD=%s", vcd_file)) begin
      $dumpfile(vcd_file);
      $dumpvars(0, dut);
    end
    start = 0;
    cyc = 0;
    last_progress = 0;
    while (dn < jb_h.size()) begin
      @(negedge clk);
      cyc++;
      // job_in, held until taken
      if (job_fired) begin
        job_in_valid = 1'b0;
        job_fired    = 1'b0;
      end
      if (!job_in_valid && jd < jb_h.size() && !chance(pct)) begin
        job_in_valid   = 1'b1;
        job_in_n_sym   = COUNT_WIDTH'(jb_n[jd]);
        job_in_n_bytes = COUNT_WIDTH'(jb_nb[jd]);
      end
      // data_in: every job's beats in order, bytes beyond the array 0
      if (data_fired) begin
        data_in_valid = 1'b0;
        data_fired    = 1'b0;
      end
      while (dd < jb_h.size() && db >= jb_beats[dd]) begin
        dd++;
        db = 0;
      end
      if (!data_in_valid && dd < jb_h.size() && !chance(pct)) begin
        data_in_valid = 1'b1;
        for (int b = 0; b < DATA_BYTES; b++) begin
          longint at;
          at = db * DBL + longint'(b);
          data_in_data[8*b +: 8] = (at < jb_nb[dd]) ? 8'(pans_stream_job_byte(jb_h[dd], at)) : 8'h00;
        end
      end
      // scale_in: every computable scale is fetched in decode order and presented after the consumer's latency
      if (scale_fired) begin
        scale_in_valid = 1'b0;
        scale_fired    = 1'b0;
        void'(sq_job.pop_front());
        void'(sq_lane.pop_front());
        void'(sq_b.pop_front());
        void'(sq_at.pop_front());
      end
      while (sf < jb_h.size() && sq_job.size() < 4 * N_LANES) begin
        int     got;
        longint b;
        if (sfi >= jb_n[sf]) begin
          sf++;
          sfi = 0;
          continue;
        end
        got = pans_stream_scale(jb_h[sf], int'(sfi % NL), sfi / NL, b);
        if (got != 1) begin
          check(got == 0, $sformatf("job %0d: pans_stream_scale(%0d, %0d) = %0d", sf, sfi % NL, sfi / NL, got));
          break;
        end
        sq_job.push_back(sf);
        sq_lane.push_back(int'(sfi % NL));
        sq_b.push_back(b);
        sq_at.push_back(cyc + longint'(rng_next() % longint'(lat_max)));
        sfi++;
      end
      if (!scale_in_valid && sq_job.size() > 0 && cyc >= sq_at[0] && !chance(p_idle)) begin
        scale_in_valid = 1'b1;
        scale_in_lane  = LANE_WIDTH'(sq_lane[0]);
        scale_in_b     = B_WIDTH'(sq_b[0]);
      end
      sym_out_ready  = !chance(pct);
      done_out_ready = !chance(pct);

      #1ps;
      progress = 1'b0;
      if (job_in_valid && job_in_ready) begin
        job_fired = 1'b1;
        progress  = 1'b1;
        jd++;
      end
      if (data_in_valid && data_in_ready) begin
        data_fired = 1'b1;
        progress   = 1'b1;
        db++;
      end
      if (scale_in_valid && scale_in_ready) begin
        scale_fired = 1'b1;
        progress    = 1'b1;
      end
      if (sym_out_valid && sym_out_ready) begin
        int     lane, slot, st, fr, esc, raw, sym, se, sm, words, errs;
        longint t, state, b;
        progress = 1'b1;
        while (oj < jb_h.size() && oi >= jb_n[oj]) begin
          oj++;
          oi = 0;
        end
        if (oj >= jb_h.size()) begin
          check(1'b0, "a symbol beyond the last job");
        end else begin
          pans_stream_job_step(jb_h[oj], oi, lane, t, slot, st, fr, esc, raw, sym, b, se, sm, words, state, errs);
          check(int'(sym_out_lane) == lane && longint'(sym_out_t) == t && int'(sym_out_symbol) == sym
                && int'(sym_out_escape) == esc,
                $sformatf("job %0d symbol %0d: lane %0d t %0d symbol %0d escape %0d, model %0d %0d %0d %0d", oj, oi,
                          sym_out_lane, sym_out_t, int'(sym_out_symbol), sym_out_escape, lane, t, sym, esc));
          check(pans_stream_observe(jb_h[oj], lane, t, int'(sym_out_symbol)) == 0,
                $sformatf("job %0d symbol %0d: the consumer refused it", oj, oi));
          oi++;
          symbols++;
          if (sym_out_escape) escapes++;
        end
      end
      if (done_out_valid && done_out_ready) begin
        int     ok, e_words, e_head;
        longint n_esc, consumed;
        progress = 1'b1;
        pans_stream_job_done(jb_h[dn], ok, n_esc, consumed, e_words, e_head);
        check(int'(done_out_ok) == ok && longint'(done_out_n_escapes) == n_esc
              && longint'(done_out_words_consumed) == consumed && int'(done_out_err_words) == e_words
              && int'(done_out_err_header) == e_head,
              $sformatf("job %0d done: ok %0d escapes %0d words %0d errors %b, model ok %0d escapes %0d words %0d",
                        dn, done_out_ok, done_out_n_escapes, done_out_words_consumed,
                        {done_out_err_words, done_out_err_header}, ok, n_esc, consumed));
        dn++;
      end
      if (progress) last_progress = cyc;
      if (cyc - last_progress > WATCHDOG) $fatal(1, "power_tb_dec_stream: no transfer for %0d cycles", WATCHDOG);
    end
    @(negedge clk);
    cyc++;
    begin
      longint all;
      all = 0;
      foreach (jb_n[j]) all += jb_n[j];
      check(symbols == all, $sformatf("%0d symbols out of %0d", symbols, all));
    end
    $display("power_tb_dec_stream: %0d jobs, %0d symbols, %0d cycles, %0d checks, %0d mismatches, %0d escapes",
             jb_h.size(), symbols, cyc - start, checks, mismatches, escapes);
    if (mismatches != 0 || symbols == 0 && !check_mode) $fatal(1, "netlist does not match the reference");
    $finish;
  end
endmodule

module power_tb_enc_stream #(
  parameter int PERIOD_PS      = 10000,
  parameter int N_LANES        = 8,  // the netlist's unit parameters, built into it
  parameter int DIV_BITS       = 2,
  parameter int DIV_STEP_BITS  = 1,
  parameter int DIV_FIRST_BITS = DIV_STEP_BITS,
  parameter int OUT_DEPTH      = 8,
  parameter int COUNT_WIDTH    = 32,
  parameter bit CLOCK_GATE     = 1
);
  import pans_pkg::*;

  import "DPI-C" function longint pans_ref_config_id();
  import "DPI-C" function void pans_stream_widths(output int word_width, output int state_width,
                                                  output int state_bytes);
  import "DPI-C" function int pans_stream_job_new(input int seed, input int n_lanes, input longint n, input int kind,
                                                  input int delay);
  import "DPI-C" function int pans_stream_job_begin(input int n_lanes, input longint n, input int kind,
                                                    input int delay);
  import "DPI-C" function void pans_stream_job_set(input int job, input longint i, input int symbol, input longint b);
  import "DPI-C" function int pans_stream_job_build(input int job);
  import "DPI-C" function void pans_stream_job_push(input int job, input longint k, output int lane, output int symbol,
                                                    output longint b);
  import "DPI-C" function longint pans_stream_job_n(input int job);
  import "DPI-C" function longint pans_stream_job_n_words(input int job);
  import "DPI-C" function longint pans_stream_job_word(input int job, input longint k);
  import "DPI-C" function longint pans_stream_job_state(input int job, input int lane);
  import "DPI-C" function longint pans_stream_job_n_escapes(input int job);

  localparam int     LANE_WIDTH  = (N_LANES > 1) ? $clog2(N_LANES) : 1;
  localparam longint NL          = longint'(N_LANES);
  localparam longint WATCHDOG    = 10000;
  localparam int     KIND_CONST = 0, KIND_EMA = 1, KIND_LIST = 2;

  logic clk   = 1'b0;
  logic rst_n = 1'b0;
  always #(PERIOD_PS / 2 * 1ps) clk = ~clk;

  logic                           job_in_valid   = 1'b0;
  logic                           job_in_ready;
  logic [COUNT_WIDTH-1:0]         job_in_n_sym   = '0;
  logic                           sym_in_valid   = 1'b0;
  logic                           sym_in_ready;
  logic [LANE_WIDTH-1:0]          sym_in_lane    = '0;
  logic signed [SYMBOL_WIDTH-1:0] sym_in_symbol  = '0;
  logic [B_WIDTH-1:0]             sym_in_b       = '0;
  logic                           word_out_valid;
  logic                           word_out_ready = 1'b1;
  logic [WORD_WIDTH-1:0]          word_out_word;
  logic                           done_out_valid;
  logic                           done_out_ready = 1'b1;
  logic [LANE_WIDTH-1:0]          done_out_lane;
  logic                           done_out_last;
  logic [STATE_WIDTH-1:0]         done_out_state;
  logic [COUNT_WIDTH:0]           done_out_n_words;
  logic [COUNT_WIDTH-1:0]         done_out_n_escapes;
  logic                           done_out_err_order;

  pans_enc_stream_timed dut (
    .clk, .rst_n, .job_in_valid, .job_in_ready, .job_in_n_sym, .sym_in_valid, .sym_in_ready, .sym_in_lane,
    .sym_in_symbol, .sym_in_b, .word_out_valid, .word_out_ready, .word_out_word, .done_out_valid,
    .done_out_ready, .done_out_lane, .done_out_last, .done_out_state, .done_out_n_words, .done_out_n_escapes,
    .done_out_err_order
  );

  longint unsigned rng_s = 64'h70616e7365656e63;
  function automatic longint unsigned rng_next();
    longint unsigned z;
    rng_s = rng_s + 64'h9e3779b97f4a7c15;
    z = rng_s;
    z = (z ^ (z >> 30)) * 64'hbf58476d1ce4e5b9;
    z = (z ^ (z >> 27)) * 64'h94d049bb133111eb;
    return z ^ (z >> 31);
  endfunction
  function automatic bit chance(input int pct);
    return (rng_next() % 100) < longint'(pct);
  endfunction

  longint checks = 0, mismatches = 0;
  function automatic void check(input bit ok, input string what);
    checks++;
    if (!ok) begin
      mismatches++;
      if (mismatches <= 10) $display("MISMATCH %s", what);
    end
  endfunction

  int     jb_h[$];
  longint jb_n[$], jb_words[$];
  function automatic void add_job(input int h);
    if (h < 0) $fatal(1, "power_tb_enc_stream: the model refused a job");
    jb_h.push_back(h);
    jb_n.push_back(pans_stream_job_n(h));
    jb_words.push_back(pans_stream_job_n_words(h));
  endfunction

  // A workload job of +STIMULUS (the header); static and random draw from rng_next before the run
  function automatic int workload_job(input int j, input string stimulus, input longint n, input int delay);
    int h;
    if (stimulus == "laplace") return pans_stream_job_new(16'h7061 + j, N_LANES, n, KIND_EMA, delay);
    if (stimulus != "static" && stimulus != "random")
      $fatal(1, "power_tb_enc_stream: +STIMULUS=%s: laplace, static or random", stimulus);
    h = pans_stream_job_begin(N_LANES, n, KIND_LIST, 0);
    for (longint i = 0; i < n; i++)
      if (stimulus == "static") pans_stream_job_set(h, i, (SYMBOL_MIN + SYMBOL_MAX) / 2, longint'(1) << (B_WIDTH - 1));
      else pans_stream_job_set(h, i, SYMBOL_MIN + int'(rng_next() % longint'(SYMBOL_MAX - SYMBOL_MIN + 1)),
                               longint'(rng_next() % (longint'(1) << B_WIDTH)));
    return pans_stream_job_build(h) == 0 ? h : -1;
  endfunction

  initial begin
    bit     check_mode;
    int     n_jobs, ww, sw, sb, pct, p_idle;
    longint n_sym, symbols, cyc, start, last_progress;
    // drivers: job jd, push job sj beat k; checkers: word job wj word w, done job dn beat d
    int     jd, sj, wj, dn, d;
    longint k, w, words;
    bit     job_fired, sym_fired, progress;
    longint escapes;
    string  vcd_file, stimulus;

    check_mode = $test$plusargs("CHECK");
    if (!$value$plusargs("JOBS=%d", n_jobs)) n_jobs = 2;
    if (!$value$plusargs("N=%d", n_sym)) n_sym = 4096;
    if (!$value$plusargs("P_IDLE=%d", p_idle)) p_idle = 0;
    if (!$value$plusargs("STIMULUS=%s", stimulus)) stimulus = "laplace";
    if (64'(pans_ref_config_id()) != CONFIG_ID)
      $fatal(1, "config mismatch: RTL package CONFIG_ID %016h, C++ reference %016h", CONFIG_ID,
             64'(pans_ref_config_id()));
    pans_stream_widths(ww, sw, sb);
    if (ww != WORD_WIDTH || sw != STATE_WIDTH || sb != STATE_BYTES) $fatal(1, "config mismatch: stream widths");

    if (check_mode) begin
      int h;
      add_job(pans_stream_job_new(101, N_LANES, 3000, KIND_LIST, 0));
      add_job(pans_stream_job_new(102, N_LANES, 1000, KIND_CONST, 0));
      add_job(pans_stream_job_new(103, N_LANES, 1000, KIND_EMA, 1));
      add_job(pans_stream_job_new(104, N_LANES, 777, KIND_EMA, 2));
      add_job(pans_stream_job_new(105, N_LANES, 0, KIND_LIST, 0));
      add_job(pans_stream_job_new(106, N_LANES, 1, KIND_LIST, 0));
      add_job(pans_stream_job_new(107, N_LANES, NL + 1, KIND_LIST, 0));
      h = pans_stream_job_begin(N_LANES, 5 * NL + 3, KIND_LIST, 0);  // an escape storm, as the decoder's
      for (longint i = 0; i < 5 * NL + 3; i++)
        pans_stream_job_set(h, i, (i % 2 == 0) ? SYMBOL_MAX - int'(i % 5) : SYMBOL_MIN + int'(i % 7), 0);
      if (pans_stream_job_build(h) != 0) $fatal(1, "power_tb_enc_stream: the storm job did not build");
      add_job(h);
    end else begin
      for (int j = 0; j < n_jobs; j++) add_job(workload_job(j, stimulus, n_sym, 0));
    end
    pct = check_mode ? 15 : 0;
    if (check_mode) p_idle = pct;  // +CHECK keeps its own stalls
    else if (p_idle != 0) $display("power_tb_enc_stream: producer idle %0d%%", p_idle);

    jd = 0; sj = 0; wj = 0; dn = 0; d = 0;
    k = 0; w = 0; words = 0;
    job_fired = 0; sym_fired = 0;
    symbols = 0;
    repeat (4) @(negedge clk);
    rst_n = 1'b1;
    if ($value$plusargs("VCD=%s", vcd_file)) begin
      $dumpfile(vcd_file);
      $dumpvars(0, dut);
    end
    start = 0;
    cyc = 0;
    last_progress = 0;
    while (dn < jb_h.size()) begin
      @(negedge clk);
      cyc++;
      if (job_fired) begin
        job_in_valid = 1'b0;
        job_fired    = 1'b0;
      end
      if (!job_in_valid && jd < jb_h.size() && !chance(pct)) begin
        job_in_valid = 1'b1;
        job_in_n_sym = COUNT_WIDTH'(jb_n[jd]);
      end
      // sym_in: every job's push beats in order (global index n - 1 down to 0)
      if (sym_fired) begin
        sym_in_valid = 1'b0;
        sym_fired    = 1'b0;
      end
      while (sj < jb_h.size() && k >= jb_n[sj]) begin
        sj++;
        k = 0;
      end
      if (!sym_in_valid && sj < jb_h.size() && !chance(p_idle)) begin
        int     lane, sym;
        longint b;
        pans_stream_job_push(jb_h[sj], k, lane, sym, b);
        sym_in_valid  = 1'b1;
        sym_in_lane   = LANE_WIDTH'(lane);
        sym_in_symbol = SYMBOL_WIDTH'(sym);
        sym_in_b      = B_WIDTH'(b);
      end
      word_out_ready = !chance(pct);
      done_out_ready = !chance(pct);

      #1ps;
      progress = 1'b0;
      if (job_in_valid && job_in_ready) begin
        job_fired = 1'b1;
        progress  = 1'b1;
        jd++;
      end
      if (sym_in_valid && sym_in_ready) begin
        sym_fired = 1'b1;
        progress  = 1'b1;
        k++;
        symbols++;
      end
      if (word_out_valid && word_out_ready) begin
        progress = 1'b1;
        while (wj < jb_h.size() && w >= jb_words[wj]) begin
          wj++;
          w = 0;
        end
        if (wj >= jb_h.size()) begin
          check(1'b0, "a word beyond the last job");
        end else begin
          check(longint'(word_out_word) == pans_stream_job_word(jb_h[wj], w),
                $sformatf("job %0d word %0d: %0h, model %0h", wj, w, word_out_word, pans_stream_job_word(jb_h[wj], w)));
          w++;
          words++;
        end
      end
      if (done_out_valid && done_out_ready) begin
        progress = 1'b1;
        check(int'(done_out_lane) == d && done_out_last == (d == N_LANES - 1)
              && longint'(done_out_state) == pans_stream_job_state(jb_h[dn], d)
              && longint'(done_out_n_words) == jb_words[dn]
              && longint'(done_out_n_escapes) == pans_stream_job_n_escapes(jb_h[dn]) && !done_out_err_order,
              $sformatf("job %0d done beat %0d: lane %0d last %0b state %0h words %0d escapes %0d err_order %0b, model state %0h words %0d escapes %0d",
                        dn, d, done_out_lane, done_out_last, done_out_state, done_out_n_words, done_out_n_escapes,
                        done_out_err_order, pans_stream_job_state(jb_h[dn], d), jb_words[dn],
                        pans_stream_job_n_escapes(jb_h[dn])));
        d++;
        if (d == N_LANES) begin
          d = 0;
          dn++;
        end
      end
      if (progress) last_progress = cyc;
      if (cyc - last_progress > WATCHDOG) $fatal(1, "power_tb_enc_stream: no transfer for %0d cycles", WATCHDOG);
    end
    @(negedge clk);
    cyc++;
    begin
      longint all;
      all = 0;
      foreach (jb_words[j]) all += jb_words[j];
      check(words == all, $sformatf("%0d words out of %0d", words, all));
    end
    escapes = 0;  // the model's, which every done beat's n_escapes matched
    foreach (jb_h[j]) escapes += pans_stream_job_n_escapes(jb_h[j]);
    $display("power_tb_enc_stream: %0d jobs, %0d symbols, %0d cycles, %0d checks, %0d mismatches, %0d escapes",
             jb_h.size(), symbols, cyc - start, checks, mismatches, escapes);
    if (mismatches != 0 || symbols == 0 && !check_mode) $fatal(1, "netlist does not match the reference");
    $finish;
  end
endmodule

