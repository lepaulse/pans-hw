// SPDX-License-Identifier: Apache-2.0
// tb_pans_dec_stream: pans_dec_stream against the stream model through its DPI (make sim-dec-stream-<cfg>).
// The model is ref/pans_stream_dpi.cpp with ref/pans1_dpi.cpp. Specification: STREAM.md §3.
// The model builds every job and is the consumer, fed with the unit's own sym_out beats.
// Every coverage corner must occur. A watchdog fails a run without a transfer for WATCHDOG cycles.
//
// Plusargs: +SEED=<n> (default 1), +TP_N=<symbols of each throughput job> (default 16384), +VERBOSE.
// Inputs change on the falling edge. Transfers are sampled 1 time unit later.
module tb_pans_dec_stream #(
  parameter int N_LANES    = 8,
  parameter int DATA_BYTES = 4,
  parameter int FIFO_WORDS  = 8,
  parameter int OUT_DEPTH   = 7,
  parameter int COUNT_WIDTH = 32,  // the unit's count width
  parameter bit CLOCK_GATE  = 1    // the unit's clock gates (0: enable flops)
);

  import pans_pkg::*;

  import "DPI-C" function longint pans_ref_config_id();
  import "DPI-C" function void pans_ref_b_widths(output int b_frac, output int b_width, output int b_k);
  import "DPI-C" function longint pans_ref_b_of_sigma(input real sigma);
  import "DPI-C" function void pans_ref_scale_of_b(input longint b, output int exp_field, output int man_field);
  import "DPI-C" function void pans_ref_encode(input int exp_field, input int man_field, input int symbol,
                                               output int start, output int freq, output int escape);
  import "DPI-C" function int pans_stream_job_new(input int seed, input int n_lanes, input longint n, input int kind,
                                                  input int delay);
  import "DPI-C" function int pans_stream_job_begin(input int n_lanes, input longint n, input int kind,
                                                    input int delay);
  import "DPI-C" function void pans_stream_job_set(input int job, input longint i, input int symbol, input longint b);
  import "DPI-C" function int pans_stream_job_build(input int job);
  import "DPI-C" function int pans_stream_job_bound(input int seed, input int n_lanes, output longint index0,
                                                    output longint index1);
  import "DPI-C" function void pans_stream_job_corrupt(input int job, input int how, input longint pos,
                                                       input int value);
  import "DPI-C" function void pans_stream_job_free(input int job);
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
  import "DPI-C" function void pans_stream_widths(output int word_width, output int state_width,
                                                  output int state_bytes);

  localparam int  LANE_WIDTH  = (N_LANES > 1) ? $clog2(N_LANES) : 1;
  localparam int  POS_WIDTH   = unsigned_width(((longint'(1) <<< COUNT_WIDTH) - 2) / longint'(N_LANES));
  localparam int  HDR_BYTES   = N_LANES * STATE_BYTES;
  localparam int  WORD_BYTES  = WORD_WIDTH / 8;
  localparam int  WATCHDOG    = 10000;
  localparam longint NL       = longint'(N_LANES);     // the same constants as 64-bit operands
  localparam longint DBL      = longint'(DATA_BYTES);
  localparam longint WBL      = longint'(WORD_BYTES);
  localparam int  RAW_BYTE    = (SYMBOL_WIDTH - 1) / 8;         // the byte of a word holding bit S - 1
  localparam int  TOP_BYTE    = STATE_WIDTH / 8;                // the byte of a state holding bit STATE_WIDTH
  localparam int  LAST_STATE  = (N_LANES - 1) * STATE_BYTES;    // the last header state's first byte
  // Two words in one escape need W + log2 E < P + S (STREAM.md §1.4).
  localparam bit  TWO_WORDS_POSSIBLE = WORD_WIDTH + $clog2(ESCAPE_SLOTS) < PRECISION + SYMBOL_WIDTH;
  localparam int  KIND_CONST = 0, KIND_EMA = 1, KIND_LIST = 2, KIND_INT = 3;
  // The least S-bit input: SYMBOL_MIN, or below a symmetric alphabet -2^(SW-1), which the encoder escapes.
  localparam int SYMBOL_IN_MIN = -(2 ** (SYMBOL_WIDTH - 1));
  localparam int  ERR_WORDS = 1;  // the step's only error bit (contract version 5)
  // b = 0 has the narrowest code. The table's last index holds b >= B_TOP_MIN.
  localparam longint B_TOP_MIN = ((longint'(1) <<< (B_K + 1)) - 1) <<< (B_WIDTH - 1 - B_K);

  // Stall profiles: percent of cycles each channel stalls, and the consumer's latency. 3 and 4 measure throughput.
  localparam int NPROF = 5;
  localparam int PROF_DATA  [NPROF] = '{0, 10, 50, 0, 0};
  localparam int PROF_OUT   [NPROF] = '{0, 10, 60, 0, 0};
  localparam int PROF_SCALE [NPROF] = '{0, 10, 40, 0, 0};
  localparam int PROF_JOB   [NPROF] = '{0, 20, 60, 0, 0};
  localparam int PROF_LAT_MIN [NPROF] = '{1, 1, 1, 1, 2};
  localparam int PROF_LAT_MAX [NPROF] = '{1, 3, 6, 1, 2};

  function automatic string prof_name(input int p);
    case (p)
      0: return "none";
      1: return "light";
      2: return "heavy";
      3: return "(a)";
      default: return "(b)";
    endcase
  endfunction

  // ---- Clock, reset, the unit
  logic clk   = 1'b0;
  logic rst_n = 1'b0;
  always #5 clk = ~clk;

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
  logic                           sym_out_ready  = 1'b0;
  logic [LANE_WIDTH-1:0]          sym_out_lane;
  logic [POS_WIDTH-1:0]           sym_out_t;
  logic signed [SYMBOL_WIDTH-1:0] sym_out_symbol;
  logic                           sym_out_escape;
  logic                           done_out_valid;
  logic                           done_out_ready = 1'b0;
  logic                           done_out_ok;
  logic [COUNT_WIDTH-1:0]         done_out_n_escapes;
  logic [COUNT_WIDTH-1:0]         done_out_words_consumed;
  logic                           done_out_err_words;
  logic                           done_out_err_header;

  pans_dec_stream #(
    .N_LANES     (N_LANES),
    .DATA_BYTES  (DATA_BYTES),
    .FIFO_WORDS  (FIFO_WORDS),
    .OUT_DEPTH   (OUT_DEPTH),
    .COUNT_WIDTH (COUNT_WIDTH),
    .CLOCK_GATE  (CLOCK_GATE)
  ) dut (
    .clk                     (clk),
    .rst_n                   (rst_n),
    .job_in_valid            (job_in_valid),
    .job_in_ready            (job_in_ready),
    .job_in_n_sym            (job_in_n_sym),
    .job_in_n_bytes          (job_in_n_bytes),
    .data_in_valid           (data_in_valid),
    .data_in_ready           (data_in_ready),
    .data_in_data            (data_in_data),
    .scale_in_valid          (scale_in_valid),
    .scale_in_ready          (scale_in_ready),
    .scale_in_lane           (scale_in_lane),
    .scale_in_b              (scale_in_b),
    .sym_out_valid           (sym_out_valid),
    .sym_out_ready           (sym_out_ready),
    .sym_out_lane            (sym_out_lane),
    .sym_out_t               (sym_out_t),
    .sym_out_symbol          (sym_out_symbol),
    .sym_out_escape          (sym_out_escape),
    .done_out_valid          (done_out_valid),
    .done_out_ready          (done_out_ready),
    .done_out_ok             (done_out_ok),
    .done_out_n_escapes      (done_out_n_escapes),
    .done_out_words_consumed (done_out_words_consumed),
    .done_out_err_words      (done_out_err_words),
    .done_out_err_header     (done_out_err_header)
  );

  // ---- The scale lookup on its own, against the model
  logic [B_WIDTH-1:0] lut_b = '0;
  scale_t             lut_scale;

  pans_scale_of_b u_lut (.b(lut_b), .scale(lut_scale));

  // ---- Checking
  longint checks = 0;
  longint errors = 0;

  function automatic bit check(input bit ok, input string what);
    checks++;
    if (!ok) begin
      errors++;
      if (errors <= 20) $display("ERROR %s", what);
    end
    return ok;
  endfunction


  // ---- Random numbers: splitmix64, the same sequence on every simulator
  typedef longint unsigned u64_t;
  u64_t rng_s;

  function automatic u64_t rng_next();
    u64_t z;
    rng_s = rng_s + 64'h9e3779b97f4a7c15;
    z = rng_s;
    z = (z ^ (z >> 30)) * 64'hbf58476d1ce4e5b9;
    z = (z ^ (z >> 27)) * 64'h94d049bb133111eb;
    return z ^ (z >> 31);
  endfunction

  function automatic longint rnd_below(input longint n);  // [0, n), n >= 1
    return longint'(rng_next() % u64_t'(n));
  endfunction

  function automatic bit chance(input int pct);
    return rnd_below(100) < longint'(pct);
  endfunction

  // pans_scale_of_b against the model at every b below 2^16, and above at each index's ends and random b.
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
      void'(check(int'(lut_scale.exp) == e && int'(lut_scale.man) == m,
                  $sformatf("pans_scale_of_b: b %0d gives {%0d, %0d}, the model {%0d, %0d}", bs[q], lut_scale.exp,
                            lut_scale.man, e, m)));
    end
    $display("scale lookup: pans_scale_of_b equals the model's scale_of_b at %0d values of b (B_WIDTH %0d, B_K %0d)",
             bs.size(), B_WIDTH, B_K);
  endtask

  // ---- The model's trace of one step
  typedef struct {
    int     lane;
    longint t;
    int     slot, start, freq, escape, raw, symbol;
    longint b;
    int     scale_exp, scale_man, words;
    longint state;
    int     errors;
  } step_t;

  function automatic step_t get_step(input int h, input longint i);
    step_t s;
    pans_stream_job_step(h, i, s.lane, s.t, s.slot, s.start, s.freq, s.escape, s.raw, s.symbol, s.b, s.scale_exp,
                         s.scale_man, s.words, s.state, s.errors);
    return s;
  endfunction

  // ---- Jobs: the model's handle and what the drivers need
  int     jb_h[$];
  longint jb_n[$];
  longint jb_nb[$];
  longint jb_beats[$];
  int     jb_prof[$];
  string  jb_label[$];

  function automatic void add_job(input int h, input int prof, input string label);
    if (!check(h >= 0, $sformatf("job %s: the model refused it", label))) return;
    jb_h.push_back(h);
    jb_n.push_back(pans_stream_job_n(h));
    jb_nb.push_back(pans_stream_job_n_bytes(h));
    jb_beats.push_back((pans_stream_job_n_bytes(h) + DBL - 1) / DBL);
    jb_prof.push_back(prof);
    jb_label.push_back(label);
  endfunction

  // A directed LIST job from the callers' arrays of symbols and scale inputs b.
  function automatic int directed(input longint n, input int sym[], input longint sb[]);
    int h;
    h = pans_stream_job_begin(N_LANES, n, KIND_LIST, 0);
    for (longint i = 0; i < n; i++) pans_stream_job_set(h, i, sym[i], sb[i]);
    void'(check(pans_stream_job_build(h) == 0, "directed job: build failed"));
    return h;
  endfunction

  function automatic int nonzero_symbol();
    int s;
    s = 0;
    while (s == 0) s = int'(longint'(SYMBOL_IN_MIN) + rnd_below(longint'(SYMBOL_MAX) - longint'(SYMBOL_IN_MIN) + 1));
    return s;
  endfunction

  // An escape storm at b = 0 on `lane`, or on every lane for lane < 0. The other symbols are 0.
  function automatic int storm(input int lane, input longint n);
    int     sym[];
    longint sb[];
    sym = new[int'(n)];
    sb  = new[int'(n)];
    for (longint i = 0; i < n; i++) begin
      sym[i] = (lane < 0 || int'(i % NL) == lane) ? nonzero_symbol() : 0;
      sb[i]  = 0;
    end
    return directed(n, sym, sb);
  endfunction

  // (b, symbol) pairs with f = 1, from the core's model at the code of a few sigmas' b.
  longint f1_b[$];
  int     f1_sym[$];

  function automatic void find_unit_freq();
    real    sigmas[4];
    longint b;
    int     e, m, st, fr, esc;
    sigmas = '{0.7, 3.0, 11.0, 45.0};
    foreach (sigmas[k]) begin
      if (sigmas[k] > real'(SYMBOL_MAX) / 2.0) continue;
      b = pans_ref_b_of_sigma(sigmas[k]);
      pans_ref_scale_of_b(b, e, m);
      for (int s = SYMBOL_MIN; s <= SYMBOL_MAX; s++) begin
        pans_ref_encode(e, m, s, st, fr, esc);
        if (esc == 0 && fr == 1) begin
          f1_b.push_back(b);
          f1_sym.push_back(s);
        end
      end
    end
  endfunction

  // Corruption `how` of job h: 0 a flipped byte anywhere, 1 a truncation anywhere, 2 an appended byte
  function automatic void corrupt(input int h, input int how);
    longint size;
    size = pans_stream_job_n_bytes(h);
    case (how)
      0: pans_stream_job_corrupt(h, 0, rnd_below(size), 1 + int'(rnd_below(255)));
      1: pans_stream_job_corrupt(h, 1, rnd_below(size + 1), 0);
      default: pans_stream_job_corrupt(h, 2, 0, int'(rnd_below(256)));
    endcase
  endfunction

  // ---- The job list of one seed
  function automatic void make_jobs(input int seed, input longint tp_n);
    longint ns[6];
    int     k, h;
    longint n;
    ns = '{0, 1, NL - 1, NL, NL + 1, 0};
    k  = 0;
    for (int prof = 0; prof < 3; prof++) begin
      ns[5] = 2 + rnd_below(4000);
      foreach (ns[q]) begin
        for (int kind = 0; kind <= KIND_LIST; kind++) begin
          for (int delay = 0; delay <= (kind == KIND_EMA ? 2 : 0); delay++) begin
            h = pans_stream_job_new(seed * 1000 + k++, N_LANES, ns[q], kind, delay);
            add_job(h, prof, $sformatf("random n %0d kind %0d delay %0d", ns[q], kind, delay));
          end
        end
      end
      // Corrupt: a flipped byte anywhere, a truncation anywhere, an appended byte
      for (int kind = 0; kind <= KIND_LIST; kind++) begin
        for (int how = 0; how < 3; how++) begin
          n = longint'(N_LANES) + 1 + rnd_below(1500);
          h = pans_stream_job_new(seed * 1000 + k++, N_LANES, n, kind, kind == KIND_EMA ? int'(rnd_below(3)) : 0);
          corrupt(h, how);
          add_job(h, prof, $sformatf("corrupt %0d n %0d kind %0d", how, n, kind));
        end
      end
    end
    // Directed
    add_job(storm(-1, 5 * N_LANES + 3), 0, "escape storm, every lane");
    add_job(storm(-1, 9 * N_LANES + 1), 2, "escape storm, every lane, heavy stalls");
    add_job(storm(N_LANES > 1 ? 1 : 0, 6 * N_LANES + 2), 1, "escape storm, one lane");
    begin  // f = 1 only
      int     sym[];
      longint sb[];
      longint r;
      n = 6 * N_LANES + 5;
      sym = new[int'(n)]; sb = new[int'(n)];
      for (longint i = 0; i < n; i++) begin
        r = rnd_below(longint'(f1_sym.size()));
        sym[i] = f1_sym[r]; sb[i] = f1_b[r];
      end
      if (f1_sym.size() > 0) add_job(directed(n, sym, sb), 1, "f = 1 only");
    end
    begin  // f = M' only: symbol 0 at b = 0 and 1 (table index 0: the narrowest code)
      int     sym[];
      longint sb[];
      n = 7 * N_LANES + 1;
      sym = new[int'(n)]; sb = new[int'(n)];
      for (longint i = 0; i < n; i++) begin
        sym[i] = 0; sb[i] = i % 2;
      end
      add_job(directed(n, sym, sb), 0, "f = M' only");
    end
    if (N_LANES >= 3) begin  // lanes without symbols
      h = pans_stream_job_new(seed * 1000 + k++, N_LANES, NL - 2, KIND_LIST, 0);
      add_job(h, 2, "lanes without symbols");
    end
    // An escape's raw value flipped. Only that symbol changes, and ok = 1 (STREAM.md §1.6).
    if (TWO_WORDS_POSSIBLE) begin
      longint words_before;
      step_t  st;
      bit     done;
      h = storm(-1, 8 * N_LANES + 3);
      words_before = 0;
      done = 0;
      for (longint i = 0; i < pans_stream_job_n(h) && !done; i++) begin
        st = get_step(h, i);
        if (st.escape != 0 && st.words == 2) begin
          pans_stream_job_corrupt(h, 0, longint'(HDR_BYTES) + words_before * WBL + longint'(RAW_BYTE),
                                  1 << ((SYMBOL_WIDTH - 1) % 8));
          done = 1;
        end
        words_before += longint'(st.words);
      end
      void'(check(done, "raw value flipped: no escape reads two words in the storm job"));
      add_job(h, 1, "raw value flipped");
    end
    // A header state out of range, above 2^STATE_WIDTH where it can be, else below L.
    h = pans_stream_job_new(seed * 1000 + k++, N_LANES, 4 * N_LANES, KIND_CONST, 0);
    if (8 * STATE_BYTES > STATE_WIDTH) pans_stream_job_corrupt(h, 0, longint'(TOP_BYTE), 1 << (STATE_WIDTH % 8));
    else                               pans_stream_job_corrupt(h, 1, longint'(LAST_STATE), 0);
    add_job(h, 0, "header state out of range");
    h = pans_stream_job_new(seed * 1000 + k++, N_LANES, 3 * N_LANES + 1, KIND_EMA, 0);
    pans_stream_job_corrupt(h, 1, longint'(HDR_BYTES) - 1 - rnd_below(longint'(HDR_BYTES)), 0);
    add_job(h, 1, "header cut short");
    h = pans_stream_job_new(seed * 1000 + k++, N_LANES, 2 * N_LANES + 3, KIND_LIST, 0);
    pans_stream_job_corrupt(h, 1, 0, 0);
    add_job(h, 2, "empty array");
    h = pans_stream_job_new(seed * 1000 + k++, N_LANES, 5 * N_LANES + 2, KIND_EMA, 1);
    for (int b = 0; b < WORD_BYTES; b++) pans_stream_job_corrupt(h, 2, 0, int'(rnd_below(256)));
    add_job(h, 0, "a whole word appended");
    // Throughput: long EMA jobs (delay 0), no stalls, the consumer 1 and 2 cycles behind the symbol
    add_job(pans_stream_job_new(seed * 1000 + k++, N_LANES, tp_n, KIND_EMA, 0), 3, "throughput (a)");
    add_job(pans_stream_job_new(seed * 1000 + k++, N_LANES, tp_n, KIND_EMA, 0), 4, "throughput (b)");
    // The example integer scale estimator's jobs (STREAM.md §1.7). They come last, so every job above keeps its seed.
    for (int prof = 0; prof < 3; prof++) begin
      ns[5] = 2 + rnd_below(4000);
      foreach (ns[q]) begin
        for (int delay = 0; delay <= 2; delay++) begin
          h = pans_stream_job_new(seed * 1000 + k++, N_LANES, ns[q], KIND_INT, delay);
          add_job(h, prof, $sformatf("random n %0d kind %0d delay %0d", ns[q], KIND_INT, delay));
        end
      end
      for (int how = 0; how < 3; how++) begin
        n = longint'(N_LANES) + 1 + rnd_below(1500);
        h = pans_stream_job_new(seed * 1000 + k++, N_LANES, n, KIND_INT, int'(rnd_below(3)));
        corrupt(h, how);
        add_job(h, prof, $sformatf("corrupt %0d n %0d kind %0d", how, n, KIND_INT));
      end
    end
    // The raw push's bound job (STREAM.md §1.9). It comes after every other job, so they keep their seeds.
    begin
      longint i0, i1;
      h = pans_stream_job_bound(seed * 1000 + k++, N_LANES, i0, i1);
      add_job(h, 1, "raw push bound");
    end
    // More words appended than the word FIFO holds, so the drain drops beats for certain. Last, to keep the seeds.
    h = pans_stream_job_new(seed * 1000 + k++, N_LANES, 4 * NL + 1, KIND_EMA, 1);
    for (int b = 0; b < (FIFO_WORDS + 2) * WORD_BYTES + DATA_BYTES; b++)
      pans_stream_job_corrupt(h, 2, 0, int'(rnd_below(256)));
    add_job(h, 0, "more words appended than the word FIFO holds");
  endfunction

  // ---- Coverage
  longint cov_symbols = 0, cov_escapes = 0, cov_two_words = 0, cov_f_one = 0, cov_f_max = 0, cov_b_zero = 0;
  longint cov_b_top = 0;
  longint cov_empty_lanes = 0, cov_dry = 0, cov_drained = 0;
  longint cov_err_words = 0, cov_err_header = 0, cov_end_only = 0, cov_ok = 0;
  longint cov_bub_scale = 0, cov_bub_words = 0, cov_bub_credit = 0, cov_bub_busy = 0;
  longint cov_outside = 0;  // escapes of the input below a symmetric alphabet

  // ---- Measurements of the throughput jobs
  longint tp_start [NPROF];
  longint tp_first [NPROF];
  longint tp_last  [NPROF];
  longint tp_done  [NPROF];
  longint tp_syms  [NPROF];
  longint tp_bub_scale [NPROF];  // issue bubbles while the job runs (after its header), by cause
  longint tp_bub_words [NPROF];
  longint tp_bub_credit [NPROF];

  // ---- The run
  initial begin
    int     seed, ww, sw, sb, bf, bw, bk;
    longint tp_n;
    // drivers and monitors
    int     jd, dd, sf, od, dn, cur;
    longint db, sfi, oi;
    bit     job_fired, data_fired, scale_fired;
    // the consumer: scales fetched from the model in decode order, each presentable from cycle sq_at
    int     sq_job[$], sq_lane[$];
    longint sq_b[$];
    longint sq_at[$];
    longint wb_i, wb_a, wb_b, beats_taken, scales_taken;
    longint cyc, last_progress;
    bit     progress;
    step_t  st;
    int     prof, span;

    seed = 1;
    tp_n = 16384;
    void'($value$plusargs("SEED=%d", seed));
    void'($value$plusargs("TP_N=%d", tp_n));
    rng_s = u64_t'(seed) * 64'h2545f4914f6cdd1d + 64'h1234567;

    // The C++ model must be the build this package was generated from.
    if (64'(pans_ref_config_id()) != CONFIG_ID)
      $fatal(1, "config mismatch: RTL package CONFIG_ID %016h, C++ reference %016h", CONFIG_ID,
             64'(pans_ref_config_id()));
    pans_stream_widths(ww, sw, sb);
    if (ww != WORD_WIDTH || sw != STATE_WIDTH || sb != STATE_BYTES)
      $fatal(1, "config mismatch: RTL package widths W %0d, state %0d bits in %0d bytes; C++ model %0d, %0d, %0d",
             WORD_WIDTH, STATE_WIDTH, STATE_BYTES, ww, sw, sb);
    pans_ref_b_widths(bf, bw, bk);
    if (bf != B_FRAC || bw != B_WIDTH || bk != B_K)
      $fatal(1, "config mismatch: RTL package B_FRAC %0d B_WIDTH %0d B_K %0d; C++ model %0d, %0d, %0d", B_FRAC,
             B_WIDTH, B_K, bf, bw, bk);

    check_lookup();

    find_unit_freq();
    make_jobs(seed, tp_n);
    $display("pans_dec_stream: PRECISION=%0d SYMBOL_WIDTH=%0d ESCAPE_SLOTS=%0d W=%0d CONFIG_ID=%016h; N_LANES=%0d DATA_BYTES=%0d FIFO_WORDS=%0d OUT_DEPTH=%0d; seed %0d, %0d jobs",
             PRECISION, SYMBOL_WIDTH, ESCAPE_SLOTS, WORD_WIDTH, CONFIG_ID, N_LANES, DATA_BYTES, FIFO_WORDS,
             OUT_DEPTH, seed, jb_h.size());

    jd = 0; dd = 0; sf = 0; od = 0; dn = 0; cur = -1;
    db = 0; sfi = 0; oi = 0;
    job_fired = 0; data_fired = 0; scale_fired = 0;
    wb_i = 0; wb_a = 0; wb_b = 0; beats_taken = 0; scales_taken = 0;
    cyc = 0; last_progress = 0;
    foreach (tp_start[p]) begin
      tp_start[p] = -1; tp_first[p] = -1; tp_last[p] = -1; tp_done[p] = -1; tp_syms[p] = 0;
      tp_bub_scale[p] = 0; tp_bub_words[p] = 0; tp_bub_credit[p] = 0;
    end

    repeat (3) @(negedge clk);
    rst_n = 1'b1;

    while (dn < jb_h.size()) begin
      // ---- Falling edge: drive
      @(negedge clk);
      cyc++;
      // job_in
      if (job_fired) begin
        job_in_valid = 1'b0;
        job_fired    = 1'b0;
      end
      if (!job_in_valid && jd < jb_h.size() && !chance(PROF_JOB[jb_prof[jd]])) begin
        job_in_valid   = 1'b1;
        job_in_n_sym   = COUNT_WIDTH'(jb_n[jd]);
        job_in_n_bytes = COUNT_WIDTH'(jb_nb[jd]);
      end
      // data_in: the beats of every job in order, bytes beyond the array random
      if (data_fired) begin
        data_in_valid = 1'b0;
        data_fired    = 1'b0;
      end
      while (dd < jb_h.size() && db >= jb_beats[dd]) begin
        dd++;
        db = 0;
      end
      if (!data_in_valid && dd < jb_h.size() && !chance(PROF_DATA[jb_prof[dd]])) begin
        data_in_valid = 1'b1;
        for (int b = 0; b < DATA_BYTES; b++) begin
          longint at;
          at = db * DBL + longint'(b);
          data_in_data[8*b +: 8] = (at < jb_nb[dd]) ? 8'(pans_stream_job_byte(jb_h[dd], at)) : 8'(rng_next());
        end
      end
      // scale_in: every computable scale is fetched now and goes out after the consumer's latency, in decode order.
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
          void'(check(got == 0, $sformatf("job %0d: pans_stream_scale(%0d, %0d) = %0d", sf, sfi % NL, sfi / NL,
                                          got)));
          break;
        end
        prof = jb_prof[sf];
        span = PROF_LAT_MAX[prof] - PROF_LAT_MIN[prof] + 1;
        sq_job.push_back(sf);
        sq_lane.push_back(int'(sfi % NL));
        sq_b.push_back(b);
        sq_at.push_back(cyc + longint'(PROF_LAT_MIN[prof]) - 1 + rnd_below(longint'(span)));
        sfi++;
      end
      if (!scale_in_valid && sq_job.size() > 0 && cyc >= sq_at[0] && !chance(PROF_SCALE[jb_prof[sq_job[0]]])) begin
        scale_in_valid = 1'b1;
        scale_in_lane  = LANE_WIDTH'(sq_lane[0]);
        scale_in_b     = B_WIDTH'(sq_b[0]);
      end
      // sym_out, done_out
      while (od < jb_h.size() && oi >= jb_n[od]) begin
        od++;
        oi = 0;
      end
      sym_out_ready  = !(od < jb_h.size() && chance(PROF_OUT[jb_prof[od]]));
      done_out_ready = !(dn < jb_h.size() && chance(PROF_OUT[jb_prof[dn]]));

      // ---- 1 time unit later: the transfers of the coming rising edge
      #1;
      progress = 1'b0;

      // White box: the steps of the job the unit works on, against the model's trace
      if (cur >= 0) begin
        if (dut.s0_issue) begin
          st = get_step(jb_h[cur], wb_i);
          // the unit carries no t, so (lane, t) is checked at sym_out
          void'(check(int'(dut.rr_lane_q) == st.lane
                      && int'(dut.s0_scale.exp) == st.scale_exp && int'(dut.s0_scale.man) == st.scale_man
                      && int'(dut.s0_slot) == st.slot,
                      $sformatf("job %0d (%s) step %0d issue: lane %0d scale {%0d, %0d} slot %0d, model lane %0d t %0d scale {%0d, %0d} slot %0d",
                                cur, jb_label[cur], wb_i, dut.rr_lane_q, dut.s0_scale.exp,
                                dut.s0_scale.man, dut.s0_slot, st.lane, st.t, st.scale_exp, st.scale_man, st.slot)));
          wb_i++;
        end
        if (dut.q4_valid) begin
          st = get_step(jb_h[cur], wb_a);
          void'(check(int'(dut.c_start) == st.start && int'(dut.c_freq) == st.freq
                      && int'(dut.c_escape) == st.escape && (st.escape != 0 || int'(dut.c_symbol) == st.symbol),
                      $sformatf("job %0d (%s) step %0d core: (%0d, %0d, %0d, esc %0d), model (%0d, %0d, %0d, esc %0d)",
                                cur, jb_label[cur], wb_a, dut.c_symbol, dut.c_start, dut.c_freq, dut.c_escape,
                                st.symbol, st.start, st.freq, st.escape)));
          wb_a++;
        end
        if (dut.q5_valid) begin
          int errs;
          st   = get_step(jb_h[cur], wb_b);
          errs = dut.s6_over ? ERR_WORDS : 0;
          void'(check(longint'(dut.s6_x) == st.state && int'(dut.s6_pops) == st.words && errs == st.errors
                      && (st.escape == 0 || int'(dut.s6_v) == st.raw),
                      $sformatf("job %0d (%s) step %0d pop: state %0h words %0d errors %0d v %0d, model %0h, %0d, %0d, %0d",
                                cur, jb_label[cur], wb_b, dut.s6_x, dut.s6_pops, errs, dut.s6_v, st.state, st.words,
                                st.errors, st.raw)));
          cov_symbols++;
          cov_escapes   += longint'(st.escape != 0);
          cov_two_words += longint'(st.words == 2);
          cov_f_one     += longint'(st.escape == 0 && st.freq == 1);
          cov_f_max     += longint'(st.escape == 0 && st.freq == CODED_SLOTS);
          cov_b_zero    += longint'(st.b == 0);
          cov_b_top     += longint'(st.b >= B_TOP_MIN);
          cov_outside   += longint'(st.escape != 0 && st.symbol < SYMBOL_MIN);
          cov_dry       += longint'(dut.s6_over);
          wb_b++;
        end
        if (int'(dut.phase_q) == 2 /* ST_RUN */ && dut.sym_left_q != '0 && !dut.s0_issue) begin
          prof = jb_prof[cur];
          if (!dut.scale_present_q[dut.rr_lane_q]) begin
            cov_bub_scale++;
            tp_bub_scale[prof]++;
          end else if (dut.lane_busy_q[dut.rr_lane_q]) begin
            cov_bub_busy++;
          end else if (!dut.s0_words_ok) begin
            cov_bub_words++;
            tp_bub_words[prof]++;
          end else if (dut.credit_q == '0) begin
            cov_bub_credit++;
            tp_bub_credit[prof]++;
          end
        end
        cov_drained += longint'(dut.in_drain && dut.in_beat);
      end
      // data_in is never ready for a beat beyond the job's array
      if (data_in_ready)
        void'(check(cur >= 0 && beats_taken < jb_beats[cur],
                    $sformatf("cycle %0d: data_in ready beyond the array of job %0d", cyc, cur)));

      // job_in
      if (job_in_valid && job_in_ready) begin
        job_fired   = 1'b1;
        progress    = 1'b1;
        cur         = jd;
        wb_i        = 0;
        wb_a        = 0;
        wb_b        = 0;
        beats_taken = 0;
        scales_taken = 0;
        if (jb_prof[jd] >= 3) tp_start[jb_prof[jd]] = cyc;
        if (jb_n[jd] < NL) cov_empty_lanes += NL - jb_n[jd];
        jd++;
      end
      // data_in
      if (data_in_valid && data_in_ready) begin
        data_fired = 1'b1;
        progress   = 1'b1;
        void'(check(dd == cur, $sformatf("job %0d took a beat of job %0d", cur, dd)));
        beats_taken++;
        db++;
      end
      // scale_in
      if (scale_in_valid && scale_in_ready) begin
        scale_fired = 1'b1;
        progress    = 1'b1;
        void'(check(sq_job[0] == cur, $sformatf("job %0d took a scale of job %0d", cur, sq_job[0])));
        scales_taken++;
      end
      // sym_out: field by field against the model, then the consumer observes it
      if (sym_out_valid && sym_out_ready) begin
        progress = 1'b1;
        if (check(od == cur && od < jb_h.size(), $sformatf("cycle %0d: a symbol beyond job %0d", cyc, cur))) begin
          st = get_step(jb_h[od], oi);
          void'(check(int'(sym_out_lane) == st.lane && longint'(sym_out_t) == st.t
                      && int'(sym_out_symbol) == st.symbol && int'(sym_out_escape) == st.escape,
                      $sformatf("job %0d (%s) symbol %0d: lane %0d t %0d symbol %0d escape %0d, model lane %0d t %0d symbol %0d escape %0d",
                                od, jb_label[od], oi, sym_out_lane, sym_out_t, sym_out_symbol, sym_out_escape,
                                st.lane, st.t, st.symbol, st.escape)));
          void'(check(pans_stream_observe(jb_h[od], int'(sym_out_lane), longint'(sym_out_t), int'(sym_out_symbol)) == 0,
                      $sformatf("job %0d symbol %0d: the consumer refused (lane %0d, t %0d)", od, oi, sym_out_lane,
                                sym_out_t)));
          prof = jb_prof[od];
          if (prof >= 3) begin
            if (tp_first[prof] < 0) tp_first[prof] = cyc;
            tp_last[prof] = cyc;
            tp_syms[prof]++;
          end
          oi++;
        end
      end
      // done_out against the model's end check
      if (done_out_valid && done_out_ready) begin
        progress = 1'b1;
        if (check(dn == cur, $sformatf("cycle %0d: done for job %0d while job %0d runs", cyc, dn, cur))) begin
          int     ok, e_words, e_head;
          longint n_esc, consumed;
          pans_stream_job_done(jb_h[dn], ok, n_esc, consumed, e_words, e_head);
          void'(check(int'(done_out_ok) == ok && longint'(done_out_n_escapes) == n_esc
                      && longint'(done_out_words_consumed) == consumed && int'(done_out_err_words) == e_words
                      && int'(done_out_err_header) == e_head,
                      $sformatf("job %0d (%s) done: ok %0d escapes %0d words %0d errors w%0d h%0d, model ok %0d escapes %0d words %0d errors w%0d h%0d",
                                dn, jb_label[dn], done_out_ok, done_out_n_escapes, done_out_words_consumed,
                                done_out_err_words, done_out_err_header, ok, n_esc, consumed, e_words, e_head)));
          void'(check(od > dn && scales_taken == jb_n[dn] && beats_taken == jb_beats[dn] && wb_b == jb_n[dn],
                      $sformatf("job %0d (%s) done with %0d of %0d beats and %0d scales taken, symbols out up to job %0d",
                                dn, jb_label[dn], beats_taken, jb_beats[dn], scales_taken, od)));
          cov_ok           += longint'(ok != 0);
          cov_err_words    += longint'(e_words != 0);
          cov_err_header   += longint'(e_head != 0);
          cov_end_only     += longint'(ok == 0 && e_words == 0 && e_head == 0);
          if (jb_prof[dn] >= 3) tp_done[jb_prof[dn]] = cyc;
          if ($test$plusargs("VERBOSE"))
            $display("job %0d (%s, %s): n %0d, %0d bytes, ok %0d, cycle %0d", dn, jb_label[dn],
                     prof_name(jb_prof[dn]), jb_n[dn], jb_nb[dn], ok, cyc);
          pans_stream_job_free(jb_h[dn]);
        end
        dn++;
      end

      if (progress) last_progress = cyc;
      if (cyc - last_progress > longint'(WATCHDOG)) begin
        $display("ERROR watchdog: no transfer for %0d cycles; job %0d (%s), phase %0d, symbols left %0d, scales present %b, bytes left %0d, FIFO %0d, reserved %0d, credit %0d, beats taken %0d of %0d",
                 WATCHDOG, cur, cur >= 0 ? jb_label[cur] : "none", dut.phase_q, dut.sym_left_q,
                 dut.scale_present_q, dut.in_bytes_left_q, dut.wf_count_q, dut.reserved_q, dut.credit_q,
                 beats_taken, cur >= 0 ? jb_beats[cur] : 0);
        $fatal(1, "FAILED: watchdog");
      end
    end

    // ---- Report
    $display("coverage: %0d symbols, %0d escapes, %0d two-word steps, %0d with f = 1, %0d with f = M', %0d at b = 0, %0d at the table's last index, %0d empty lanes, %0d reads past the array, %0d beats drained",
             cov_symbols, cov_escapes, cov_two_words, cov_f_one, cov_f_max, cov_b_zero, cov_b_top, cov_empty_lanes,
             cov_dry, cov_drained);
    $display("coverage: jobs ok %0d; err_words %0d, err_header %0d, end check only %0d; issue bubbles: scale %0d, words %0d, credit %0d, busy lane %0d",
             cov_ok, cov_err_words, cov_err_header, cov_end_only, cov_bub_scale, cov_bub_words, cov_bub_credit,
             cov_bub_busy);
    void'(check(cov_escapes > 0 && cov_f_one > 0 && cov_f_max > 0 && cov_b_zero > 0 && cov_b_top > 0
                && cov_empty_lanes > 0 && (cov_two_words > 0 || !TWO_WORDS_POSSIBLE) && cov_dry > 0 && cov_drained > 0,
                "coverage: escapes, f = 1, f = M', b = 0, the table's last index, empty lanes, two-word steps (W + log2 E < P + S), reads past the array and drained beats must all occur"));
    void'(check(cov_ok > 0 && cov_err_words > 0 && cov_err_header > 0 && cov_end_only > 0,
                "coverage: clean jobs, every error flag (words, header) and an end-check-only failure must occur"));
    if (SYMBOL_IN_MIN < SYMBOL_MIN) begin
      $display("coverage: %0d escapes of the input %0d outside the symmetric alphabet", cov_outside, SYMBOL_IN_MIN);
      void'(check(cov_outside > 0, "coverage: an escape of an input outside the symmetric alphabet must occur"));
    end
    // Issue bubbles for each cause. Below 6 lanes a busy lane masks the other causes.
    if (N_LANES >= 6)
      void'(check(cov_bub_scale > 0 && cov_bub_words > 0 && cov_bub_credit > 0,
                  "coverage: issue bubbles for want of a scale, of words and of credit must occur"));
    else if (OUT_DEPTH > N_LANES)
      void'(check(cov_bub_busy > 0, "coverage: issue bubbles for a busy lane must occur (N_LANES < 6)"));
    for (int p = 3; p < NPROF; p++) begin
      if (tp_done[p] < 0 || tp_syms[p] < 2) continue;
      $display("throughput %s (EMA delay 0, no stalls, scale_in %0d cycle(s) after the symbol's transfer): %0d symbols, %.4f symbols/cycle steady (first to last symbol), %.4f with header and drain (job_in to done_out)",
               prof_name(p), PROF_LAT_MIN[p], tp_syms[p],
               real'(tp_syms[p] - 1) / real'(tp_last[p] - tp_first[p]),
               real'(tp_syms[p]) / real'(tp_done[p] - tp_start[p] + 1));
      $display("  issue bubbles in it: %0d for want of a scale, %0d of FIFO words, %0d of output credit",
               tp_bub_scale[p], tp_bub_words[p], tp_bub_credit[p]);
    end
    if (errors != 0) $fatal(1, "FAILED: %0d errors in %0d checks", errors, checks);
    $display("PASSED: %0d checks, 0 errors (%0d jobs, %0d cycles)", checks, jb_h.size(), cyc);
    $finish;
  end

endmodule
