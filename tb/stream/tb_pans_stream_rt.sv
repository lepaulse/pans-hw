// SPDX-License-Identifier: Apache-2.0
// tb_pans_stream_rt: the RTL round trip of the stream units (make sim-stream-rt-<cfg>).
// pans_enc_stream encodes each job, the testbench assembles the byte array, and pans_dec_stream decodes it back.
// Specification: STREAM.md §3. The model only makes the stimulus and is the decoder's consumer.
// A job passes iff it decodes to its symbols with ok, consuming the encoder's words and finding its escapes.
// Every coverage corner must occur.
//
// Plusargs: +SEED=<n> (default 1), +VERBOSE. Parameters: the units', DEC_ and ENC_ where both units have one.
// Inputs change on the falling edge. Transfers are sampled 1 time unit later.
module tb_pans_stream_rt #(
  parameter int N_LANES         = 8,
  parameter int DATA_BYTES      = 4,
  parameter int FIFO_WORDS      = 8,
  parameter int DEC_OUT_DEPTH   = 7,
  parameter int DEC_COUNT_WIDTH = 32,
  parameter bit DEC_CLOCK_GATE  = 1,
  parameter int DIV_BITS        = 2,
  parameter int DIV_STEP_BITS   = 1,
  parameter int DIV_FIRST_BITS  = DIV_STEP_BITS,
  parameter int ENC_OUT_DEPTH   = 8,
  parameter int ENC_COUNT_WIDTH = 32,
  parameter bit ENC_CLOCK_GATE  = 1
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
  import "DPI-C" function int pans_stream_job_bound(input int seed, input int n_lanes, output longint index0,
                                                    output longint index1);
  import "DPI-C" function void pans_stream_job_free(input int job);
  import "DPI-C" function longint pans_stream_job_n(input int job);
  import "DPI-C" function void pans_stream_job_push(input int job, input longint k, output int lane, output int symbol,
                                                    output longint b);
  import "DPI-C" function int pans_stream_scale(input int job, input int lane, input longint t, output longint b);
  import "DPI-C" function int pans_stream_observe(input int job, input int lane, input longint t, input int symbol);

  localparam int     LANE_WIDTH  = (N_LANES > 1) ? $clog2(N_LANES) : 1;
  localparam int     POS_WIDTH   = unsigned_width(((longint'(1) <<< DEC_COUNT_WIDTH) - 2) / longint'(N_LANES));
  localparam int     WORD_BYTES  = WORD_WIDTH / 8;
  localparam longint NL          = longint'(N_LANES);
  localparam longint DBL         = longint'(DATA_BYTES);
  localparam longint WATCHDOG    = 20000;
  localparam int     KIND_CONST = 0, KIND_EMA = 1, KIND_LIST = 2;
  // The least S-bit input: SYMBOL_MIN, or below a symmetric alphabet -2^(SW-1), which the encoder escapes.
  localparam int SYMBOL_IN_MIN = -(2 ** (SYMBOL_WIDTH - 1));
  localparam bit     TWO_WORDS_POSSIBLE = WORD_WIDTH + $clog2(ESCAPE_SLOTS) < PRECISION + SYMBOL_WIDTH;

  // Stall profiles: percent of cycles each channel stalls, and the consumer's latency range.
  localparam int NPROF = 3;
  localparam int PROF_GAP   [NPROF] = '{0, 10, 40};
  localparam int PROF_STALL [NPROF] = '{0, 10, 50};
  localparam int PROF_LAT   [NPROF] = '{1, 3, 6};

  // ---- Clock, reset, the units
  logic clk   = 1'b0;
  logic rst_n = 1'b0;
  always #5 clk = ~clk;

  // pans_enc_stream
  logic                           e_job_valid  = 1'b0;
  logic                           e_job_ready;
  logic [ENC_COUNT_WIDTH-1:0]     e_job_n_sym  = '0;
  logic                           e_sym_valid  = 1'b0;
  logic                           e_sym_ready;
  logic [LANE_WIDTH-1:0]          e_sym_lane   = '0;
  logic signed [SYMBOL_WIDTH-1:0] e_sym_symbol = '0;
  logic [B_WIDTH-1:0]             e_sym_b      = '0;
  logic                           e_word_valid;
  logic                           e_word_ready = 1'b0;
  logic [WORD_WIDTH-1:0]          e_word_word;
  logic                           e_done_valid;
  logic                           e_done_ready = 1'b0;
  logic [LANE_WIDTH-1:0]          e_done_lane;
  logic                           e_done_last;
  logic [STATE_WIDTH-1:0]         e_done_state;
  logic [ENC_COUNT_WIDTH:0]       e_done_n_words;
  logic [ENC_COUNT_WIDTH-1:0]     e_done_n_escapes;
  logic                           e_done_err_order;

  pans_enc_stream #(.N_LANES(N_LANES), .DIV_BITS(DIV_BITS), .DIV_STEP_BITS(DIV_STEP_BITS),
                    .DIV_FIRST_BITS(DIV_FIRST_BITS), .OUT_DEPTH(ENC_OUT_DEPTH), .COUNT_WIDTH(ENC_COUNT_WIDTH),
                    .CLOCK_GATE(ENC_CLOCK_GATE)) u_enc (
    .clk, .rst_n,
    .job_in_valid(e_job_valid), .job_in_ready(e_job_ready), .job_in_n_sym(e_job_n_sym),
    .sym_in_valid(e_sym_valid), .sym_in_ready(e_sym_ready), .sym_in_lane(e_sym_lane), .sym_in_symbol(e_sym_symbol),
    .sym_in_b(e_sym_b),
    .word_out_valid(e_word_valid), .word_out_ready(e_word_ready), .word_out_word(e_word_word),
    .done_out_valid(e_done_valid), .done_out_ready(e_done_ready), .done_out_lane(e_done_lane),
    .done_out_last(e_done_last), .done_out_state(e_done_state), .done_out_n_words(e_done_n_words),
    .done_out_n_escapes(e_done_n_escapes), .done_out_err_order(e_done_err_order)
  );

  // pans_dec_stream
  logic                           d_job_valid   = 1'b0;
  logic                           d_job_ready;
  logic [DEC_COUNT_WIDTH-1:0]     d_job_n_sym   = '0;
  logic [DEC_COUNT_WIDTH-1:0]     d_job_n_bytes = '0;
  logic                           d_data_valid  = 1'b0;
  logic                           d_data_ready;
  logic [8*DATA_BYTES-1:0]        d_data_data   = '0;
  logic                           d_scale_valid = 1'b0;
  logic                           d_scale_ready;
  logic [LANE_WIDTH-1:0]          d_scale_lane  = '0;
  logic [B_WIDTH-1:0]             d_scale_b     = '0;
  logic                           d_sym_valid;
  logic                           d_sym_ready   = 1'b0;
  logic [LANE_WIDTH-1:0]          d_sym_lane;
  logic [POS_WIDTH-1:0]           d_sym_t;
  logic signed [SYMBOL_WIDTH-1:0] d_sym_symbol;
  logic                           d_sym_escape;
  logic                           d_done_valid;
  logic                           d_done_ready  = 1'b0;
  logic                           d_done_ok;
  logic [DEC_COUNT_WIDTH-1:0]     d_done_n_escapes;
  logic [DEC_COUNT_WIDTH-1:0]     d_done_words_consumed;
  logic                           d_done_err_words;
  logic                           d_done_err_header;

  pans_dec_stream #(.N_LANES(N_LANES), .DATA_BYTES(DATA_BYTES), .FIFO_WORDS(FIFO_WORDS), .OUT_DEPTH(DEC_OUT_DEPTH),
                    .COUNT_WIDTH(DEC_COUNT_WIDTH), .CLOCK_GATE(DEC_CLOCK_GATE))
  u_dec (
    .clk, .rst_n,
    .job_in_valid(d_job_valid), .job_in_ready(d_job_ready), .job_in_n_sym(d_job_n_sym),
    .job_in_n_bytes(d_job_n_bytes),
    .data_in_valid(d_data_valid), .data_in_ready(d_data_ready), .data_in_data(d_data_data),
    .scale_in_valid(d_scale_valid), .scale_in_ready(d_scale_ready), .scale_in_lane(d_scale_lane),
    .scale_in_b(d_scale_b),
    .sym_out_valid(d_sym_valid), .sym_out_ready(d_sym_ready), .sym_out_lane(d_sym_lane), .sym_out_t(d_sym_t),
    .sym_out_symbol(d_sym_symbol), .sym_out_escape(d_sym_escape),
    .done_out_valid(d_done_valid), .done_out_ready(d_done_ready), .done_out_ok(d_done_ok),
    .done_out_n_escapes(d_done_n_escapes), .done_out_words_consumed(d_done_words_consumed),
    .done_out_err_words(d_done_err_words), .done_out_err_header(d_done_err_header)
  );

  // ---- Checking
  longint checks = 0;
  longint errors = 0;
  bit     verbose;

  function automatic bit check(input bit ok, input string what);
    checks++;
    if (!ok) begin
      errors++;
      if (errors <= 20) $display("ERROR %s", what);
    end
    return ok;
  endfunction

  // ---- Random numbers: splitmix64, one stream per process, independent of their interleaving
  typedef longint unsigned u64_t;

  function automatic u64_t splitmix(ref u64_t s);
    u64_t z;
    s = s + 64'h9e3779b97f4a7c15;
    z = s;
    z = (z ^ (z >> 30)) * 64'hbf58476d1ce4e5b9;
    z = (z ^ (z >> 27)) * 64'h94d049bb133111eb;
    return z ^ (z >> 31);
  endfunction

  u64_t rng_main, rng_enc, rng_dec;

  function automatic bit enc_chance(input int pct);
    return longint'(splitmix(rng_enc) % 100) < longint'(pct);
  endfunction

  function automatic bit dec_chance(input int pct);
    return longint'(splitmix(rng_dec) % 100) < longint'(pct);
  endfunction

  function automatic longint main_below(input longint n);  // [0, n), n >= 1
    return longint'(splitmix(rng_main) % u64_t'(n));
  endfunction

  // ---- Jobs (handles of the model's stimulus) and what the encoder hands the decoder
  int     jb_h[$];
  int     jb_prof[$];
  string  jb_label[$];

  typedef struct {
    int     job;           // index into jb_*
    longint n_words;       // the encoder's done_out counts
    longint n_escapes;
    byte unsigned bytes[$];  // the assembled array
  } array_t;

  array_t to_dec[$];       // the encoder's arrays, in job order
  int     enc_done = 0;    // jobs encoded
  int     dec_done = 0;    // jobs decoded
  longint cycle = 0;
  longint last_progress = 0;

  // ---- Coverage
  longint cov_symbols = 0, cov_escapes = 0, cov_two_push = 0, cov_two_pop = 0, cov_empty_lanes = 0, cov_n0 = 0;
  longint cov_words = 0, cov_bytes = 0;
  longint cov_outside = 0;  // inputs below a symmetric alphabet, back as escapes

  always @(negedge clk) begin
    if (rst_n) begin
      cycle++;
      if (u_enc.q2_valid && u_enc.s3_push == 2'd2) cov_two_push++;
      if (u_dec.q5_valid && u_dec.s6_pops == 2'd2) cov_two_pop++;
      if (cycle - last_progress > WATCHDOG)
        $fatal(1, "FAILED: watchdog: no transfer for %0d cycles (%0d jobs encoded, %0d decoded of %0d)", WATCHDOG,
               enc_done, dec_done, jb_h.size());
    end
  end

  function automatic void add_job(input int h, input int prof, input string label);
    if (!check(h >= 0, $sformatf("job %s: the model refused it", label))) return;
    jb_h.push_back(h);
    jb_prof.push_back(prof);
    jb_label.push_back(label);
  endfunction

  // A directed LIST job at b = 0. The symbols of `lane`, or of every lane for lane < 0, escape.
  // The other symbols are 0, coded with f = M'.
  function automatic int storm(input int lane, input longint n);
    int h, s;
    h = pans_stream_job_begin(N_LANES, n, KIND_LIST, 0);
    for (longint i = 0; i < n; i++) begin
      s = 0;
      if (lane < 0 || longint'(lane) == i % NL)
        while (s == 0)
          s = int'(longint'(SYMBOL_IN_MIN) + main_below(longint'(SYMBOL_MAX) - longint'(SYMBOL_IN_MIN) + 1));
      pans_stream_job_set(h, i, s, 0);
    end
    void'(check(pans_stream_job_build(h) == 0, "directed job: build failed"));
    return h;
  endfunction

  function automatic void make_jobs(input int seed);
    longint ns[6];
    int     k;
    k = 0;
    for (int prof = 0; prof < NPROF; prof++) begin
      ns = '{0, 1, NL - 1, NL, NL + 1, 2 + main_below(2999)};
      foreach (ns[q])
        for (int kind = 0; kind <= KIND_LIST; kind++)
          for (int delay = 0; delay <= (kind == KIND_EMA ? 2 : 0); delay++)
            add_job(pans_stream_job_new(seed * 1000 + k++, N_LANES, ns[q], kind, delay), prof,
                    $sformatf("n %0d kind %0d delay %0d", ns[q], kind, delay));
    end
    add_job(storm(-1, 6 * NL + 5), 0, "escape storm, every lane");
    add_job(storm(-1, 9 * NL + 1), 2, "escape storm, every lane, heavy stalls");
    add_job(storm(N_LANES > 1 ? 1 : 0, 7 * NL + 3), 1, "escape storm, one lane");
    add_job(storm(N_LANES, 5 * NL + 2), 1, "f = M' only");
    begin  // the raw push's bound job, last to keep the seeds above
      longint i0, i1;
      add_job(pans_stream_job_bound(seed * 1000 + k++, N_LANES, i0, i1), 1, "raw push bound");
    end
  endfunction

  // ---- The encoder's process: every job through pans_enc_stream, its output assembled into the array
  initial begin : enc_proc
    bit job_fired, sym_fired;
    job_fired = 0;
    sym_fired = 0;
    wait (rst_n);
    for (int j = 0; j < jb_h.size(); j++) begin
      longint n, k, n_words, n_escapes;
      int     d, prof, lane, sym;
      longint b;
      bit     job_sent, err_order;
      logic [WORD_WIDTH-1:0]  words[$];
      logic [STATE_WIDTH-1:0] states[N_LANES];
      array_t a;
      // variables of a block in an initial process are static, so every one is set here
      words.delete();
      a.bytes.delete();
      n = pans_stream_job_n(jb_h[j]);
      prof = jb_prof[j];
      k = 0;
      d = 0;
      job_sent = 0;
      n_words = 0;
      n_escapes = 0;
      err_order = 0;
      while (d < N_LANES) begin
        @(negedge clk);
        if (job_fired) begin
          e_job_valid = 1'b0;
          job_fired   = 1'b0;
        end
        if (!job_sent && !e_job_valid && !enc_chance(PROF_GAP[prof])) begin
          e_job_valid = 1'b1;
          e_job_n_sym = ENC_COUNT_WIDTH'(n);
        end
        if (sym_fired) begin
          e_sym_valid = 1'b0;
          sym_fired   = 1'b0;
        end
        if (job_sent && !e_sym_valid && k < n && !enc_chance(PROF_GAP[prof])) begin
          pans_stream_job_push(jb_h[j], k, lane, sym, b);
          e_sym_valid  = 1'b1;
          e_sym_lane   = LANE_WIDTH'(lane);
          e_sym_symbol = SYMBOL_WIDTH'(sym);
          e_sym_b      = B_WIDTH'(b);
        end
        e_word_ready = !enc_chance(PROF_STALL[prof]);
        e_done_ready = !enc_chance(PROF_STALL[prof]);
        #1;
        if (e_job_valid && e_job_ready) begin
          job_fired = 1'b1;
          job_sent  = 1'b1;
          last_progress = cycle;
        end
        if (e_sym_valid && e_sym_ready) begin
          sym_fired = 1'b1;
          k++;
          last_progress = cycle;
        end
        if (e_word_valid && e_word_ready) begin
          words.push_back(e_word_word);
          last_progress = cycle;
        end
        if (e_done_valid && e_done_ready) begin
          void'(check(job_sent && k == n && int'(e_done_lane) == d && e_done_last == (d == N_LANES - 1),
                      $sformatf("job %0d (%s) encoder done beat %0d: lane %0d last %0b after %0d of %0d symbols", j,
                                jb_label[j], d, e_done_lane, e_done_last, k, n)));
          states[d] = e_done_state;
          n_words   = longint'(e_done_n_words);
          n_escapes = longint'(e_done_n_escapes);
          err_order = err_order || e_done_err_order;
          d++;
          last_progress = cycle;
        end
      end
      void'(check(n_words == longint'(words.size()) && !err_order,
                  $sformatf("job %0d (%s) encoder: done_out says %0d words, word_out gave %0d; err_order %0b", j,
                            jb_label[j], n_words, words.size(), err_order)));
      // The array (STREAM.md §1.5): the final states, lane 0 first, then the words last-emitted first, LE
      a.job       = j;
      a.n_words   = n_words;
      a.n_escapes = n_escapes;
      for (int l = 0; l < N_LANES; l++)
        for (int b = 0; b < STATE_BYTES; b++) a.bytes.push_back(8'(longint'(states[l]) >> (8 * b)));
      for (int w = words.size() - 1; w >= 0; w--)
        for (int b = 0; b < WORD_BYTES; b++) a.bytes.push_back(8'(words[w] >> (8 * b)));
      cov_words += longint'(words.size());
      cov_bytes += longint'(a.bytes.size());
      to_dec.push_back(a);
      enc_done++;
    end
    @(negedge clk);
    e_job_valid  = 1'b0;
    e_sym_valid  = 1'b0;
    e_word_ready = 1'b0;
    e_done_ready = 1'b0;
  end

  // ---- The decoder's process: every array through pans_dec_stream, the model as consumer
  initial begin : dec_proc
    bit job_fired, data_fired, scale_fired;
    job_fired = 0;
    data_fired = 0;
    scale_fired = 0;
    wait (rst_n);
    for (int j = 0; j < jb_h.size(); j++) begin
      array_t a;
      longint n, nb, beats, db, sfi, oi, escapes, cyc;
      int     h, prof;
      bit     job_sent, done;
      int     expect_sym[$];
      int     sq_lane[$];
      longint sq_b[$];
      longint sq_at[$];
      wait (to_dec.size() > 0);
      // variables of a block in an initial process are static, so every one is set here
      expect_sym.delete();
      sq_lane.delete();
      sq_b.delete();
      sq_at.delete();
      a = to_dec.pop_front();
      h = jb_h[a.job];
      prof = jb_prof[a.job];
      n = pans_stream_job_n(h);
      nb = longint'(a.bytes.size());
      beats = (nb + DBL - 1) / DBL;
      // the job's symbols in decode order: global index i is push beat n - 1 - i
      for (longint i = 0; i < n; i++) begin
        int     lane, sym;
        longint b;
        pans_stream_job_push(h, n - 1 - i, lane, sym, b);
        expect_sym.push_back(sym);
      end
      if (n < NL) cov_empty_lanes += NL - n;
      if (n == 0) cov_n0++;
      db = 0; sfi = 0; oi = 0; escapes = 0; cyc = 0;
      job_sent = 0;
      done = 0;
      while (!done) begin
        @(negedge clk);
        cyc++;
        if (job_fired) begin
          d_job_valid = 1'b0;
          job_fired   = 1'b0;
        end
        if (!job_sent && !d_job_valid && !dec_chance(PROF_GAP[prof])) begin
          d_job_valid   = 1'b1;
          d_job_n_sym   = DEC_COUNT_WIDTH'(n);
          d_job_n_bytes = DEC_COUNT_WIDTH'(nb);
        end
        if (data_fired) begin
          d_data_valid = 1'b0;
          data_fired   = 1'b0;
        end
        if (job_sent && !d_data_valid && db < beats && !dec_chance(PROF_GAP[prof])) begin
          d_data_valid = 1'b1;
          for (int b = 0; b < DATA_BYTES; b++) begin
            longint at;
            at = db * DBL + longint'(b);
            d_data_data[8*b +: 8] = (at < nb) ? a.bytes[at] : 8'(splitmix(rng_dec));  // beyond the array: noise
          end
        end
        // the consumer: every scale it can compute, in decode order, presentable from its latency on
        if (scale_fired) begin
          d_scale_valid = 1'b0;
          scale_fired   = 1'b0;
          void'(sq_lane.pop_front());
          void'(sq_b.pop_front());
          void'(sq_at.pop_front());
        end
        while (sfi < n && sq_lane.size() < 4 * N_LANES) begin
          int     got;
          longint b;
          got = pans_stream_scale(h, int'(sfi % NL), sfi / NL, b);
          if (got != 1) begin
            void'(check(got == 0, $sformatf("job %0d: pans_stream_scale(%0d, %0d) = %0d", a.job, sfi % NL, sfi / NL,
                                            got)));
            break;
          end
          sq_lane.push_back(int'(sfi % NL));
          sq_b.push_back(b);
          sq_at.push_back(cyc + longint'(splitmix(rng_dec) % u64_t'(PROF_LAT[prof])));
          sfi++;
        end
        if (job_sent && !d_scale_valid && sq_lane.size() > 0 && cyc >= sq_at[0] && !dec_chance(PROF_GAP[prof])) begin
          d_scale_valid = 1'b1;
          d_scale_lane  = LANE_WIDTH'(sq_lane[0]);
          d_scale_b     = B_WIDTH'(sq_b[0]);
        end
        d_sym_ready  = !dec_chance(PROF_STALL[prof]);
        d_done_ready = !dec_chance(PROF_STALL[prof]);
        #1;
        if (d_job_valid && d_job_ready) begin
          job_fired = 1'b1;
          job_sent  = 1'b1;
          last_progress = cycle;
        end
        if (d_data_valid && d_data_ready) begin
          data_fired = 1'b1;
          db++;
          last_progress = cycle;
        end
        if (d_scale_valid && d_scale_ready) begin
          scale_fired = 1'b1;
          last_progress = cycle;
        end
        if (d_sym_valid && d_sym_ready) begin
          if (check(oi < n, $sformatf("job %0d (%s): a symbol beyond its %0d", a.job, jb_label[a.job], n))) begin
            void'(check(longint'(d_sym_lane) == oi % NL && longint'(d_sym_t) == oi / NL
                        && int'(d_sym_symbol) == expect_sym[oi],
                        $sformatf("job %0d (%s) symbol %0d: lane %0d t %0d symbol %0d, the job's lane %0d t %0d symbol %0d",
                                  a.job, jb_label[a.job], oi, d_sym_lane, d_sym_t, int'(d_sym_symbol), oi % NL, oi / NL,
                                  expect_sym[oi])));
            void'(check(pans_stream_observe(h, int'(oi % NL), oi / NL, int'(d_sym_symbol)) == 0,
                        $sformatf("job %0d symbol %0d: the consumer refused it", a.job, oi)));
            escapes += longint'(d_sym_escape);
            if (int'(d_sym_symbol) < SYMBOL_MIN)
              cov_outside += longint'(check(d_sym_escape == 1'b1,
                                            $sformatf("job %0d symbol %0d: %0d outside the alphabet not an escape",
                                                      a.job, oi, int'(d_sym_symbol))));
            oi++;
          end
          last_progress = cycle;
        end
        if (d_done_valid && d_done_ready) begin
          void'(check(oi == n && db == beats && d_done_ok && !d_done_err_words && !d_done_err_header,
                      $sformatf("job %0d (%s) decoder done: ok %0b errors w%0b h%0b after %0d of %0d symbols, %0d of %0d beats",
                                a.job, jb_label[a.job], d_done_ok, d_done_err_words, d_done_err_header, oi, n, db,
                                beats)));
          void'(check(longint'(d_done_words_consumed) == a.n_words && longint'(d_done_n_escapes) == a.n_escapes
                      && escapes == a.n_escapes,
                      $sformatf("job %0d (%s): decoder consumed %0d words, %0d escapes (%0d flagged on sym_out); the encoder wrote %0d words, %0d escapes",
                                a.job, jb_label[a.job], d_done_words_consumed, d_done_n_escapes, escapes, a.n_words,
                                a.n_escapes)));
          cov_symbols += n;
          cov_escapes += escapes;
          done = 1;
          last_progress = cycle;
          if (verbose)
            $display("job %0d (%s, profile %0d): n %0d, %0d words, %0d bytes, %0d escapes, decoded in %0d cycles",
                     a.job, jb_label[a.job], prof, n, a.n_words, nb, escapes, cyc);
        end
      end
      pans_stream_job_free(h);
      dec_done++;
    end
    @(negedge clk);
    d_job_valid   = 1'b0;
    d_data_valid  = 1'b0;
    d_scale_valid = 1'b0;
    d_sym_ready   = 1'b0;
    d_done_ready  = 1'b0;
  end

  // ---- The run
  initial begin
    int seed, ww, sw, sb;
    seed = 1;
    void'($value$plusargs("SEED=%d", seed));
    verbose = $test$plusargs("VERBOSE");
    rng_main = u64_t'(seed) * 64'h2545f4914f6cdd1d + 64'h7274;
    rng_enc  = rng_main ^ 64'h656e63;
    rng_dec  = rng_main ^ 64'h646563;
    if (64'(pans_ref_config_id()) != CONFIG_ID)
      $fatal(1, "config mismatch: RTL package CONFIG_ID %016h, C++ reference %016h", CONFIG_ID,
             64'(pans_ref_config_id()));
    pans_stream_widths(ww, sw, sb);
    if (ww != WORD_WIDTH || sw != STATE_WIDTH || sb != STATE_BYTES)
      $fatal(1, "config mismatch: RTL package widths W %0d, state %0d bits in %0d bytes; C++ model %0d, %0d, %0d",
             WORD_WIDTH, STATE_WIDTH, STATE_BYTES, ww, sw, sb);
    make_jobs(seed);
    $display("tb_pans_stream_rt: PRECISION=%0d SYMBOL_WIDTH=%0d ESCAPE_SLOTS=%0d W=%0d CONFIG_ID=%016h; N_LANES=%0d DATA_BYTES=%0d FIFO_WORDS=%0d DEC_OUT_DEPTH=%0d DEC_COUNT_WIDTH=%0d DEC_CLOCK_GATE=%0d DIV_BITS=%0d DIV_STEP_BITS=%0d DIV_FIRST_BITS=%0d ENC_OUT_DEPTH=%0d ENC_COUNT_WIDTH=%0d ENC_CLOCK_GATE=%0d; seed %0d, %0d jobs",
             PRECISION, SYMBOL_WIDTH, ESCAPE_SLOTS, WORD_WIDTH, CONFIG_ID, N_LANES, DATA_BYTES, FIFO_WORDS,
             DEC_OUT_DEPTH, DEC_COUNT_WIDTH, DEC_CLOCK_GATE, DIV_BITS, DIV_STEP_BITS, DIV_FIRST_BITS, ENC_OUT_DEPTH,
             ENC_COUNT_WIDTH, ENC_CLOCK_GATE, seed, jb_h.size());
    repeat (3) @(negedge clk);
    rst_n = 1'b1;
    wait (dec_done == jb_h.size());
    repeat (2) @(negedge clk);
    $display("coverage: %0d symbols round-tripped through %0d words (%0d bytes); %0d escapes, %0d two-word pushes, %0d two-word pops, %0d empty lanes, %0d jobs with n = 0",
             cov_symbols, cov_words, cov_bytes, cov_escapes, cov_two_push, cov_two_pop, cov_empty_lanes, cov_n0);
    void'(check(cov_escapes > 0 && cov_empty_lanes > 0 && cov_n0 > 0
                && ((cov_two_push > 0 && cov_two_pop > 0) || !TWO_WORDS_POSSIBLE),
                "coverage: escapes, empty lanes, n = 0 and two-word pushes and pops (W + log2 E < P + S) must all occur"));
    if (SYMBOL_IN_MIN < SYMBOL_MIN) begin
      $display("coverage: %0d symbols %0d outside the symmetric alphabet round-tripped as escapes", cov_outside,
               SYMBOL_IN_MIN);
      void'(check(cov_outside > 0, "coverage: a symbol outside the symmetric alphabet must round-trip"));
    end
    if (errors != 0) $fatal(1, "FAILED: %0d errors in %0d checks", errors, checks);
    $display("PASSED: %0d checks, 0 errors (%0d jobs, %0d cycles)", checks, jb_h.size(), cycle);
    $finish;
  end

endmodule
