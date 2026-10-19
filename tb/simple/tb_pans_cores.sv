// SPDX-License-Identifier: Apache-2.0
// tb_pans_cores: directed encode/decode test of pans_enc_core and pans_dec_core against the C++ reference.
// The reference is ref/pans1_dpi.cpp built for the same config. The CONFIG_IDs must match.
// Specification: SPEC.md §6. +ALL_SCALES runs every scale code instead of the default set.
// Single transactions change the inputs on the falling edge and wait for out_valid.
module tb_pans_cores;

  import pans_pkg::*;

  import "DPI-C" function longint pans_ref_config_id();
  import "DPI-C" function void pans_ref_scale(input real sigma, output int exp_field, output int man_field);
  import "DPI-C" function void pans_ref_encode(input int exp_field, input int man_field, input int symbol,
                                               output int start, output int freq, output int escape);
  import "DPI-C" function void pans_ref_decode(input int exp_field, input int man_field, input int slot,
                                               output int symbol, output int start, output int freq,
                                               output int escape);

  logic clk   = 1'b0;
  logic rst_n = 1'b0;
  always #5 clk = ~clk;

  // ---- Encoder
  logic                           enc_in_valid  = 1'b0;
  scale_t                         enc_in_scale  = '0;
  logic signed [SYMBOL_WIDTH-1:0] enc_in_symbol = '0;
  logic                           enc_out_valid;
  logic [PRECISION-1:0]           enc_out_start;
  logic [PRECISION-1:0]           enc_out_freq;
  logic                           enc_out_escape;

  pans_enc_core u_enc (
    .clk        (clk),
    .rst_n      (rst_n),
    .in_valid   (enc_in_valid),
    .in_scale   (enc_in_scale),
    .in_symbol  (enc_in_symbol),
    .out_valid  (enc_out_valid),
    .out_start  (enc_out_start),
    .out_freq   (enc_out_freq),
    .out_escape (enc_out_escape)
  );

  // ---- Decoder
  logic                           dec_in_valid = 1'b0;
  scale_t                         dec_in_scale = '0;
  logic [PRECISION-1:0]           dec_in_slot  = '0;
  logic                           dec_out_valid;
  logic signed [SYMBOL_WIDTH-1:0] dec_out_symbol;
  logic [PRECISION-1:0]           dec_out_start;
  logic [PRECISION-1:0]           dec_out_freq;
  logic                           dec_out_escape;

  pans_dec_core u_dec (
    .clk        (clk),
    .rst_n      (rst_n),
    .in_valid   (dec_in_valid),
    .in_scale   (dec_in_scale),
    .in_slot    (dec_in_slot),
    .out_valid  (dec_out_valid),
    .out_symbol (dec_out_symbol),
    .out_start  (dec_out_start),
    .out_freq   (dec_out_freq),
    .out_escape (dec_out_escape)
  );

  // The encoder's S-bit inputs, the alphabet and, below a symmetric one, -2^(SW-1).
  localparam int SYMBOL_IN_MIN = -(2 ** (SYMBOL_WIDTH - 1));

  // ---- Checking
  int unsigned checks = 0;
  int unsigned errors = 0;

  function automatic void check(input bit ok, input string what);
    checks++;
    if (!ok) begin
      errors++;
      if (errors <= 20) $display("ERROR %s", what);
    end
  endfunction

  // ---- Transactions
  // One transaction at a time. LATENCY_MAX bounds the wait for out_valid.
  localparam int LATENCY_MAX = 8;

  task automatic encode(input scale_t scale, input int symbol, output int start, output int freq, output bit escape);
    int wait_cycles;
    @(negedge clk);
    enc_in_valid  = 1'b1;
    enc_in_scale  = scale;
    enc_in_symbol = SYMBOL_WIDTH'(symbol);
    @(negedge clk);
    enc_in_valid = 1'b0;
    wait_cycles  = 1;
    while (enc_out_valid !== 1'b1 && wait_cycles < LATENCY_MAX) begin
      @(negedge clk);
      wait_cycles++;
    end
    check(enc_out_valid === 1'b1, "encoder: no output within LATENCY_MAX cycles");
    start  = int'(enc_out_start);
    freq   = int'(enc_out_freq);
    escape = enc_out_escape;
    @(negedge clk);
  endtask

  task automatic decode(input scale_t scale, input int slot, output int symbol, output int start, output int freq,
                        output bit escape);
    int wait_cycles;
    @(negedge clk);
    dec_in_valid = 1'b1;
    dec_in_scale = scale;
    dec_in_slot  = PRECISION'(slot);
    @(negedge clk);
    dec_in_valid = 1'b0;
    wait_cycles  = 1;
    while (dec_out_valid !== 1'b1 && wait_cycles < LATENCY_MAX) begin
      @(negedge clk);
      wait_cycles++;
    end
    check(dec_out_valid === 1'b1, "decoder: no output within LATENCY_MAX cycles");
    symbol = int'(dec_out_symbol);
    start  = int'(dec_out_start);
    freq   = int'(dec_out_freq);
    escape = dec_out_escape;
    @(negedge clk);
  endtask

  // Encode symbol, compare with the reference, then decode every slot of its interval (or its escape slot).
  task automatic symbol_round_trip(input scale_t scale, input int symbol, input string label);
    int    start, freq, ref_start, ref_freq, ref_escape;
    int    dec_symbol, dec_start, dec_freq;
    bit    escape, dec_escape;
    string where;
    encode(scale, symbol, start, freq, escape);
    pans_ref_encode(int'(scale.exp), int'(scale.man), symbol, ref_start, ref_freq, ref_escape);
    where = $sformatf("%s symbol=%0d", label, symbol);
    check(start == ref_start && freq == ref_freq && escape == bit'(ref_escape),
          $sformatf("%s: encoder (%0d, %0d, esc %0d), reference (%0d, %0d, esc %0d)",
                    where, start, freq, escape, ref_start, ref_freq, ref_escape));
    if (symbol < SYMBOL_MIN) check(escape, $sformatf("%s: an input outside the alphabet was not escaped", where));
    if (escape) begin
      decode(scale, CODED_SLOTS, dec_symbol, dec_start, dec_freq, dec_escape);
      check(dec_escape, $sformatf("%s: escape slot not decoded as the escape", where));
      return;
    end
    for (int slot = start; slot < start + freq; slot++) begin
      decode(scale, slot, dec_symbol, dec_start, dec_freq, dec_escape);
      check(!dec_escape && dec_symbol == symbol && dec_start == start && dec_freq == freq,
            $sformatf("%s slot=%0d: decoded (%0d, %0d, %0d, esc %0d)",
                      where, slot, dec_symbol, dec_start, dec_freq, dec_escape));
    end
  endtask

  // Decode slot and compare every output with the reference.
  task automatic slot_vs_reference(input scale_t scale, input int slot, input string label);
    int    symbol, start, freq, ref_symbol, ref_start, ref_freq, ref_escape;
    bit    escape;
    decode(scale, slot, symbol, start, freq, escape);
    pans_ref_decode(int'(scale.exp), int'(scale.man), slot, ref_symbol, ref_start, ref_freq, ref_escape);
    check(symbol == ref_symbol && start == ref_start && freq == ref_freq && escape == bit'(ref_escape),
          $sformatf("%s slot=%0d: decoder (%0d, %0d, %0d, esc %0d), reference (%0d, %0d, %0d, esc %0d)",
                    label, slot, symbol, start, freq, escape, ref_symbol, ref_start, ref_freq, ref_escape));
    if (!escape)
      check(symbol >= SYMBOL_MIN && symbol <= SYMBOL_MAX,
            $sformatf("%s slot=%0d: decoded symbol %0d outside the alphabet", label, slot, symbol));
  endtask

  // ---- Streaming: back to back, idle every 7th cycle
  // out_valid must be in_valid delayed by the core's latency, and every response the reference's.
  localparam int ENC_LATENCY = 2;
  localparam int DEC_LATENCY = 4;

  task automatic stream(input scale_t scale, input string label);
    bit enc_v [$], dec_v [$];                 // in_valid history
    int enc_q [$][3], dec_q [$][4];           // expected responses in order
    int n_sym, n_slot, cycle, si, sl;
    int r_start, r_freq, r_escape, r_symbol;
    n_sym  = SYMBOL_MAX - SYMBOL_IN_MIN + 1;
    n_slot = NUM_SLOTS;
    si = 0;
    sl = 0;
    for (cycle = 0; si < n_sym || sl < n_slot || enc_q.size() > 0 || dec_q.size() > 0; cycle++) begin
      @(negedge clk);
      // responses for the edge just past
      if (enc_v.size() >= ENC_LATENCY) begin
        check(enc_out_valid === enc_v[0], $sformatf("%s: encoder out_valid %b, expected %b", label, enc_out_valid, enc_v[0]));
        if (enc_v[0]) begin
          check(int'(enc_out_start) == enc_q[0][0] && int'(enc_out_freq) == enc_q[0][1] && int'(enc_out_escape) == enc_q[0][2],
                $sformatf("%s: streamed encoder (%0d, %0d, %0d), reference (%0d, %0d, %0d)", label, enc_out_start,
                          enc_out_freq, enc_out_escape, enc_q[0][0], enc_q[0][1], enc_q[0][2]));
          enc_q.pop_front();
        end
        enc_v.pop_front();
      end
      if (dec_v.size() >= DEC_LATENCY) begin
        check(dec_out_valid === dec_v[0], $sformatf("%s: decoder out_valid %b, expected %b", label, dec_out_valid, dec_v[0]));
        if (dec_v[0]) begin
          check(int'(dec_out_symbol) == dec_q[0][0] && int'(dec_out_start) == dec_q[0][1] && int'(dec_out_freq) == dec_q[0][2]
                && int'(dec_out_escape) == dec_q[0][3],
                $sformatf("%s: streamed decoder (%0d, %0d, %0d, %0d), reference (%0d, %0d, %0d, %0d)", label,
                          dec_out_symbol, dec_out_start, dec_out_freq, dec_out_escape, dec_q[0][0], dec_q[0][1],
                          dec_q[0][2], dec_q[0][3]));
          dec_q.pop_front();
        end
        dec_v.pop_front();
      end
      // the next requests (idle every 7th cycle and after the last)
      enc_in_valid = (cycle % 7 != 6) && si < n_sym;
      dec_in_valid = (cycle % 7 != 6) && sl < n_slot;
      enc_in_scale = scale;
      dec_in_scale = scale;
      if (enc_in_valid) begin
        enc_in_symbol = SYMBOL_WIDTH'(SYMBOL_IN_MIN + si);
        pans_ref_encode(int'(scale.exp), int'(scale.man), SYMBOL_IN_MIN + si, r_start, r_freq, r_escape);
        enc_q.push_back('{r_start, r_freq, r_escape});
        si++;
      end
      if (dec_in_valid) begin
        dec_in_slot = PRECISION'(sl);
        pans_ref_decode(int'(scale.exp), int'(scale.man), sl, r_symbol, r_start, r_freq, r_escape);
        dec_q.push_back('{r_symbol, r_start, r_freq, r_escape});
        sl++;
      end
      enc_v.push_back(enc_in_valid);
      dec_v.push_back(dec_in_valid);
    end
    enc_in_valid = 1'b0;
    dec_in_valid = 1'b0;
    check(si == n_sym && sl == n_slot, $sformatf("%s: streamed %0d symbols, %0d slots", label, si, sl));
    if ($test$plusargs("STREAM_DEBUG")) $display("%s: %0d cycles, %0d symbols, %0d slots", label, cycle, si, sl);
    repeat (DEC_LATENCY + 1) @(negedge clk);
  endtask

  // ---- Test
  initial begin
    real    sigmas[$];
    scale_t scale;
    int     exp_field, man_field;

    sigmas.push_back(0.0);
    sigmas.push_back(1.0 / 0.0);
    for (int k = -16; k <= 32; k++) sigmas.push_back(2.0 ** (k / 4.0));

    // The C++ reference must be the build this package was generated from.
    if (64'(pans_ref_config_id()) != CONFIG_ID)
      $fatal(1, "config mismatch: RTL package CONFIG_ID %016h, C++ reference %016h", CONFIG_ID, 64'(pans_ref_config_id()));

    repeat (3) @(posedge clk);
    rst_n = 1'b1;

    if ($test$plusargs("ALL_SCALES")) begin
      $display("pans cores: PRECISION=%0d SYMBOL_WIDTH=%0d ESCAPE_SLOTS=%0d alphabet [%0d, %0d] CONFIG_ID=%016h, %s",
               PRECISION, SYMBOL_WIDTH, ESCAPE_SLOTS, SYMBOL_MIN, SYMBOL_MAX, CONFIG_ID,
               $sformatf("all %0d scale codes", 2 ** (SCALE_EXP_WIDTH + SCALE_MAN_WIDTH)));
      for (int e = 0; e < 2 ** SCALE_EXP_WIDTH; e++) begin
        for (int m = 0; m < 2 ** SCALE_MAN_WIDTH; m++) begin
          scale = '{exp: SCALE_EXP_WIDTH'(e), man: SCALE_MAN_WIDTH'(m)};
          for (int symbol = SYMBOL_IN_MIN; symbol <= SYMBOL_MAX; symbol++)
            symbol_round_trip(scale, symbol, $sformatf("exp=%0d man=%0d", e, m));
          for (int slot = 0; slot < NUM_SLOTS; slot++) slot_vs_reference(scale, slot, $sformatf("exp=%0d man=%0d", e, m));
          stream(scale, $sformatf("exp=%0d man=%0d stream", e, m));
        end
      end
    end else begin
      $display("pans cores: PRECISION=%0d SYMBOL_WIDTH=%0d ESCAPE_SLOTS=%0d alphabet [%0d, %0d] CONFIG_ID=%016h, %s",
               PRECISION, SYMBOL_WIDTH, ESCAPE_SLOTS, SYMBOL_MIN, SYMBOL_MAX, CONFIG_ID,
               $sformatf("%0d scales", sigmas.size()));
      foreach (sigmas[i]) begin
        pans_ref_scale(sigmas[i], exp_field, man_field);
        scale = '{exp: SCALE_EXP_WIDTH'(exp_field), man: SCALE_MAN_WIDTH'(man_field)};
        for (int symbol = SYMBOL_IN_MIN; symbol <= SYMBOL_MAX; symbol++)
          symbol_round_trip(scale, symbol, $sformatf("sigma=%g", sigmas[i]));
        for (int slot = 0; slot < NUM_SLOTS; slot++) slot_vs_reference(scale, slot, $sformatf("sigma=%g", sigmas[i]));
        stream(scale, $sformatf("sigma=%g stream", sigmas[i]));
      end
    end

    if (errors != 0) $fatal(1, "FAILED: %0d errors in %0d checks", errors, checks);
    $display("PASSED: %0d checks, 0 errors", checks);
    $finish;
  end

endmodule
