// SPDX-License-Identifier: Apache-2.0
// power_tb.sv: a core's post-route netlist against the C++ reference, one transaction per clock (make power-<cfg>).
// It replays a stimulus file of ppa/power_stimulus.cpp and checks every response and out_valid.
// Specification: SPEC.md §6 and §7. The corner set checks the netlist. The workload's activity goes to the VCD.
// The last line is "<top>: <n> transactions, <m> mismatches, <e> escapes", the escapes counted on out_escape.
//
//   +STIMULUS=<hex> +EXPECTED=<hex> +COUNT=<n> [+VCD=<file>]

module power_tb_enc;
  import pans_pkg::*;
  parameter int PERIOD_PS = 10000;  // an integer: OpenSTA's VCD reader mis-parses some real-valued parameters
  parameter int MAX_COUNT = 1 << 17;

  localparam int REQ_WIDTH = SCALE_EXP_WIDTH + SCALE_MAN_WIDTH + SYMBOL_WIDTH;
  localparam int RSP_WIDTH = 2 * PRECISION + 1;
  // A request set before edge i is in the output register after edge i + 2.
  localparam int LATENCY   = 2;

  logic                    clk = 1'b0;
  logic                    rst_n = 1'b0;
  logic                    in_valid = 1'b0;
  logic [REQ_WIDTH-1:0]    request = '0;
  logic [REQ_WIDTH-1:0]    stimulus [MAX_COUNT];
  logic [RSP_WIDTH-1:0]    expected [MAX_COUNT];
  int                      count = 0;
  int                      errors = 0;
  int                      escapes = 0;
  string                   stimulus_file, expected_file, vcd_file;
  logic                    out_valid, out_escape;
  logic [PRECISION-1:0]    out_start, out_freq;

  always #(PERIOD_PS / 2 * 1ps) clk = ~clk;

  pans_enc_core_timed dut (
    .clk, .rst_n, .in_valid,
    .in_scale(request[REQ_WIDTH-1 -: SCALE_EXP_WIDTH + SCALE_MAN_WIDTH]), .in_symbol(request[SYMBOL_WIDTH-1:0]),
    .out_valid, .out_start, .out_freq, .out_escape
  );

  initial begin
    if (!$value$plusargs("STIMULUS=%s", stimulus_file) || !$value$plusargs("EXPECTED=%s", expected_file)
        || !$value$plusargs("COUNT=%d", count) || count < 1 || count > MAX_COUNT)
      $fatal(1, "power_tb_enc: +STIMULUS, +EXPECTED and +COUNT (1 .. %0d) are required", MAX_COUNT);
    $readmemh(stimulus_file, stimulus, 0, count - 1);
    $readmemh(expected_file, expected, 0, count - 1);
    repeat (4) @(posedge clk);
    rst_n    <= 1'b1;
    in_valid <= 1'b1;
    if ($value$plusargs("VCD=%s", vcd_file)) begin
      $dumpfile(vcd_file);
      $dumpvars(0, dut);
    end
    for (int i = 0; i < count + LATENCY; i++) begin
      if (i < count) request <= stimulus[i];
      @(posedge clk);
      #(PERIOD_PS / 4 * 1ps);
      if (i >= LATENCY && out_escape === 1'b1) escapes++;
      if (i >= LATENCY && (out_valid !== 1'b1 || {out_start, out_freq, out_escape} !== expected[i - LATENCY])) begin
        errors++;
        if (errors <= 5) $display("MISMATCH %0d: netlist %h, reference %h", i - LATENCY, {out_start, out_freq, out_escape},
                                  expected[i - LATENCY]);
      end
    end
    $display("power_tb_enc: %0d transactions, %0d mismatches, %0d escapes", count, errors, escapes);
    if (errors != 0) $fatal(1, "netlist does not match the reference");
    $finish;
  end
endmodule

module power_tb_dec;
  import pans_pkg::*;
  parameter int PERIOD_PS = 10000;
  parameter int MAX_COUNT = 1 << 17;

  localparam int REQ_WIDTH = SCALE_EXP_WIDTH + SCALE_MAN_WIDTH + PRECISION;
  localparam int RSP_WIDTH = SYMBOL_WIDTH + 2 * PRECISION + 1;
  localparam int LATENCY   = 4;  // as in power_tb_enc, with the decoder's four stages

  logic                           clk = 1'b0;
  logic                           rst_n = 1'b0;
  logic                           in_valid = 1'b0;
  logic [REQ_WIDTH-1:0]           request = '0;
  logic [REQ_WIDTH-1:0]           stimulus [MAX_COUNT];
  logic [RSP_WIDTH-1:0]           expected [MAX_COUNT];
  int                             count = 0;
  int                             errors = 0;
  int                             escapes = 0;
  string                          stimulus_file, expected_file, vcd_file;
  logic                           out_valid, out_escape;
  logic signed [SYMBOL_WIDTH-1:0] out_symbol;
  logic [PRECISION-1:0]           out_start, out_freq;

  always #(PERIOD_PS / 2 * 1ps) clk = ~clk;

  pans_dec_core_timed dut (
    .clk, .rst_n, .in_valid,
    .in_scale(request[REQ_WIDTH-1 -: SCALE_EXP_WIDTH + SCALE_MAN_WIDTH]), .in_slot(request[PRECISION-1:0]),
    .out_valid, .out_symbol, .out_start, .out_freq, .out_escape
  );

  initial begin
    if (!$value$plusargs("STIMULUS=%s", stimulus_file) || !$value$plusargs("EXPECTED=%s", expected_file)
        || !$value$plusargs("COUNT=%d", count) || count < 1 || count > MAX_COUNT)
      $fatal(1, "power_tb_dec: +STIMULUS, +EXPECTED and +COUNT (1 .. %0d) are required", MAX_COUNT);
    $readmemh(stimulus_file, stimulus, 0, count - 1);
    $readmemh(expected_file, expected, 0, count - 1);
    repeat (4) @(posedge clk);
    rst_n    <= 1'b1;
    in_valid <= 1'b1;
    if ($value$plusargs("VCD=%s", vcd_file)) begin
      $dumpfile(vcd_file);
      $dumpvars(0, dut);
    end
    for (int i = 0; i < count + LATENCY; i++) begin
      if (i < count) request <= stimulus[i];
      @(posedge clk);
      #(PERIOD_PS / 4 * 1ps);
      if (i >= LATENCY && out_escape === 1'b1) escapes++;
      if (i >= LATENCY && (out_valid !== 1'b1
                           || {out_symbol, out_start, out_freq, out_escape} !== expected[i - LATENCY])) begin
        errors++;
        if (errors <= 5) $display("MISMATCH %0d: netlist %h, reference %h", i - LATENCY,
                                  {out_symbol, out_start, out_freq, out_escape}, expected[i - LATENCY]);
      end
    end
    $display("power_tb_dec: %0d transactions, %0d mismatches, %0d escapes", count, errors, escapes);
    if (errors != 0) $fatal(1, "netlist does not match the reference");
    $finish;
  end
endmodule
