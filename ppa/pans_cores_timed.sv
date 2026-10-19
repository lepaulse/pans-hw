// SPDX-License-Identifier: Apache-2.0
// pans_cores_timed.sv: timing wrappers of the cores for the PPA flow, not part of the design.
// Each registers its core's inputs, so every timed path runs register to register.

module pans_enc_core_timed
  import pans_pkg::*;
(
  input  logic                           clk,
  input  logic                           rst_n,
  input  logic                           in_valid,
  input  scale_t                         in_scale,
  input  logic signed [SYMBOL_WIDTH-1:0] in_symbol,
  output logic                           out_valid,
  output logic [PRECISION-1:0]           out_start,
  output logic [PRECISION-1:0]           out_freq,
  output logic                           out_escape
);
  logic                           valid_q;
  scale_t                         scale_q;
  logic signed [SYMBOL_WIDTH-1:0] symbol_q;

  always_ff @(posedge clk) begin
    valid_q  <= in_valid;
    scale_q  <= in_scale;
    symbol_q <= in_symbol;
  end

  pans_enc_core u_core (
    .clk, .rst_n, .in_valid(valid_q), .in_scale(scale_q), .in_symbol(symbol_q),
    .out_valid, .out_start, .out_freq, .out_escape
  );
endmodule

module pans_dec_core_timed
  import pans_pkg::*;
(
  input  logic                           clk,
  input  logic                           rst_n,
  input  logic                           in_valid,
  input  scale_t                         in_scale,
  input  logic [PRECISION-1:0]           in_slot,
  output logic                           out_valid,
  output logic signed [SYMBOL_WIDTH-1:0] out_symbol,
  output logic [PRECISION-1:0]           out_start,
  output logic [PRECISION-1:0]           out_freq,
  output logic                           out_escape
);
  logic                 valid_q;
  scale_t               scale_q;
  logic [PRECISION-1:0] slot_q;

  always_ff @(posedge clk) begin
    valid_q <= in_valid;
    scale_q <= in_scale;
    slot_q  <= in_slot;
  end

  pans_dec_core u_core (
    .clk, .rst_n, .in_valid(valid_q), .in_scale(scale_q), .in_slot(slot_q),
    .out_valid, .out_symbol, .out_start, .out_freq, .out_escape
  );
endmodule
