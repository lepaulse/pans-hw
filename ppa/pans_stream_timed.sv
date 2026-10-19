// SPDX-License-Identifier: Apache-2.0
// pans_stream_timed.sv: timing wrappers of the stream units for the ASIC PPA flow, not part of the design.
// A register slice on every channel makes every timed path run register to register (STREAM.md §4.1).
// A slice adds one cycle of latency and keeps valid/ready, so a testbench drives a wrapper as it drives the unit.
// The unit is instance u_dut, kept through synthesis, so its cells carry the prefix u_dut/.

// A full register slice, a two-entry skid buffer. Every port is a register's or ends in one.
// The skid register catches the item taken in the cycle the output stalls. Reset clears the valid bits only.
module pans_stream_slice #(
  parameter int WIDTH = 1
) (
  input  logic             clk,
  input  logic             rst_n,
  input  logic             in_valid,
  output logic             in_ready,
  input  logic [WIDTH-1:0] in_data,
  output logic             out_valid,
  input  logic             out_ready,
  output logic [WIDTH-1:0] out_data
);
  logic             main_valid, skid_valid;
  logic [WIDTH-1:0] main_data, skid_data;
  logic             main_load;  // the main register takes the next item (the skid's, else the input's) or empties

  assign in_ready  = !skid_valid;
  assign out_valid = main_valid;
  assign out_data  = main_data;
  assign main_load = out_ready || !main_valid;

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      main_valid <= 1'b0;
      skid_valid <= 1'b0;
    end else if (main_load) begin
      main_valid <= skid_valid || in_valid;
      skid_valid <= 1'b0;
    end else if (in_valid && !skid_valid) begin
      skid_valid <= 1'b1;
    end
  end

  always_ff @(posedge clk) begin
    if (main_load) main_data <= skid_valid ? skid_data : in_data;
    if (!skid_valid) skid_data <= in_data;
  end
endmodule

module pans_dec_stream_timed
  import pans_pkg::*;
#(
  parameter  int N_LANES     = 8,
  parameter  int DATA_BYTES  = 4,
  parameter  int FIFO_WORDS  = 8,
  parameter  int OUT_DEPTH   = 7,
  parameter  int COUNT_WIDTH = 32,
  parameter  bit CLOCK_GATE  = 1,
  localparam int LANE_WIDTH  = (N_LANES > 1) ? $clog2(N_LANES) : 1,
  localparam int POS_WIDTH   = unsigned_width(((longint'(1) <<< COUNT_WIDTH) - 2) / longint'(N_LANES))
) (
  input  logic                           clk,
  input  logic                           rst_n,
  input  logic                           job_in_valid,
  output logic                           job_in_ready,
  input  logic [COUNT_WIDTH-1:0]         job_in_n_sym,
  input  logic [COUNT_WIDTH-1:0]         job_in_n_bytes,
  input  logic                           data_in_valid,
  output logic                           data_in_ready,
  input  logic [8*DATA_BYTES-1:0]        data_in_data,
  input  logic                           scale_in_valid,
  output logic                           scale_in_ready,
  input  logic [LANE_WIDTH-1:0]          scale_in_lane,
  input  logic [B_WIDTH-1:0]             scale_in_b,
  output logic                           sym_out_valid,
  input  logic                           sym_out_ready,
  output logic [LANE_WIDTH-1:0]          sym_out_lane,
  output logic [POS_WIDTH-1:0]           sym_out_t,
  output logic signed [SYMBOL_WIDTH-1:0] sym_out_symbol,
  output logic                           sym_out_escape,
  output logic                           done_out_valid,
  input  logic                           done_out_ready,
  output logic                           done_out_ok,
  output logic [COUNT_WIDTH-1:0]         done_out_n_escapes,
  output logic [COUNT_WIDTH-1:0]         done_out_words_consumed,
  output logic                           done_out_err_words,
  output logic                           done_out_err_header
);
  localparam int JOB_W   = 2 * COUNT_WIDTH;
  localparam int DATA_W  = 8 * DATA_BYTES;
  localparam int SCALE_W = LANE_WIDTH + B_WIDTH;
  localparam int SYM_W   = LANE_WIDTH + POS_WIDTH + SYMBOL_WIDTH + 1;
  localparam int DONE_W  = 1 + 2 * COUNT_WIDTH + 2;

  // The unit's side of the slices
  logic                           job_valid, job_ready, data_valid, data_ready, scale_valid, scale_ready;
  logic                           sym_valid, sym_ready, done_valid, done_ready;
  logic [COUNT_WIDTH-1:0]         job_n_sym, job_n_bytes;
  logic [DATA_W-1:0]              data_data;
  logic [LANE_WIDTH-1:0]          scale_lane;
  logic [B_WIDTH-1:0]             scale_b;
  logic [LANE_WIDTH-1:0]          sym_lane;
  logic [POS_WIDTH-1:0]           sym_t;
  logic signed [SYMBOL_WIDTH-1:0] sym_symbol;
  logic                           sym_escape;
  logic                           done_ok, done_err_words, done_err_header;
  logic [COUNT_WIDTH-1:0]         done_n_escapes, done_words_consumed;

  pans_stream_slice #(.WIDTH(JOB_W)) s_job (
    .clk, .rst_n, .in_valid(job_in_valid), .in_ready(job_in_ready), .in_data({job_in_n_sym, job_in_n_bytes}),
    .out_valid(job_valid), .out_ready(job_ready), .out_data({job_n_sym, job_n_bytes}));
  pans_stream_slice #(.WIDTH(DATA_W)) s_data (
    .clk, .rst_n, .in_valid(data_in_valid), .in_ready(data_in_ready), .in_data(data_in_data),
    .out_valid(data_valid), .out_ready(data_ready), .out_data(data_data));
  pans_stream_slice #(.WIDTH(SCALE_W)) s_scale (
    .clk, .rst_n, .in_valid(scale_in_valid), .in_ready(scale_in_ready), .in_data({scale_in_lane, scale_in_b}),
    .out_valid(scale_valid), .out_ready(scale_ready), .out_data({scale_lane, scale_b}));
  pans_stream_slice #(.WIDTH(SYM_W)) s_sym (
    .clk, .rst_n, .in_valid(sym_valid), .in_ready(sym_ready), .in_data({sym_lane, sym_t, sym_symbol, sym_escape}),
    .out_valid(sym_out_valid), .out_ready(sym_out_ready),
    .out_data({sym_out_lane, sym_out_t, sym_out_symbol, sym_out_escape}));
  pans_stream_slice #(.WIDTH(DONE_W)) s_done (
    .clk, .rst_n, .in_valid(done_valid), .in_ready(done_ready),
    .in_data({done_ok, done_n_escapes, done_words_consumed, done_err_words, done_err_header}),
    .out_valid(done_out_valid), .out_ready(done_out_ready),
    .out_data({done_out_ok, done_out_n_escapes, done_out_words_consumed, done_out_err_words, done_out_err_header}));

  pans_dec_stream #(
    .N_LANES(N_LANES), .DATA_BYTES(DATA_BYTES), .FIFO_WORDS(FIFO_WORDS), .OUT_DEPTH(OUT_DEPTH),
    .COUNT_WIDTH(COUNT_WIDTH), .CLOCK_GATE(CLOCK_GATE)
  ) u_dut (
    .clk, .rst_n,
    .job_in_valid(job_valid), .job_in_ready(job_ready), .job_in_n_sym(job_n_sym), .job_in_n_bytes(job_n_bytes),
    .data_in_valid(data_valid), .data_in_ready(data_ready), .data_in_data(data_data),
    .scale_in_valid(scale_valid), .scale_in_ready(scale_ready), .scale_in_lane(scale_lane),
    .scale_in_b(scale_b),
    .sym_out_valid(sym_valid), .sym_out_ready(sym_ready), .sym_out_lane(sym_lane), .sym_out_t(sym_t),
    .sym_out_symbol(sym_symbol), .sym_out_escape(sym_escape),
    .done_out_valid(done_valid), .done_out_ready(done_ready), .done_out_ok(done_ok),
    .done_out_n_escapes(done_n_escapes), .done_out_words_consumed(done_words_consumed),
    .done_out_err_words(done_err_words), .done_out_err_header(done_err_header));
endmodule

module pans_enc_stream_timed
  import pans_pkg::*;
#(
  parameter  int N_LANES        = 8,
  parameter  int DIV_BITS       = 2,
  parameter  int DIV_STEP_BITS  = 1,
  parameter  int DIV_FIRST_BITS = DIV_STEP_BITS,
  parameter  int OUT_DEPTH      = 8,
  parameter  int COUNT_WIDTH    = 32,
  parameter  bit CLOCK_GATE     = 1,
  localparam int LANE_WIDTH     = (N_LANES > 1) ? $clog2(N_LANES) : 1
) (
  input  logic                           clk,
  input  logic                           rst_n,
  input  logic                           job_in_valid,
  output logic                           job_in_ready,
  input  logic [COUNT_WIDTH-1:0]         job_in_n_sym,
  input  logic                           sym_in_valid,
  output logic                           sym_in_ready,
  input  logic [LANE_WIDTH-1:0]          sym_in_lane,
  input  logic signed [SYMBOL_WIDTH-1:0] sym_in_symbol,
  input  logic [B_WIDTH-1:0]             sym_in_b,
  output logic                           word_out_valid,
  input  logic                           word_out_ready,
  output logic [WORD_WIDTH-1:0]          word_out_word,
  output logic                           done_out_valid,
  input  logic                           done_out_ready,
  output logic [LANE_WIDTH-1:0]          done_out_lane,
  output logic                           done_out_last,
  output logic [STATE_WIDTH-1:0]         done_out_state,
  output logic [COUNT_WIDTH:0]           done_out_n_words,
  output logic [COUNT_WIDTH-1:0]         done_out_n_escapes,
  output logic                           done_out_err_order
);
  localparam int SYM_W  = LANE_WIDTH + SYMBOL_WIDTH + B_WIDTH;
  localparam int DONE_W = LANE_WIDTH + 1 + STATE_WIDTH + (COUNT_WIDTH + 1) + COUNT_WIDTH + 1;

  logic                           job_valid, job_ready, sym_valid, sym_ready, word_valid, word_ready;
  logic                           done_valid, done_ready;
  logic [COUNT_WIDTH-1:0]         job_n_sym;
  logic [LANE_WIDTH-1:0]          sym_lane;
  logic signed [SYMBOL_WIDTH-1:0] sym_symbol;
  logic [B_WIDTH-1:0]             sym_b;
  logic [WORD_WIDTH-1:0]          word_word;
  logic [LANE_WIDTH-1:0]          done_lane;
  logic                           done_last, done_err_order;
  logic [STATE_WIDTH-1:0]         done_state;
  logic [COUNT_WIDTH:0]           done_n_words;
  logic [COUNT_WIDTH-1:0]         done_n_escapes;

  pans_stream_slice #(.WIDTH(COUNT_WIDTH)) s_job (
    .clk, .rst_n, .in_valid(job_in_valid), .in_ready(job_in_ready), .in_data(job_in_n_sym),
    .out_valid(job_valid), .out_ready(job_ready), .out_data(job_n_sym));
  pans_stream_slice #(.WIDTH(SYM_W)) s_sym (
    .clk, .rst_n, .in_valid(sym_in_valid), .in_ready(sym_in_ready),
    .in_data({sym_in_lane, sym_in_symbol, sym_in_b}),
    .out_valid(sym_valid), .out_ready(sym_ready), .out_data({sym_lane, sym_symbol, sym_b}));
  pans_stream_slice #(.WIDTH(WORD_WIDTH)) s_word (
    .clk, .rst_n, .in_valid(word_valid), .in_ready(word_ready), .in_data(word_word),
    .out_valid(word_out_valid), .out_ready(word_out_ready), .out_data(word_out_word));
  pans_stream_slice #(.WIDTH(DONE_W)) s_done (
    .clk, .rst_n, .in_valid(done_valid), .in_ready(done_ready),
    .in_data({done_lane, done_last, done_state, done_n_words, done_n_escapes, done_err_order}),
    .out_valid(done_out_valid), .out_ready(done_out_ready),
    .out_data({done_out_lane, done_out_last, done_out_state, done_out_n_words, done_out_n_escapes,
               done_out_err_order}));

  pans_enc_stream #(
    .N_LANES(N_LANES), .DIV_BITS(DIV_BITS), .DIV_STEP_BITS(DIV_STEP_BITS), .DIV_FIRST_BITS(DIV_FIRST_BITS),
    .OUT_DEPTH(OUT_DEPTH), .COUNT_WIDTH(COUNT_WIDTH), .CLOCK_GATE(CLOCK_GATE)
  ) u_dut (
    .clk, .rst_n,
    .job_in_valid(job_valid), .job_in_ready(job_ready), .job_in_n_sym(job_n_sym),
    .sym_in_valid(sym_valid), .sym_in_ready(sym_ready), .sym_in_lane(sym_lane), .sym_in_symbol(sym_symbol),
    .sym_in_b(sym_b),
    .word_out_valid(word_valid), .word_out_ready(word_ready), .word_out_word(word_word),
    .done_out_valid(done_valid), .done_out_ready(done_ready), .done_out_lane(done_lane), .done_out_last(done_last),
    .done_out_state(done_state), .done_out_n_words(done_n_words), .done_out_n_escapes(done_n_escapes),
    .done_out_err_order(done_err_order));
endmodule

