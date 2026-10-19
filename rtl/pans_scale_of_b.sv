// SPDX-License-Identifier: Apache-2.0
// pans_scale_of_b: the scale input b to the cores' scale code, a lookup in SCALE_TABLE.
// Mirrors scale_of_b of ref/pans1_ref.h bit for bit.
// Specification: SPEC.md §1 and §3.
// Combinational. The stream units register the code.
module pans_scale_of_b
  import pans_pkg::*;
(
  input  logic [B_WIDTH-1:0] b,
  output scale_t             scale
);

  logic [B_WIDTH+B_K-1:0]      wide;   // b over B_K zeros
  logic [B_LEAD_BITS-1:0]      lead;   // the position of b's leading one
  logic [B_K-1:0]              below;  // the B_K bits below it
  logic [SCALE_INDEX_BITS-1:0] index;

  always_comb begin
    wide  = {b, {B_K{1'b0}}};
    lead  = '0;
    below = '0;
    for (int l = 0; l < B_WIDTH; l++) begin
      if (b[l]) begin  // the highest set bit is the last to assign
        lead  = B_LEAD_BITS'(l);
        below = wide[l +: B_K];
      end
    end
    index = {lead, below};
    scale = SCALE_TABLE[index];
  end

  // ---- Elaboration checks
  if (B_LEAD_BITS < 1 || (1 << B_LEAD_BITS) < B_WIDTH || SCALE_INDEX_BITS != B_LEAD_BITS + B_K
      || SCALE_TABLE_SIZE != B_WIDTH * (1 << B_K)) begin : g_check_widths
    $error("pans_scale_of_b: B_LEAD_BITS, SCALE_INDEX_BITS or SCALE_TABLE_SIZE does not match B_WIDTH and B_K");
  end
  // The contract version this lookup implements (SPEC.md §4).
  if (CONTRACT_VERSION != 5) begin : g_check_contract
    $error("pans_scale_of_b: implements contract version 5 (SPEC.md §4)");
  end

endmodule
