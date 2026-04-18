/**
 * @file saturn_field.v
 * @brief Decodes a 5-bit Saturn field code into its (start_nib, end_nib)
 *        inclusive nibble range.
 *
 * The HP Saturn CPU operates on 64-bit registers as 16 nibbles; most
 * arithmetic, logic, and data-move instructions apply only to a
 * contiguous slice of those nibbles called a "field". The field code
 * table mirrors the `start_fields[]` / `end_fields[]` arrays in
 * `x48ng/src/core/registers.c` — entries marked "-1" in the C source
 * ("use P register") correspond to the @ref FC_P / @ref FC_WP codes here.
 *
 * Codes 8..14 are mirrors of codes 0..6 (same start/end), preserved for
 * bit-exact opcode decoding fidelity.
 */
`include "saturn_pkg.vh"

/**
 * @brief Saturn field decoder (combinational).
 *
 * @param code      5-bit field code; see saturn_pkg.vh `FC_*` constants.
 * @param p         Current P register (0..15). Used by @ref FC_P and
 *                  @ref FC_WP to compute a P-dependent slice.
 * @param start_nib First in-field nibble index (inclusive, 0..15).
 * @param end_nib   Last  in-field nibble index (inclusive, 0..15).
 */
module saturn_field (
    input  wire [4:0] code,      ///< field code 0..18 (see FC_* in saturn_pkg.vh)
    input  wire [3:0] p,         ///< P register value (used by FC_P / FC_WP)
    output reg  [3:0] start_nib, ///< inclusive start of field
    output reg  [3:0] end_nib    ///< inclusive end of field
);
    always @(*) begin
        case (code)
            5'd0:  begin start_nib = p;      end_nib = p;      end  // FC_P
            5'd1:  begin start_nib = 4'd0;   end_nib = p;      end  // FC_WP
            5'd2:  begin start_nib = 4'd2;   end_nib = 4'd2;   end  // FC_XS
            5'd3:  begin start_nib = 4'd0;   end_nib = 4'd2;   end  // FC_X
            5'd4:  begin start_nib = 4'd15;  end_nib = 4'd15;  end  // FC_S
            5'd5:  begin start_nib = 4'd3;   end_nib = 4'd14;  end  // FC_M
            5'd6:  begin start_nib = 4'd0;   end_nib = 4'd1;   end  // FC_B
            5'd7:  begin start_nib = 4'd0;   end_nib = 4'd15;  end  // FC_W
            5'd8:  begin start_nib = p;      end_nib = p;      end  // FC_P  mirror
            5'd9:  begin start_nib = 4'd0;   end_nib = p;      end  // FC_WP mirror
            5'd10: begin start_nib = 4'd2;   end_nib = 4'd2;   end  // FC_XS mirror
            5'd11: begin start_nib = 4'd0;   end_nib = 4'd2;   end  // FC_X  mirror
            5'd12: begin start_nib = 4'd15;  end_nib = 4'd15;  end  // FC_S  mirror
            5'd13: begin start_nib = 4'd3;   end_nib = 4'd14;  end  // FC_M  mirror
            5'd14: begin start_nib = 4'd0;   end_nib = 4'd1;   end  // FC_B  mirror
            5'd15: begin start_nib = 4'd0;   end_nib = 4'd4;   end  // FC_A
            5'd16: begin start_nib = 4'd0;   end_nib = 4'd3;   end  // FC_IN
            5'd17: begin start_nib = 4'd0;   end_nib = 4'd2;   end  // FC_OUT
            5'd18: begin start_nib = 4'd0;   end_nib = 4'd0;   end  // FC_OUTS
            default: begin start_nib = 4'd0; end_nib = 4'd15;  end
        endcase
    end
endmodule
