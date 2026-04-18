// saturn_field.v — field code -> (start_nib, end_nib) decoder
// Ref: x48ng registers.c start_fields[] / end_fields[].
// -1 entries in the C tables mean "use P register".
`include "saturn_pkg.vh"

module saturn_field (
    input  wire [4:0] code,      // field code 0..18
    input  wire [3:0] p,         // P register (0..15)
    output reg  [3:0] start_nib, // inclusive
    output reg  [3:0] end_nib    // inclusive
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
