// ============================================================
// color_saved_display : memory SAVED by the colour compression, in %,
// on the DE2-115 seven-segment displays.
//
//   HEX3 HEX2 : saved percentage, whole part
//   HEX1      : blank (acts as the decimal point)
//   HEX0      : saved percentage, first decimal
//   HEX7..HEX4: off
//   Example:  "96 4"  ->  96.4 % of the memory was saved
//
//   original colour image = 320 x 320 x 24 bits = 2,457,600 bits
//   compressed (in tenths of %) = total_bits x 1000 / 2,457,600
//                               ~ total_bits x 3413 / 2^23
//   (multiply + shift instead of a divider; error < 0.01 %)
//   saved = 100 % - compressed
//   While the engine is still working, dashes are shown.
// ============================================================
module color_saved_display (
    input  wire        done,          // engine finished
    input  wire [23:0] total_bits,    // compressed size of Y + Cb + Cr in bits
    output wire [6:0]  HEX0, HEX1, HEX2, HEX3, HEX4, HEX5, HEX6, HEX7
);

    // ---------- 1. Percentages in tenths (e.g. 964 = 96.4 %) ----------
    wire [39:0] prod          = {16'd0, total_bits} * 40'd3413 + 40'd4194304;  // + 2^22 rounds
    wire [39:0] comp_x10_full = prod >> 23;
    wire [9:0]  comp_x10      = (comp_x10_full > 40'd1000) ? 10'd1000 : comp_x10_full[9:0];
    wire [9:0]  saved_x10     = 10'd1000 - comp_x10;

    // ---------- 2. Split into digits ----------
    wire [3:0] d_tens   = saved_x10 / 100;
    wire [3:0] d_ones   = (saved_x10 / 10) % 10;
    wire [3:0] d_tenths = saved_x10 % 10;

    // ---------- 3. Digit -> 7-segment pattern (segments are ON when 0) ----------
    localparam [6:0] SEG_BLANK = 7'b1111111;
    localparam [6:0] SEG_DASH  = 7'b0111111;

    function [6:0] seg;
        input [3:0] d;
        case (d)
            4'd0: seg = 7'b1000000;
            4'd1: seg = 7'b1111001;
            4'd2: seg = 7'b0100100;
            4'd3: seg = 7'b0110000;
            4'd4: seg = 7'b0011001;
            4'd5: seg = 7'b0010010;
            4'd6: seg = 7'b0000010;
            4'd7: seg = 7'b1111000;
            4'd8: seg = 7'b0000000;
            4'd9: seg = 7'b0010000;
            default: seg = SEG_BLANK;
        endcase
    endfunction

    assign HEX3 = !done ? SEG_DASH : ((d_tens == 4'd0) ? SEG_BLANK : seg(d_tens));
    assign HEX2 = !done ? SEG_DASH : seg(d_ones);
    assign HEX1 = !done ? SEG_DASH : SEG_BLANK;
    assign HEX0 = !done ? SEG_DASH : seg(d_tenths);

    assign HEX4 = SEG_BLANK;
    assign HEX5 = SEG_BLANK;
    assign HEX6 = SEG_BLANK;
    assign HEX7 = SEG_BLANK;

endmodule
