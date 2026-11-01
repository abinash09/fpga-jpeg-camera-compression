// ============================================================
// color_dct_engine : JPEG-style COLOUR compression
//
// Input pixels are RGB565. Each pixel is split into
//   Y  (brightness)   = 0.299 R + 0.587 G + 0.114 B
//   Cb (blue colour)  = -0.169 R - 0.331 G + 0.500 B   (+128)
//   Cr (red colour)   =  0.500 R - 0.419 G - 0.081 B   (+128)
//
// The engine works in 3 passes (one per channel):
//   pass 0 : Y  at full size  320 x 320 -> 40 x 40 = 1600 blocks of 8x8
//   pass 1 : Cb at half size  160 x 160 -> 20 x 20 =  400 blocks of 8x8
//   pass 2 : Cr at half size  160 x 160 -> 20 x 20 =  400 blocks of 8x8
//   (half size = every 2x2 group of pixels is averaged into one colour
//    sample, "4:2:0 chroma subsampling" - the eye hardly notices)
//
// For every 8x8 block:
//   LOAD -> DCT (F = C*X*C') -> QUANTIZE in zigzag order + count bits
//        -> DEQUANT -> IDCT (X' = C'*F*C) -> WRITE (+128, clip 0..255)
//
// Bits counted per block (JPEG-style run-length coding, fixed 8-bit symbols):
//   DC  = 4-bit size + amplitude bits of (DC - previous DC)
//   AC  = 8-bit (run,size) symbol + amplitude bits for each non-zero,
//         8-bit ZRL for every 16 zeros skipped, 8-bit EOB at the end
//
// Quantization: JPEG luminance table for Y, JPEG chrominance table for
// Cb/Cr, both multiplied by 1, 2, 4 or 8 (quality input = SW[1:0]).
// ============================================================
module color_dct_engine (
    input  wire        clk,
    input  wire        reset,
    input  wire [1:0]  quality,     // 0 = light ... 3 = heavy compression
    output reg  [16:0] rd_addr,     // pixel address in the original image
    input  wire [15:0] rd_rgb,      // RGB565 pixel (arrives 1 clock later)
    output reg  [16:0] wr_addr,     // address in the result memory
    output reg  [7:0]  wr_data,     // value to write
    output reg         we,          // write enable
    output reg  [1:0]  wr_chan,     // which result memory: 0 = Y, 1 = Cb, 2 = Cr
    output reg         done,        // 1 when all 3 channels are finished
    output reg  [23:0] total_bits   // compressed size of Y + Cb + Cr in bits
);

    // ---------------- states ----------------
    localparam S_START = 3'd0,
               S_LOAD  = 3'd1,
               S_MUL   = 3'd2,
               S_QUANT = 3'd3,
               S_WRITE = 3'd4,
               S_NEXT  = 3'd5,
               S_DONE  = 3'd6;

    reg [2:0] state;
    reg [1:0] chan;          // 0 = Y, 1 = Cb, 2 = Cr
    reg [1:0] mode;          // which matrix multiply (0..3)
    reg [8:0] cnt;           // step counter inside a state
    reg [5:0] bx, by;        // block column / row
    reg [1:0] shift;         // quality latched at start
    reg signed [31:0] acc;   // multiply-accumulate register
    reg signed [12:0] prev_dc;   // DC value of the previous block
    reg [5:0]         run;       // zeros seen since the last non-zero

    reg signed [15:0] bufA [0:63];   // 8x8 working buffer A
    reg signed [15:0] bufB [0:63];   // 8x8 working buffer B

    wire is_y = (chan == 2'd0);

    // ---------------- tables ----------------
    // DCT matrix C[u][i] * 4096, index = {u, i}
    function signed [13:0] coef;
        input [5:0] n;
        case (n)
            6'd0:  coef =  14'sd1448; 6'd1:  coef =  14'sd1448; 6'd2:  coef =  14'sd1448; 6'd3:  coef =  14'sd1448;
            6'd4:  coef =  14'sd1448; 6'd5:  coef =  14'sd1448; 6'd6:  coef =  14'sd1448; 6'd7:  coef =  14'sd1448;
            6'd8:  coef =  14'sd2009; 6'd9:  coef =  14'sd1703; 6'd10: coef =  14'sd1138; 6'd11: coef =  14'sd400;
            6'd12: coef = -14'sd400;  6'd13: coef = -14'sd1138; 6'd14: coef = -14'sd1703; 6'd15: coef = -14'sd2009;
            6'd16: coef =  14'sd1892; 6'd17: coef =  14'sd784;  6'd18: coef = -14'sd784;  6'd19: coef = -14'sd1892;
            6'd20: coef = -14'sd1892; 6'd21: coef = -14'sd784;  6'd22: coef =  14'sd784;  6'd23: coef =  14'sd1892;
            6'd24: coef =  14'sd1703; 6'd25: coef = -14'sd400;  6'd26: coef = -14'sd2009; 6'd27: coef = -14'sd1138;
            6'd28: coef =  14'sd1138; 6'd29: coef =  14'sd2009; 6'd30: coef =  14'sd400;  6'd31: coef = -14'sd1703;
            6'd32: coef =  14'sd1448; 6'd33: coef = -14'sd1448; 6'd34: coef = -14'sd1448; 6'd35: coef =  14'sd1448;
            6'd36: coef =  14'sd1448; 6'd37: coef = -14'sd1448; 6'd38: coef = -14'sd1448; 6'd39: coef =  14'sd1448;
            6'd40: coef =  14'sd1138; 6'd41: coef = -14'sd2009; 6'd42: coef =  14'sd400;  6'd43: coef =  14'sd1703;
            6'd44: coef = -14'sd1703; 6'd45: coef = -14'sd400;  6'd46: coef =  14'sd2009; 6'd47: coef = -14'sd1138;
            6'd48: coef =  14'sd784;  6'd49: coef = -14'sd1892; 6'd50: coef =  14'sd1892; 6'd51: coef = -14'sd784;
            6'd52: coef = -14'sd784;  6'd53: coef =  14'sd1892; 6'd54: coef = -14'sd1892; 6'd55: coef =  14'sd784;
            6'd56: coef =  14'sd400;  6'd57: coef = -14'sd1138; 6'd58: coef =  14'sd1703; 6'd59: coef = -14'sd2009;
            6'd60: coef =  14'sd2009; 6'd61: coef = -14'sd1703; 6'd62: coef =  14'sd1138; 6'd63: coef = -14'sd400;
        endcase
    endfunction

    // JPEG standard luminance quantization table, index = {u, v}
    function [6:0] qtab;
        input [5:0] n;
        case (n)
            6'd0:  qtab = 7'd16;  6'd1:  qtab = 7'd11;  6'd2:  qtab = 7'd10;  6'd3:  qtab = 7'd16;
            6'd4:  qtab = 7'd24;  6'd5:  qtab = 7'd40;  6'd6:  qtab = 7'd51;  6'd7:  qtab = 7'd61;
            6'd8:  qtab = 7'd12;  6'd9:  qtab = 7'd12;  6'd10: qtab = 7'd14;  6'd11: qtab = 7'd19;
            6'd12: qtab = 7'd26;  6'd13: qtab = 7'd58;  6'd14: qtab = 7'd60;  6'd15: qtab = 7'd55;
            6'd16: qtab = 7'd14;  6'd17: qtab = 7'd13;  6'd18: qtab = 7'd16;  6'd19: qtab = 7'd24;
            6'd20: qtab = 7'd40;  6'd21: qtab = 7'd57;  6'd22: qtab = 7'd69;  6'd23: qtab = 7'd56;
            6'd24: qtab = 7'd14;  6'd25: qtab = 7'd17;  6'd26: qtab = 7'd22;  6'd27: qtab = 7'd29;
            6'd28: qtab = 7'd51;  6'd29: qtab = 7'd87;  6'd30: qtab = 7'd80;  6'd31: qtab = 7'd62;
            6'd32: qtab = 7'd18;  6'd33: qtab = 7'd22;  6'd34: qtab = 7'd37;  6'd35: qtab = 7'd56;
            6'd36: qtab = 7'd68;  6'd37: qtab = 7'd109; 6'd38: qtab = 7'd103; 6'd39: qtab = 7'd77;
            6'd40: qtab = 7'd24;  6'd41: qtab = 7'd35;  6'd42: qtab = 7'd55;  6'd43: qtab = 7'd64;
            6'd44: qtab = 7'd81;  6'd45: qtab = 7'd104; 6'd46: qtab = 7'd113; 6'd47: qtab = 7'd92;
            6'd48: qtab = 7'd49;  6'd49: qtab = 7'd64;  6'd50: qtab = 7'd78;  6'd51: qtab = 7'd87;
            6'd52: qtab = 7'd103; 6'd53: qtab = 7'd121; 6'd54: qtab = 7'd120; 6'd55: qtab = 7'd101;
            6'd56: qtab = 7'd72;  6'd57: qtab = 7'd92;  6'd58: qtab = 7'd95;  6'd59: qtab = 7'd98;
            6'd60: qtab = 7'd112; 6'd61: qtab = 7'd100; 6'd62: qtab = 7'd103; 6'd63: qtab = 7'd99;
        endcase
    endfunction

    // 65536 / Q  (so "divide by Q" becomes "multiply, then shift right 16")
    function [12:0] rtab;
        input [5:0] n;
        case (n)
            6'd0:  rtab = 13'd4096; 6'd1:  rtab = 13'd5958; 6'd2:  rtab = 13'd6554; 6'd3:  rtab = 13'd4096;
            6'd4:  rtab = 13'd2731; 6'd5:  rtab = 13'd1638; 6'd6:  rtab = 13'd1285; 6'd7:  rtab = 13'd1074;
            6'd8:  rtab = 13'd5461; 6'd9:  rtab = 13'd5461; 6'd10: rtab = 13'd4681; 6'd11: rtab = 13'd3449;
            6'd12: rtab = 13'd2521; 6'd13: rtab = 13'd1130; 6'd14: rtab = 13'd1092; 6'd15: rtab = 13'd1192;
            6'd16: rtab = 13'd4681; 6'd17: rtab = 13'd5041; 6'd18: rtab = 13'd4096; 6'd19: rtab = 13'd2731;
            6'd20: rtab = 13'd1638; 6'd21: rtab = 13'd1150; 6'd22: rtab = 13'd950;  6'd23: rtab = 13'd1170;
            6'd24: rtab = 13'd4681; 6'd25: rtab = 13'd3855; 6'd26: rtab = 13'd2979; 6'd27: rtab = 13'd2260;
            6'd28: rtab = 13'd1285; 6'd29: rtab = 13'd753;  6'd30: rtab = 13'd819;  6'd31: rtab = 13'd1057;
            6'd32: rtab = 13'd3641; 6'd33: rtab = 13'd2979; 6'd34: rtab = 13'd1771; 6'd35: rtab = 13'd1170;
            6'd36: rtab = 13'd964;  6'd37: rtab = 13'd601;  6'd38: rtab = 13'd636;  6'd39: rtab = 13'd851;
            6'd40: rtab = 13'd2731; 6'd41: rtab = 13'd1872; 6'd42: rtab = 13'd1192; 6'd43: rtab = 13'd1024;
            6'd44: rtab = 13'd809;  6'd45: rtab = 13'd630;  6'd46: rtab = 13'd580;  6'd47: rtab = 13'd712;
            6'd48: rtab = 13'd1337; 6'd49: rtab = 13'd1024; 6'd50: rtab = 13'd840;  6'd51: rtab = 13'd753;
            6'd52: rtab = 13'd636;  6'd53: rtab = 13'd542;  6'd54: rtab = 13'd546;  6'd55: rtab = 13'd649;
            6'd56: rtab = 13'd910;  6'd57: rtab = 13'd712;  6'd58: rtab = 13'd690;  6'd59: rtab = 13'd669;
            6'd60: rtab = 13'd585;  6'd61: rtab = 13'd655;  6'd62: rtab = 13'd636;  6'd63: rtab = 13'd662;
        endcase
    endfunction

    // zigzag order: position 0..63 -> index {u, v} in the 8x8 block
    function [5:0] zigzag;
        input [5:0] p;
        case (p)
            6'd0:  zigzag = 6'd0;   6'd1:  zigzag = 6'd1;   6'd2:  zigzag = 6'd8;   6'd3:  zigzag = 6'd16;
            6'd4:  zigzag = 6'd9;   6'd5:  zigzag = 6'd2;   6'd6:  zigzag = 6'd3;   6'd7:  zigzag = 6'd10;
            6'd8:  zigzag = 6'd17;  6'd9:  zigzag = 6'd24;  6'd10: zigzag = 6'd32;  6'd11: zigzag = 6'd25;
            6'd12: zigzag = 6'd18;  6'd13: zigzag = 6'd11;  6'd14: zigzag = 6'd4;   6'd15: zigzag = 6'd5;
            6'd16: zigzag = 6'd12;  6'd17: zigzag = 6'd19;  6'd18: zigzag = 6'd26;  6'd19: zigzag = 6'd33;
            6'd20: zigzag = 6'd40;  6'd21: zigzag = 6'd48;  6'd22: zigzag = 6'd41;  6'd23: zigzag = 6'd34;
            6'd24: zigzag = 6'd27;  6'd25: zigzag = 6'd20;  6'd26: zigzag = 6'd13;  6'd27: zigzag = 6'd6;
            6'd28: zigzag = 6'd7;   6'd29: zigzag = 6'd14;  6'd30: zigzag = 6'd21;  6'd31: zigzag = 6'd28;
            6'd32: zigzag = 6'd35;  6'd33: zigzag = 6'd42;  6'd34: zigzag = 6'd49;  6'd35: zigzag = 6'd56;
            6'd36: zigzag = 6'd57;  6'd37: zigzag = 6'd50;  6'd38: zigzag = 6'd43;  6'd39: zigzag = 6'd36;
            6'd40: zigzag = 6'd29;  6'd41: zigzag = 6'd22;  6'd42: zigzag = 6'd15;  6'd43: zigzag = 6'd23;
            6'd44: zigzag = 6'd30;  6'd45: zigzag = 6'd37;  6'd46: zigzag = 6'd44;  6'd47: zigzag = 6'd51;
            6'd48: zigzag = 6'd58;  6'd49: zigzag = 6'd59;  6'd50: zigzag = 6'd52;  6'd51: zigzag = 6'd45;
            6'd52: zigzag = 6'd38;  6'd53: zigzag = 6'd31;  6'd54: zigzag = 6'd39;  6'd55: zigzag = 6'd46;
            6'd56: zigzag = 6'd53;  6'd57: zigzag = 6'd60;  6'd58: zigzag = 6'd61;  6'd59: zigzag = 6'd54;
            6'd60: zigzag = 6'd47;  6'd61: zigzag = 6'd55;  6'd62: zigzag = 6'd62;  6'd63: zigzag = 6'd63;
        endcase
    endfunction

    // how many bits are needed to write the number v (0 -> 0, 1 -> 1, 2..3 -> 2, 4..7 -> 3, ...)
    function [3:0] nbits;
        input [13:0] v;
        integer t;
        begin
            nbits = 4'd0;
            for (t = 0; t < 14; t = t + 1)
                if (v[t]) nbits = t + 1;
        end
    endfunction

    // JPEG standard CHROMINANCE quantization table (all other entries are 99)
    function [6:0] qtab_c;
        input [5:0] n;
        case (n)
            6'd0:  qtab_c = 7'd17;  6'd1:  qtab_c = 7'd18;  6'd2:  qtab_c = 7'd24;  6'd3:  qtab_c = 7'd47;
            6'd8:  qtab_c = 7'd18;  6'd9:  qtab_c = 7'd21;  6'd10: qtab_c = 7'd26;  6'd11: qtab_c = 7'd66;
            6'd16: qtab_c = 7'd24;  6'd17: qtab_c = 7'd26;  6'd18: qtab_c = 7'd56;
            6'd24: qtab_c = 7'd47;  6'd25: qtab_c = 7'd66;
            default: qtab_c = 7'd99;
        endcase
    endfunction

    // 65536 / Q for the chrominance table
    function [12:0] rtab_c;
        input [5:0] n;
        case (n)
            6'd0:  rtab_c = 13'd3855; 6'd1:  rtab_c = 13'd3641; 6'd2:  rtab_c = 13'd2731; 6'd3:  rtab_c = 13'd1394;
            6'd8:  rtab_c = 13'd3641; 6'd9:  rtab_c = 13'd3121; 6'd10: rtab_c = 13'd2521; 6'd11: rtab_c = 13'd993;
            6'd16: rtab_c = 13'd2731; 6'd17: rtab_c = 13'd2521; 6'd18: rtab_c = 13'd1170;
            6'd24: rtab_c = 13'd1394; 6'd25: rtab_c = 13'd993;
            default: rtab_c = 13'd662;
        endcase
    endfunction

    // ---------------- RGB565 -> Y / Cb / Cr ----------------
    wire [7:0] r8 = {rd_rgb[15:11], rd_rgb[15:13]};   // 5/6/5 bits -> 8 bits
    wire [7:0] g8 = {rd_rgb[10:5],  rd_rgb[10:9]};
    wire [7:0] b8 = {rd_rgb[4:0],   rd_rgb[4:2]};
    wire signed [17:0] rs = {10'd0, r8};
    wire signed [17:0] gs = {10'd0, g8};
    wire signed [17:0] bs = {10'd0, b8};

    // x256 integer versions of the JPEG colour formulas
    wire signed [17:0] y_t  =  18'sd77  * rs + 18'sd150 * gs + 18'sd29  * bs;
    wire signed [17:0] cb_t = -18'sd43  * rs - 18'sd85  * gs + 18'sd128 * bs;
    wire signed [17:0] cr_t =  18'sd128 * rs - 18'sd107 * gs - 18'sd21  * bs;

    wire signed [17:0] y_r  = (y_t  + 18'sd128) >>> 8;      // 0..255
    wire signed [17:0] cb_r = (cb_t + 18'sd128) >>> 8;      // about -128..127 (= Cb - 128)
    wire signed [17:0] cr_r = (cr_t + 18'sd128) >>> 8;      // about -128..127 (= Cr - 128)

    wire signed [15:0] y_val = $signed(y_r[15:0]) - 16'sd128;              // level shift
    wire signed [15:0] c_val = (chan == 2'd1) ? $signed(cb_r[15:0]) : $signed(cr_r[15:0]);

    // ---------------- addresses ----------------
    // Y pass   : 8x8 pixels per block,   pixel (i,j) = cnt[5:3], cnt[2:0]
    // Cb/Cr pass: 16x16 pixels per block, pixel (i,j) = cnt[7:4], cnt[3:0]
    wire [8:0] y_row = {by, 3'b000}       + {6'd0, cnt[5:3]};
    wire [8:0] y_col = {bx, 3'b000}       + {6'd0, cnt[2:0]};
    wire [8:0] p_row = {by[4:0], 4'b0000} + {5'd0, cnt[7:4]};
    wire [8:0] p_col = {bx[4:0], 4'b0000} + {5'd0, cnt[3:0]};
    wire [8:0] rrow  = is_y ? y_row : p_row;
    wire [8:0] rcol  = is_y ? y_col : p_col;

    always @(*) rd_addr = ({8'd0, rrow} << 8) + ({8'd0, rrow} << 6) + {8'd0, rcol};   // row*320+col

    // write addresses: Y image is 320 wide, Cb/Cr images are 160 wide
    wire [16:0] y_wr_addr = ({8'd0, y_row} << 8) + ({8'd0, y_row} << 6) + {8'd0, y_col};
    wire [7:0]  c_row     = {by[4:0], 3'b000} + {5'd0, cnt[5:3]};       // 0..159
    wire [7:0]  c_col     = {bx[4:0], 3'b000} + {5'd0, cnt[2:0]};
    wire [16:0] c_wr_addr = ({9'd0, c_row} << 7) + ({9'd0, c_row} << 5) + {9'd0, c_col};  // row*160+col

    // ---------------- LOAD helpers ----------------
    wire [5:0] cnt_m1 = cnt[5:0] - 6'd1;              // Y: data arriving now is pixel cnt-1
    wire [7:0] p_m1   = cnt[7:0] - 8'd1;              // Cb/Cr: pixel cnt-1 of the 16x16 area
    wire [5:0] c_idx  = {p_m1[7:5], p_m1[3:1]};       // colour sample = (row/2, col/2)
    wire signed [15:0] c_acc = bufA[c_idx] + c_val;   // running sum of the 2x2 group
    wire signed [15:0] c_avg = (c_acc + 16'sd2) >>> 2; // average of 4

    // ---------------- matrix multiply datapath ----------------
    //   mode 0 : bufB = C  * bufA       (DCT, step 1)
    //   mode 1 : bufA = bufB * C'       (DCT, step 2)
    //   mode 2 : bufB = C' * bufA       (IDCT, step 1)
    //   mode 3 : bufA = bufB * C        (IDCT, step 2)
    wire [2:0] a = cnt[8:6];
    wire [2:0] b = cnt[5:3];
    wire [2:0] k = cnt[2:0];

    reg signed [13:0] c_op;
    reg signed [15:0] d_op;
    always @(*) begin
        case (mode)
            2'd0:    begin c_op = coef({a, k}); d_op = bufA[{k, b}]; end
            2'd1:    begin c_op = coef({b, k}); d_op = bufB[{a, k}]; end
            2'd2:    begin c_op = coef({k, a}); d_op = bufA[{k, b}]; end
            default: begin c_op = coef({k, b}); d_op = bufB[{a, k}]; end
        endcase
    end

    wire signed [29:0] prod     = d_op * c_op;
    wire signed [31:0] acc_next = (k == 3'd0) ? prod : acc + prod;
    wire signed [31:0] acc_rnd  = acc_next + 32'sd2048;
    wire signed [15:0] mul_out  = acc_rnd[27:12];               // divide by 4096

    // ---------------- quantize + dequantize (zigzag order) ----------------
    wire [5:0]         zz_n   = zigzag(cnt[5:0]);
    wire [6:0]         q_tab  = is_y ? qtab(zz_n) : qtab_c(zz_n);
    wire [12:0]        r_tab  = is_y ? rtab(zz_n) : rtab_c(zz_n);
    wire signed [15:0] q_in   = bufA[zz_n];
    wire               q_neg  = q_in[15];
    wire [15:0]        q_mag  = q_neg ? -q_in : q_in;
    wire [31:0]        q_num  = q_mag * r_tab + (32'd32768 << shift);
    wire [31:0]        q_val  = q_num >> (5'd16 + shift);           // round(|F| / Q)
    wire [15:0]        q_step = {9'd0, q_tab} << shift;
    wire [31:0]        dq     = q_val * q_step;
    wire signed [15:0] dq_out = q_neg ? -dq[15:0] : dq[15:0];

    // ---------------- compressed-size counter ----------------
    wire [11:0]        q_small  = q_val[11:0];
    wire               q_zero   = (q_small == 12'd0);
    wire signed [12:0] q_signed = q_neg ? -$signed({1'b0, q_small}) : $signed({1'b0, q_small});
    wire signed [13:0] dc_diff  = q_signed - prev_dc;
    wire [13:0]        dc_mag   = dc_diff[13] ? -dc_diff : dc_diff;
    wire [3:0]         dc_size  = nbits(dc_mag);
    wire [3:0]         ac_size  = nbits({2'b00, q_small});

    reg [9:0] bits_add;
    always @(*) begin
        if (cnt[5:0] == 6'd0)
            bits_add = 10'd4 + {6'd0, dc_size};                     // DC
        else if (q_zero)
            bits_add = (cnt[5:0] == 6'd63) ? 10'd8 : 10'd0;         // EOB at the end
        else
            bits_add = {5'd0, run[5:4], 3'b000}                     // ZRL per 16 zeros
                     + 10'd8 + {6'd0, ac_size};                     // symbol + amplitude
    end

    // ---------------- output value (add 128, limit 0..255) ----------------
    wire signed [15:0] o_val   = bufA[cnt[5:0]] + 16'sd128;
    wire [7:0]         o_pixel = o_val[15]          ? 8'd0   :
                                 (o_val > 16'sd255) ? 8'd255 : o_val[7:0];

    wire [5:0] last_blk = is_y ? 6'd39 : 6'd19;       // 40x40 blocks for Y, 20x20 for Cb/Cr

    // ---------------- control state machine ----------------
    initial begin
        state = S_START; chan = 0; mode = 0; cnt = 0; bx = 0; by = 0; shift = 0;
        acc = 0; wr_addr = 0; wr_data = 0; we = 0; wr_chan = 0; done = 0;
        prev_dc = 0; run = 0; total_bits = 0;
    end

    always @(posedge clk) begin
        we <= 1'b0;

        if (reset) begin
            state <= S_START;
            done  <= 1'b0;
        end else begin
            case (state)

            S_START: begin
                shift      <= quality;
                chan       <= 2'd0;
                bx         <= 6'd0;
                by         <= 6'd0;
                cnt        <= 9'd0;
                done       <= 1'b0;
                total_bits <= 24'd0;
                prev_dc    <= 13'sd0;
                state      <= S_LOAD;
            end

            S_LOAD: begin
                if (is_y) begin                                   // Y: 64 pixels
                    if (cnt != 9'd0)
                        bufA[cnt_m1] <= y_val;
                    if (cnt == 9'd64) begin
                        cnt <= 9'd0; mode <= 2'd0; state <= S_MUL;
                    end else
                        cnt <= cnt + 9'd1;
                end else begin                                    // Cb/Cr: 256 pixels -> 64 samples
                    if (cnt != 9'd0) begin
                        if (!p_m1[4] && !p_m1[0])     bufA[c_idx] <= c_val;  // 1st of 2x2
                        else if (p_m1[4] && p_m1[0])  bufA[c_idx] <= c_avg;  // 4th -> average
                        else                          bufA[c_idx] <= c_acc;  // 2nd, 3rd
                    end
                    if (cnt == 9'd256) begin
                        cnt <= 9'd0; mode <= 2'd0; state <= S_MUL;
                    end else
                        cnt <= cnt + 9'd1;
                end
            end

            S_MUL: begin
                acc <= acc_next;
                if (k == 3'd7) begin
                    if (mode[0] == 1'b0) bufB[{a, b}] <= mul_out;
                    else                 bufA[{a, b}] <= mul_out;
                end
                if (cnt == 9'd511) begin
                    cnt <= 9'd0;
                    case (mode)
                        2'd0: mode  <= 2'd1;
                        2'd1: state <= S_QUANT;
                        2'd2: mode  <= 2'd3;
                        2'd3: state <= S_WRITE;
                    endcase
                end else
                    cnt <= cnt + 9'd1;
            end

            S_QUANT: begin
                bufA[zz_n] <= dq_out;
                total_bits <= total_bits + {14'd0, bits_add};
                if (cnt[5:0] == 6'd0) begin
                    prev_dc <= q_signed;
                    run     <= 6'd0;
                end else if (q_zero)
                    run <= run + 6'd1;
                else
                    run <= 6'd0;
                if (cnt == 9'd63) begin
                    cnt <= 9'd0; mode <= 2'd2; state <= S_MUL;
                end else
                    cnt <= cnt + 9'd1;
            end

            S_WRITE: begin
                we      <= 1'b1;
                wr_chan <= chan;
                wr_addr <= is_y ? y_wr_addr : c_wr_addr;
                wr_data <= o_pixel;
                if (cnt == 9'd63) begin
                    cnt <= 9'd0; state <= S_NEXT;
                end else
                    cnt <= cnt + 9'd1;
            end

            S_NEXT: begin
                cnt   <= 9'd0;
                state <= S_LOAD;
                if (bx == last_blk) begin
                    bx <= 6'd0;
                    if (by == last_blk) begin
                        by <= 6'd0;
                        if (chan == 2'd2)
                            state <= S_DONE;                      // all 3 channels done
                        else begin
                            chan    <= chan + 2'd1;               // next channel
                            prev_dc <= 13'sd0;
                        end
                    end else
                        by <= by + 6'd1;
                end else
                    bx <= bx + 6'd1;
            end

            S_DONE: done <= 1'b1;

            default: state <= S_START;
            endcase
        end
    end

endmodule
