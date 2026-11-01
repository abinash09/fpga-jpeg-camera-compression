// ============================================================
// cam_jpeg_top : LIVE CAMERA + JPEG-style colour compression (DE2-115 + TRDB-D5M)
//
//   LEFT  of screen : camera snapshot (320 x 320 colour)
//   RIGHT of screen : the same snapshot after compression
//   HEX3..HEX0      : memory saved in %
//
//   KEY0        : reset (set up camera again, take a new picture)
//   KEY1        : take a new picture and compress it
//   SW17        : 1 = live mode (keep taking + compressing pictures)
//   SW16..SW13  : exposure (brightness) - press KEY0 after changing
//   SW3  SW2    : colour order of the camera pixels (try until colours look right)
//   SW1  SW0    : compression strength
//   LEDG0       : compression finished
//   LEDG1       : camera set up
//   LEDG2       : capturing a picture
//
// Flow:  camera -> cam_capture (camera clock) -> cam_async_fifo
//        -> orig_mem (25 MHz) -> color_dct_engine -> res_y / res_cb / res_cr
//        -> VGA
// ============================================================
module cam_jpeg_top (
    input  wire        CLOCK_50,
    input  wire [1:0]  KEY,
    input  wire [17:0] SW,

    output reg  [7:0]  VGA_R,
    output reg  [7:0]  VGA_G,
    output reg  [7:0]  VGA_B,
    output reg         VGA_HS,
    output reg         VGA_VS,
    output wire        VGA_CLK,
    output reg         VGA_BLANK_N,
    output wire        VGA_SYNC_N,

    output wire [2:0]  LEDG,
    output wire [6:0]  HEX0, HEX1, HEX2, HEX3, HEX4, HEX5, HEX6, HEX7,

    // TRDB-D5M camera (on the GPIO connector)
    input  wire [11:0] D5M_D,
    input  wire        D5M_FVAL,
    input  wire        D5M_LVAL,
    input  wire        D5M_PIXLCK,
    output wire        D5M_RESET_N,
    output wire        D5M_SCLK,
    inout  wire        D5M_SDATA,
    output wire        D5M_TRIGGER,
    output wire        D5M_XCLKIN
);

    wire reset = ~KEY[0];

    // ---------- 25 MHz clock (VGA, engine, and the camera's input clock) ----------
    reg clk25 = 1'b0;
    always @(posedge CLOCK_50) clk25 <= ~clk25;

    assign D5M_XCLKIN  = clk25;
    assign D5M_TRIGGER = 1'b1;

    // ---------- 1. Camera set-up over I2C ----------
    wire cfg_done;
    cam_config config_unit (
        .clk          (clk25),
        .reset        (reset),
        .exposure_sel (SW[16:13]),
        .cam_reset_n  (D5M_RESET_N),
        .sclk         (D5M_SCLK),
        .sdata        (D5M_SDATA),
        .done         (cfg_done)
    );

    // ---------- 2. Capture one frame (camera clock domain) ----------
    reg         cap_req = 1'b0;
    wire        cap_done;
    wire        cam_wen;
    wire [32:0] cam_wdata;

    cam_capture capture_unit (
        .pclk        (D5M_PIXLCK),
        .d           (D5M_D),
        .lval        (D5M_LVAL),
        .fval        (D5M_FVAL),
        .bayer_phase (SW[3:2]),
        .cap_req     (cap_req),
        .cap_done    (cap_done),
        .wen         (cam_wen),
        .wdata       (cam_wdata)
    );

    // ---------- 3. Cross from camera clock to 25 MHz ----------
    wire [32:0] fifo_q;
    wire        fifo_empty;
    wire        fifo_full;

    cam_async_fifo #(.W(33), .AW(4)) fifo_unit (
        .wclk   (D5M_PIXLCK),
        .wen    (cam_wen),
        .wdata  (cam_wdata),
        .wfull  (fifo_full),
        .rclk   (clk25),
        .ren    (1'b1),                // always take a pixel when one is there
        .rdata  (fifo_q),
        .rempty (fifo_empty)
    );

    wire        pix_we   = ~fifo_empty;
    wire [16:0] pix_addr = fifo_q[32:16];
    wire [15:0] pix_rgb  = fifo_q[15:0];

    // ---------- 4. Control: capture -> compress -> (repeat in live mode) ----------
    localparam T_CFG = 3'd0, T_CAP = 3'd1, T_DRAIN = 3'd2,
               T_CSTART = 3'd3, T_CWAIT = 3'd4, T_IDLE = 3'd5;

    reg [2:0] ctl = T_CFG;
    reg [1:0] capdone_s = 2'b00;      // cap_done synchronised to 25 MHz
    reg [2:0] key1_s = 3'b000;        // KEY1 synchronised, for edge detection
    reg [3:0] dly = 4'd0;
    reg       eng_kick = 1'b0;        // 1-clock pulse: start the engine
    reg       first = 1'b1;           // no picture compressed yet
    wire      eng_done;

    always @(posedge clk25) begin
        capdone_s <= {capdone_s[0], cap_done};
        key1_s    <= {key1_s[1:0], ~KEY[1]};
        eng_kick  <= 1'b0;

        if (reset) begin
            ctl     <= T_CFG;
            cap_req <= 1'b0;
            first   <= 1'b1;
        end else begin
            case (ctl)
            T_CFG:                                          // wait for camera set-up
                if (cfg_done) ctl <= T_CAP;

            T_CAP: begin                                    // ask for one frame
                cap_req <= 1'b1;
                if (capdone_s[1]) begin
                    cap_req <= 1'b0;
                    dly     <= 4'd0;
                    ctl     <= T_DRAIN;
                end
            end

            T_DRAIN:                                        // let the FIFO empty out
                if (!capdone_s[1] && fifo_empty) begin
                    dly <= dly + 4'd1;
                    if (dly == 4'd15) ctl <= T_CSTART;
                end else
                    dly <= 4'd0;

            T_CSTART: begin                                 // start compression
                eng_kick <= 1'b1;
                first    <= 1'b0;
                dly      <= 4'd0;
                ctl      <= T_CWAIT;
            end

            T_CWAIT: begin                                  // wait until compression is done
                if (dly != 4'd15) dly <= dly + 4'd1;
                else if (eng_done) ctl <= SW[17] ? T_CAP : T_IDLE;
            end

            T_IDLE:                                         // wait for KEY1
                if (key1_s[1] && !key1_s[2]) ctl <= T_CAP;

            default: ctl <= T_CFG;
            endcase
        end
    end

    assign LEDG[0] = eng_done & ~first;
    assign LEDG[1] = cfg_done;
    assign LEDG[2] = (ctl == T_CAP);

    // ---------- VGA 640x480 timing ----------
    localparam H_VISIBLE = 640, H_FRONT = 16, H_SYNC = 96, H_BACK = 48;
    localparam V_VISIBLE = 480, V_FRONT = 10, V_SYNC = 2,  V_BACK = 33;
    localparam H_TOTAL = H_VISIBLE + H_FRONT + H_SYNC + H_BACK;
    localparam V_TOTAL = V_VISIBLE + V_FRONT + V_SYNC + V_BACK;

    reg [9:0] x = 0, y = 0;
    always @(posedge clk25) begin
        if (x == H_TOTAL - 1) begin
            x <= 0;
            y <= (y == V_TOTAL - 1) ? 10'd0 : y + 10'd1;
        end else
            x <= x + 10'd1;
    end

    wire hs_now      = ~((x >= H_VISIBLE + H_FRONT) && (x < H_VISIBLE + H_FRONT + H_SYNC));
    wire vs_now      = ~((y >= V_VISIBLE + V_FRONT) && (y < V_VISIBLE + V_FRONT + V_SYNC));
    wire visible_now = (x < H_VISIBLE) && (y < V_VISIBLE);
    wire left_now    = (x < 320) && (y < 320);
    wire right_now   = (x >= 320) && (x < 640) && (y < 320);

    wire [9:0]  x_right    = x - 10'd320;
    wire [16:0] row_base   = ({7'd0, y} << 8) + ({7'd0, y} << 6);       // y * 320
    wire [16:0] addr_left  = left_now  ? row_base + {7'd0, x}       : 17'd0;
    wire [16:0] addr_right = right_now ? row_base + {7'd0, x_right} : 17'd0;

    wire [8:0]  cy     = y[9:1];
    wire [8:0]  cx     = x_right[9:1];
    wire [16:0] addr_c = right_now ? ({8'd0, cy} << 7) + ({8'd0, cy} << 5) + {8'd0, cx} : 17'd0;

    // ---------- Original picture memory (RGB565) ----------
    //   port A : camera writes (capture) / engine reads (compress)
    //   port B : VGA reads
    reg [15:0] orig_mem [0:102399];

    wire [16:0] eng_rd_addr;
    wire [16:0] a_addr = pix_we ? pix_addr : eng_rd_addr;
    reg  [15:0] orig_eng, orig_vga;

    always @(posedge clk25) begin
        if (pix_we) orig_mem[a_addr] <= pix_rgb;
        orig_eng <= orig_mem[a_addr];
    end
    always @(posedge clk25)
        orig_vga <= orig_mem[addr_left];

    // ---------- Result memories (engine writes, VGA reads) ----------
    reg [7:0] res_y  [0:102399];
    reg [7:0] res_cb [0:25599];
    reg [7:0] res_cr [0:25599];

    wire [16:0] eng_wr_addr;
    wire [7:0]  eng_wr_data;
    wire        eng_we;
    wire [1:0]  eng_wr_chan;

    reg [7:0] y_vga, cb_vga, cr_vga;
    always @(posedge clk25) begin
        if (eng_we && eng_wr_chan == 2'd0) res_y[eng_wr_addr] <= eng_wr_data;
        y_vga <= res_y[addr_right];
    end
    always @(posedge clk25) begin
        if (eng_we && eng_wr_chan == 2'd1) res_cb[eng_wr_addr[14:0]] <= eng_wr_data;
        cb_vga <= res_cb[addr_c[14:0]];
    end
    always @(posedge clk25) begin
        if (eng_we && eng_wr_chan == 2'd2) res_cr[eng_wr_addr[14:0]] <= eng_wr_data;
        cr_vga <= res_cr[addr_c[14:0]];
    end

    // ---------- Compression engine (same as the colour project) ----------
    wire [23:0] total_bits;

    color_dct_engine engine (
        .clk        (clk25),
        .reset      (reset | eng_kick | first),
        .quality    (SW[1:0]),
        .rd_addr    (eng_rd_addr),
        .rd_rgb     (orig_eng),
        .wr_addr    (eng_wr_addr),
        .wr_data    (eng_wr_data),
        .we         (eng_we),
        .wr_chan    (eng_wr_chan),
        .done       (eng_done),
        .total_bits (total_bits)
    );

    color_saved_display display (
        .done       (eng_done & ~first),
        .total_bits (total_bits),
        .HEX0 (HEX0), .HEX1 (HEX1), .HEX2 (HEX2), .HEX3 (HEX3),
        .HEX4 (HEX4), .HEX5 (HEX5), .HEX6 (HEX6), .HEX7 (HEX7)
    );

    // ---------- Left side: RGB565 -> 8 bits per colour ----------
    wire [7:0] o_r = {orig_vga[15:11], orig_vga[15:13]};
    wire [7:0] o_g = {orig_vga[10:5],  orig_vga[10:9]};
    wire [7:0] o_b = {orig_vga[4:0],   orig_vga[4:2]};

    // ---------- Right side: Y Cb Cr -> R G B ----------
    wire signed [17:0] yv  = {10'd0, y_vga};
    wire signed [17:0] cbd = $signed({10'd0, cb_vga}) - 18'sd128;
    wire signed [17:0] crd = $signed({10'd0, cr_vga}) - 18'sd128;

    wire signed [17:0] r_t = yv + ((18'sd359 * crd) >>> 8);
    wire signed [17:0] g_t = yv - ((18'sd88 * cbd + 18'sd183 * crd) >>> 8);
    wire signed [17:0] b_t = yv + ((18'sd454 * cbd) >>> 8);

    function [7:0] clip;
        input signed [17:0] v;
        clip = (v < 0) ? 8'd0 : (v > 255) ? 8'd255 : v[7:0];
    endfunction

    // ---------- Delay control signals 1 clock (memory latency) ----------
    reg left_d, right_d, visible_d;
    always @(posedge clk25) begin
        VGA_HS      <= hs_now;
        VGA_VS      <= vs_now;
        VGA_BLANK_N <= visible_now;
        visible_d   <= visible_now;
        left_d      <= left_now;
        right_d     <= right_now;
    end

    always @(*) begin
        if (visible_d && left_d) begin
            VGA_R = o_r;  VGA_G = o_g;  VGA_B = o_b;
        end else if (visible_d && right_d) begin
            VGA_R = clip(r_t);  VGA_G = clip(g_t);  VGA_B = clip(b_t);
        end else begin
            VGA_R = 8'd0; VGA_G = 8'd0; VGA_B = 8'd0;
        end
    end

    assign VGA_CLK    = clk25;
    assign VGA_SYNC_N = 1'b0;

endmodule
