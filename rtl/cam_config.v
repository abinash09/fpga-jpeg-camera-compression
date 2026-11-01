// ============================================================
// cam_config : resets the TRDB-D5M camera and sets it up over I2C
//
// 1. Holds the camera in reset for 2 ms, then waits 20 ms.
// 2. Writes 15 camera registers using I2C (2 wires: SCLK + SDATA,
//    about 100 kHz). Every write is:
//       START, 0xBA (camera address), register number,
//       data high byte, data low byte, STOP
// 3. Sets 'done' = 1. The camera then sends frames by itself.
//
// Camera window: a 1280 x 1280 square from the middle of the sensor,
// reading every 2nd pixel and every 2nd row -> 640 x 640 frame.
// (The capture module then turns every 2x2 group into 1 pixel -> 320 x 320.)
// ============================================================
module cam_config (
    input  wire       clk,            // 25 MHz
    input  wire       reset,          // KEY0: reset camera and send settings again
    input  wire [3:0] exposure_sel,   // SW16..SW13: 0 = darkest ... 15 = brightest
    output reg        cam_reset_n,    // camera reset (active low)
    output reg        sclk,           // I2C clock
    inout  wire       sdata,          // I2C data (open drain)
    output reg        done            // 1 = camera is set up
);

    localparam NREG = 15;

    // ---------------- register list {register, value} ----------------
    function [23:0] entry;
        input [4:0]  i;
        input [15:0] expo;
        case (i)
            5'd0:  entry = {8'h20, 16'hC000};   // read mode: mirror rows + columns (picture upright)
            5'd1:  entry = {8'h09, expo};       // exposure time (shutter width, in rows)
            5'd2:  entry = {8'h05, 16'h0000};   // horizontal blanking (minimum)
            5'd3:  entry = {8'h06, 16'h0019};   // vertical blanking (25 rows)
            5'd4:  entry = {8'h0A, 16'h8000};   // invert pixel clock (data stable on rising edge)
            5'd5:  entry = {8'h2B, 16'h0013};   // green 1 gain
            5'd6:  entry = {8'h2C, 16'h009A};   // blue gain
            5'd7:  entry = {8'h2D, 16'h019C};   // red gain
            5'd8:  entry = {8'h2E, 16'h0013};   // green 2 gain
            5'd9:  entry = {8'h01, 16'd386};    // first row    (centre of the 1944-row sensor)
            5'd10: entry = {8'h02, 16'd672};    // first column (centre of the 2592-column sensor)
            5'd11: entry = {8'h03, 16'd1279};   // window height - 1  (1280 rows)
            5'd12: entry = {8'h04, 16'd1279};   // window width  - 1  (1280 columns)
            5'd13: entry = {8'h22, 16'h0001};   // rows:    skip 2x -> 640 rows out
            5'd14: entry = {8'h23, 16'h0001};   // columns: skip 2x -> 640 columns out
            default: entry = {8'h20, 16'hC000};
        endcase
    endfunction

    // ---------------- I2C timing: 1 bit = 4 quarters of 63 clocks (~99 kHz) ----------------
    reg  [5:0] qdiv = 6'd0;
    wire       tick = (qdiv == 6'd62);
    always @(posedge clk) qdiv <= tick ? 6'd0 : qdiv + 6'd1;

    reg sda_o;
    assign sdata = sda_o ? 1'bz : 1'b0;          // open drain: release for 1, pull low for 0

    localparam C_RST = 3'd0, C_WAIT = 3'd1, C_START = 3'd2, C_BIT = 3'd3,
               C_STOP = 3'd4, C_DONE = 3'd5;

    reg [2:0]  st;
    reg [19:0] wcnt;
    reg [1:0]  q;           // quarter of the current bit (0..3)
    reg [4:0]  idx;         // which register is being written
    reg [1:0]  byte_i;      // byte 0..3 of the current write
    reg [3:0]  bit_i;       // bit 0..7, 8 = acknowledge slot
    reg [31:0] sh;          // bits still to send
    reg [15:0] exposure;

    initial begin
        st = C_RST; wcnt = 0; q = 0; idx = 0; byte_i = 0; bit_i = 0; sh = 0;
        cam_reset_n = 1'b0; sclk = 1'b1; sda_o = 1'b1; done = 1'b0;
        exposure = 16'h0100;
    end

    always @(posedge clk) begin
        if (reset) begin
            st          <= C_RST;
            wcnt        <= 20'd0;
            q           <= 2'd0;
            idx         <= 5'd0;
            done        <= 1'b0;
            cam_reset_n <= 1'b0;
            sclk        <= 1'b1;
            sda_o       <= 1'b1;
            exposure    <= ({12'd0, exposure_sel} + 16'd1) << 8;   // 256 .. 4096 rows
        end else begin
            case (st)

            C_RST: begin                                   // camera in reset for 2 ms
                cam_reset_n <= 1'b0;
                wcnt <= wcnt + 20'd1;
                if (wcnt == 20'd50000) begin wcnt <= 20'd0; st <= C_WAIT; end
            end

            C_WAIT: begin                                  // release reset, wait 20 ms
                cam_reset_n <= 1'b1;
                wcnt <= wcnt + 20'd1;
                if (wcnt == 20'd500000) begin q <= 2'd0; st <= C_START; end
            end

            C_START: if (tick) begin                       // START: SDA falls while SCL high
                case (q)
                    2'd0: begin sda_o <= 1'b1; sclk <= 1'b1; end
                    2'd1: sda_o <= 1'b0;
                    2'd2: sclk  <= 1'b0;
                    2'd3: begin
                        sh     <= {8'hBA, entry(idx, exposure)};
                        byte_i <= 2'd0;
                        bit_i  <= 4'd0;
                        st     <= C_BIT;
                    end
                endcase
                q <= q + 2'd1;
            end

            C_BIT: if (tick) begin                         // one data bit or the ACK slot
                case (q)
                    2'd0: begin
                        sclk  <= 1'b0;
                        sda_o <= (bit_i == 4'd8) ? 1'b1 : sh[31];   // release SDA for ACK
                    end
                    2'd1: sclk <= 1'b1;
                    2'd2: sclk <= 1'b1;
                    2'd3: begin
                        sclk <= 1'b0;
                        if (bit_i == 4'd8) begin
                            bit_i <= 4'd0;
                            if (byte_i == 2'd3) st <= C_STOP;
                            else                byte_i <= byte_i + 2'd1;
                        end else begin
                            sh    <= {sh[30:0], 1'b0};
                            bit_i <= bit_i + 4'd1;
                        end
                    end
                endcase
                q <= q + 2'd1;
            end

            C_STOP: if (tick) begin                        // STOP: SDA rises while SCL high
                case (q)
                    2'd0: begin sclk <= 1'b0; sda_o <= 1'b0; end
                    2'd1: sclk  <= 1'b1;
                    2'd2: sda_o <= 1'b1;
                    2'd3: begin
                        if (idx == NREG - 1) st <= C_DONE;
                        else begin idx <= idx + 5'd1; st <= C_START; end
                    end
                endcase
                q <= q + 2'd1;
            end

            C_DONE: done <= 1'b1;

            default: st <= C_RST;
            endcase
        end
    end

endmodule
