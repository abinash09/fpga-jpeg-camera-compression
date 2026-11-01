// ============================================================
// tb_color_dct_engine : ModelSim testbench for color_dct_engine
//
// Feeds a known 320 x 320 test picture to the engine, waits until it
// finishes, then prints the compressed size, the memory saved and a few
// decompressed pixel values.
//
//   TEST = 1 : uniform gray picture (every pixel RGB565 16'h8410)
//   TEST = 2 : vertical edge - black for columns 0..163, white after
//   QUALITY  : 0..3, same as SW1 SW0 on the board
//
// Run in ModelSim:
//   vlib work
//   vlog color_dct_engine.v tb_color_dct_engine.v
//   vsim -gTEST=1 -gQUALITY=0 tb_color_dct_engine
//   run -all
// ============================================================
`timescale 1ns/1ps
module tb_color_dct_engine;

    parameter TEST    = 1;
    parameter [1:0] QUALITY = 2'd0;

    reg clk   = 1'b0;
    reg reset = 1'b1;
    always #20 clk = ~clk;                       // 25 MHz

    // ---------- picture and result memories (same as in the top module) ----------
    reg [15:0] img    [0:102399];
    reg [7:0]  res_y  [0:102399];
    reg [7:0]  res_cb [0:25599];
    reg [7:0]  res_cr [0:25599];

    wire [16:0] rd_addr, wr_addr;
    wire [7:0]  wr_data;
    wire        we, done;
    wire [1:0]  wr_chan;
    wire [23:0] total_bits;
    reg  [15:0] rd_rgb;

    color_dct_engine dut (
        .clk        (clk),
        .reset      (reset),
        .quality    (QUALITY),
        .rd_addr    (rd_addr),
        .rd_rgb     (rd_rgb),
        .wr_addr    (wr_addr),
        .wr_data    (wr_data),
        .we         (we),
        .wr_chan    (wr_chan),
        .done       (done),
        .total_bits (total_bits)
    );

    // memory model: read data arrives one clock after the address, like block RAM
    always @(posedge clk) begin
        rd_rgb <= img[rd_addr];
        if (we) begin
            case (wr_chan)
                2'd0: res_y[wr_addr]         <= wr_data;
                2'd1: res_cb[wr_addr[14:0]]  <= wr_data;
                2'd2: res_cr[wr_addr[14:0]]  <= wr_data;
            endcase
        end
    end

    integer i, c;
    real    saved;

    initial begin
        // ---------- build the test picture ----------
        for (i = 0; i < 102400; i = i + 1) begin
            c = i % 320;
            if (TEST == 1) img[i] = 16'h8410;                         // gray
            else           img[i] = (c < 164) ? 16'h0000 : 16'hFFFF;  // black | white
        end

        // ---------- run the engine ----------
        repeat (4) @(posedge clk);
        reset = 1'b0;
        wait (done == 1'b1);
        @(posedge clk);

        // ---------- report ----------
        saved = 100.0 - (total_bits * 100.0) / 2457600.0;
        $display("--------------------------------------------------");
        $display("TEST %0d, QUALITY %0d finished at %0t ns", TEST, QUALITY, $time);
        $display("total_bits   = %0d", total_bits);
        $display("memory saved = %0.1f %%", saved);
        $display("res_y[0..3]  = %0d %0d %0d %0d", res_y[0], res_y[1], res_y[2], res_y[3]);
        $write  ("row 0, Y, columns 158-169 :");
        for (c = 158; c < 170; c = c + 1) $write(" %0d", res_y[c]);
        $display("");
        $display("res_cb[0] = %0d   res_cr[0] = %0d", res_cb[0], res_cr[0]);
        $display("--------------------------------------------------");
        $finish;
    end

endmodule
