// ============================================================
// cam_async_fifo : small "first in, first out" buffer that safely
// moves data from the camera clock (PIXCLK) to our 25 MHz clock.
//
// The two clocks are not related, so the read and write positions
// are passed across in Gray code (only 1 bit changes per step) through
// 2 flip-flops each. This is the standard safe way to cross clocks.
// Depth 16 is plenty: the camera produces 1 pixel every 4 clocks on
// average, and the 25 MHz side removes 1 pixel every clock.
// ============================================================
module cam_async_fifo #(
    parameter W  = 33,                 // data width
    parameter AW = 4                   // 2^AW = 16 entries
)(
    input  wire          wclk,
    input  wire          wen,
    input  wire [W-1:0]  wdata,
    output wire          wfull,

    input  wire          rclk,
    input  wire          ren,
    output wire [W-1:0]  rdata,        // valid whenever rempty = 0
    output wire          rempty
);

    reg [W-1:0] mem [0:(1<<AW)-1];

    reg [AW:0] wbin = 0, wgray = 0, rbin = 0, rgray = 0;
    reg [AW:0] rgray_w1 = 0, rgray_w2 = 0;     // read pointer seen by the write side
    reg [AW:0] wgray_r1 = 0, wgray_r2 = 0;     // write pointer seen by the read side

    // ---------- write side (camera clock) ----------
    wire [AW:0] wbin_n  = wbin + {{AW{1'b0}}, (wen & ~wfull)};
    wire [AW:0] wgray_n = (wbin_n >> 1) ^ wbin_n;

    always @(posedge wclk) begin
        if (wen & ~wfull) mem[wbin[AW-1:0]] <= wdata;
        wbin     <= wbin_n;
        wgray    <= wgray_n;
        rgray_w1 <= rgray;
        rgray_w2 <= rgray_w1;
    end

    assign wfull = (wgray == {~rgray_w2[AW:AW-1], rgray_w2[AW-2:0]});

    // ---------- read side (25 MHz) ----------
    wire [AW:0] rbin_n  = rbin + {{AW{1'b0}}, (ren & ~rempty)};
    wire [AW:0] rgray_n = (rbin_n >> 1) ^ rbin_n;

    always @(posedge rclk) begin
        rbin     <= rbin_n;
        rgray    <= rgray_n;
        wgray_r1 <= wgray;
        wgray_r2 <= wgray_r1;
    end

    assign rempty = (rgray == wgray_r2);
    assign rdata  = mem[rbin[AW-1:0]];

endmodule
