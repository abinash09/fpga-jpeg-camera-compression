// ============================================================
// cam_capture : takes ONE frame from the camera and turns it into
//               320 x 320 RGB565 pixels.  Runs on the camera's own
//               clock (PIXCLK).
//
// The camera sends a 640 x 640 "Bayer" picture: every pixel is only
// ONE colour, in a 2 x 2 pattern such as
//        G R
//        B G
// Each 2 x 2 group becomes one RGB pixel:
//        R = the red one, B = the blue one, G = average of the 2 greens
// -> 320 x 320 colour pixels, exactly the size our engine uses.
//
// Because a 2 x 2 group spans two rows, the first row of each pair is
// kept in a small line buffer (one memory block).
//
// Handshake with the 25 MHz side:
//   cap_req goes 1  -> wait for the next frame start (FVAL rising),
//                      capture the whole frame,
//                      then cap_done = 1
//   cap_req goes 0  -> cap_done = 0
// Every finished pixel is sent out as {address, RGB565} with wen = 1.
// ============================================================
module cam_capture (
    input  wire        pclk,          // camera pixel clock
    input  wire [11:0] d,             // camera pixel data (12 bits)
    input  wire        lval,          // line valid
    input  wire        fval,          // frame valid
    input  wire [1:0]  bayer_phase,   // SW3 SW2: which corner of 2x2 is red
    input  wire        cap_req,       // from the 25 MHz side (not synchronised yet)
    output reg         cap_done,      // to the 25 MHz side
    output reg         wen,           // 1 = wdata holds a new pixel
    output reg  [32:0] wdata          // {address[16:0], RGB565[15:0]}
);

    // ---------- register the camera inputs ----------
    reg [7:0] d_r;                    // top 8 of the 12 bits are enough for display
    reg       lval_r, fval_r, lval_d, fval_d;
    reg       req_1, req_2;           // 2-stage synchroniser for cap_req

    always @(posedge pclk) begin
        d_r    <= d[11:4];
        lval_r <= lval;
        fval_r <= fval;
        lval_d <= lval_r;
        fval_d <= fval_r;
        req_1  <= cap_req;
        req_2  <= req_1;
    end

    // ---------- row / column counters ----------
    reg [10:0] col = 11'd0;
    reg [10:0] row = 11'd0;
    always @(posedge pclk) begin
        if (lval_r) col <= col + 11'd1;
        else        col <= 11'd0;

        if (!fval_r)                row <= 11'd0;
        else if (lval_d && !lval_r) row <= row + 11'd1;   // end of a line
    end

    // ---------- capture exactly one whole frame ----------
    reg active = 1'b0;
    initial cap_done = 1'b0;
    always @(posedge pclk) begin
        if (!req_2) begin
            active   <= 1'b0;
            cap_done <= 1'b0;
        end else if (!cap_done) begin
            if (fval_r && !fval_d)
                active <= 1'b1;                            // frame starts
            else if (!fval_r && fval_d && active) begin
                active   <= 1'b0;                          // frame ends
                cap_done <= 1'b1;
            end
        end
    end

    // ---------- line buffer: top row of each 2x2 group ----------
    reg [15:0] lb [0:511];            // {top-left, top-right} for each column pair
    reg [15:0] lb_q;
    reg [7:0]  prev;                  // pixel to the left of the current one
    wire [8:0] pair = col[9:1];

    always @(posedge pclk) begin
        if (lval_r && !row[0] && col[0])
            lb[pair] <= {prev, d_r};                      // even row: store the pair
        lb_q <= lb[pair];                                 // read it back on the odd row
        if (lval_r && !col[0])
            prev <= d_r;
    end

    // ---------- 2x2 group -> R G B ----------
    wire [7:0] p00 = lb_q[15:8];      // top-left
    wire [7:0] p01 = lb_q[7:0];       // top-right
    wire [7:0] p10 = prev;            // bottom-left
    wire [7:0] p11 = d_r;             // bottom-right (arriving now)
    wire [8:0] g_a = p00 + p11;
    wire [8:0] g_b = p01 + p10;

    reg [7:0] R, G, B;
    always @(*) begin
        case (bayer_phase)
            2'd0: begin R = p01; B = p10; G = g_a[8:1]; end    // G R / B G
            2'd1: begin R = p00; B = p11; G = g_b[8:1]; end    // R G / G B
            2'd2: begin R = p11; B = p00; G = g_b[8:1]; end    // B G / G R
            default: begin R = p10; B = p01; G = g_a[8:1]; end // G B / R G
        endcase
    end

    // ---------- output pixel (row/2, col/2) ----------
    wire [9:0]  orow = row[10:1];
    wire [9:0]  ocol = col[10:1];
    wire [16:0] oaddr = ({7'd0, orow} << 8) + ({7'd0, orow} << 6) + {7'd0, ocol};   // orow*320+ocol

    always @(posedge pclk) begin
        wen <= 1'b0;
        if (active && lval_r && row[0] && col[0] && (orow < 10'd320) && (ocol < 10'd320)) begin
            wen   <= 1'b1;
            wdata <= {oaddr, R[7:3], G[7:2], B[7:3]};
        end
    end

endmodule
