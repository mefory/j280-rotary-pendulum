// ============================================================================
// qenc4x.v — 正交编码器四倍频解码（32 位计数）
// 状态机沿用官方 encoder.v 的四状态转移表，计数扩为 s32（连续旋转不回绕）。
// POL_FLIP=1 时计数方向取反（到手极性确认开关，见 j280_defs.vh）。
// ============================================================================
`include "j280_defs.vh"
module qenc4x #(
    parameter POL_FLIP = 0
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        enc_a,
    input  wire        enc_b,
    output reg  signed [31:0] pos
);
    reg [1:0] sync_a, sync_b;
    reg [1:0] st_now, st_last;
    wire signed [31:0] step = POL_FLIP ? -delta : delta;
    reg signed [31:0] delta;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sync_a <= 2'd0; sync_b <= 2'd0; st_now <= 2'd0;
        end else begin
            sync_a <= {sync_a[0], enc_a};
            sync_b <= {sync_b[0], enc_b};
            st_now <= {sync_a[1], sync_b[1]};
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st_last <= 2'd0; delta <= 32'sd0; pos <= 32'sd0;
        end else begin
            st_last <= st_now;
            case ({st_last, st_now})
                4'b00_01, 4'b01_11, 4'b11_10, 4'b10_00: delta <= -32'sd1;
                4'b00_10, 4'b10_11, 4'b11_01, 4'b01_00: delta <= +32'sd1;
                default:                                 delta <=  32'sd0;
            endcase
            pos <= pos + step;
        end
    end
endmodule
