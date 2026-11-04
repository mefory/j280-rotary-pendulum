// ============================================================================
// cnt2deg.v — 编码器计数 → 0.01° 角度换算（两级流水，DSP 友好）
// deg = (cnt × 36000/CPR)，拆 hi/lo 两个 16×16 乘法 + 汇加，2 拍延迟。
// ============================================================================
`include "j280_defs.vh"
module cnt2deg #(
    parameter CPR = `CNT_PER_REV
) (
    input  wire               clk,
    input  wire               rst_n,
    input  wire signed [31:0] cnt,
    output reg  signed [31:0] deg        // 0.01°
);
    // C = round(36000×1024/CPR)：deg = (cnt×C)>>>10
    // 35446 does not fit signed 16 bits. Low half is unsigned,
    // high half is signed; keeping both signs explicit also fixes cnt < 0.
    localparam signed [16:0] C = (36000*1024 + CPR/2) / CPR;

    reg signed [33:0] plo;
    reg signed [32:0] phi;
    wire signed [48:0] full = $signed({phi, 16'b0}) +
                              $signed({{15{plo[33]}}, plo});

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            plo <= 34'sd0; phi <= 33'sd0; deg <= 32'sd0;
        end else begin
            plo <= $signed({1'b0, cnt[15:0]}) * C;
            phi <= $signed(cnt[31:16]) * C;
            deg <= full[41:10];         // >>>10
        end
    end
endmodule
