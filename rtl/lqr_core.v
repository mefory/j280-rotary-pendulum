// ============================================================================
// lqr_core.v — 全状态反馈 u = K1·α + K2·α̇ + K3·θe + K4·θ̇（3 级流水，4 乘 3 加）
// 结构 = 官方"倾角环+位置环叠加"（PPT §6.4）的最优化版本；
// 增益 s16 Q3.13（scale 8192，2026-09-18 自 Q2.14 左移一位以容纳实机先验
// 增益域 ±4.0），由 matlab/lqr_to_rtl.m 生成写入 j280_gains.vh 或经 UART 在线覆盖。
// 【注意】增益符号已含负反馈方向：MATLAB 侧保证 u = K·x 为镇定律（非 -K·x）。
// ============================================================================
`include "j280_defs.vh"
module lqr_core (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               start,
    input  wire signed [15:0] alpha,       // 0.01°
    input  wire signed [15:0] alpha_dot,   // 0.1°/s
    input  wire signed [15:0] theta_e,     // 0.01°
    input  wire signed [15:0] theta_dot,   // 0.1°/s
    input  wire signed [15:0] k1, k2, k3, k4, // Q3.13
    output reg  signed [15:0] u,           // 0.01% duty
    output reg                done
);
    // S1: 4 个 16×16 有符号乘（推断 DSP）
    reg signed [31:0] p1, p2, p3, p4;
    reg [1:0] valid;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n)valid<=0;
        else valid<={valid[0],start};
    end
    always @(posedge clk) begin
        p1 <= alpha     * k1;
        p2 <= alpha_dot * k2;
        p3 <= theta_e   * k3;
        p4 <= theta_dot * k4;
    end
    // S2: 求和（乘积单位 = duty×2^13）
    reg signed [33:0] acc;
    always @(posedge clk) acc <= p1 + p2 + p3 + p4;
    // S3: Q3.13 右移 + 饱和
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            u <= 16'sd0; done <= 1'b0;
        end else begin
            done <= valid[1];
            if(valid[1])u<=sat14((acc+34'sd4096) >>> 13);
        end
    end

    function signed [15:0] sat14(input signed [33:0] v);
        sat14 = (v >  34'sd9500)  ? 16'sd9500  :
                (v < -34'sd9500)  ? -16'sd9500 : v[15:0];
    endfunction
endmodule
