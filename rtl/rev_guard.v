// ============================================================================
// rev_guard.v — 换向驻留守卫（2026-10-22 用户硬约束）
//
// "电机的齿轮箱不允许快速换向"。这不是整定问题：任何一版控制律只要把方向
// 判断放在噪声放大的通路上（比如 θ̇ 的量化台阶 931 = 93°/s 一穿零就改号），
// 就会在 20~45Hz 上满幅翻向，硬件必坏。所以这里做**结构性**的守卫：
//
// 任何一次输出极性变化之后，GUARD_MS 内禁止再次变号；窗口内若上游要求反向，
// 输出 0（滑行）等待——宁可少推一次，也不让桥臂翻向。
//
// 语义细节：
//   · "极性"只看最近一次**非零**输出的符号；0 不是极性，不重置驻留计时
//     （0 → 反向 也算一次换向，因为扭矩确实反了）。
//   · 驻留计时从最近一次极性**变化**开始累计，与是否输出 0 无关。
//   · GUARD_MS=0 时整级直通（J280/DE10 行为逐位不变）。
//   · 正常甩击只含一次极性变化（加速→刹车），守卫不介入；它只在"反复反向"
//     这种病态情形下生效，把翻转率硬性压到 ≤ 1000/GUARD_MS 次/秒。
// ============================================================================
`timescale 1ns / 1ps
module rev_guard #(
    parameter [9:0] GUARD_MS = 10'd0    // 0 = 直通；典型 300（≤3.3 次/秒）
) (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               tick,     // 1ms
    input  wire signed [15:0] duty_in,
    output wire signed [15:0] duty_out
);
    generate if (GUARD_MS == 10'd0) begin : g_bypass
        assign duty_out = duty_in;
    end else begin : g_guard
        reg [1:0]  pol;     // 最近一次非零输出的极性：0=从未, 1=+, 2=−
        reg [9:0]  dwell;   // 自上次极性变化以来的拍数
        wire [1:0] req_pol = (duty_in > 16'sd0) ? 2'd1 : ((duty_in < 16'sd0) ? 2'd2 : 2'd0);
        wire block = (req_pol != 2'd0) && (pol != 2'd0) && (req_pol != pol) &&
                     (dwell < GUARD_MS);
        assign duty_out = block ? 16'sd0 : duty_in;
        always @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin
                pol <= 2'd0; dwell <= 10'd0;
            end else if (tick) begin
                if (dwell != 10'd1023) dwell <= dwell + 10'd1;
                // 允许换向：首次驱动，或距上次变化已过窗口
                if ((req_pol != 2'd0) && (req_pol != pol) &&
                    ((pol == 2'd0) || (dwell >= GUARD_MS))) begin
                    pol   <= req_pol;
                    dwell <= 10'd0;
                end
            end
        end
    end endgenerate
endmodule
