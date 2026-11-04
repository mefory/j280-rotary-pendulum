// ============================================================================
// obs_core.v — 4 状态稳态 Kalman 观测器（Luenberger 形式，双编码器方案）
//
// 结构（j280_obs_gains.vh 由 matlab/obs_to_rtl.m 自动生成，Q12.20）：
//   x̂ = [θ(c01), α(c01), θ̇(0.1°/s), α̇(0.1°/s)]（j280_linear_model 原生状态序），
//   内部 ×256 定标（s32）。每 1ms tick：
//   x̂⁺ = Adu·x̂ + Bdu·duty_prev + Lu·(y − x̂(θ,α))
//   y = [θ 链 c01, α 链 c01]（现行差分链输出，SWING 段继续由它服务）
//
// 模式（设计定案 2026-10-16，docs/MT6835_GMR_双编码器与观测器方案.md §5）：
//   inactive（SWING/SAFE/|α|≥30°）：x̂ 从动量测（θ/α 取量测、θ̇/α̇ 取差分链），
//     切换瞬间天然连续、零瞬态；active（BALANCE/POS 且 |α|<30°）：自由运行。
//   obs_enable 由 ctrl_core 的 OBS_USE 参数驱动（默认 0 = 全程从动，行为不变）。
//
// 实现：1ms 拍内 28 周期 MAC 微序列器（4 状态 × 7 项），64 位累加、
//   Q12.20 移位舍入、饱和到 s32；50MHz 下 28 拍 ≪ 50000 拍裕量。
// ============================================================================
`timescale 1ns / 1ps
module obs_core #(
    parameter signed [15:0] OBS_ALPHA_MAX = 16'sd3000   // |α|<30° 线性工作窗
)(
    input  wire               clk,
    input  wire               rst_n,
    input  wire               tick,            // 1ms 控制拍
    input  wire               obs_enable,      // ctrl_core OBS_USE 门控
    input  wire signed [31:0] theta_meas,      // 差分链 θ（c01）
    input  wire signed [15:0] theta_dot_meas,  // 差分链 θ̇（0.1°/s）
    input  wire signed [15:0] alpha_meas,      // 差分链 α（c01）
    input  wire signed [15:0] alpha_dot_meas,  // 差分链 α̇（0.1°/s）
    input  wire signed [15:0] duty_prev,       // 上一拍 duty（±10000）
    output reg                obs_active,
    output wire signed [31:0] obs_theta,       // c01
    output wire signed [15:0] obs_theta_dot,   // 0.1°/s
    output wire signed [15:0] obs_alpha,       // c01
    output wire signed [15:0] obs_alpha_dot    // 0.1°/s
);
    `include "j280_obs_gains.vh"   // localparam + obs_coef()（模块作用域）
    // 状态（×256 域，模型原生序 x0=θ x1=α x2=θ̇ x3=α̇）与锁存量测
    reg signed [31:0] x0, x1, x2, x3;
    reg signed [31:0] xn0, xn1, xn2, xn3;
    reg signed [31:0] y1s, y2s, uq;
    reg signed [31:0] e1, e2;
    reg        busy;
    reg        commit_q;   // 末行 xn3 写入与整体提交隔一拍：同拍提交会读到陈旧 xn
    reg [1:0]  mi;          // 状态行 0..3
    reg [2:0]  mj;          // 项 0..6（0-3=A 列，4=Bdu，5/6=Lu θ/α）

    localparam signed [63:0] RND = 64'sd524288;   // +2^19 舍入（Q12.20）

    // MAC 项选择
    reg signed [31:0] term;
    always @* begin
        case (mj)
            3'd0:    term = x0;
            3'd1:    term = x1;
            3'd2:    term = x2;
            3'd3:    term = x3;
            3'd4:    term = uq;
            3'd5:    term = e1;
            default: term = e2;
        endcase
    end
    // 2026-10-20 时序修复（Fmax 45.12MHz 违规）：原 32×32→64 乘法与 64 位加
    // 串在一拍内。此处改为"乘法拆两拍"：第一个忙拍锁存乘积，第二拍累加。
    // 48 拍/状态的微序列预算充裕（1ms=50000 拍），加拍零代价。
    // 位宽保持满 32×32：不能截断项——θ 是 c01 域 ×256 的多圈累积，实测
    // -36125 cdeg 即 s24 溢出（§7 的 12:52 日志），截断会破坏 θ 估计。
    reg signed [63:0] prod_q;
    reg               mul_done;
    // 累加器拆成高低 32 位：每拍只过一个 32 位加法器（64 位全宽加也是慢路径）。
    // 补码正确性（已用 Python 30 万随机例验证 0 错配）：低位按**无符号**加取
    // 进位——位级加法与符号解释无关，只要和 64 位一次加成的位模式一致即可。
    reg signed [31:0] acc_lo;
    reg signed [31:0] acc_hi;
    wire        [32:0] slo     = {1'b0, acc_lo} + {1'b0, prod_q[31:0]};
    wire signed [31:0] sum_lo  = slo[31:0];
    wire signed [31:0] sum_hi  = acc_hi + prod_q[63:32] + {{31{1'b0}}, slo[32]};
    wire signed [63:0] sum64   = {sum_hi, sum_lo};
    wire signed [63:0] acc     = {acc_hi, acc_lo};   // 仅 OBS_DEBUG 显示用
    wire signed [63:0] accp    = acc + prod_q;       // 仅 OBS_DEBUG 显示用

    function signed [31:0] sat32(input signed [63:0] v);
        sat32 = (v > 64'sd2147483647) ? 32'sd2147483647 :
                (v < -64'sd2147483648) ? -32'sd2147483648 : v[31:0];
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            x0 <= 0; x1 <= 0; x2 <= 0; x3 <= 0;
            xn0 <= 0; xn1 <= 0; xn2 <= 0; xn3 <= 0;
            y1s <= 0; y2s <= 0; uq <= 0; e1 <= 0; e2 <= 0;
            busy <= 1'b0; mi <= 2'd0; mj <= 3'd0;
            acc_lo <= 32'sd0; acc_hi <= 32'sd0;
            commit_q <= 1'b0; prod_q <= 64'd0; mul_done <= 1'b0;
            obs_active <= 1'b0;
        end else begin
            if (tick && !busy) begin
                // 拍入口：锁存量测与使能（×256 域）
                y1s <= theta_meas <<< 8;
                y2s <= {{16{alpha_meas[15]}}, alpha_meas} <<< 8;
                uq  <= {{16{duty_prev[15]}}, duty_prev};
                obs_active <= obs_enable && (alpha_meas < OBS_ALPHA_MAX) &&
                              (alpha_meas > -OBS_ALPHA_MAX);
                if (obs_enable && (alpha_meas < OBS_ALPHA_MAX) &&
                    (alpha_meas > -OBS_ALPHA_MAX)) begin
                    e1   <= (theta_meas <<< 8) - x0;      // innovation（旧状态）
                    e2   <= ({{16{alpha_meas[15]}}, alpha_meas} <<< 8) - x1;
                    mi   <= 2'd0;
                    mj   <= 3'd0;
                    acc_lo <= 32'sd0; acc_hi <= 32'sd0;
                    mul_done <= 1'b0;
                    busy <= 1'b1;
                end else begin
                    // 从动：θ/α 取量测、θ̇/α̇ 取差分链（切换零瞬态）
                    x0 <= theta_meas <<< 8;
                    x1 <= {{16{alpha_meas[15]}}, alpha_meas} <<< 8;
                    x2 <= {{16{theta_dot_meas[15]}}, theta_dot_meas} <<< 8;
                    x3 <= {{16{alpha_dot_meas[15]}}, alpha_dot_meas} <<< 8;
                end
            end else if (commit_q) begin
                busy <= 1'b0;
                commit_q <= 1'b0;
                x0 <= xn0; x1 <= xn1; x2 <= xn2; x3 <= xn3;
            end else if (busy) begin
                `ifdef OBS_DEBUG
                $display("MAC mi=%0d mj=%0d term=%0d coef=%0d acc=%0d accp=%0d",
                         mi, mj, term, obs_coef({1'b0, mi}, mj), acc, accp);
                `endif
                // 两拍 MAC：第 1 拍只锁乘积（32×32→64 独立成拍，断开关键路径），
                // 第 2 拍做 64 位累加 + 行末移位/提交。
                if (!mul_done) begin
                    prod_q   <= term * obs_coef({1'b0, mi}, mj);
                    mul_done <= 1'b1;
                end else begin
                    mul_done <= 1'b0;
                    // 拆分累加：低 32 位和（带进位）+ 高 32 位（含进位）。
                    // 行末（mj==6）把累加器清零，下一行从头开始——否则
                    // acc_lo 会带着上一行的 sum_lo 进入新行（历史 bug）。
                    acc_lo <= (mj == 3'd6) ? 32'sd0 : sum_lo;
                    acc_hi <= (mj == 3'd6) ? 32'sd0 : sum_hi;
                    if (mj == 3'd6) begin
                        case (mi)
                            2'd0: xn0 <= sat32((sum64 + RND) >>> 20);
                            2'd1: xn1 <= sat32((sum64 + RND) >>> 20);
                            2'd2: xn2 <= sat32((sum64 + RND) >>> 20);
                            2'd3: xn3 <= sat32((sum64 + RND) >>> 20);
                        endcase
                        mj <= 3'd0;
                        if (mi == 2'd3) begin
                            commit_q <= 1'b1;   // xn3 本拍才写入，提交推迟一拍
                        end else begin
                            mi <= mi + 2'd1;
                        end
                    end else begin
                        mj <= mj + 3'd1;
                    end
                end
            end
        end
    end

    // 输出：×256 域 → 控制口径（θ s32，α/速度饱和 s16）
    // 2026-10-20 时序修复：原先直接组合输出，观测器使能后 x0 成为 ctrl_core 里
    // θ 减法/饱和链的源端，关键路径变成 x0 → 31 位进位链 → the32 → theta_e
    // 比较器（Fmax 48.4MHz 违规）。此处加一级输出寄存器：线长锯断，观测器
    // 输出多 1ms 延迟（与从动模式本来就有的一拍同量级），逻辑语义不变。
    // 观测器更新本来就在 tick 后几拍内完成，此寄存不改变实际可用时刻。
    function signed [15:0] sat16(input signed [31:0] v);
        sat16 = (v > 32'sd32767) ? 16'sd32767 :
                (v < -32'sd32768) ? -16'sd32768 : v[15:0];
    endfunction

    reg signed [31:0] o_theta;
    reg signed [15:0] o_alpha, o_theta_dot, o_alpha_dot;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            o_theta <= 0; o_alpha <= 0; o_theta_dot <= 0; o_alpha_dot <= 0;
        end else begin
            o_theta     <= (x0 + 32'sd128) >>> 8;
            o_alpha     <= sat16((x1 + 32'sd128) >>> 8);
            o_theta_dot <= sat16((x2 + 32'sd128) >>> 8);
            o_alpha_dot <= sat16((x3 + 32'sd128) >>> 8);
        end
    end
    assign obs_theta     = o_theta;
    assign obs_alpha     = o_alpha;
    assign obs_theta_dot = o_theta_dot;
    assign obs_alpha_dot = o_alpha_dot;

    wire unused_ok = &{1'b0, x2[31:16], 1'b0};
endmodule
