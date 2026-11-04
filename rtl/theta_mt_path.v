// ============================================================================
// theta_mt_path.v — 电机侧 MT6701 绝对角 → 摆臂 θ/θ̇（experiment/mt6701-theta）
//
// 硬件背景（2026-10-19 用户定案）：电机保留 13 线霍尔正交编码器（qenc4x 原链
// 不动），电机尾部同轴加装 MT6701（SSI 14 位绝对角，径向磁铁叠装在霍尔磁铁
// 上——存在磁场干扰风险，故霍尔链保留为默认源，本模块为可选第二源）。
// 减速比 1:20.45（= 409/20，精确有理数）。摆侧 α 已由 MT6835 承担（V4）。
//
// 与霍尔链（qenc4x→arm_vel，CPR=1040）的关系：
//   · 分辨率：1 电机 LSB = 360°/(16384×20.45) = 0.0010745° 摆臂角 = 0.10745 c01，
//     比霍尔 1 计数（34.6 c01）细 322 倍；M 法 1ms 差分台阶 10.7（0.1°/s 单位），
//     对比霍尔 4ms 差分台阶 865——T 法测速在本链上没有必要（USE_T 退役）。
//   · 尺度：本链按真实 20.45 折算；霍尔链 CNT_PER_REV=1040=52×20.0，把 θ 放大
//     2.2%（52×20.45=1063.4 才是真值）。POS 精度与辨识以本链为准。
//   · 零点：上电位置 = θ 0（与 qenc4x 复位清零同语义），两链同瞬间起算，
//     源切换的瞬态只来自 2.2% 尺度差×累计角，量级无害。
//   · 抗扰：绝对角传感器，单帧毛刺不累积（下一帧自愈）；Δ 限幅 ±2048 LSB
//     只堵溢出/断线毛刺（物理最大 ~725 LSB/ms @ 44 电机转/s）。
//
// 数据通路：
//   angle14 变化沿（clk 域）→ 15 位模 16384 回绕差分 d（signed 截位自动回绕）
//   → 限幅 ±D_MAX → acc(s64) += d × K_Q16      （K_Q16=7042 = round(2^16×36000
//   ×20/409/16384)，θ 累计在 Q16-c01 域，>>16 即 0.01°）
//   θ 输出 = sat32(acc>>>16)，组合直出（acc 事件驱动更新，tick 消费时与霍尔链
//   cnt2deg 同等新鲜度）；θ̇ 在 tick 域 1ms 差分 + Q8 EMA + 限斜率，乘法段按
//   clk 单独流水一级（arm_vel 教训：串进 EMA 链会 39 级逻辑卡 33MHz）——群延迟
//   ~3ms（霍尔链 4ms 差分+EMA ~5ms）。
//
// 首拍 primed 标志：复位后首个采样只记基准不积分（否则上电停在任意角会算出
// 假 Δ）。故障语义继承 mt6701_ssi_if 的 hold-last-good：坏帧时 angle14 保持，
// 本链 Δ=0 → θ 冻结、θ̇ 自然归零（安全链兜底见 t20k_top：选中本源且传感
// 故障 → otr_fault）。
// ============================================================================
`timescale 1ns / 1ps
module theta_mt_path #(
    parameter integer GEAR_NUM   = 409,   // 减速比 20.45 = 409/20（电机转数:臂转数）
    parameter integer GEAR_DEN   = 20,
    parameter        DIR_FLIP    = 1'b0,  // 1: Δ 取反（实机方向核对后定案）
    parameter integer EMA_SH     = 2,     // θ̇ EMA 移位（对齐 arm_vel 默认）
    parameter signed [15:0] SLEW_LIM = 16'sd0, // θ̇ 每拍限斜率（0.1°/s/拍，0=禁用；
                                           //  语义同 arm_vel，透传板级 THDOT_SLEW_LIM）
    parameter integer D_MAX      = 2048,  // 单事件 Δ 限幅（电机 LSB）
    // 判停阈值（拍，1ms）。与 arm_vel 的 STALL_TICKS 同语义：这么久没有任何
    // 角度事件就认为臂真停。2026-10-22 霍尔编码器拆除后，θ 只剩 MT6701 一个
    // 源，模式机的 POS 入场"臂真停"门（arm_stalled）必须由本链提供。
    parameter integer STALL_TICKS = 300
)(
    input  wire               clk,
    input  wire               rst_n,
    input  wire               tick,       // 1ms 控制拍
    input  wire [13:0]        angle14,    // MT6701 电机轴绝对角 0..16383
    output wire signed [31:0] theta_c01,  // 摆臂角 0.01°（上电位置=0，多圈连续）
    output reg  signed [15:0] theta_dot,  // 0.1°/s（1ms 差分 + EMA + 限斜率）
    output reg                arm_stalled // 1=连续 STALL_TICKS 拍无角度事件（臂真停）
);
    // K_Q16 = round(2^16 × 36000/(16384 × GEAR_NUM/GEAR_DEN))
    //       = round(144000×GEAR_DEN/GEAR_NUM) = round(2880000/409) = 7042（误差 −0.006%）
    localparam signed [31:0] K_Q16 =
        (144000 * GEAR_DEN + GEAR_NUM / 2) / GEAR_NUM;

    // ================= 采样事件域：回绕差分 + Q16 积分 =================
    reg [13:0] prev14;
    reg        primed;          // 首个采样只建基准
    reg signed [63:0] acc;      // Q16-c01 累计（s64：±2.9e8 电机圈不溢出）
    wire raw_chg = primed && (angle14 != prev14);
    // 14 位差按 14 位有符号解释（符号位=bit13）= 模 16384 回绕差分：
    // 模值 8192..16383 → −8192..−1。勿零扩到 15 位——那会让 bit14 当符号恒 0，
    // 反向大跳变被算成正向（tb 毛刺用例抓出）。
    wire signed [13:0] d_s14  = $signed(angle14 - prev14);
    wire signed [13:0] d_clamped = (d_s14 >  D_MAX) ?  D_MAX :
                                    (d_s14 < -D_MAX) ? -D_MAX : d_s14;
    wire signed [13:0] d_eff = DIR_FLIP ? -d_clamped : d_clamped;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            prev14 <= 14'd0;
            primed <= 1'b0;
            acc    <= 64'sd0;
        end else if (!primed) begin
            prev14 <= angle14;      // 首采样：建基准，不积分
            primed <= 1'b1;
        end else if (raw_chg) begin
            acc    <= acc + 64'sd1 * d_eff * K_Q16;
            prev14 <= angle14;
        end
    end

    // θ 组合直出（消费方都在 tick 沿读，事件更新的 acc 保证新鲜度与 cnt2deg 等价）
    wire signed [63:0] acc_shr = acc >>> 16;
    assign theta_c01 = (acc_shr >  64'sd2147483647) ? 32'sd2147483647 :
                       (acc_shr < -64'sd2147483648) ? -32'sd2147483648 :
                                                      acc_shr[31:0];

    // ================= tick 域：1ms 差分 θ̇ + EMA + 限斜率（两级流水）=================
    // 流水切分承 arm_vel 的教训：乘法必须从 EMA/限斜率链隔离出来单独打一拍，
    // 否则 64 位减+×100+EMA+限斜率串成一拍（实测 39 级逻辑，Fmax 33.1MHz）。
    // 拍 A（tick）：d_acc_q = sat(acc − acc_prev)；拍 B（clk）：dps_raw_q 乘法段；
    // EMA/限斜率在下一 tick 消费 dps_raw_q —— θ̇ 比 θ 晚 1ms 反映运动，群延迟
    // ~3ms（霍尔链 4ms 差分+EMA ~5ms，仍有优势）。
    reg signed [31:0] d_acc_q;      // 拍 A：相邻 tick 的 acc 差
    reg signed [15:0] dps_raw_q;    // 拍 B：d_acc×100>>>16（s16 饱和）
    reg signed [63:0] acc_prev;     // 上 tick 的 acc 快照
    wire signed [63:0] d_acc64 = acc - acc_prev;
    // 正常 ≤ ±2048×7042 = ±14.4M；s32 截位足够且护住毛刺双跳（+2048 后 −2048）
    wire signed [31:0] d_acc = (d_acc64 >  64'sd2147483647) ? 32'sd2147483647 :
                               (d_acc64 < -64'sd2147483648) ? -32'sd2147483648 :
                                                              d_acc64[31:0];
    wire signed [63:0] dps_ext = d_acc_q * 64'sd100;
    wire signed [63:0] dps_shr = dps_ext >>> 16;
    wire signed [31:0] dps_sat = (dps_shr >  64'sd32767) ? 32'sd32767 :
                                 (dps_shr < -64'sd32768) ? -32'sd32768 :
                                                            dps_shr[31:0];
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            acc_prev  <= 64'sd0;
            d_acc_q   <= 32'sd0;
            dps_raw_q <= 16'sd0;
        end else begin
            dps_raw_q <= dps_sat[15:0];       // 乘法段按 clk 流水（同 arm_vel vprod）
            if (tick) begin
                d_acc_q  <= d_acc;
                acc_prev <= acc;
            end
        end
    end

    // EMA（Q8 内部，对齐 arm_vel 公式）+ 限斜率（对上一拍限幅输出限差）
    reg  signed [31:0] ema;
    wire signed [31:0] dps_ext16 = {{16{dps_raw_q[15]}}, dps_raw_q};
    wire signed [31:0] ema_next = ema + (((dps_ext16 <<< 8) - ema) >>> EMA_SH);
    wire signed [15:0] ema_out = sat16((ema_next + 32'sd128) >>> 8);

    reg  signed [15:0] slew_q;
    wire signed [16:0] d_slew = {ema_out[15], ema_out} - {slew_q[15], slew_q};
    wire signed [16:0] lim_ext = $signed({{9{SLEW_LIM[15]}}, SLEW_LIM});
    wire signed [16:0] d_clip  = (d_slew >  lim_ext) ? lim_ext :
                                 (d_slew < -lim_ext) ? -lim_ext : d_slew;
    wire signed [15:0] slew_next = (SLEW_LIM == 0) ? ema_out
                                                   : slew_q + d_clip[15:0];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ema      <= 32'sd0;
            slew_q   <= 16'sd0;
            theta_dot<= 16'sd0;
        end else if (tick) begin
            ema      <= ema_next;
            slew_q   <= slew_next;
            theta_dot<= slew_next;
        end
    end

    function signed [15:0] sat16(input signed [31:0] v);
        sat16 = (v >  32'sd32767) ? 16'sd32767 :
                (v < -32'sd32768) ? -16'sd32768 : v[15:0];
    endfunction

    // ================= 判停：连续 STALL_TICKS 拍无角度事件 =================
    // raw_chg 在 clk 域（事件驱动），stall 计数按控制拍推进——语义照抄
    // arm_vel 的 v_t_zero（那边判的是无计数边沿，这边判的是无角度变化）。
    // 注意：MT6701 掉线（磁场干扰/ CRC 坏帧）时 angle14 保持旧值 ⇒ raw_chg 不出现
    // ⇒ 本标志会误判"真停"。这是已知残余风险：掉线期间 θ̇ 本来也塌到 0，
    // 判停门与 θ̇ 门同源退化，不额外引入新的错误（POS 入场还另有 |α|/|α̇| 门）。
    reg [15:0] stall_cnt;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            stall_cnt   <= 16'd0;
            arm_stalled <= 1'b0;
        end else if (raw_chg) begin
            stall_cnt   <= 16'd0;
            arm_stalled <= 1'b0;
        end else if (tick) begin
            if (stall_cnt < 16'd65535) stall_cnt <= stall_cnt + 16'd1;
            if (stall_cnt >= STALL_TICKS) arm_stalled <= 1'b1;
        end
    end
endmodule
