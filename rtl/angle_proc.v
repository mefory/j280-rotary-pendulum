// ============================================================================
// angle_proc.v — 摆角标定 / 换算 / 差分角速度 + 竖直零位自校准
// 链路：alpha_avg(u16) → 0.01° 原始角 → 减下垂点 offset（按键标定，均值 2s）
//       → +倒立点偏置(±18000) → 回绕 (−18000,18000] → alpha s16
//       → 4ms 跨拍差分 ×25 → EMA → alpha_dot s16
// 极性开关 POL_ALPHA_FLIP / ALPHA_UP_OFF 见 j280_defs.vh（到手实测后改）。
//
// 竖直零位自校准（2026-10-20 定案）：物理上稳摆唯一准静态平衡位是物理竖直
// （α_true 均值=0），故 BALANCE 期间测得 α 的长时间均值=传感器零偏。慢 EMA
// 只累积不改控（会话内控制行为零变化），退出 BALANCE 沿一次性提交进 offset。
// 刻意不做控制律 α 积分：积分不区分零偏与误差，会把假读数顶给 θe（臂漂移
// 缠绕），修 offset 才是治本。AZ 参数仅整定用，ctrl_core AZ_ENABLE 门控。
// ============================================================================
`include "j280_defs.vh"
module angle_proc #(
    parameter DIFF_MS  = 4,      // 差分跨距（ms）
    parameter EMA_SH   = 2,
    parameter CAL_TICKS = `CAL_AVG_TICKS,
    // ---- 自校准整定参数（tb 缩短时标用小值，板级默认即实机值）----
    parameter [4:0]  AZ_EMA_SH    = 5'd13,    // α 均值 EMA 移位，τ≈2^13 ms=8s
    parameter [15:0] AZ_MIN_TICKS = 16'd15000, // 有效采样时长门 15s
    parameter [10:0] AZ_DELTA_MAX = 11'd500,   // 提交限幅 ±5°（防异常值跑飞）
    parameter [15:0] AZ_ALPHA_WIN = 16'd1500,  // |α|<15° 才计入（滤捕获瞬态）
    parameter        AZ_TRIM_EN   = 1'b1,      // 0=完全禁用 AZ 提交路径（保留代码）
    // 满量程口径（Q16：counts→0.01°）。默认 65472↔345.00° 电位器链
    // （LTC2308/ADS1220）→ round(34500*65536/65472)=34534；T20K+MT6701
    // 全 360° 直读（angle14<<2）传 36000（65536↔360.00°）。
    parameter [15:0] SCALE_Q16 = 16'd34534
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        tick,          // 1ms
    input  wire [15:0] alpha_avg,     // adc_if 输出（更新率 ~48kHz，tick 时取最新）
    input  wire        calib_start,   // 按键触发，摆杆自然下垂时按下
    input  wire        az_en,         // 零位自校准会话门（ctrl_core：M_BALANCE）
    input  wire signed [15:0] trim_c01, // 运行时零位微调（参数 id13），与 offset 相加
    output reg  signed [15:0] alpha,      // 0.01°，零点=倒立点
    output wire signed [15:0] alpha_pre,  // 0.01°，trim 之前（= raw − offset）
    output reg  signed [15:0] alpha_dot,  // 0.1°/s
    output reg  signed [15:0] offset_c01, // 标定值（遥测用）
    output reg                calib_done
);
    wire [31:0] raw_prod = alpha_avg * SCALE_Q16;
    wire signed [31:0] raw_c01 = $signed({16'd0, raw_prod[31:16]});
    reg  signed [31:0] raw_s;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) raw_s <= 32'sd0;
        else        raw_s <= raw_c01; // 0..34500 must not saturate at 32767
    end

    // ---- 下垂点标定：CAL_TICKS 均值 ----
    // 乘 Q24 倒数后四舍五入，避免综合出长组合除法链。
    localparam [31:0] RECIP_K = (64'd16777216 + CAL_TICKS/2)/CAL_TICKS;
    reg        calib_run;
    reg [31:0] cal_cnt;
    reg [31:0] cal_sum;
    wire [31:0] cal_total = cal_sum + raw_c01;
    reg [31:0] cal_total_r;
    reg [63:0] cal_product_r;
    reg signed [31:0] cal_mean;
    wire [63:0] cal_rounded = cal_product_r + 64'd8388608;
    wire signed [31:0] cal_offset = (cal_mean > 32'sd18000) ?
                                     cal_mean - 32'sd36000 : cal_mean;
    reg [3:0] cal_valid;
    // ---- 竖直零位自校准：会话内只累积，退出沿一次性提交 ----
    reg  [15:0] az_cnt;             // 窗内有效 tick 累计
    reg  signed [31:0] az_ema_q8;   // α 均值 EMA（Q8，防整数舍入直流偏）
    reg         az_en_d;
    localparam signed [16:0] AZ_WIN = $signed({1'b0, AZ_ALPHA_WIN});
    localparam signed [31:0] AZ_DMAX = $signed({{21{1'b0}}, AZ_DELTA_MAX});
    wire signed [31:0] az_alpha_q8 = {{16{alpha[15]}}, alpha} <<< 8;
    wire        az_valid = az_en && (alpha < AZ_WIN) && (alpha > -AZ_WIN);
    wire signed [31:0] az_mean  = (az_ema_q8 + 32'sd128) >>> 8;
    wire signed [31:0] az_delta = (az_mean > AZ_DMAX)  ? AZ_DMAX  :
                                  (az_mean < -AZ_DMAX) ? -AZ_DMAX : az_mean;
    wire signed [31:0] az_off_sum = {{16{offset_c01[15]}}, offset_c01} + az_delta;
    // 提交后仍回绕到 (-18000,18000]（|Δ|≤5° ⇒ 单次 ±36000 环绕足够）
    wire signed [31:0] az_off_new = (az_off_sum > 32'sd18000)   ? az_off_sum - 32'sd36000 :
                                    (az_off_sum <= -32'sd18000) ? az_off_sum + 32'sd36000 :
                                                                  az_off_sum;
    // Four 50MHz clocks after the final sample; no extra control-period delay.
    // Separate sum, multiply, rounding and wrapping to meet 50MHz.
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n)begin cal_total_r<=0;cal_product_r<=0;cal_mean<=0;cal_valid<=0;end
        else begin
            if(calib_run && tick && cal_cnt==CAL_TICKS-1)cal_total_r<=cal_total;
            cal_product_r<=cal_total_r*RECIP_K;
            cal_mean<=$signed(cal_rounded[55:24]);
            cal_valid<={cal_valid[2:0],(calib_run && tick && cal_cnt==CAL_TICKS-1)};
        end
    end
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            calib_run <= 1'b0; cal_cnt <= 0; cal_sum <= 0;
            offset_c01 <= 16'sd8344;   // 默认 ≈ 官方下垂口径 990@12bit 换算，实机按键重标
            calib_done <= 1'b0;
            az_cnt <= 16'd0; az_ema_q8 <= 32'd0; az_en_d <= 1'b0;
        end else begin
            if (cal_valid[3]) begin
                offset_c01 <= cal_offset[15:0]; // 已回绕到 (-18000,18000]，可用 s16 表示
                calib_done <= 1'b1;
            end
            // 自校准：EMA/计数按 tick 累积（会话内不改 offset，控制行为零变化）；
            // az_en 下降沿过时长门一次性提交。与 cal 路天然互斥（cal 仅
            // SAFE/IDLE 触发采样，az_en 仅 BALANCE 有效），offset 无同拍竞争。
            az_en_d <= az_en;
            if (tick && az_valid) begin
                az_ema_q8 <= az_ema_q8 + ((az_alpha_q8 - az_ema_q8) >>> AZ_EMA_SH);
                if (az_cnt != 16'hFFFF) az_cnt <= az_cnt + 16'd1;
            end
            if (AZ_TRIM_EN && az_en_d && !az_en) begin
                if (az_cnt >= AZ_MIN_TICKS)
                    offset_c01 <= az_off_new[15:0];
                az_cnt    <= 16'd0;   // 会话状态清零（无论是否提交）
                az_ema_q8 <= 32'd0;
            end
            if (calib_start && !calib_run && cal_valid==0) begin
                calib_run <= 1'b1; cal_cnt <= 0; cal_sum <= 0; calib_done <= 1'b0;
            end else if (calib_run && tick) begin
                cal_sum <= cal_sum + raw_c01;
                cal_cnt <= cal_cnt + 1;
                if (cal_cnt == CAL_TICKS-1) begin
                    calib_run  <= 1'b0;
                end
            end
        end
    end

    // ---- 换算到倒立点零点（第 2 拍寄存）----
    wire signed [31:0] delta = raw_s - {{16{offset_c01[15]}},offset_c01};
    wire signed [31:0] d0 = `POL_ALPHA_FLIP ? -delta : delta;
    // 运行时零位微调（参数 id13）：直接叠加，正 trim ⇒ α 读数减小。
    // 语义：offset 增大使 delta 减小 ⇒ α 减小；trim 与 offset 同向同量，
    // 故等价于 offset + trim，但走独立通路上位机可写。
    wire signed [31:0] up = d0 - {{16{trim_c01[15]}}, trim_c01} + 32'sd18000;
    // 遥测用：trim 之前的角（= raw − offset，±18000 内）。有了它就能从记录
    // 直接读出"当轮真实零偏是多少"，不必再用 offset−trim 反推；排查 POS
    // 掉摆时正是卡在这（见 docs/记录格式v3.md）。
    assign alpha_pre = sat16(d0);
    // 回绕到 (−18000, 18000]
    reg signed [31:0] wrapped;
    always @* begin
        if      (up >  32'sd54000) wrapped = up - 32'sd72000;
        else if (up >  32'sd18000) wrapped = up - 32'sd36000;
        else if (up <= -32'sd54000) wrapped = up + 32'sd72000;
        else if (up <= -32'sd18000) wrapped = up + 32'sd36000;
        else                        wrapped = up;
    end
    reg signed [15:0] wrapped_r;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) wrapped_r <= 16'sd0;
        else        wrapped_r <= sat16(wrapped);
    end

    // ---- 差分角速度（DIFF_MS 跨拍）+ EMA ----
    reg signed [15:0] a0, a1, a2, a3;
    wire signed [15:0] a_old = (DIFF_MS == 4) ? a3 : a0;
    // 回绕感知差分：跨 ±180° 时修正 ±36000，否则微分尖峰污染能量估计
    wire signed [31:0] d4r   = wrapped_r - a_old;       // 0.01° per 4ms
    wire signed [31:0] d4    = (d4r >  32'sd18000) ? d4r - 32'sd36000 :
                               (d4r < -32'sd18000) ? d4r + 32'sd36000 : d4r;
    // 寄存一拍切断组合链；α̇ 多 1ms 延迟，已纳入模型和验收。
    reg signed [31:0] d4_q;
    wire signed [31:0] ad_raw = d4_q * 16'sd25;         // ×25 → 0.1°/s
    reg signed [31:0] ema;
    wire signed [31:0] ad_clip = {{16{sat_ad[15]}},sat_ad};
    wire signed [15:0] sat_ad = sat16(ad_raw);
    // Q8 EMA state avoids the negative DC bias of integer >>>2 feedback.
    wire signed [31:0] ema_next = ema + (((ad_clip <<< 8)-ema) >>> EMA_SH);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            a0<=0; a1<=0; a2<=0; a3<=0; d4_q<=0; ema<=0;
            alpha<=0; alpha_dot<=0;
        end else if (tick) begin
            a0 <= wrapped_r; a1 <= a0; a2 <= a1; a3 <= a2;
            d4_q <= d4;
            ema <= ema_next;
            alpha     <= wrapped_r;
            alpha_dot <= sat16((ema_next+32'sd128) >>> 8);
        end
    end

    function signed [15:0] sat16(input signed [31:0] v);
        sat16 = (v >  32'sd32767) ? 16'sd32767 :
                (v < -32'sd32768) ? -16'sd32768 : v[15:0];
    endfunction
endmodule
