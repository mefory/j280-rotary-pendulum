// ============================================================================
// arm_vel.v — 摆臂角速度与角度换算（M/T 混合 + Q8 EMA）
//
// 【低速 T 法：已实现并验证，发布默认启用（USE_T=1）】
//   动机：M 法低速每计数台阶 865 单位（86.5°/s），定位段摆臂只有 ~2°/s，θ̇ 在
//   0/±865 间跳；K4 是位置环里的大项，会把它放大成大指令驱动摆臂来回摆。
//   T 法（量相邻计数边沿间隔）在低速连续平滑，实现与参数都已验证（tb_arm_vel）。
//   当前 RTL 四方向闭环、Python 镜像和 MATLAB 控制器均已同步；串口任务
//   host.test_rtl_mission 在 USE_T=1 配置下通过（progress +1420）。
//   T 法仍需真实编码器低速和齿隙数据做最终板上确认。
//   T 法的两个要点（都不是原实现的样子）：
//     · 时间基准是**控制拍**（1ms）而不是 50MHz 时钟——仿真 tb 常把一拍压成几十个
//       时钟，用时钟计数会让 v 放大 1000 倍而失效；用拍计数则仿真与实机自动同口径。
//     · 除数取 `max(上次边沿周期, 距上次边沿的实时拍数)`：原"保持上次边沿周期"会在
//       臂停下后把陈旧高速值挂住 STALL_TICKS(300ms)——起摆结束、捕捉瞬间最危险
//       （K4 会把它放大成大指令）；因此必须使用实时经过拍数参与除数。
//   混合切换点：|4ms 差分| ≥ 2 计数（≥173°/s）走 M（保持已验证行为），否则走 T；
//   两法在交叉点数值一致（1 计数/4ms ⇔ 865），切换无台阶。
//
// 【差分窗口 = 4 拍，别去"修"它】
//   h0..h3 是 4 级移位链，`dpos = pos - h3` 取的是"当前 pos"与"4 拍前的 pos"之差，
//   在硬件上就是 4ms 差分，与 M_SCALE 的 4ms 定标、MATLAB 采样环模型
//   （lqr_lag_check 的 th(1)-th(5)）三者一致。
//   注意：在按 tick 锁步改 pos 的 testbench 里，DUT 的寄存器与 tb 的赋值在同一沿竞争，
//   会读成 3 拍（1 计数/拍 → 2595 而非 3462）——那是 tb 的相位假象，**不是**实现问题。
//   判据以硬件口径（4 拍）为准，tb 期望值按 tb 可观测值标注。
//
// 【时间基准：控制拍，不是 50MHz 时钟】
//   T 法用 **tick（1ms 控制拍）** 计数边沿间隔，而不是原始时钟周期。
//   原因：仿真里 tb 常把一拍压缩成几十个时钟（tb_closedloop 是 50 时钟/拍），
//   用时钟计数会让 v 放大 1000 倍而失效；用拍计数则仿真与实机**自动同口径**
//   （每拍恒等于 1ms 的物理约定），不再需要"VNUM 与时钟周期同比缩小"的配套 hack。
//   代价：低速分辨率 1 拍（1ms）——对定位段足够（2°/s 时边沿间隔 180 拍）。
//
// 时序：dpos 先饱和到 18bit 再乘（寄存）；角度换算走 cnt2deg 两级流水；
//       除法器 28 周期顺序恢复余数法，50MHz 下 560ns ≪ 1ms 节拍。
// ============================================================================
`include "j280_defs.vh"
module arm_vel #(
    parameter WIN_MS       = 4,        // M 法差分窗口（ms）
    parameter EMA_SH       = 2,        // EMA 移位 2^n
    parameter CPR          = `CNT_PER_REV,
    parameter USE_T        = 1,        // 1: 低速段用 T 法。2026-09-23 实机证据：M 法低速
                                       // 量化（1 计数=865 单位）经 K4 放大成 ±9000 duty
                                       // 的输出抖动（实机 duty 峰峰 14770、换向 34.5 次/s、
                                       // 咔咔共振），且 K4 不可砍（砍到 8000 捕获后立不住）。
                                       // T 法只在 <173°/s 低速段接管，高速捕获仍走 M 法。
    parameter STALL_TICKS  = 300,      // 无边沿判停阈值（拍）：臂停下 300ms 内 θ̇ 归零
    parameter signed [15:0] SLEW_LIM = 0, // θ̇ 每拍最大变化（0.1°/s/拍）；0=禁用。
                                       // T20K 零点刹车配置 1000：削 T 法单拍伪迹
                                       // （1730+）而物理加速（≤900/拍）不受限
    // 1=保留原先按控制拍更新流水寄存器；0=按 FPGA 时钟更新，减少两拍测速延迟。
    // EMA 与限斜率状态仍只在 tick 更新。默认兼容，T20K 单独启用时钟流水。
    parameter PIPELINE_ON_TICK = 1,
    // VNUM = 10*360*1000/CPR（tick=1ms 口径；CPR=1040 → 3461.5）
    // 即"1 计数/1 拍"对应的 0.1°/s 值
    parameter [27:0] VNUM  = 3_600_000 / CPR
) (
    input  wire                clk,
    input  wire                rst_n,
    input  wire                tick,        // 1ms
    input  wire signed [31:0]  pos,         // 编码器计数
    output wire signed [31:0]  theta_c01,   // 0.01°（2 拍延迟，cnt2deg）
    output reg  signed [15:0]  theta_dot,   // 0.1°/s（M/T 混合 + EMA 后）
    output wire                arm_stalled  // 臂真停标志=v_t_zero（2026-09-30 导出：
                                           // T 法 θ̇ 读数在静止粘滑下 0↔400°/s 尖峰，
                                           // 阈值判不住物理静止，判停标志才判得住）
);
    // ================= M 法：4 拍窗差分（高速段）=================
    // h0..h3 四级移位链 ⇒ dpos = pos - h3 是 4ms 差分（见文件头说明）。
    reg signed [31:0] h0, h1, h2, h3;
    wire signed [31:0] dpos = pos - h3;

    // dpos 饱和到 ±131071 计数/ms（远超物理可能），压乘法位宽
    wire signed [17:0] dpos_c = (dpos >  32'sd131071) ? 18'sd131071 :
                                (dpos < -32'sd131072) ? -18'sd131072 : dpos[17:0];
    // 0.1°/s per count per 4ms = 360000*10/(CPR*4) ≈ 865.4 → ×1731 >>>1
    reg signed [31:0] vprod;
    localparam signed [15:0] M_SCALE=(7200000+CPR*WIN_MS/2)/(CPR*WIN_MS);
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) vprod <= 32'sd0;
        else        vprod <= dpos_c * M_SCALE;
    end
    wire signed [31:0] v_raw = vprod >>> 1;

    // ================= T 法：边沿间按控制拍计时（低速段）=================
    // 除数 den 取「上次边沿周期 p_cap」与「距上次边沿的实时时间」的较大者：
    //   · 臂匀速运动时二者相等 ⇒ 估计有效（v = VNUM/T）；
    //   · 臂减速或停下时实时时间变大 ⇒ 估计**平滑衰减**，不会像"只保持上次周期"
    //     那样在停车后继续输出一个陈旧的高速值（那会让 K4 在停车后打出大指令，
    //     起摆结束转入稳摆的瞬间尤其危险——这也是当初 USE_T 被排除出发版的原因）。
    // 除法每拍重触发一次（28 clk ≪ 50000 clk/拍，不冲突）；边界处由判停直接归零。
    // n 拍(ms) 内走 1 计数 → v[0.1°/s] = 10*360*1000/(CPR*n) = VNUM/n
    localparam [11:0] P_MAX = 12'd4095;    // 边沿周期上限（>4s 视为不动，v=0）
    reg [23:0]  tcnt;                  // tick 计数（1kHz，4.6 小时回绕）
    reg [23:0]  t_edge;                // 上次边沿时刻（拍）
    reg signed [31:0] pos_d;
    wire        pos_chg = (pos != pos_d);
    reg         dir_pos;               // 本边沿方向（正计数=1）
    reg [15:0]  stall;                 // 距上次边沿的 tick 数（判停）
    reg         v_t_zero;              // 判停标志 → T 法输出 0
    assign arm_stalled = v_t_zero;     // 对外导出判停（POS 入场确定性门）
    reg  [25:0] p_cap;                 // 锁存的边沿周期（拍）
    wire [23:0] elapsed = tcnt - t_edge;
    wire [25:0] den = (elapsed > p_cap[23:0]) ? {2'd0, elapsed} : p_cap;

    // 周期捕获 + 方向（每边沿一次）
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tcnt <= 24'd0; t_edge <= 24'd0; pos_d <= 32'sd0;
            dir_pos <= 1'b1; p_cap <= {14'd0, P_MAX};
        end else begin
            pos_d <= pos;
            if (tick) tcnt <= tcnt + 24'd1;
            if (pos_chg) begin
                t_edge  <= tcnt;
                p_cap   <= ((tcnt - t_edge) > {12'd0, P_MAX})
                           ? {14'd0, P_MAX}
                           : {2'd0, tcnt - t_edge};
                dir_pos <= (pos > pos_d);
            end
        end
    end

    // 判停：无边沿 STALL_TICKS 拍 → T 法输出 0（同时防 tcnt 回绕混叠）
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)                  stall <= 16'd0;
        else if (pos_chg)            stall <= 16'd0;
        else if (tick && stall < 16'hFFFF) stall <= stall + 16'd1;
    end
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)                  v_t_zero <= 1'b0;
        else if (pos_chg)            v_t_zero <= 1'b0;
        else if (tick && stall >= STALL_TICKS) v_t_zero <= 1'b1;
    end

    // 顺序恢复余数除法：商 = VNUM / den（28 周期）
    // 边沿触发 + 每拍重触发（跟踪 den 的增长 ⇒ 停车后估计平滑衰减）
    reg        div_run;
    reg        div_fin;
    reg [5:0]  div_cnt;
    reg [27:0] div_num;
    reg [26:0] div_rem;
    reg signed [15:0] v_t_q;           // 商（饱和到 s16）
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            div_run <= 1'b0; div_fin <= 1'b0; div_cnt <= 0;
            div_num <= 0; div_rem <= 0; v_t_q <= 16'sd0;
        end else if (div_fin) begin
            // 商已就绪（上周期完成末次移位）：饱和锁存
            v_t_q   <= (|div_num[27:15]) ? 16'sd32767 : div_num[15:0];
            div_fin <= 1'b0;
        end else if (div_run) begin
            // 恢复余数法一步：{rem, num MSB} 试减除数
            if ({div_rem[25:0], div_num[27]} >= {1'b0, den}) begin
                div_rem <= {div_rem[25:0], div_num[27]} - {1'b0, den};
                div_num <= {div_num[26:0], 1'b1};
            end else begin
                div_rem <= {div_rem[25:0], div_num[27]};
                div_num <= {div_num[26:0], 1'b0};
            end
            div_cnt <= div_cnt + 6'd1;
            if (div_cnt == 6'd27) begin
                div_run <= 1'b0;
                div_fin <= 1'b1;
            end
        end else if (pos_chg || (tick && !v_t_zero)) begin
            div_num  <= VNUM;
            div_rem  <= 27'd0;
            div_cnt  <= 6'd0;
            div_run  <= 1'b1;
        end else if (v_t_zero) begin
            v_t_q <= 16'sd0;           // 判停：T 法输出 0
        end
    end
    wire signed [15:0] v_t = v_t_zero ? 16'sd0 :
                             (dir_pos ? v_t_q : -v_t_q);

    // ================= M/T 混合：|dpos|≥2 计数用 M，否则用 T =================
    wire use_m = (dpos_c >= 18'sd2) || (dpos_c <= -18'sd2);
    // 旧写法 (!USE_T || use_m)：USE_T=0 时恒取 v_raw，等于把 T 法整段短路；
    // 修正为"仅当启用 T 且处于低速段才用 T"。
    wire signed [31:0] v_blend = (USE_T && !use_m) ? {{16{v_t[15]}}, v_t} : v_raw;

    // v_blend 先打一拍再进 EMA：把 28 周期除法器/M 乘法从 EMA 的同拍组合前级
    // 中切出来（加限制器后综合器重定时把 qenc→除法→EMA→限制器串成一条
    // 26ns 关键路径，Fmax 掉到 38MHz；寄存后每段独立满足 50MHz）。
    // PIPELINE_ON_TICK=1 时这里多 1ms、slew_raw 再多 1ms；不能当成 20ns 流水。
    // T20K 选择 0，将两级寄存改为时钟流水；滤波和限斜率仍保持控制拍状态。
    reg signed [15:0] blend_q;
    wire signed [15:0] blend_sat = sat16(v_blend);
    reg signed [31:0] ema;
    // blend_q 的符号扩展必须是独立 signed wire——内联拼接 {{16{...}},...}
    // 是无符号的，会把整个表达式传染成无符号算术（减法 wrap + >>> 变逻辑
    // 移位，EMA 爆炸，实测 ema 3534839→1077057657）。同理见限制器 lim_ext 注释。
    wire signed [31:0] blend_ext = {{16{blend_q[15]}}, blend_q};
    wire signed [31:0] ema_next=ema+((( blend_ext <<< 8)-ema) >>> EMA_SH);

    // ================= θ̇ 斜率限制器（2026-10-09，方案文档 §5-1）=================
    // 目标：掐断 K4×T法×刹车 的 20-60ms 短脉冲自激（171403 日志：439 次/84s，
    // θ̇ 单拍跳变 750~900 单位/拍，T 法量化伪迹单拍可达 1730+）。物理约束：
    // 底座电机全力推进 θ̇ 也只能 ~900 单位/拍，故每拍变化钳到 SLEW_LIM 只削
    // 伪迹不伤真运动（对比一阶低通会吃相位裕量）。SLEW_LIM=0 禁用（DE10 默认）。
    // 时序：ema→slew_raw 流水，slew_q 每个控制拍限幅。首版把 EMA
    // 计算与限制器串成同拍组合，qenc→slew_q 关键路径 25ns 致 Fmax 39MHz。
    // 流水按控制拍还是时钟更新由 PIPELINE_ON_TICK 决定，需分别验证相位与 STA。
    reg signed [15:0] slew_raw, slew_q;
    wire signed [15:0] ema_out_raw = sat16((ema_next+32'sd128) >>> 8);
    // Limit against the previous LIMITED output, using the registered target.
    // Using slew_raw as state leaks the raw jump through on the next tick.
    wire signed [16:0] d_slew = {slew_raw[15], slew_raw}
                              - {slew_q[15], slew_q};
    // 注意：拼接结果恒为无符号，直接参与有符号比较会把两侧都无符号化
    // （d_slew=0 被判 < +SLEW_LIM → 错取 -SLEW_LIM，实测 td=-1000）。故先
    // 转成 signed 局部 wire 再比较。
    wire signed [16:0] lim_ext = $signed({{9{SLEW_LIM[15]}}, SLEW_LIM});
    wire signed [16:0] d_clip  = (d_slew >  lim_ext) ? lim_ext :
                                 (d_slew < -lim_ext) ? -lim_ext : d_slew;
    wire signed [15:0] slew_next = SLEW_LIM == 0 ? slew_raw
                                                 : slew_q + d_clip[15:0];
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            blend_q <= 16'sd0;
            slew_raw <= 16'sd0;
        end else if (!PIPELINE_ON_TICK || tick) begin
            blend_q <= blend_sat;
            slew_raw <= ema_out_raw;
        end
    end
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            h0 <= 0; h1 <= 0; h2 <= 0; h3 <= 0;
            ema <= 0;
            slew_q <= 16'sd0; theta_dot <= 16'sd0;
        end else if (tick) begin
            h0 <= pos; h1 <= h0; h2 <= h1; h3 <= h2;
            ema       <= ema_next;
            slew_q    <= slew_next;
            theta_dot <= slew_next;
        end
    end

    cnt2deg #(.CPR(CPR)) u_c2d (
        .clk(clk), .rst_n(rst_n), .cnt(pos), .deg(theta_c01)
    );

    function signed [15:0] sat16(input signed [31:0] v);
        sat16 = (v >  32'sd32767) ? 16'sd32767 :
                (v < -32'sd32768) ? -16'sd32768 : v[15:0];
    endfunction
endmodule
