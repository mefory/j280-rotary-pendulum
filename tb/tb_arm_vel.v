// ============================================================================
// tb_arm_vel.v — arm_vel M/T 混合测速验收（**拍计时 + 4 拍差分窗**口径）
//
// 2026-09-23 随位置环重设计同步改写：
//   ① T 法时间基准改为控制拍（1ms），因此在加速时钟的 tb 里与实机同口径；
//      判停由 STALL_TICKS(=300 拍) 决定，臂停下后 300ms 内 θ̇ 归零。
//   ② 边沿驱动改为"tick 之后改 pos"，与 tb_control_path 同口径，避免赋值与 DUT
//      移位链同沿竞争把 4ms 差分读成 3 计数（那是 tb 的相位假象）。
// tb 在**稳态运动**中采样 θ̇（周期测量法必须在运动中采样）。
//
// 期望 v = VNUM/n，VNUM = 3_600_000/CPR = 3461（1 计数/n 拍 ⇒ 3461/n，单位 0.1°/s）；
// M 法期望 v = dpos×1731>>>1 = dpos×865（dpos 是 4ms 差分计数）：
//   n=250 → 13      n=115 → 30      n=60 → 57      n=12 → 288
//   n=4   → 865     （dpos=1 仍走 T，与 M 的 1 计数/4ms 同值：交叉点）
//   n=2   → 1731    （dpos=2 起走 M）
//   n=1   → 3462    （dpos=4，纯 M）
// 容差随 n 变化：拍计时的时间分辨率是 1 拍，折算到速度上是 ±VNUM/n² 的量化误差
// （n=115 → ±0.3，n=60 → ±1，n=12 → ±24）。低速容差小、高速容差大是该方案的固有
// 特性，不是回归；高速段本身由 M 法接管。
// ============================================================================
`timescale 1ns / 1ps
`include "j280_defs.vh"
module tb_arm_vel;
    localparam CLK_HALF = 100;          // 5MHz 测试时钟（200ns），仅用于提速
    localparam TICK_DIV = 4_999;        // 5000 个测试时钟 = 1 个控制拍

    reg clk = 0, rst_n = 0, tick = 0;
    reg signed [31:0] pos = 0;
    wire signed [31:0] theta_c01;
    wire signed [15:0] theta_dot;

    // 显式 USE_T(1)：T 法路径要保住验证（发布默认是 USE_T=0，即纯 M 法）。
    // 之所以发布不启用 T：RTL 四方向闭环与两个模型镜像都显示"低速 θ̇ 变准"会重新激活
    // K4 的阻尼、把最终定位残差顶大（`host.acceptance --all` 的 +3000 会从 0.81° 顶到
    // 2.31°，超过 2° 判据）。发版门禁 host.test_rtl_mission 在 USE_T=0/1 下都通过
    // （progress +1489 / +1420），说明定位修好靠的是增益重设计，不是这套 T 法。
    // 是否上板启用 T，需实机证据（实机才有真正的低速量化噪声，仿真 tb 里 dpos 恒 0）。
    arm_vel #(.USE_T(1)) dut (
        .clk(clk), .rst_n(rst_n), .tick(tick),
        .pos(pos), .theta_c01(theta_c01), .theta_dot(theta_dot)
    );

    // 第二实例：SLEW_LIM=1000（T20K 零点刹车配置）——斜率限制器不伤稳态跟踪，
    // 高速段（物理加速 900/拍 以内）完全放行，只有 T 法单拍伪迹（1730+）被削。
    wire signed [15:0] theta_dot_sl;
    arm_vel #(.USE_T(1), .SLEW_LIM(1000)) dut_sl (
        .clk(clk), .rst_n(rst_n), .tick(tick),
        .pos(pos), .theta_c01(), .theta_dot(theta_dot_sl)
    );
    always #(CLK_HALF) clk = ~clk;

    integer tick_div = 0;
    always @(posedge clk) begin
        tick_div <= (tick_div == TICK_DIV) ? 0 : tick_div + 1;
        tick     <= (tick_div == TICK_DIV - 1);
    end

    integer errors = 0;

    // 稳态边沿发生器（与 tb_control_path 同口径）。
    // 关键：用**阻塞赋值**、在 tick 到来前 10 个测试时钟改 pos，错开 DUT 的 tick 采样沿。
    // 若在 tick 同沿用非阻塞赋值改 pos，DUT 的 h 移位链会读成 3 计数（1 计数/拍 → 2590
    // 而非 3462）——那是 tb 的相位假象，不是实现问题。
    integer period_ticks = 115;
    integer step_delta   = 1;
    reg     running      = 0;
    integer tick_cnt     = 0;
    initial begin
        forever begin
            @(negedge clk);
            if (tick_div == TICK_DIV - 10) begin
                tick_cnt = tick_cnt + 1;
                if (running && tick_cnt >= period_ticks) begin
                    tick_cnt = 0;
                    pos = pos + step_delta;
                end
            end
        end
    end

    task settle(input integer ticks);
        integer t;
        begin
            for (t = 0; t < ticks; t = t + 1) @(posedge tick);
        end
    endtask

    task check(input integer expected, input integer tol, input [255:0] name);
        begin
            if (theta_dot < expected - tol || theta_dot > expected + tol) begin
                $display("FAIL: %0s expect %0d±%0d got %0d", name, expected, tol, theta_dot);
                errors = errors + 1;
            end else begin
                $display("  OK  %0s: theta_dot=%0d (expect %0d±%0d)", name, theta_dot, expected, tol);
            end
        end
    endtask

    // SLEW_LIM 实例与主实例的跟随性检查（容差为绝对差）
    task check_sl(input integer master, input integer tol, input [255:0] name);
        begin
            if (theta_dot_sl < master - tol || theta_dot_sl > master + tol) begin
                $display("FAIL: %0s master=%0d±%0d got %0d", name, master, tol, theta_dot_sl);
                errors = errors + 1;
            end else begin
                $display("  OK  %0s: slew=%0d (master %0d±%0d)", name, theta_dot_sl, master, tol);
            end
        end
    endtask

    // 设定周期与方向，等两个边沿让 p_cap/差分窗/EMA 稳定后判读
    task measure(input integer period, input integer delta, input integer expected,
                 input integer tol, input [255:0] name);
        begin
            period_ticks = period; step_delta = delta; running = 1;
            settle(2 * period + 20);
            check(expected, tol, name);
        end
    endtask

    initial begin
        rst_n = 0; #2000; rst_n = 1;
        settle(20);

        measure(250,  1,   13,   2, "T 1/250tick +13");   // 1.38°/s 定位段量级
        measure(115,  1,   30,   3, "T 1/115tick +30");   // 3.0°/s
        measure(60,   1,   57,   2, "T 1/60tick  +57");   // 5.8°/s
        measure(12,   1,  288,  25, "T 1/12tick +288");   // 28.8°/s
        measure(4,    1,  865,  20, "T/M cross 1/4tick +865");
        measure(2,    1, 1731,  30, "M cross 1/2tick +1731");
        measure(1,    1, 3462,  30, "M fast 1/1tick +3462");

        // 停车后估计必须**平滑衰减**，不能挂着上次的边沿速度：K4 会把陈旧高速值放大
        // 成大指令（起摆结束臂很快，捕捉瞬间尤其危险，这也是当初 USE_T 被排除的原因）。
        // 除数取 max(p_cap, 距上次边沿的实时时间) ⇒ 80 拍后应从 3462 降到 ~43。
        // 紧跟高速段做这个检查才有意义：若挂陈旧值会读到 ~3454。
        running = 0; step_delta = 1;
        settle(80); check(0, 80, "decay after stop（无陈旧值）");

        measure(115, -1,  -30,   3, "T 1/115tick -30");   // 反向

        // 判停：无边沿 STALL_TICKS(300) 拍后硬归零，再等 EMA 尾巴（~30 拍）落净
        running = 0;
        settle(360); check(0, 1, "stall -> 0");

        // 判停后能重新测出来（实机上就是"停下再爬行"这个真实故障模式）
        measure(115, 1, 30, 3, "re-acquire +30");

        // ---- SLEW_LIM=1000 实例：稳态跟踪不受限 + 高速段放行 + 复位回零 ----
        // 稳态点全部跟着主实例走（物理加速 < 900/拍）
        check_sl(theta_dot, 2, "slew: T 30 稳态不受限");
        if (theta_dot_sl < theta_dot - 40 || theta_dot_sl > theta_dot + 40) begin
            $display("FAIL: slew 288 跟随偏差大 %0d vs %0d", theta_dot_sl, theta_dot);
            errors = errors + 1;
        end else $display("  OK  slew 288 跟随: %0d vs %0d", theta_dot_sl, theta_dot);
        measure(4, 1, 865, 20, "slew: cross 865");
        if (theta_dot_sl < theta_dot - 60 || theta_dot_sl > theta_dot + 60) begin
            $display("FAIL: slew 865 跟随偏差大 %0d vs %0d", theta_dot_sl, theta_dot);
            errors = errors + 1;
        end else $display("  OK  slew 865 跟随: %0d vs %0d", theta_dot_sl, theta_dot);
        // 高速段 dpos=4/拍 → θ̇ 3462，爬升 1000/拍，2-4 拍内到达 —— 延迟有界
        measure(1, 1, 3462, 30, "slew: M fast 3462");
        if (theta_dot_sl < theta_dot - 2000) begin
            $display("FAIL: slew 高速爬升过慢 %0d vs %0d", theta_dot_sl, theta_dot);
            errors = errors + 1;
        end else $display("  OK  slew 高速爬升: %0d (target %0d)", theta_dot_sl, theta_dot);
        // 复位后 slew_q 回零。注意：复位清 h 链而 pos 保持 → 释放后头 3 拍有
        // dpos 假脉冲，EMA 尾巴按"停车衰减"曲线（T 法除法器每拍重触发）要走
        // ~STALL_TICKS(300) 拍判停才硬归零——这是 arm_vel 既有语义（两实例
        // 行为一致），不是斜率限制器的问题。判据=判停后回零。
        running = 0;
        settle(360);
        rst_n = 0;
        repeat (10) @(negedge clk);
        rst_n = 1;
        settle(330);
        if (theta_dot_sl !== 0) begin
            $display("FAIL: slew 复位判停未回零 %0d", theta_dot_sl);
            errors = errors + 1;
        end else $display("  OK  slew 复位判停回零");

        if (errors == 0) $display("tb_arm_vel PASS");
        else             $display("tb_arm_vel FAIL errors=%0d", errors);
        $finish;
    end

    initial begin
        #5_000_000_000;   // 超时保护
        $display("TIMEOUT"); $finish;
    end
endmodule
