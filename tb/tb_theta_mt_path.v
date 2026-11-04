// tb_theta_mt_path.v — 电机侧 MT6701 θ 链单元测试（experiment/mt6701-theta）
//
// 镜像模型：tb 用实数累计复刻 DUT 的整数语义（模 16384 回绕差分 + ±2048 限幅
// + ×7042 累计 + >>>16 取角），θ 允差 ±1.5 c01（移位取整），θ̇ 稳态允差覆盖
// EMA 收敛残差。DIR_FLIP 对称性用第二实例逐值断言（恒等于 −DUT0）。
`timescale 1ns / 1ps
module tb_theta_mt_path;
    reg clk = 0, rst_n = 0, tick = 0;
    reg [13:0] angle = 14'd0;
    wire signed [31:0] th0, th1;
    wire signed [15:0] td0, td1;
    integer errors = 0;

    theta_mt_path #(.SLEW_LIM(0)) dut0 (
        .clk(clk), .rst_n(rst_n), .tick(tick), .angle14(angle),
        .theta_c01(th0), .theta_dot(td0)
    );
    theta_mt_path #(.SLEW_LIM(0), .DIR_FLIP(1'b1)) dut1 (
        .clk(clk), .rst_n(rst_n), .tick(tick), .angle14(angle),
        .theta_c01(th1), .theta_dot(td1)
    );

    always #10 clk = ~clk;

    // 镜像：exp_acc 单位 c01×2^16；step 复刻回绕差分 + ±2048 限幅 + ×7042
    real exp_acc, dtmp;
    integer prev_ang;
    task model_step(input integer newang);
        begin
            dtmp = newang - prev_ang;
            if (dtmp > 8191)  dtmp = dtmp - 16384;
            if (dtmp < -8191) dtmp = dtmp + 16384;
            if (dtmp > 2048)  dtmp = 2048;
            if (dtmp < -2048) dtmp = -2048;
            exp_acc  = exp_acc + dtmp * 7042.0;
            prev_ang = newang;
        end
    endtask
    // 换角 → 事件域吸收 → 单拍 tick 脉冲
    task step_tick(input [13:0] newang);
        begin
            angle = newang;
            model_step(newang);
            repeat (3) @(posedge clk);
            @(negedge clk) tick = 1;
            @(negedge clk) tick = 0;
            @(posedge clk);
        end
    endtask
    task chk_theta(input [127:0] tag);
        real expect_c01;
        begin
            expect_c01 = exp_acc / 65536.0;
            if ($itor(th0) > expect_c01 + 1.5 || $itor(th0) < expect_c01 - 1.5) begin
                $display("FAIL %0s: th0=%0d expect~%f", tag, th0, expect_c01);
                errors = errors + 1;
            end
            if (th1 + th0 > 1 || th1 + th0 < -1) begin
                // 算术右移取整：acc 与 −acc 的 >>>16 相差 ≤1（floor 不对称）
                $display("FAIL %0s: DIR_FLIP 对称性 th1=%0d -th0=%0d", tag, th1, -th0);
                errors = errors + 1;
            end
        end
    endtask

    integer k, a, a2;
    initial begin
        exp_acc = 0.0; prev_ang = 0;
        // ---- 复位期角度停在 5000：首采样只建基准，θ 必须 0 ----
        angle = 14'd5000;
        repeat (5) @(posedge clk);
        rst_n = 1;
        repeat (3) @(posedge clk);
        prev_ang = 5000;            // 镜像同语义：释放复位后的首个值为基准
        repeat (5) step_tick(14'd5000);
        if (th0 !== 32'sd0 || td0 !== 16'sd0) begin
            $display("FAIL: 上电基准 th0=%0d td0=%0d（期望 0/0）", th0, td0);
            errors = errors + 1;
        end else $display("PASS: 上电任意角只建基准（θ=0）");

        // ---- 恒速 +16 LSB/ms（0.977 电机转/s → 臂 17.2°/s）×320 拍 ----
        a = 5000;
        for (k = 0; k < 320; k = k + 1) begin
            a = a + 16; if (a >= 16384) a = a - 16384;
            step_tick(a[13:0]);
        end
        chk_theta("ramp320");
        // 稳态 θ̇：16×7042×100/65536 = 171.9
        if (td0 > 178 || td0 < 166) begin
            $display("FAIL: 恒速 θ̇=%0d（期望 ~172）", td0);
            errors = errors + 1;
        end else $display("PASS: 恒速 θ̇=%0d（期望 ~172）", td0);
        if (td1 + td0 > 2 || td1 + td0 < -2) begin
            $display("FAIL: DIR_FLIP θ̇ 对称性 td0=%0d td1=%0d", td0, td1);
            errors = errors + 1;
        end

        // ---- 继续同一斜坡 800 拍：跨 16384 回绕至少一次，θ 连续 ----
        for (k = 0; k < 800; k = k + 1) begin
            a = a + 16; if (a >= 16384) a = a - 16384;
            step_tick(a[13:0]);
            if (k % 100 == 99) chk_theta("wrap");
        end
        $display("PASS: 回绕连续 θ=%0d c01（%0f°）", th0, $itor(th0)/100.0);

        // ---- 冻结：25 拍后 θ̇ 精确归零、θ 保持 ----
        for (k = 0; k < 25; k = k + 1) step_tick(a[13:0]);
        if (td0 !== 16'sd0) begin
            $display("FAIL: 冻结后 θ̇=%0d（期望 0）", td0);
            errors = errors + 1;
        end else $display("PASS: 冻结后 θ̇ 归零");
        chk_theta("freeze");

        // ---- 高速 +725 LSB/ms（44 电机转/s 峰值口径）×80 拍 ----
        for (k = 0; k < 80; k = k + 1) begin
            a = a + 725; if (a >= 16384) a = a - 16384;
            step_tick(a[13:0]);
        end
        chk_theta("fast725");
        // 稳态 θ̇：725×7042×100/65536 = 7790
        if (td0 > 7800 || td0 < 7780) begin
            $display("FAIL: 高速 θ̇=%0d（期望 ~7790）", td0);
            errors = errors + 1;
        end else $display("PASS: 高速 θ̇=%0d（期望 ~7790）", td0);

        // ---- 毛刺对称抵消：+5000/−5000（两侧都各自限幅 ±2048）净零 ----
        a2 = a + 5000; if (a2 >= 16384) a2 = a2 - 16384;
        step_tick(a2[13:0]);
        chk_theta("glitch+");     // 镜像同限幅记账：θ=+219.9 c01 台阶
        step_tick(a[13:0]);       // 跳回 → 净变化归零（绝对角不累积毛刺）
        chk_theta("glitch-");
        $display("PASS: 毛刺 ±5000 对称抵消（θ 净零，th0=%0d）", th0);

        // ---- 长程继续正向 300 拍：累计一致性 ----
        for (k = 0; k < 300; k = k + 1) begin
            a = a + 16; if (a >= 16384) a = a - 16384;
            step_tick(a[13:0]);
        end
        chk_theta("long300");

        if (errors == 0) $display("tb_theta_mt_path PASS (THETA_MT_PATH ALL PASS)");
        else $display("tb_theta_mt_path FAIL (%0d ERRORS)", errors);
        $finish;
    end
endmodule
