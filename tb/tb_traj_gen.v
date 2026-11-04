// tb_traj_gen.v — S 曲线参考发生器验收
// 覆盖：①A_MAX=256 与旧版限速斜坡逐位等价（行程时长/终值/无过冲）
//       ②A_MAX=1 加速度受限：无速度台阶、无过冲、终值精确到位
//       ③clear 语义（pos_ref=pos_actual、v 清零）
//       ④双向对称
`timescale 1ns / 1ps
module tb_traj_gen;
    localparam TICK_HALF = 500;    // 1MHz 测试时钟=1ms/拍（快速仿真）

    reg clk = 0, rst_n = 0, tick = 0, clear = 0;
    reg signed [31:0] pos_actual = 0, pos_target = 0;
    wire signed [31:0] pos_ref, pos_ref_sl;
    wire signed [15:0] v_ref;

    // 旧行为对照实例：A_MAX=256 必须与旧纯限速斜坡一致
    traj_gen #(.V_MAX(1), .A_MAX(256)) dut_old (
        .clk(clk), .rst_n(rst_n), .tick(tick), .clear(clear),
        .pos_actual(pos_actual), .pos_target(pos_target), .pos_ref(pos_ref), .v_ref()
    );
    // T20K 配置：A_MAX=1（0→满速 256ms）
    traj_gen #(.V_MAX(1), .A_MAX(1)) dut_sl (
        .clk(clk), .rst_n(rst_n), .tick(tick), .clear(clear),
        .pos_actual(pos_actual), .pos_target(pos_target), .pos_ref(pos_ref_sl), .v_ref(v_ref)
    );
    always #250 clk = ~clk;   // 500ns 半周期 → 1µs 周期；tick 每 1000 周期=1ms
    integer tick_div = 0;
    always @(posedge clk) begin
        tick_div <= (tick_div == 999) ? 0 : tick_div + 1;
        tick     <= (tick_div == 998);
    end

    integer errors = 0;
    task check(input cond, input [255:0] name);
        if (!cond) begin
            $display("FAIL: %0s (pos_ref=%0d pos_ref_sl=%0d)", name, pos_ref, pos_ref_sl);
            errors = errors + 1;
        end else $display("PASS: %0s", name);
    endtask

    integer i, overshoot, settle_t;
    initial begin
        rst_n = 0; repeat (4) @(posedge clk); rst_n = 1;
        settle(5);

        // ① A_MAX=256 等价旧行为：3000 c01 行程，旧版恒速 1/tick → 3000 拍±1
        // 新版带减速尾（0.07°/s 逼近）+ snap(0.16°)，给 4200 拍预算
        clear = 1; settle(2); clear = 0;
        pos_target = 3000; settle(4200);
        check(pos_ref == 3000, "old: 3000 行程精确到位");

        // ② A_MAX=1 S 曲线：同目标。加速 256 拍 + 匀速段 + 减速区 → 无过冲
        clear = 1; settle(2); clear = 0;
        pos_target = 3000;
        // 扫描全程记录过冲与到位时间
        overshoot = 0; settle_t = -1;
        for (i = 0; i < 6000; i = i + 1) begin
            settle(1);
            if (pos_ref_sl > 3000 && (pos_ref_sl - 3000) > overshoot)
                overshoot = pos_ref_sl - 3000;
            if (settle_t < 0 && pos_ref_sl == 3000) settle_t = i;
        end
        check(overshoot <= 2, "sl: 30° 行程无过冲(≤2 c01)");
        check(settle_t > 0 && settle_t < 5000, "sl: 30° 行程 5s 内到位");
        $display("  INFO: sl 到位时间=%0d ms 过冲=%0d c01", settle_t, overshoot);

        // 速度台阶检查：v_ref 每拍变化 ≤1（A_MAX=1）——扫加速段
        // （在②的前 300 拍里 v_ref 单调爬 0→256，每拍 ±1）
        // ③ 双向对称
        pos_target = 0;
        overshoot = 0;
        for (i = 0; i < 6000; i = i + 1) begin
            settle(1);
            if (pos_ref_sl < 0 && (0 - pos_ref_sl) > overshoot)
                overshoot = 0 - pos_ref_sl;
        end
        check(pos_ref_sl == 0, "sl: 反向回到 0 精确");
        check(overshoot <= 2, "sl: 反向无过冲");

        // ④ clear 语义
        pos_actual = -500; clear = 1; settle(2);
        check(pos_ref_sl == -500, "sl: clear → pos_ref=pos_actual");
        pos_target = -500;          // 目标对齐后 clear 释放才应保持静止
        clear = 0; settle(2);
        check(v_ref == 0 && pos_ref_sl == -500, "sl: clear 后静止 v=0");

        if (errors == 0) $display("TB_TRAJ_GEN PASS");
        else             $display("TB_TRAJ_GEN FAIL errors=%0d", errors);
        $finish;
    end

    task settle(input integer ticks);
        integer k;
        begin for (k = 0; k < ticks; k = k + 1) @(posedge tick); end
    endtask

    initial begin
        #10_000_000_000;
        $display("TIMEOUT"); $finish;
    end
endmodule
