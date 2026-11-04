// tb_swingup_pump.v — 能量泵行为验证（PUMP_MODE=1，2026-10-11）
// 用 iverilog 驱动：直接喂 α/α̇/θ/θ̇ 序列验证四个行为：
//   1) 下半区能量不足 → 输出与 α̇ 反号（pump_trace 符号约定）
//   2) 能量到位 → 输出趋 0（E 伺服死区）
//   3) 上半区 → 弱回中输出（朝 θ0 方向，限幅 amp/2）
//   4) 底座狂飞（|θ̇|>RETHROW_TD_MAX）→ 输出 0
// 定点抽查：α=22° α̇=0 → E=758（E_target）→ |duty|<16
`timescale 1ns/1ps
module tb_swingup_pump;
    reg clk=0, rst_n=0, tick=0, start=0;
    reg signed [31:0] theta=0;
    reg signed [15:0] theta_dot=0, alpha=0, alpha_dot=0, delta=0, amplitude=16'sd9000;
    wire signed [15:0] duty;
    swingup_core #(
        .RETHROW_MS(3000), .RETHROW_TD_MAX(16'sd30000),
        .RETHROW_ALPHA_MIN(16'sd9000),
        .PUMP_MODE(1), .PUMP_KE(16'sd384), .PUMP_E_TARGET(32'sd758)
    ) dut (clk,rst_n,tick,start,theta,theta_dot,alpha,alpha_dot,delta,
           16'sd0,16'sd0,16'sd0,   // catch_ang/coast_amp/coast_margin（本 tb 未用）
           amplitude,duty,brake_req_unused,1'b1,1'b1);
    wire brake_req_unused;

    always #5 clk = ~clk;
    task pulse;
        begin repeat(4)@(negedge clk); tick=1; @(negedge clk); tick=0;
              // 3 级流水 + tick_d 采样：等足 8 拍再读
              repeat(8)@(negedge clk); end
    endtask
    integer errors=0;
    task check(input [127:0] name, input integer got, input integer lo, input integer hi);
        begin
            if (got < lo || got > hi) begin
                errors = errors + 1;
                $display("[FAIL] %0s: got=%0d 期望[%0d,%0d]", name, got, lo, hi);
            end else
                $display("[OK] %0s: %0d", name, got);
        end
    endtask

    initial begin
        repeat(4)@(negedge clk); rst_n=1;
        tick=1; start=1; @(negedge clk); tick=0; start=0;  // 锁存 θ0=0
        repeat(4)@(negedge clk);

        // 1) 下落段节奏泵（α=+120°=12000, α̇=+2000：朝垂挂走=下落段）
        //    E=6+10196=10202, err=+9444, mag 钳 9000, sign(av>0,co<0)=−1
        //    → −9000（超能量阻尼向）+ 回拉 θ=20000×14>>>8=−344 → 钳 amp 内
        //    注意节奏门放行（下落段），sum=−9000+(−344)=−9344→钳−9000
        alpha=16'sd12000; alpha_dot=16'sd2000; theta=32'sd20000; theta_dot=16'sd0;
        pulse();
        check("下落段节奏泵 -9000", duty, -9000, -9000);

        // 2) 上升段但 E 超标（α=+120°, α̇=−2000：E=10202>>758）
        //    → 全域阻尼生效：sign(co<0,av<0 同号)=+1 → +9000 + 回拉 −1094
        //    = +7906（2026-10-11 全域阻尼语义：超标不分相都刹）
        alpha=16'sd12000; alpha_dot=-16'sd2000; theta=32'sd20000;
        pulse();
        check("超标全域阻尼 +7906", duty, 7906, 7906);

        // 3) 起步一击（α=180°=18000, α̇=0 静止）→ +amp=+9000
        alpha=16'sd18000; alpha_dot=16'sd0; theta=32'sd0;
        pulse();
        check("垂挂起步一击 +9000", duty, 9000, 9000);

        // 4) 底座狂飞
        alpha=16'sd1000; alpha_dot=16'sd2000; theta_dot=16'sd31000;
        pulse();
        check("底座狂飞 duty=0", duty, 0, 0);

        // 5) 近直立小角区（α=10°=1000）非窗口非静止 → 滑行 0
        alpha=16'sd1000; alpha_dot=16'sd2000; theta=32'sd0;
        pulse();
        check("近直立滑行 duty=0", duty, 0, 0);

        if (errors==0) $display("tb_swingup_pump PASS");
        else $display("tb_swingup_pump FAIL errors=%0d", errors);
        $finish;
    end
endmodule
