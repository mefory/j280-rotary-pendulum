// tb_swingup_res.v — 共振激振模式验证（PUMP_MODE=2 连续三角波版 v8，2026-10-12）
// 结构：osc_ref 斜坡积分（slope 8/7/6 c01/拍三段扫频，±3000 c01），KP 伺服
//       连续目标静音跟踪；臂速闸 |θ̇|>120°/s 撤力。
`timescale 1ns/1ps
module tb_swingup_res;
    reg clk=0, rst_n=0, tick=0, start=0;
    reg signed [31:0] theta=0;
    reg signed [15:0] theta_dot=0, alpha=0, alpha_dot=0, delta=0, amplitude=16'sd9000;
    wire signed [15:0] duty;
    swingup_core #(
        .RETHROW_MS(3000), .RETHROW_TD_MAX(16'sd30000),
        .PUMP_MODE(2), .RES_DRIVE_SHAPING_EN(1)
    ) dut (clk,rst_n,tick,start,theta,theta_dot,alpha,alpha_dot,delta,
           16'sd0,16'sd0,16'sd0,   // catch_ang/coast_amp/coast_margin（本 tb 未用）
           amplitude,duty,brake_req_unused,1'b1,1'b1);
    wire brake_req_unused;

    always #5 clk = ~clk;
    task pulse(input integer n);
        begin repeat(n) begin
            repeat(4)@(negedge clk); tick=1; @(negedge clk); tick=0;
        end end
    endtask
    integer errors=0;
    task check(input [127:0] name, input integer got, input integer lo, input integer hi);
        begin
            if ((^got === 1'bx) || got < lo || got > hi) begin
                errors = errors + 1;
                $display("[FAIL] %0s: got=%0d 期望[%0d,%0d]", name, got, lo, hi);
            end else
                $display("[OK] %0s: %0d", name, got);
        end
    endtask

    initial begin
        repeat(4)@(negedge clk); rst_n=1;
        tick=1; start=1; @(negedge clk); tick=0; start=0;
        pulse(6);

        // 1) 段0 斜坡起步：osc_ref 从 0 往 −3000 积（osc_dir=0），θ=0 静止
        //    6 拍后 osc_ref=−48 → err=−48×9=−432（小输出=静音证据）
        alpha=16'sd12000; alpha_dot=16'sd2000; theta=32'sd0; theta_dot=16'sd0;
        pulse(6);
        check("斜坡起步小输出", duty, -1200, -300);

        // 2) 臂速闸（|θ̇|=1500>1200）→ 撤力 0
        theta_dot=16'sd1500;
        pulse(4);
        check("臂速闸撤力 0", duty, 0, 0);

        // 3) 臂速恢复 → 恢复输出
        theta_dot=16'sd300;
        pulse(4);
        check("臂速恢复输出", duty, -2000, 2000);

        // 4) 长推 400ms：osc_ref 走完下行 + 100ms 上行（折返发生、无爆冲）
        pulse(400);
        check("400ms 斜坡折返无爆冲", duty, -9000, 9000);

        if (errors==0) $display("tb_swingup_res PASS");
        else $fatal(1,"tb_swingup_res FAIL errors=%0d", errors);
        $finish;
    end
endmodule
