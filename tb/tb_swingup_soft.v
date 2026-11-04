`timescale 1ns/1ps
module tb_swingup_soft;
    reg clk=0, rst_n=0, tick=0, start=0, active=0;
    always #10 clk=~clk;
    reg signed [31:0] theta=10000;
    reg signed [15:0] td=0, ad=0;
    reg valid=1;
    wire signed [15:0] duty;
    swingup_core #(.PUMP_MODE(2), .ALPHA_QUALITY_EN(1), .RES_DRIVE_SHAPING_EN(1)) dut(
        .clk(clk), .rst_n(rst_n), .tick(tick), .start(start), .theta(theta),
        .theta_dot(td), .alpha(16'sd18000), .alpha_dot(ad), .delta(16'sd11800),
        .amplitude(16'sd8800), .duty(duty), .alpha_valid(valid), .swing_active(active));
    integer previous=0, current, i, checked=0;
    task require(input reg condition, input [255:0] name);
        begin if(condition !== 1'b1) $fatal(1,"tb_swingup_soft FAIL: %0s",name); end
    endtask
    task pulse;
        begin
            repeat(8) @(negedge clk); tick=1;
            @(negedge clk); tick=0; repeat(12) @(negedge clk);
            require((^duty !== 1'bx), "known output");
            current=$signed(duty);
            require(current>=-8800 && current<=8800, "amplitude bounded");
            if(active && !start) begin
                if(current!=0 && previous!=0)
                    require((current<0)==(previous<0), "reversal must pass zero");
                if((current<0 ? -current : current) > (previous<0 ? -previous : previous))
                    require((current<0 ? -current : current)-(previous<0 ? -previous : previous)<=150,
                            "push rise <=1.5pct/ms");
            end
            previous=current; checked=checked+1;
        end
    endtask
    initial begin
        #100; rst_n=1;
        repeat(400) begin pulse; require(duty===16'sd0,"inactive output zero"); end
        active=1; start=1; pulse;
        require(duty===16'sd0 && dut.res_err_f===32'sd0,"start clears stale command");
        start=0; repeat(100) pulse;
        require(duty < -1000,"normal drive retained");
        // 高速撤力时 theta 非零：整个指令必须归零，不能留下回中残值。
        td=1500; repeat(3) pulse;
        require(duty===16'sd0,"overspeed total output zero");
        td=0; repeat(6) pulse;
        require(duty>=-900 && duty<0,"re-enable rises gently");
        for(i=0;i<80;i=i+1) begin td=(i%2)?0:1500; pulse; end
        td=0; repeat(20) pulse;
        // 实际位置跨过目标，驱动换向必须先给一拍零。
        theta=-20000; repeat(20) pulse;
        require(duty>1000,"opposite direction drive retained");
        ad=30000; repeat(5) pulse;
        require(dut.res_hold===1'b1 && duty===16'sd0,"hold total output zero");
        active=0; repeat(3) @(negedge clk);
        require(duty===16'sd0 && dut.res_ready===1'b0,"stop clears immediately");
        repeat(300) pulse;
        theta=-1039; ad=0; active=1; start=1; pulse; start=0;
        require(duty===16'sd0,"restart has no stale pulse");
        repeat(4) pulse;
        require(duty>=-600 && duty<=600,"restart ramp bounded");
        $display("tb_swingup_soft PASS: %0d samples, quiet start/restart, ramp, zero, reversal, hold",checked);
        $finish;
    end
    initial begin #2000000;$fatal(1,"tb_swingup_soft TIMEOUT");end
endmodule
