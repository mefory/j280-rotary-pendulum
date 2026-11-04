// 5Hz 已知臂角输入，比较两种流水更新粒度的测速相位；不使用对象模型。
`timescale 1ns/1ps
module tb_arm_vel_latency;
    reg clk=0, rst_n=0, tick=0;
    reg signed [31:0] pos=0;
    wire signed [15:0] slow_speed, fast_speed;
    always #10 clk=~clk;
    arm_vel #(.SLEW_LIM(1000)) old_pipe (
        .clk(clk), .rst_n(rst_n), .tick(tick), .pos(pos),
        .theta_c01(), .theta_dot(slow_speed), .arm_stalled()
    );
    arm_vel #(.SLEW_LIM(1000), .PIPELINE_ON_TICK(0)) fast_pipe (
        .clk(clk), .rst_n(rst_n), .tick(tick), .pos(pos),
        .theta_c01(), .theta_dot(fast_speed), .arm_stalled()
    );
    integer n, fd;
    real counts, truth_speed, co, si;
    real slow_co=0.0, slow_si=0.0, fast_co=0.0, fast_si=0.0;
    real delay_reduction, amplitude_ratio;
    reg [1023:0] trace="sim/out/arm_vel_latency.csv";
    initial begin
        if ($value$plusargs("TRACE=%s",trace)) begin end
        fd=$fopen(trace,"w");
        if (fd==0) $fatal(1,"Cannot open trace");
        $fdisplay(fd,"tick,encoder,true_speed,slow_speed,fast_speed");
        repeat(5) @(negedge clk); rst_n=1;
        for(n=0;n<2400;n=n+1) begin
            counts=10.0*$sin(2.0*3.141592653589793*5.0*n*0.001)*1040.0/360.0;
            truth_speed=10.0*2.0*3.141592653589793*5.0*$cos(2.0*3.141592653589793*5.0*n*0.001)*10.0;
            @(negedge clk);
            pos=(counts>=0) ? $rtoi(counts+0.5) : $rtoi(counts-0.5);
            repeat(80) @(negedge clk);
            tick=1; @(posedge clk); #1;
            if ((^slow_speed===1'bx) || (^fast_speed===1'bx)) $fatal(1,"Unknown speed");
            $fdisplay(fd,"%0d,%0d,%f,%0d,%0d",n,pos,truth_speed,slow_speed,fast_speed);
            if(n>=400) begin
                co=$cos(2.0*3.141592653589793*5.0*n*0.001);
                si=$sin(2.0*3.141592653589793*5.0*n*0.001);
                slow_co=slow_co+$itor(slow_speed)*co;
                slow_si=slow_si+$itor(slow_speed)*si;
                fast_co=fast_co+$itor(fast_speed)*co;
                fast_si=fast_si+$itor(fast_speed)*si;
            end
            @(negedge clk); tick=0;
            repeat(20) @(negedge clk);
        end
        $fclose(fd);
        delay_reduction=($atan2(slow_si,slow_co)-$atan2(fast_si,fast_co))/(2.0*3.141592653589793*5.0)*1000.0;
        amplitude_ratio=$sqrt(fast_co*fast_co+fast_si*fast_si)/$sqrt(slow_co*slow_co+slow_si*slow_si);
        if(delay_reduction<1.5 || delay_reduction>2.5 || amplitude_ratio<0.95 || amplitude_ratio>1.05)
            $fatal(1,"phase/gain mismatch: reduction=%fms amplitude_ratio=%f",delay_reduction,amplitude_ratio);
        $display("tb_arm_vel_latency PASS: delay reduction=%fms amplitude_ratio=%f",delay_reduction,amplitude_ratio);
        $finish;
    end
    initial begin #10_000_000; $fatal(1,"TIMEOUT"); end
endmodule
