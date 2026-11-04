`timescale 1ns/1ps
module tb_units;
    reg clk=0, rst_n=0, tick=0, calib_start=0;
    always #10 clk=~clk;
    reg signed [31:0] cnt=0;
    reg [15:0] adc=0;
    wire signed [31:0] deg;
    wire signed [15:0] alpha, adot, offset;
    wire done;
    cnt2deg cv(clk,rst_n,cnt,deg);
    angle_proc #(.CAL_TICKS(8)) ap(clk,rst_n,tick,adc,calib_start,1'b0,16'sd0,alpha,ap_pre,adot,offset,done);
    wire signed [15:0] ap_pre;   // alpha_pre（v3 遥测新增输出，本 tb 未用）
    integer errors=0, i;
    reg signed [63:0] expected;
    task count_check(input integer value);
        begin
            @(negedge clk);cnt=value;
            repeat(5) @(negedge clk);
            expected=$signed(value)*64'sd35446;
            expected=expected >>> 10;
            if(deg !== expected[31:0]) begin
                $display("FAIL cnt=%0d deg=%0d expected=%0d",cnt,deg,expected);errors=errors+1;
            end
        end
    endtask
    task pulse;
        begin @(negedge clk);tick=1;@(negedge clk);tick=0;repeat(4)@(negedge clk);end
    endtask
    task sample(input integer value);
        begin adc=value;repeat(8)@(negedge clk);pulse;end
    endtask
    initial begin
        #100;rst_n=1;
        count_check(0);count_check(1);count_check(-1);
        count_check(1040);count_check(-1040);
        count_check(65535);count_check(65536);count_check(-65535);count_check(-65536);
        count_check(1000000);count_check(-1000000);
        sample(65472);
        if(alpha < 8140 || alpha > 8180) begin
            $display("FAIL full-scale angle %0d",alpha);errors=errors+1;
        end
        adc=65472;repeat(8)@(negedge clk);
        calib_start=1;@(negedge clk);calib_start=0;
        for(i=0;i<8;i=i+1)pulse;
        if(!done || offset < -1502 || offset > -1498) begin
            $display("FAIL calibration done=%d offset=%d",done,offset);errors=errors+1;
        end
        repeat(8)@(negedge clk);pulse;
        if(alpha < 17998 && alpha > -17998) begin
            $display("FAIL hanging after calibration %d",alpha);errors=errors+1;
        end
        if(errors)$fatal(1,"tb_units FAIL errors=%d",errors);
        $display("tb_units PASS: signed angle conversion, full scale, calibration");
        $finish;
    end
    initial begin #100000;$fatal(1,"tb_units TIMEOUT");end
endmodule
