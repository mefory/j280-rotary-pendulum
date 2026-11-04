`timescale 1ns/1ps
// 固定实测状态输入比较指令整形；不是新控制器闭环仿真。
module tb_swingup_drive_replay;
    reg clk=0, rst_n=0, tick=0, start=0, active=0;
    always #10 clk=~clk;
    reg signed [31:0] theta=0;
    reg signed [15:0] td=0, alpha=0, ad=0;
    wire signed [15:0] old_duty, new_duty;
    wire signed [15:0] old_output, new_output;
    swingup_core #(.PUMP_MODE(2), .ALPHA_QUALITY_EN(1)) old(
        .clk(clk), .rst_n(rst_n), .tick(tick), .start(start), .theta(theta), .theta_dot(td),
        .alpha(alpha), .alpha_dot(ad), .delta(16'sd11800), .amplitude(16'sd8800),
        .duty(old_duty), .alpha_valid(1'b1), .swing_active(active));
    swingup_core #(.PUMP_MODE(2), .ALPHA_QUALITY_EN(1), .RES_DRIVE_SHAPING_EN(1)) shaped(
        .clk(clk), .rst_n(rst_n), .tick(tick), .start(start), .theta(theta), .theta_dot(td),
        .alpha(alpha), .alpha_dot(ad), .delta(16'sd11800), .amplitude(16'sd8800),
        .duty(new_duty), .alpha_valid(1'b1), .swing_active(active));
    out_stage old_out(.clk(clk),.rst_n(rst_n),.tick(tick),.bypass(1'b0),
        .dz_pos(16'sd1280),.dz_neg(16'sd1480),.duty_in(active ? old_duty : 16'sd0),.duty_out(old_output));
    out_stage new_out(.clk(clk),.rst_n(rst_n),.tick(tick),.bypass(1'b0),
        .dz_pos(16'sd1280),.dz_neg(16'sd1480),.duty_in(active ? new_duty : 16'sd0),.duty_out(new_output));
    task pulse;
        begin repeat(8) @(negedge clk); tick=1; @(negedge clk); tick=0; repeat(12) @(negedge clk); end
    endtask
    reg [4095:0] input_path, output_path;
    integer fi,fo,status,t,m,th,tv,a,av,u,previous_mode=0,frames=0;
    initial begin
        if(!$value$plusargs("INPUT=%s",input_path) || !$value$plusargs("OUTPUT=%s",output_path))
            $fatal(1,"tb_swingup_drive_replay FAIL: paths required");
        fi=$fopen(input_path,"r");fo=$fopen(output_path,"w");
        if(!fi || !fo)$fatal(1,"tb_swingup_drive_replay FAIL: files");
        $fdisplay(fo,"tick,mode,theta,theta_dot,actual_duty,old_duty,new_duty,old_command,new_command");
        #100; rst_n=1;
        // Gowin 初值假设，仅用于旧算法对照；旧版未记录启动前的 EMA/扫频状态。
        old.res_err_f=0;
        while(!$feof(fi)) begin
            status=$fscanf(fi,"%d %d %d %d %d %d %d\n",t,m,th,tv,a,av,u);
            if(status!=7)$fatal(1,"tb_swingup_drive_replay FAIL: malformed input");
            theta=th;td=tv;alpha=a;ad=av;active=(m==1);
            start=(m==1 && previous_mode!=1);pulse;start=0;pulse;
            if($isunknown({old_duty,new_duty,old_output,new_output}))$fatal(1,"tb_swingup_drive_replay FAIL: X");
            $fdisplay(fo,"%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d",t,m,theta,td,u,old_output,new_output,old_duty,new_duty);
            previous_mode=m;frames=frames+1;
        end
        $fclose(fi);$fclose(fo);
        $display("tb_swingup_drive_replay PASS: %0d frames, measured state zero-order hold",frames);
        $finish;
    end
endmodule
