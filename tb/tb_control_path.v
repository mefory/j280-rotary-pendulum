`timescale 1ns/1ps
module tb_control_path;
    reg clk=0,rst_n=0,tick=0,clear=1;
    always #10 clk=~clk;
    reg signed [31:0] pos=0,target=0,count=0;
    wire signed [31:0] refpos,theta;
    wire signed [15:0] vref_unused;
    wire signed [15:0] speed,sw;
    reg signed [15:0] alpha=0,ad=0;
    reg start_cmd=0,mode_cmd=0,stop_cmd=0;
    reg signed [15:0] fsm_alpha=18000,fsm_speed=0;
    wire [2:0] mode;
    wire av_stalled;
    // 超时覆盖 2000ms（RTL 定案 25000ms，仅为缩短仿真；触发拍=超时值+1）
    mode_fsm #(.SWING_TIMEOUT_MS(16'd2000)) fsm(.clk(clk),.rst_n(rst_n),.tick(tick),.key_start(start_cmd),
        .key_mode(mode_cmd),.key_stop(stop_cmd),.fault(1'b0),
        .alpha(fsm_alpha),.alpha_dot(ad),.theta_dot(fsm_speed),
        .arm_stalled(av_stalled),
        .catch_ang(16'sd3000),.catch_ang_fall(16'sd6000),.mode(mode));
    reg start_swing=0;
    reg signed [31:0] fling_theta=0;
    reg signed [15:0] fling_delta=16'sd11800, fling_amp=16'sd9000;
    reg signed [15:0] fling_speed=0;   // 补甩门控（0=静止底座，恒允许补甩）
    traj_gen tr(clk,rst_n,tick,clear,pos,target,refpos,vref_unused);
    arm_vel av(.clk(clk),.rst_n(rst_n),.tick(tick),.pos(count),.theta_c01(theta),.theta_dot(speed),.arm_stalled(av_stalled));
    swingup_core swing(clk,rst_n,tick,start_swing,fling_theta,fling_speed,16'sd0,16'sd0,fling_delta,
                   16'sd0,16'sd0,16'sd0,   // catch_ang/coast_amp/coast_margin（本 tb 未用）
                   fling_amp,sw,swing_brake_w,1'b1,1'b1);
    wire swing_brake_w;
    integer i,errors=0,previous;
    task pulse;
        begin repeat(8)@(negedge clk);tick=1;@(negedge clk);tick=0;repeat(8)@(negedge clk);end
    endtask
    task swingstart;
        begin repeat(2)@(negedge clk);tick=1;start_swing=1;@(negedge clk);tick=0;start_swing=0;repeat(2)@(negedge clk);end
    endtask
    task swingcheck(input integer th,expected);
        begin
            fling_theta=th;repeat(4)@(negedge clk);
            if(sw!==expected)begin $display("FAIL swing theta=%d u=%d expected=%d",th,sw,expected);errors=errors+1;end
        end
    endtask
    initial begin
        #100;rst_n=1;
        pos=72000;pulse;clear=0;target=75000;
        // S 曲线（2026-10-09）：巡航 2700 拍 + 减速尾 ~880 拍（线性逼近+snap≤16）
        for(i=0;i<4200;i=i+1)begin
            previous=refpos;pulse;
            if(refpos<previous || refpos>previous+16 || refpos>75000)errors=errors+1;
        end
        if(refpos!=75000)errors=errors+1;
        target=69000;
        for(i=0;i<7200;i=i+1)begin
            previous=refpos;pulse;
            if(refpos>previous || refpos<previous-16 || refpos<69000)errors=errors+1;
        end
        if(refpos!=69000)errors=errors+1;
        // M 法段：1 计数/拍 ⇒ 4ms 差分 4 计数 ⇒ 3462（M_SCALE 按 4ms 定标）
        for(i=0;i<80;i=i+1)begin count=count+1;pulse;end
        if(speed<3460 || speed>3464)begin $display("FAIL M positive %d",speed);errors=errors+1;end
        for(i=0;i<80;i=i+1)begin count=count-1;pulse;end
        if(speed > -3460 || speed < -3464)begin $display("FAIL M negative %d",speed);errors=errors+1;end
        // 2026-09-23：低速段改用 T 法后，停下时由 STALL_TICKS(=300 拍) 判停归零，
        // 因此必须等超过 300 拍才能要求 speed==0（旧实现 M 法 8 拍就自然归零）。
        repeat(320)pulse;
        if(speed!=0)begin $display("FAIL stale speed %d",speed);errors=errors+1;end
        // 甩摆伺服：进入沿锁存 target=theta0+delta，u=sat(9*(target-theta), amp)
        swingstart;
        swingcheck(0,9000);        // err=+11800 → 饱和
        swingcheck(11800,0);       // 到达目标
        swingcheck(12300,-4500);   // 过冲 500，线性区 9*500
        swingcheck(16800,-9000);   // err=-5000 → 反向饱和
        swingcheck(11200,5400);    // 线性区 9*600
        fling_amp=16'sd5000;
        swingcheck(0,5000);        // 运行时限幅生效（+9*11800 → +5000）
        fling_amp=16'sd9000;
        swingcheck(0,9000);
        fling_theta=-16'sd5000;    // 先设进入点臂角，再给进入沿
        swingstart;                // 新的进入沿重锁存 target=-5000+11800=6800
        swingcheck(-5000,9000);
        swingcheck(6800,0);
        // 手扶稳摆：IDLE 下必须同时满足 |alpha|<5°、|alpha_dot|<30°/s、
        // |theta_dot|<20°/s；成功后直接进 BALANCE，不经过 SWING。
        start_cmd=1;pulse;start_cmd=0; // SAFE -> IDLE
        fsm_alpha=0;fsm_speed=0;ad=400;mode_cmd=1;pulse;mode_cmd=0;
        if(mode!=0)begin $display("FAIL manual balance accepted high alpha_dot mode=%d",mode);errors=errors+1;end
        ad=0;fsm_speed=300;mode_cmd=1;pulse;mode_cmd=0;
        if(mode!=0)begin $display("FAIL manual balance accepted moving arm mode=%d",mode);errors=errors+1;end
        fsm_speed=0;fsm_alpha=600;mode_cmd=1;pulse;mode_cmd=0;
        if(mode!=0)begin $display("FAIL manual balance accepted tilted rod mode=%d",mode);errors=errors+1;end
        fsm_alpha=0;mode_cmd=1;pulse;mode_cmd=0;
        if(mode!=2)begin $display("FAIL manual balance did not enter BALANCE mode=%d",mode);errors=errors+1;end
        stop_cmd=1;pulse;stop_cmd=0;
        start_cmd=1;pulse;start_cmd=0; // SAFE -> IDLE
        start_cmd=1;pulse;start_cmd=0; // IDLE -> SWING
        fsm_alpha=0;fsm_speed=6000;pulse;
        if(mode!=1)begin $display("FAIL overspeed capture");errors=errors+1;end
        fsm_speed=0;pulse;
        if(mode!=2)begin $display("FAIL stable capture");errors=errors+1;end
        mode_cmd=1;fsm_speed=700;pulse;
        if(mode!=2)begin $display("FAIL POS entered while arm moving mode=%d",mode);errors=errors+1;end
        fsm_speed=0;fsm_alpha=600;pulse;
        if(mode!=2)begin $display("FAIL POS entered while tilted");errors=errors+1;end
        fsm_alpha=0;pulse;mode_cmd=0;
        if(mode!=3)begin $display("FAIL stable POS request");errors=errors+1;end
        // 单次起摆：稳摆/定位掉摆后必须进 SAFE，不得自动再次甩摆。
        fsm_alpha=7000;pulse;
        if(mode!=4)begin $display("FAIL drop-out did not enter SAFE mode=%d",mode);errors=errors+1;end
        start_cmd=1;pulse;start_cmd=0; // SAFE -> IDLE，重新武装下一次起摆
        start_cmd=1;pulse;start_cmd=0; // IDLE -> SWING
        if(mode!=1)begin $display("FAIL re-armed start did not enter SWING mode=%d",mode);errors=errors+1;end
        fsm_alpha=18000;
        repeat(1999)pulse;
        if(mode!=1)begin $display("FAIL swing timed out too early mode=%d",mode);errors=errors+1;end
        repeat(2)pulse;
        if(mode!=4)begin $display("FAIL swing timeout did not enter SAFE mode=%d",mode);errors=errors+1;end
        repeat(10)pulse;
        if(mode!=4)begin $display("FAIL swing timeout restarted mode=%d",mode);errors=errors+1;end
        if(errors)$fatal(1,"tb_control_path FAIL %d",errors);
        $display("tb_control_path PASS: trajectory, reverse, stop, fling, saturation, mode guards");
        $finish;
    end
    initial begin #10000000;$fatal(1,"control_path timeout");end
endmodule
