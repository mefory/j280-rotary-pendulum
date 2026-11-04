// Nonlinear plant in the testbench, ACTUAL top-level control RTL in the loop.
// 100 clocks = one simulated 1ms plant step. ADC average/encoder counts are
// injected; ADC serial timing, quadrature and PWM pins are covered separately.
`timescale 1ns/1ps
`include "tb/j280_plant_params.vh"
module tb_closedloop;
    reg clk=0,rst_n=0,plant_tick=0;
    always #10 clk=~clk;
    reg [15:0] adc_sample=0;
    reg signed [31:0] encoder_sample=0;
    reg start_cmd=0,stop_cmd=0,pos_cmd=0;
    reg [1:0] dip_cmd=0;
    wire pwm,an1,an2;
    j280_top dut(.clk_50m(clk),.rst_n(rst_n),.key_sw(4'hf),.dip_sw(dip_cmd),
        .uart_rx(1'b1),.adc_data_in(10'd248),.adc_otr(1'b0),
        .enc1_a(1'b0),.enc1_b(1'b0),.enc2_a(1'b0),.enc2_b(1'b0),
        .PWMA(pwm),.AN1(an1),.AN2(an2));
    real x0=0,x1=3.141592653589793,x2=0,x3=0;
    real j0=`PLANT_J0,jp=`PLANT_JP,c=`PLANT_C,mg=`PLANT_MG;
    real kt=`PLANT_KT,rm=`PLANT_R,bv=`PLANT_BV,bp=`PLANT_BP;
    real a0,a1,a2,a3,b0,b1,b2,b3,c0,c1,c2,c3,d0,d1,d2,d3;
    real tau,h=0.0001,ra,wrapped,theta,err,peak=0,maxerr=0,latest_target;
    integer n,sub,logfile,errors=0,catch_tick=-1,dropped=0;
    integer seed=7,noise_state,noise,quiet_ticks;
    integer relative=-1;
    integer mt_src=0;
    reg [13:0] mt_a14=0;
    reg [1023:0] trace_file="sim/out/closedloop.csv";
    task derivative(input real q0,q1,q2,q3,force0,output real f0,f1,f2,f3);
        real s,co,m11,m12,r1,r2,det;
        begin
            s=$sin(q1);co=$cos(q1);m11=j0+jp*s*s;m12=c*co;
            r1=force0-2*jp*s*co*q2*q3+c*s*q3*q3-bv*q2;
            r2=mg*s+jp*s*co*q2*q2-bp*q3;det=m11*jp-m12*m12;
            f0=q2;f1=q3;f2=(jp*r1-m12*r2)/det;f3=(-m12*r1+m11*r2)/det;
        end
    endtask
    function real wrapdeg(input real angle);
        real r;
        begin r=angle;while(r>180)r=r-360;while(r<=-180)r=r+360;wrapdeg=r;end
    endfunction
    initial begin
        if($value$plusargs("SEED=%d",seed))begin end
        if($value$plusargs("DIP=%d",relative))dip_cmd=relative[1:0];
        if($value$plusargs("MTSRC=%d",mt_src))begin end
        if($value$plusargs("TRACE=%s",trace_file))begin end
        noise_state=seed;
        force dut.u_core.tick=plant_tick;
        force dut.alpha_avg=adc_sample;
        force dut.enc_pos=encoder_sample;
        force dut.u_core.key_start=start_cmd;
        force dut.u_core.key_stop=stop_cmd;
        force dut.u_core.key_mode=pos_cmd;
        // +MTSRC=1：θ/θ̇ 改由电机侧 MT6701 绝对角链提供（experiment/mt6701-theta）。
        // 电机轴角 = 臂角 x0 × 减速比 20.45；14 位模 16384 送 mt_angle14。
        if(mt_src)begin
            force dut.u_core.th_src_mt=1'b1;
            force dut.u_core.mt_angle14=mt_a14;
        end
        logfile=$fopen(trace_file,"w");
        if(logfile==0)$fatal(1,"Cannot open closed-loop trace");
        $fdisplay(logfile,"time,theta,alpha,theta_dot,alpha_dot,duty,mode,ref,target");
        #100;rst_n=1;
        // Sensors warm in SAFE, clear power-on latch, then request startup.
        for(n=-20;n<25000;n=n+1)begin
            ra=wrapdeg(x1*180/3.141592653589793)+263.44;
            while(ra>=360)ra=ra-360;
            while(ra<0)ra=ra+360;
            if(ra>345)ra=345;
            noise=($random(noise_state)%4);
            quiet_ticks=$rtoi(ra/345*65472+0.5)+noise;
            if(quiet_ticks<0)quiet_ticks=0;if(quiet_ticks>65472)quiet_ticks=65472;
            adc_sample=quiet_ticks;
            ra=x0/(2*3.141592653589793)*1040;
            encoder_sample=(ra>=0) ? $rtoi(ra+0.5) : $rtoi(ra-0.5);
            if(mt_src)begin
                ra=x0*20.45/(2*3.141592653589793);    // 电机圈数（实数，含负角）
                ra=ra-$floor(ra);                      // 小数部分 → 14 位绝对角
                mt_a14=$rtoi(ra*16384+0.5);
            end
            stop_cmd=(n>=-15 && n<=-12);start_cmd=(n==0);pos_cmd=(n==8000);
            repeat(50)@(negedge clk);
            plant_tick=1;@(negedge clk);plant_tick=0;
            repeat(49)@(negedge clk);
            if(dut.u_core.mode==2 && catch_tick<0)catch_tick=n;
            if(catch_tick>=0 && (dut.u_core.mode==1 || dut.u_core.mode==4))dropped=1;
            if(dut.u_core.run_en)begin
                if(dut.u_core.duty_out>350)tau=(dut.u_core.duty_out-350)*12.0/10000*kt/rm;
                else if(dut.u_core.duty_out < -350)tau=(dut.u_core.duty_out+350)*12.0/10000*kt/rm;
                else tau=0;
            end else tau=0;
            for(sub=0;sub<10;sub=sub+1)begin
                derivative(x0,x1,x2,x3,tau,a0,a1,a2,a3);
                derivative(x0+h*a0/2,x1+h*a1/2,x2+h*a2/2,x3+h*a3/2,tau,b0,b1,b2,b3);
                derivative(x0+h*b0/2,x1+h*b1/2,x2+h*b2/2,x3+h*b3/2,tau,c0,c1,c2,c3);
                derivative(x0+h*c0,x1+h*c1,x2+h*c2,x3+h*c3,tau,d0,d1,d2,d3);
                x0=x0+h*(a0+2*b0+2*c0+d0)/6;x1=x1+h*(a1+2*b1+2*c1+d1)/6;
                x2=x2+h*(a2+2*b2+2*c2+d2)/6;x3=x3+h*(a3+2*b3+2*c3+d3)/6;
            end
            wrapped=wrapdeg(x1*180/3.141592653589793);theta=x0*180/3.141592653589793;
            if(dut.u_core.mode==3 && n>8002)begin
                if($abs(wrapped)>peak)peak=$abs(wrapped);
                latest_target=$itor(dut.u_core.pos_tgt)/100;
                err=$abs(theta-latest_target);
                if(n>=23000 && err>maxerr)maxerr=err;
            end
            if(n>=0 && n%10==0)$fdisplay(logfile,"%f,%f,%f,%f,%f,%d,%d,%f,%f",
                 n*0.001,theta,wrapped,x2*180/3.141592653589793,x3*180/3.141592653589793,
                dut.u_core.duty_out,dut.u_core.mode,$itor(dut.u_core.pos_ref)/100,$itor(dut.u_core.pos_tgt)/100);
        end
        $fclose(logfile);
        $display("RTL nonlinear seed=%d dip=%d capture=%dms dropped=%d moving_alpha=%f late_error=%f final_mode=%d",
            seed,dip_cmd,catch_tick,dropped,peak,maxerr,dut.u_core.mode);
        if(catch_tick<0 || catch_tick>5000 || dropped || dut.u_core.mode!=3 || peak>3 || maxerr>2)
            $fatal(1,"tb_closedloop FAIL");
        $display("tb_closedloop PASS");$finish;
    end
endmodule
