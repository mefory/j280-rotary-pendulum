// End-to-end mission: commands travel real UART pins from a Python-generated
// timeline; the actual top-level RTL closes the loop on the nonlinear Furuta
// plant; every TX byte is logged for host-side telemetry verdicts.
// Simulation is accelerated: one control tick per 1500 clocks (x33) and
// UART at BAUD_DIV=5 (div=4 would sample on bit boundaries; 5 keeps the
// 1.5-bit mid-point sampling of uart_rx 20 ns away from every edge). A
// 50-byte frame occupies 2500 clocks < the 3000-clock telemetry period, so
// snapshots are never overwritten and priority ACKs wait at most one frame.
// iverilog manages ~20k clocks/s on this design, so the mission is kept
// compact (capture + POS entry/anchor/ramp + stop, ~6.3 s); long-hold
// metrics stay covered by tb_closedloop and host.acceptance.
// ADC serial timing and quadrature pins stay covered by tb_adc_if/tb_qenc4x;
// the plant injects the averaged ADC word and encoder count like tb_closedloop.
// Stimulus format (sim/out/uart_mission_stimulus.txt):
//   "0 <tick>"      advance until the control-tick counter reaches <tick>
//   "<len> <hex…>"  send <len> raw bytes over the RX pin
`timescale 1ns/1ps
`include "tb/j280_plant_params.vh"
module tb_uart_mission;
    localparam integer BAUD=5;    // clocks per UART bit
    localparam integer TICK=1500; // clocks per 1 ms control tick
    reg clk=0,rst_n=0,rx=1,control_tick=0;
    always #10 clk=~clk;
    wire tx;
    j280_top dut(.clk_50m(clk),.rst_n(rst_n),.key_sw(4'hf),.dip_sw(2'b00),
        .uart_rx(rx),.uart_tx(tx),.adc_data_in(10'd781),.adc_otr(1'b0),
        .enc1_a(1'b0),.enc1_b(1'b0),.enc2_a(1'b0),.enc2_b(1'b0));
    defparam dut.u_core.u_tx.BAUD_DIV=BAUD;
    defparam dut.u_core.u_rx.BAUD_DIV=BAUD;
    defparam dut.u_core.u_debug.rx.TIMEOUT_CLOCKS=50000;
    integer tick_clocks=0;
    integer tick_count=0;
    always @(negedge clk)begin
        control_tick=0;
        if(tick_clocks==TICK-1)begin tick_clocks=0;control_tick=1;tick_count=tick_count+1;end
        else tick_clocks=tick_clocks+1;
    end

    reg [15:0] adc_sample=0;
    reg signed [31:0] encoder_sample=0;
    // Furuta plant, same equations/parameters as tb_closedloop.
    real x0=0,x1=3.141592653589793,x2=0,x3=0;
    real j0=`PLANT_J0,jp=`PLANT_JP,c=`PLANT_C,mg=`PLANT_MG;
    real kt=`PLANT_KT,rm=`PLANT_R,bv=`PLANT_BV,bp=`PLANT_BP;
    real a0,a1,a2,a3,b0,b1,b2,b3,c0,c1,c2,c3,d0,d1,d2,d3;
    real tau,h=0.0001,ra;
    integer sub,seed=7,noise_state,noise,quiet_ticks;
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

    // Plant: refresh sensors just before the tick edge, then integrate one
    // control period under the torque the DUT computed for the previous tick.
    initial begin
        force dut.u_core.tick=control_tick;
        force dut.alpha_avg=adc_sample;
        force dut.enc_pos=encoder_sample;
        noise_state=seed;
        #100;rst_n=1;
        forever begin
            wait(tick_clocks==TICK-10);
            ra=wrapdeg(x1*180/3.141592653589793)+263.44;
            while(ra>=360)ra=ra-360;
            while(ra<0)ra=ra+360;
            if(ra>345)ra=345;
            noise=($random(noise_state)%4);
            quiet_ticks=$rtoi(ra/345*65472+0.5)+noise;
            if(quiet_ticks<0)quiet_ticks=0;if(quiet_ticks>65472)quiet_ticks=65472;
            adc_sample=quiet_ticks[15:0];
            ra=x0/(2*3.141592653589793)*1040;
            encoder_sample=(ra>=0) ? $rtoi(ra+0.5) : $rtoi(ra-0.5);
            @(posedge control_tick);
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
        end
    end

    integer f,scan,n,j,value,t_wait;
    reg [7:0] byte_rx;
    integer k;
    always @(negedge tx)begin
        #(BAUD*30);  // 1.5 bit periods to the first data-bit centre
        for(k=0;k<8;k=k+1)begin byte_rx[k]=tx;#(BAUD*20);end
        #(BAUD*10);
        if(tx!==1'b1)$fatal(1,"UART stop bit invalid");
        $display("RX %02h",byte_rx);
    end
    task send_byte(input [7:0] data);
        integer b;
        begin
            rx=0;#(BAUD*20);
            for(b=0;b<8;b=b+1)begin rx=data[b];#(BAUD*20);end
            rx=1;#(BAUD*20);
        end
    endtask
    initial begin
        f=$fopen("sim/out/uart_mission_stimulus.txt","r");
        if(f==0)$fatal(1,"Missing UART mission stimuli");
        wait(rst_n==1);
        while(!$feof(f))begin
            scan=$fscanf(f,"%d",n);
            if(scan==1)begin
                if(n==0)begin
                    scan=$fscanf(f,"%d",t_wait);
                    if(scan==1)wait(tick_count>=t_wait);
                end else begin
                    for(j=0;j<n;j=j+1)begin
                        scan=$fscanf(f,"%h",value);
                        if(scan!=1)$fatal(1,"Bad mission stimuli");
                        send_byte(value[7:0]);
                    end
                end
            end
        end
        $fclose(f);
        #20000000;  // flush trailing ACK and a few hundred telemetry frames
        $display("tb_uart_mission DONE");
        $finish;
    end
    initial begin #1500000000;$fatal(1,"tb_uart_mission TIMEOUT");end
endmodule
