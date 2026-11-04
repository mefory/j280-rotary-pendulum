// POS 保持段 move_blend/k3_eff 专项:j280_top + force 静态传感器,手动切 POS,
// 盯 move_blend/k3_eff_m 是否复现板上 2044/-471。
`timescale 1ns/1ps
module tb_pos_hold_mb;
    reg clk=0,rst_n=0,plant_tick=0;
    always #10 clk=~clk;
    reg [7:0] div=0;
    always @(posedge clk) begin
        div <= (div==99) ? 8'd0 : div+8'd1;
        plant_tick <= (div==99);
    end
    reg [15:0] adc_sample=16'd65472;
    reg signed [31:0] encoder_sample=0;
    reg start_cmd=0, stop_cmd=0, pos_cmd=0;
    reg [1:0] dip_cmd=0;
    wire pwm,an1,an2;
    j280_top dut(.clk_50m(clk),.rst_n(rst_n),.key_sw(4'hf),.dip_sw(dip_cmd),
        .uart_rx(1'b1),.adc_data_in(10'd248),.adc_otr(1'b0),
        .enc1_a(1'b0),.enc1_b(1'b0),.enc2_a(1'b0),.enc2_b(1'b0),
        .PWMA(pwm), .PWMB(), .BN1(), .BN2(),
        .AN1(an1), .AN2(an2));
    integer n;
    integer mb_max=0, k3e_min=32767, k3e_max=-32768;
    initial begin
        force dut.u_core.tick=plant_tick;
        force dut.alpha_avg=adc_sample;
        force dut.enc_pos=encoder_sample;
        force dut.u_core.key_start=start_cmd;
        force dut.u_core.key_stop=stop_cmd;
        force dut.u_core.key_mode=pos_cmd;
        force dut.u_core.alpha_i = 16'sd100;
        force dut.u_core.alpha_dot_i = 16'sd30;
        force dut.u_core.theta_c01_i = 32'sd15000;
        force dut.u_core.theta_dot_i = 16'sd60;
        force dut.u_core.pos_tgt = 32'sd15000;
        #100; rst_n=1;
        repeat(20) @(posedge plant_tick);
        stop_cmd=1; repeat(4) @(posedge plant_tick); stop_cmd=0; repeat(10) @(posedge plant_tick);
        $display("CLEAR 后 mode=%0d fault=%b", dut.mode, dut.u_core.fault_latch);
        start_cmd=1; repeat(2) @(posedge plant_tick); start_cmd=0;
        repeat(10) @(posedge plant_tick);
        $display("mode=%0d alpha=%0d fault=%b run_en=%b", dut.mode, dut.u_core.alpha, dut.u_core.fault, dut.u_core.run_en);
        pos_cmd=1; repeat(2) @(posedge plant_tick); pos_cmd=0;
        repeat(10) @(posedge plant_tick);
        $display("mode=%0d alpha=%0d", dut.mode, dut.u_core.alpha);
        pos_cmd=1; repeat(2) @(posedge plant_tick); pos_cmd=0;
        repeat(10) @(posedge plant_tick);
        $display("mode=%0d (POS=3?) t=%0t", dut.mode, $time);
        if (dut.mode != 3'd3) begin $display("未进 POS,退出"); $finish; end
        for (n=0;n<3000;n=n+1) begin
            @(posedge plant_tick); #1;
            if (dut.u_core.move_blend > mb_max) begin
                mb_max = dut.u_core.move_blend;
                $display("t=%0dms mb 峰=%0d thdot=%0d k3e_m=%0d settle=%0d rehold=%b",
                    n, mb_max, dut.u_core.theta_dot, dut.u_core.k3_eff_m,
                    dut.u_core.pos_settle_ms, dut.u_core.pos_rehold);
            end
            if (dut.u_core.k3_eff_m > k3e_max) k3e_max=dut.u_core.k3_eff_m;
            if (dut.u_core.k3_eff_m < k3e_min) k3e_min=dut.u_core.k3_eff_m;
        end
        $display("3s 保持:mb_max=%0d k3e range [%0d,%0d]", mb_max, k3e_min, k3e_max);
        for (n=0;n<2000;n=n+1) begin
            @(posedge plant_tick); #1;
            force dut.u_core.theta_dot_i = (n%2) ? 16'sd600 : -16'sd950;
            if (dut.u_core.move_blend > mb_max) begin
                mb_max = dut.u_core.move_blend;
                $display("t2=%0dms mb 峰=%0d k3e_m=%0d", n, mb_max, dut.u_core.k3_eff_m);
            end
        end
        $display("抖动 2s 后:mb=%0d mb_max=%0d k3e range [%0d,%0d]",
                 dut.u_core.move_blend, mb_max, k3e_min, k3e_max);
        $finish;
    end
endmodule
