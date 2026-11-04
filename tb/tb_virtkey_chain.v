// tb_virtkey_chain.v — 全链路仿真：fb_channel → key_sw → ui → mode_fsm
// 验证 HPS 虚拟键命令能否驱动真实模式状态机
`timescale 1ns / 1ns
module tb_virtkey_chain;
    reg clk = 0, rst_n = 0, tick = 0;
    reg [9:0] led_pio = 10'd0;
    reg [15:0] alpha = -16'sd2000;   // -20°：远离捕获域，防意外迁移
    reg [15:0] alpha_dot = 0, duty = 0, theta_dot = 0;
    reg [31:0] theta = 0;
    reg [2:0] mode;
    reg fault = 1'b0, hil_use = 1'b0, run_en = 1'b0;
    reg [3:0] KEY = 4'b1111;         // 全部物理键空闲（低有效）
    reg [9:0] SW = 10'b0000000000;   // SW8=0（非标定）SW9=0（真实模式）
    wire [3:0] virt_key_n;
    wire [3:0] key_sw = KEY & virt_key_n;   // de10_top 同款接法
    wire [3:0] hps_dipsw, hps_button;

    fb_channel u_fbch (
        .clk(clk), .rst_n(rst_n), .tick(tick),
        .alpha(alpha), .alpha_dot(alpha_dot), .theta(theta), .theta_dot(theta_dot),
        .duty(duty), .mode(mode), .fault(fault), .hil_use(hil_use), .run_en(run_en),
        .led_pio(led_pio), .dipsw_pio(hps_dipsw), .button_pio(hps_button),
        .virt_key_n(virt_key_n)
    );

    wire key_start, key_mode, key_stop, key_calib, calib_done;
    wire [3:0] led;
    ui u_ui (
        .clk(clk), .rst_n(rst_n), .tick(tick),
        .key_sw(key_sw), .dip_sw(2'b00), .mode(mode),
        .fault_latch(fault), .calib_done(calib_done),
        .key_start(key_start), .key_mode(key_mode),
        .key_calib(key_calib), .key_stop(key_stop),
        .dip(), .led(led)
    );

    mode_fsm u_fsm (
        .clk(clk), .rst_n(rst_n), .tick(tick),
        .key_start(key_start), .key_mode(key_mode), .key_stop(key_stop),
        .fault(fault), .alpha(alpha), .alpha_dot(alpha_dot), .theta_dot(theta_dot),
        .catch_ang(16'sd3000), .catch_ang_fall(16'sd6000), .mode(mode)
    );

    always #10 clk = ~clk;
    reg [5:0] tick_div = 0;
    always @(posedge clk) begin
        if (tick_div == 49) begin tick_div <= 0; tick <= 1'b1; end
        else begin tick_div <= tick_div + 1'b1; tick <= 1'b0; end
    end

    task tick_n(input integer n);
        integer k;
        begin
            for (k = 0; k < n; k = k + 1) @(posedge tick);
            @(negedge tick);
        end
    endtask

    initial begin
        repeat (5) @(posedge clk);
        rst_n = 1;
        tick_n(20);
        if (mode !== 3'd4) begin $display("FAIL: 上电非 SAFE, mode=%0d", mode); $finish; end
        $display("PASS: 上电 SAFE");

        // 虚拟 start：SAFE → IDLE
        @(negedge tick); led_pio = 10'd1;
        tick_n(80); led_pio = 10'd0;
        if (mode === 3'd0) $display("PASS: 虚拟启动 SAFE->IDLE");
        else begin $display("FAIL: 虚拟启动后 mode=%0d (期望 0)", mode); $display("VIRTUAL-KEY-BROKEN"); end

        // 虚拟 start：IDLE → SWING
        @(negedge tick); led_pio = 10'd1;
        tick_n(80); led_pio = 10'd0;
        if (mode === 3'd1) $display("PASS: 虚拟启动 IDLE->SWING");
        else $display("INFO: 二次启动后 mode=%0d (1=SWING)", mode);

        // 虚拟急停：SWING → SAFE
        @(negedge tick); led_pio = 10'd3;
        tick_n(80); led_pio = 10'd0;
        if (mode === 3'd4) $display("PASS: 虚拟急停 -> SAFE");
        else $display("FAIL: 急停后 mode=%0d (期望 4)", mode);

        $finish;
    end
endmodule
