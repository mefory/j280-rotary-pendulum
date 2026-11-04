// ============================================================================
// j280_top.v — J280 旋转倒立摆 板级顶层（GW2A-55 硬件外壳）
//
// 本文件只做板级接线，控制逻辑全部在 rtl/ctrl_core.v（板级无关）：
//   3PA1030 ADC 串行时序 → adc_if → alpha_avg ┐
//   WDD35D4 正交编码器    → qenc4x → enc_pos   ├→ ctrl_core
//   SW1..SW4 / DIP / LED  → ui                 ┤
//   CH340 UART            ↔ uart_rx/tx         ┤
//   TB6612 电机           ← motor_pwm ← duty_out┘
//
// 【到手 bring-up 检查】见架构文档 §6：极性开关 POL_*、CPR、电机插座通道。
// 通道 B（PWMB/BN1/BN2）保持安全关断；enc2 预留。
// ============================================================================
`include "j280_defs.vh"
module j280_top (
    input  wire        clk_50m,
    input  wire        rst_n,        // SW5，低有效

    input  wire [3:0]  key_sw,       // SW1..SW4 低有效
    input  wire [1:0]  dip_sw,
    output wire [3:0]  led,

    output wire        uart_tx,
    input  wire        uart_rx,

    // 3PA1030 ADC（摆角 WDD35D4）
    output wire        adc_clk,
    output wire        adc_oe_n,
    output wire        adc_stby_n,
    input  wire [9:0]  adc_data_in,
    input  wire        adc_otr,

    // 编码器 + TB6612 通道 A（电机）
    input  wire        enc1_a,
    input  wire        enc1_b,
    output wire        PWMA,
    output wire        AN1,
    output wire        AN2,
    // 通道 B 预留（保持关断）
    input  wire        enc2_a,
    input  wire        enc2_b,
    output wire        PWMB,
    output wire        BN1,
    output wire        BN2
);
    // ---------------- 传感前端 ----------------
    wire [15:0] alpha_avg;
    wire        otr_event;
    adc_if u_adc (
        .clk(clk_50m), .rst_n(rst_n),
        .adc_clk(adc_clk), .adc_oe_n(adc_oe_n), .adc_stby_n(adc_stby_n),
        .adc_data_in(adc_data_in), .adc_otr(adc_otr),
        .alpha_avg(alpha_avg), .otr_event(otr_event)
    );

    wire signed [31:0] enc_pos;
    qenc4x #(.POL_FLIP(`POL_ENC_FLIP)) u_enc (
        .clk(clk_50m), .rst_n(rst_n), .enc_a(enc1_a), .enc_b(enc1_b), .pos(enc_pos)
    );

    // ---------------- 控制核心（板级无关） ----------------
    wire signed [15:0] duty_out;
    wire               motor_enable;
    wire [2:0]         mode;

    ctrl_core u_core (
        .clk(clk_50m), .rst_n(rst_n),
        .adc_raw(alpha_avg), .otr_fault(otr_event), .enc_pos(enc_pos),
        .mt_angle14(14'd0), .th_src_mt(1'b0),   // 无电机侧 MT6701：恒霍尔源
        .mt6_frames(16'd0), .mt6_good(16'd0), .mt6_crc_err(16'd0),
        .mt6_mg_err(16'd0), .mt6_mg_code(3'd0), .mt6_angle_raw(14'd0),
        .mt6_fault(1'b0), .mt6_cause(2'd0), .mt6_max_no_good(16'd0),
        .mt68_frames(16'd0), .mt68_good(16'd0), .mt68_crc_err(16'd0),
        .mt68_status_err(16'd0), .mt68_status(3'd0), .mt68_angle_raw(21'd0),
        .mt68_fault(1'b0),
        .hil_use(1'b0),
        .key_sw(key_sw), .dip_sw(dip_sw), .led(led),
        .uart_rx(uart_rx), .uart_tx(uart_tx),
        .duty_out(duty_out), .motor_enable(motor_enable), .mode(mode),
        .alpha(), .alpha_dot(), .theta_c01(), .theta_dot(),
        .tick_out(), .fault()
    );

    // ---------------- 输出 ----------------
    // BRAKE_TH(0)：J280/DE10 驱动器死区小（375），零点刹车是 T20K+TB6612 专属
    motor_pwm #(.CARRIER(`PWM_CARRIER), .BRAKE_TH(0)) u_pwmA (
        .clk(clk_50m), .rst_n(rst_n),
        .run_en(motor_enable), .brake_en(1'b0),
        .duty(duty_out),
        .in1(AN1), .in2(AN2), .pwm(PWMA)
    );
    assign PWMB = 1'b0;  assign BN1 = 1'b0;  assign BN2 = 1'b0;  // 通道 B 关断
endmodule
