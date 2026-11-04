// ============================================================================
// tb_top_smoke.v — 顶层冒烟测试（虚拟整机）
// 1) 复位后 SAFE→(上电故障需确认)→ 按下急停键确认清除 → IDLE
// 2) SW1 启动 → SWING（假 ADC 悬垂值）→ PWM 有输出、遥测帧出现
// 3) 假 ADC 给倒立值 → 捕获域 → BALANCE
// 4) 全程监控 X 态
// 仿真时长 ~120ms 实际时间（iverilog 数秒内完成）
// ============================================================================
`timescale 1ns / 1ps
`include "j280_defs.vh"
module tb_top_smoke;
    // ★2026-11-04 修复：rst_n 初值必须为 1，再由 initial 块拉低产生真实 negedge。
    // 若初值直接是 0，time-0 没有 1→0 下降沿事件 ⇒ 所有
    // `always @(posedge clk or negedge rst_n)` 的复位分支从未执行，寄存器停在 X
    // （实测 duty_out=X → PWMA=X，误报"输出存在 X"）。
    reg clk = 0, rst_n = 1;
    reg [3:0] key_sw = 4'hF;      // 全松开（低有效）
    reg [1:0] dip_sw = 2'b00;
    reg uart_rx = 1'b1;
    reg [9:0] adc_data_in = 10'd248;   // 悬垂（官方口径）
    reg adc_otr = 0;
    reg enc1_a = 0, enc1_b = 0;

    wire uart_tx, adc_clk, adc_oe_n, adc_stby_n;
    wire [3:0] led;
    wire PWMA, AN1, AN2, PWMB, BN1, BN2;

    j280_top dut (
        .clk_50m(clk), .rst_n(rst_n),
        .key_sw(key_sw), .dip_sw(dip_sw), .led(led),
        .uart_tx(uart_tx), .uart_rx(uart_rx),
        .adc_clk(adc_clk), .adc_oe_n(adc_oe_n), .adc_stby_n(adc_stby_n),
        .adc_data_in(adc_data_in), .adc_otr(adc_otr),
        .enc1_a(enc1_a), .enc1_b(enc1_b),
        .PWMA(PWMA), .AN1(AN1), .AN2(AN2),
        .enc2_a(1'b0), .enc2_b(1'b0),
        .PWMB(PWMB), .BN1(BN1), .BN2(BN2)
    );
    always #10 clk = ~clk;

    integer errors = 0;
    integer tel_frames = 0;
    integer pwm_edges = 0;

    // PWM 边沿计数
    always @(posedge PWMA) pwm_edges <= pwm_edges + 1;

    // 编码器慢速正转（A 超前 B）：1 计数 / 200µs = 5000 计数/s
    reg [1:0] enc_st = 0;
    reg enc_run=1;
    always #200000 begin
        if(enc_run) begin
            enc_st <= (enc_st == 3) ? 0 : enc_st + 1;
            case (enc_st)
            0: begin enc1_a = 0; enc1_b = 0; end
            1: begin enc1_a = 1; enc1_b = 0; end
            2: begin enc1_a = 1; enc1_b = 1; end
            3: begin enc1_a = 0; enc1_b = 1; end
            endcase
        end
    end

    // UART 位接收：监视遥测帧头 A5 5A
    localparam BITW = `UART_BAUD_DIV * 20;   // 1Mbaud @50MHz → 1µs
    reg [7:0] rbyte;
    task rx_byte_task;
        integer k;
        begin
            #(BITW/2);
            for (k = 0; k < 8; k = k + 1) begin
                #(BITW);
                rbyte = {uart_tx, rbyte[7:1]};
            end
            #(BITW); // 停止位
            if (rbyte == 8'hA5) tel_frames = tel_frames + 1;
        end
    endtask
    always @(negedge uart_tx) rx_byte_task;

    task press_key(input integer idx, input integer hold_ms);
        begin
            key_sw[idx] = 1'b0;
            #(hold_ms * 1_000_000);
            key_sw[idx] = 1'b1;
            #500_000;
        end
    endtask

    initial begin
        rst_n = 0; #2000; rst_n = 1;
        #1000;

        // ---- 上电安全态 ----
        if (dut.u_core.u_safe.fault_latch !== 1'b1) begin
            $display("FAIL: 上电 fault_latch 应为 1"); errors = errors + 1;
        end
        if (PWMA !== 1'b0 || AN1 !== 1'b0 || AN2 !== 1'b0) begin
            $display("FAIL: 上电输出未关断"); errors = errors + 1;
        end
        // 按急停键确认清除上电故障（SW4）
        press_key(3, 30);
        if (dut.u_core.u_safe.fault_latch !== 1'b0) begin
            $display("FAIL: 确认后 fault_latch 应清零"); errors = errors + 1;
        end
        if (dut.u_core.mode !== `M_IDLE) begin
            $display("FAIL: 应为 IDLE, mode=%0d", dut.u_core.mode); errors = errors + 1;
        end

        // ---- 悬垂值下启动 → SWING ----
        press_key(0, 30);
        #20_000_000;
        if (dut.u_core.mode !== `M_SWING) begin
            $display("FAIL: 应为 SWING, mode=%0d", dut.u_core.mode); errors = errors + 1;
        end
        if (pwm_edges == 0) begin
            $display("FAIL: SWING 下 PWM 无输出"); errors = errors + 1;
        end
        if (tel_frames < 5) begin
            $display("FAIL: 遥测帧过少 = %0d", tel_frames); errors = errors + 1;
        end

        // ---- 倒立值 → 捕获 → BALANCE ----
        adc_data_in = 10'd781;   // ≈ 倒立点
        #30_000_000;
        if(dut.u_core.mode!==`M_SWING)begin
            $display("FAIL: overspeed must block capture");errors=errors+1;
        end
        enc_run=0;
        #30_000_000;
        if (dut.u_core.mode !== `M_BALANCE) begin
            $display("FAIL: 应为 BALANCE, mode=%0d alpha=%0d adot=%0d",
                     dut.u_core.mode, dut.u_core.alpha, dut.u_core.alpha_dot);
            errors = errors + 1;
        end
        // ★2026-11-04 修复：原断言检查 pos_tgt（当前斜坡位置）− θ == 3000，
        // 但 pos_tgt 是从 θ 起步按 20°/s 斜坡爬向 pos_tgt_cmd 的，刚进 POS 时
        // 差值必为 0（需 ~750ms 才爬完）。应检查【目标】pos_tgt_cmd − θ。
        press_key(1,30);
        if(dut.u_core.mode!==`M_POS || dut.u_core.pos_tgt_cmd-dut.u_core.theta_c01!=3000)begin
            $display("FAIL: POS relative target mode=%d cmd=%d theta=%d diff=%d",
                     dut.u_core.mode, dut.u_core.pos_tgt_cmd, dut.u_core.theta_c01,
                     dut.u_core.pos_tgt_cmd-dut.u_core.theta_c01);
            errors=errors+1;
        end
        // 目标一旦锁存，改变 DIP 不应追逐臂（V1 无拨码：dip_sw 在 top 内硬编码
        // 2'b00，此处 TB 改自己的 reg 对 DUT 无影响——保留该检查以验证锁存语义）
        dip_sw=2'b11;
        #10_000_000;
        if(dut.u_core.pos_tgt_cmd-dut.u_core.theta_c01!=3000)begin
            $display("FAIL: POS target not latched (cmd=%d theta=%d)",
                     dut.u_core.pos_tgt_cmd, dut.u_core.theta_c01);errors=errors+1;
        end

        // ---- 无 X 检查 ----
        if (^{led, PWMA, AN1, AN2, uart_tx} === 1'bx) begin
            $display("FAIL: 输出存在 X"); errors = errors + 1;
        end

        $display("tel_frames=%0d pwm_edges=%0d alpha=%0d theta_dot=%0d duty=%0d",
                 tel_frames, pwm_edges, dut.u_core.alpha, dut.u_core.theta_dot, dut.u_core.duty_out);
        if (errors == 0) $display("tb_top_smoke PASS");
        else             $display("tb_top_smoke FAIL errors=%0d", errors);
        $finish;
    end

    // 超时保护
    initial begin
        #300_000_000;   // 300ms 仿真时间
        $display("TIMEOUT"); $finish;
    end
endmodule
