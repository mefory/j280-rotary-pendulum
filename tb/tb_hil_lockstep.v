// ============================================================================
// tb_hil_lockstep.v — HIL 锁步机制回归（DE10 无硬件算法验证的核心契约）
//
// 被测契约（host/hil.py 完全依赖它）：
//   1) 每条 STEP 恰推进**一个**控制毫秒：遥测的 tick 严格逐条 +1，不多不少。
//   2) STEP 的 ACK 携带该拍算出的 duty/mode/fault，且 mode 与当前模式一致。
//   3) 传感器字确实被采用：把 ADC 置为倒立点、编码器置零时，模式机应当从
//      IDLE→SWING 走到 BALANCE（捕获），证明注入值真的进了 angle_proc/arm_vel。
//   4) STEP 时无游离节拍：HIL 模式下自由计数器不驱动控制核，故两次 STEP
//      之间 tick_count 不自行增长（靠 (1) 的严格 +1 间接断言）。
//
// 激励由 host/test_hil_lockstep.py 生成（含真实 CRC），经 RX 引脚回放；
// TX 以 "RX <hex>" 打印，Python 侧解码并逐条判定。走真实 UART 引脚，
// 因此 UART 收发的位时序也一并被覆盖。
// ============================================================================
`timescale 1ns/1ps
module tb_hil_lockstep;
    localparam integer BAUD = 10;   // 每位 200ns（与 tb_debug_uart 同口径）

    reg clk = 0, rx = 1;
    always #10 clk = ~clk;

    wire tx;
    wire [9:0] LEDR;
    // KEY[0] 在 de10_top 内即 rst_n（低有效）。上电先按住，再释放。
    reg  [3:0] KEY = 4'b0;      // KEY0=0 → 复位有效；KEY1..3 松开
    reg  [9:0] SW  = 10'h200;   // SW[9]=1 → HIL 锁步；DIP 位 SW[1:0]=0

    // 用真实 DE10 顶层，从而验证 de10_top → ctrl_core → debug_link 的完整连线
    de10_top dut (
        .CLOCK_50(clk), .KEY(KEY), .SW(SW), .LEDR(LEDR),
        .HEX0(), .HEX1(), .HEX2(), .HEX3(), .HEX4(), .HEX5(),
        .uart_rx(rx), .uart_tx(tx),
        .adc_convst(), .adc_sck(), .adc_sdi(), .adc_sdo(),
        .enc_a(1'b0), .enc_b(1'b0),
        .mot_ain1(), .mot_ain2(), .mot_pwma(),
        .dbg_tick(), .dbg_state()
    );
    defparam dut.u_core.u_tx.BAUD_DIV = BAUD;
    defparam dut.u_core.u_rx.BAUD_DIV = BAUD;
    defparam dut.u_core.u_debug.rx.TIMEOUT_CLOCKS = 50000;

    integer f, scan, n, j, value, k;
    reg [7:0] byte_rx;

    // UART 接收：按位采样 TX 并打印为 RX <hex>
    // 采样点在**位中点**：起始位下降沿后等 1.5 位，随后每 1 位采一次，
    // 采完 8 位正好落在停止位中点。写成 0.5 位会采到起始位，导致全帧错位。
    always @(negedge tx) begin
        #(BAUD*20*3/2);
        for (k = 0; k < 8; k = k + 1) begin byte_rx[k] = tx; #(BAUD*20); end
        if (tx !== 1'b1) $fatal(1, "UART stop bit invalid");
        $display("RX %02h", byte_rx);
    end

    task send_byte(input [7:0] data);
        integer b;
        begin
            rx = 0; #(BAUD*20);
            for (b = 0; b < 8; b = b + 1) begin rx = data[b]; #(BAUD*20); end
            rx = 1; #(BAUD*20);
        end
    endtask

    initial begin
        KEY = 4'b0;          // 复位有效
        #2000;
        KEY = 4'hF;          // 释放复位（KEY0 松开）
        #2000000;            // 等控制核与 UART 稳定
        f = $fopen("sim/out/hil_stimulus.txt", "r");
        if (f == 0) $fatal(1, "Missing HIL stimuli");
        while (!$feof(f)) begin
            scan = $fscanf(f, "%d", n);
            if (scan == 1) begin
                for (j = 0; j < n; j = j + 1) begin
                    scan = $fscanf(f, "%h", value);
                    send_byte(value[7:0]);
                end
            end
            // 帧间留白必须覆盖上一帧的**完整应答**再发下一帧：debug_link 在
            // reply 待发期间会丢弃新到达的帧（stop-and-wait）。STEP 应答 32 字节、
            // HIL 遥测 50 字节，@200ns/bit 分别为 64us / 100us，取 300us 留足余量。
            #300000;
        end
        $fclose(f);
        #1000000;
        $display("tb_hil_lockstep PASS");
        $finish;
    end

    initial begin #4000000000; $fatal(1, "tb_hil_lockstep TIMEOUT"); end
endmodule
