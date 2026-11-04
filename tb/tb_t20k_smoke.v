// tb_t20k_smoke.v — Tang Primer 20K 外壳冒烟测试（V5.1 端口）
// 验证：PLL 复位链释放、上电 SAFE、UART 空闲电平、两路 SPI（MT6835 α +
// 电机侧 MT6701 SSI）持续轮询、θ 源=MT6701 独占（TH_SRC_FORCE_MT=1，A14 失效）。
// UART 协议层由 ctrl_core 的既有回归（tb_debug_uart/tb_uart_mission）覆盖。
`timescale 1ns / 1ns
module tb_t20k_smoke;
    reg clk27 = 0;
    reg [4:0] key_n = 5'b11111;
    reg spi_miso = 1'b1;    // MT6835 MISO 空闲（帧间上拉读 1）
    reg mt6701_do = 1'b0;   // 全零帧：CRC 恒过（零数据零校验），角度恒 0 不 fault
    reg th_src_n = 1'b1;    // 悬空（内部上拉仿真）= 霍尔默认源
    reg enc_a = 1'b0, enc_b = 1'b0;
    wire [5:0] led;
    wire uart_tx;
    wire spi_csn, spi_sck, spi_mosi;
    wire mt6701_csn, mt6701_sck;
    wire mot_ain1, mot_ain2, mot_pwma, mot_stby;

    t20k_top dut (
        .clk27(clk27), .key_n(key_n), .led(led), .uart_tx(uart_tx), .uart_rx(1'b1),
        .spi_csn(spi_csn), .spi_sck(spi_sck), .spi_mosi(spi_mosi), .spi_miso(spi_miso),
        .mt6701_csn(mt6701_csn), .mt6701_sck(mt6701_sck), .mt6701_do(mt6701_do),
        .th_src_n(th_src_n),
        .mot_ain1(mot_ain1), .mot_ain2(mot_ain2), .mot_pwma(mot_pwma),
        .mot_stby(mot_stby)
    );

    // 加速仿真：板级 27MHz 周期缩短（时钟比由模型保证，绝对时间无意义）
    always #18.5 clk27 = ~clk27;

    // 两路传感总线的活动计数（轮询在跑 = FSM 活着）
    integer sck6835_edges = 0, sck6701_edges = 0;
    always @(posedge spi_sck)    sck6835_edges = sck6835_edges + 1;
    always @(posedge mt6701_sck) sck6701_edges = sck6701_edges + 1;

    integer errors = 0;
    initial begin
        // 上电：T10（key_n[4]）按住复位
        key_n[4] = 1'b0;
        repeat (20) @(posedge clk27);
        key_n[4] = 1'b1;

        // 等 PLL lock + 两级同步 → 复位释放，越过两路驱动上电等待（各 2ms 等效）
        wait (dut.rst_n === 1'b1);
        repeat (200000) @(posedge dut.clk50);

        if (dut.u_core.mode === 3'd4)
            $display("PASS: 上电 SAFE");
        else begin $display("FAIL: 上电 mode=%0d (期望 4)", dut.u_core.mode); errors = errors + 1; end

        if (uart_tx === 1'b1)
            $display("PASS: UART 空闲电平为高");
        else begin $display("FAIL: uart_tx=%b", uart_tx); errors = errors + 1; end

        if (sck6835_edges > 100 && sck6701_edges > 100)
            $display("PASS: MT6835 与电机侧 MT6701 均在轮询（%0d/%0d 边沿）",
                     sck6835_edges, sck6701_edges);
        else begin
            $display("FAIL: SPI 轮询停滞（MT6835=%0d, MT6701=%0d 边沿）",
                     sck6835_edges, sck6701_edges);
            errors = errors + 1;
        end

        // θ 源（V5.1 定案 MT6701 独占）：TH_SRC_FORCE_MT=1 恒选 MT 源，
        // A14 电平从复位起即被忽略（失效保留脚）。
        if (dut.th_src_mt === 1'b1)
            $display("PASS: 复位即 θ 源=MT6701（V5.1 独占）");
        else begin $display("FAIL: 默认 th_src_mt=%b", dut.th_src_mt); errors = errors + 1; end
        th_src_n = 1'b0;
        repeat (4) @(posedge dut.clk50);
        if (dut.th_src_mt === 1'b1)
            $display("PASS: A14 拉地 → 仍为 MT 源（脚已失效保留）");
        else begin $display("FAIL: 拉地后 th_src_mt=%b", dut.th_src_mt); errors = errors + 1; end
        th_src_n = 1'b1;
        repeat (4) @(posedge dut.clk50);
        if (dut.th_src_mt === 1'b1)
            $display("PASS: A14 松开 → 仍为 MT 源");
        else begin $display("FAIL: 松开后 th_src_mt=%b", dut.th_src_mt); errors = errors + 1; end

        // 观测器接管（2026-10-20 OBS_USE=1）：SAFE 下 obs_en 必须为 0（只挂
        // BALANCE/POS），从动模式输出逐拍等于量测 => ctrl_core 的 α/θ 仍来自
        // 差分链，上电行为与 OBS_USE=0 完全一致（不改变 SAFE/起摆语义）。
        if (dut.u_core.u_obs.obs_active === 1'b0)
            $display("PASS: OBS_USE=1 但 SAFE 段观测器从动（obs_active=0）");
        else begin
            $display("FAIL: SAFE 段 obs_active=%b（应仅 BALANCE/POS 接管）",
                     dut.u_core.u_obs.obs_active);
            errors = errors + 1;
        end
        // 从动直通：ctrl_core 对外 α 应等于 angle_proc 的量测值
        if (dut.u_core.alpha === dut.u_core.alpha_i)
            $display("PASS: 从动直通 α=量测（%0d）", dut.u_core.alpha);
        else begin
            $display("FAIL: 从动段 α=%0d 但量测=%0d", dut.u_core.alpha, dut.u_core.alpha_i);
            errors = errors + 1;
        end

        if (errors == 0) $display("tb_t20k_smoke PASS (TB_T20K_SMOKE ALL PASS)");
        else $display("tb_t20k_smoke FAIL (%0d ERRORS)", errors);
        $finish;
    end
endmodule
