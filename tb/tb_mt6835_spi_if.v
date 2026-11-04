// tb_mt6835_spi_if.v — MT6835 SPI 接口行为模型测试台
// 协议模型（数据手册 Rev1.3 §7.6 + SimpleFOC 驱动交叉验证）：
//   SPI mode 3：SCK 空闲高；MOSI/MISO 都在下降沿更新、上升沿被采样；
//   CSN↓ 启动、CSN↑ 结束。OP_ANGLE 帧 48 拍：拍 1..16 = 命令应答（MISO 无关），
//   拍 17..48 = 32 位响应 [21 位角 | 3 位状态 | 8 位 CRC8]。
//   CRC8 = poly 0x07，init 0x00，无最终异或，MSB 先行，覆盖 {angle21, status[2:0]}。
// 覆盖：基本读数、扫角含 360° 回绕、单帧 CRC 错（保持+不置 fault）、连续坏帧
//       fault、状态位告警、断线 fault、自愈恢复，以及协议形状（每帧 48 升/48
//       降沿、TL≥80ns、TH≥40ns、CSN 高时 SCK 静止、MOSI 命令字正确）。
`timescale 1ns / 1ps
module tb_mt6835_spi_if;
    reg clk = 1'b0;
    reg rst_n = 1'b0;
    wire spi_csn, spi_sck, spi_mosi;
    wire spi_miso;
    wire [15:0] adc_raw;
    wire [20:0] alpha21;
    wire adc_valid, adc_fault;
    wire [15:0] st_frames, st_good, st_crc_err, st_status_err;
    wire [2:0]  st_status;
    wire [20:0] st_angle_raw;

    mt6835_spi_if #(
        .SPI_HALF_DIV(2),       // 半周期 40ns（周期 80ns ≥ 手册 TSCK 60ns）
        .TL_TICKS(3),           // 120ns（与 RTL 新默认一致，监视器阈值 80ns）
        .TH_TICKS(2),           // 80ns ≥ 0.5×TSCK
        .FRAME_GAP_TICKS(100),  // 帧周期 ~8µs（仿真加速）
        .POWERUP_CLKS(200),
        .WATCHDOG_CLKS(50000)   // 1ms
    ) dut (
        .clk(clk), .rst_n(rst_n),
        .spi_csn(spi_csn), .spi_sck(spi_sck),
        .spi_mosi(spi_mosi), .spi_miso(spi_miso),
        .adc_raw(adc_raw), .alpha21(alpha21),
        .adc_valid(adc_valid), .adc_fault(adc_fault),
        .st_frames(st_frames), .st_good(st_good),
        .st_crc_err(st_crc_err), .st_status_err(st_status_err),
        .st_status(st_status), .st_angle_raw(st_angle_raw)
    );

    always #10 clk = ~clk;      // 50MHz

    integer errors = 0;

    // ---------------- MT6835 从机行为模型 ----------------
    function [7:0] crc8(input [23:0] d);   // poly 0x07, init 0, MSB first
        reg [7:0] c;
        integer i;
        begin
            c = 8'd0;
            for (i = 23; i >= 0; i = i - 1) begin
                if (c[7] ^ d[i]) c = {c[6:0], 1'b0} ^ 8'h07;
                else             c = {c[6:0], 1'b0};
            end
            crc8 = c;
        end
    endfunction

    reg [20:0] angle  = 21'd1000;
    reg [2:0]  status = 3'b000;
    reg        corrupt = 1'b0;   // 置位后下一帧 CRC 字节反转
    reg        online  = 1'b1;   // 0 = 模拟断线（MISO 恒高，不推数据）
    reg [31:0] resp;             // [31:11]=angle [10:8]=status [7:0]=crc
    reg [5:0]  clk_idx;          // 本帧已发生的 SCK 下降沿数
    reg        miso_r = 1'b1;    // 空闲=1（等效 FPGA 侧上拉）

    assign spi_miso = miso_r;

    // CSN↓ 锁存本帧内容（帧间更新，模型帧内稳定）
    always @(negedge spi_csn) begin
        resp = {angle, status, crc8({angle, status})};
        if (corrupt) resp[7:0] = ~resp[7:0];
        clk_idx = 6'd0;
    end

    // 每个 SCK 下降沿推出一位：拍 1..16 = 命令应答（驱 0），拍 17..48 = 响应
    always @(negedge spi_sck) begin
        if (!spi_csn) begin
            clk_idx = clk_idx + 6'd1;
            if (!online)
                miso_r = 1'b1;
            else if (clk_idx >= 6'd17)
                miso_r = resp[48 - clk_idx];
            else
                miso_r = 1'b0;
        end
    end

    // ---------------- 协议形状监视 ----------------
    // armed：复位期间 X→1 跳变不是真实沿，首个 CSN↓ 之后才开始计数/检查
    reg armed = 1'b0;
    always @(negedge spi_csn) if (rst_n) armed = 1'b1;

    integer frame_count = 0;
    integer rise_cnt = 0, fall_cnt = 0;
    real    t_csn_fall, t_last_rise;
    reg     saw_angle_cmd = 1'b0;
    reg [47:0] mosi_sr;

    always @(negedge spi_sck) if (armed) mosi_sr = {mosi_sr[46:0], spi_mosi};

    always @(posedge spi_csn) begin
        if (armed) begin
            frame_count = frame_count + 1;
            if (rise_cnt != 48 || fall_cnt != 48) begin
                $display("FAIL: 帧%0d 沿数 rise=%0d fall=%0d（期望 48/48）",
                         frame_count - 1, rise_cnt, fall_cnt);
                errors = errors + 1;
            end
            if (($realtime - t_last_rise) < 40.0) begin
                $display("FAIL: 帧%0d TH=%0.1fns < 0.5xTSCK", frame_count - 1,
                         $realtime - t_last_rise);
                errors = errors + 1;
            end
            if (mosi_sr[47:24] !== 24'hA00300) begin
                $display("FAIL: 帧%0d MOSI 命令字 %h（期望 A00300）",
                         frame_count - 1, mosi_sr[47:24]);
                errors = errors + 1;
            end else
                saw_angle_cmd = 1'b1;
        end
        rise_cnt = 0;
        fall_cnt = 0;
    end

    always @(negedge spi_csn) t_csn_fall = $realtime;

    always @(negedge spi_sck) begin
        if (armed) begin
            if (spi_csn) begin
                $display("FAIL: CSN 高期间 SCK 活动");
                errors = errors + 1;
            end else if (fall_cnt == 0) begin
                if (($realtime - t_csn_fall) < 80.0) begin   // TL_TICKS=2 → 80ns
                    $display("FAIL: 帧%0d TL=%0.1fns", frame_count,
                             $realtime - t_csn_fall);
                    errors = errors + 1;
                end
            end
            fall_cnt = fall_cnt + 1;
        end
    end

    always @(posedge spi_sck) begin
        if (armed) begin
            if (spi_csn) begin
                $display("FAIL: CSN 高期间 SCK 活动");
                errors = errors + 1;
            end
            rise_cnt = rise_cnt + 1;
            t_last_rise = $realtime;
        end
    end

    integer valid_count = 0;
    always @(posedge clk) if (adc_valid) valid_count = valid_count + 1;

    // ---------------- 测试序列（帧号 = 已完成帧数 - 1） ----------------
    initial begin
        repeat (3) @(posedge clk);
        rst_n = 1'b1;

        // A. 基本读数：angle=1000 → raw=31(1000>>5)、alpha21=1000
        wait (frame_count == 2);
        if (adc_raw !== 16'd31 || alpha21 !== 21'd1000) begin
            $display("FAIL: 基本 raw=%0d alpha21=%0d 期望 31/1000",
                     adc_raw, alpha21); errors = errors + 1;
        end else $display("PASS: 基本帧 raw=%0d alpha21=%0d", adc_raw, alpha21);
        if (adc_fault) begin $display("FAIL: 正常帧后 fault=1"); errors = errors + 1; end
        if (valid_count != 2) begin
            $display("FAIL: valid_count=%0d 期望 2", valid_count); errors = errors + 1;
        end
        // 健康统计：2 好帧后 frames/good=2、无错、原始角=1000
        if (st_frames !== 16'd2 || st_good !== 16'd2 ||
            st_crc_err !== 16'd0 || st_status_err !== 16'd0 ||
            st_angle_raw !== 21'd1000 || st_status !== 3'd0) begin
            $display("FAIL: 健康 f=%0d g=%0d crc=%0d st=%0d raw=%0d code=%b（期望 2/2/0/0/1000/000）",
                     st_frames, st_good, st_crc_err, st_status_err, st_angle_raw, st_status);
            errors = errors + 1;
        end else $display("PASS: 健康统计基线 f=2 g=2 无错");

        // B. 扫角 + 360° 回绕
        angle = 21'd2097151; wait (frame_count == 3);
        if (adc_raw !== 16'd65535) begin
            $display("FAIL: 满量程 raw=%0d 期望 65535", adc_raw); errors = errors + 1;
        end else $display("PASS: 满量程 raw=%0d", adc_raw);
        angle = 21'd0; wait (frame_count == 4);
        if (adc_raw !== 16'd0 || alpha21 !== 21'd0) begin
            $display("FAIL: 零点 raw=%0d 期望 0", adc_raw); errors = errors + 1;
        end else $display("PASS: 满量程→零点回绕 raw=%0d", adc_raw);
        angle = 21'd1048576; wait (frame_count == 5);
        if (adc_raw !== 16'd32768) begin
            $display("FAIL: 半圈 raw=%0d 期望 32768", adc_raw); errors = errors + 1;
        end else $display("PASS: 半圈 raw=%0d", adc_raw);

        // C. 单帧 CRC 错：保持旧值，不置 fault
        angle = 21'd262144; corrupt = 1'b1;      // 期望好帧 raw=8192
        wait (frame_count == 6);
        if (adc_raw !== 16'd32768) begin
            $display("FAIL: CRC 坏帧更新了 raw=%0d（应保持 32768）", adc_raw); errors = errors + 1;
        end else $display("PASS: 单帧 CRC 错保持旧值");
        if (adc_fault) begin $display("FAIL: 单帧 CRC 错即置 fault"); errors = errors + 1; end
        // 健康统计：坏帧也计入 frames 并留下原始角，good 不动
        if (st_frames !== 16'd6 || st_good !== 16'd5 || st_crc_err !== 16'd1 ||
            st_status_err !== 16'd0 || st_angle_raw !== 21'd262144) begin
            $display("FAIL: 坏帧健康 f=%0d g=%0d crc=%0d st=%0d raw=%0d（期望 6/5/1/0/262144）",
                     st_frames, st_good, st_crc_err, st_status_err, st_angle_raw);
            errors = errors + 1;
        end else $display("PASS: 坏帧计数与原始角捕获（含坏帧）");

        // D. 恢复 + 连续两帧坏 → fault → 好帧自愈
        corrupt = 1'b0;
        wait (frame_count == 7);
        if (adc_raw !== 16'd8192) begin
            $display("FAIL: 恢复帧 raw=%0d 期望 8192", adc_raw); errors = errors + 1;
        end else $display("PASS: 恢复帧 raw=%0d", adc_raw);
        corrupt = 1'b1;
        wait (frame_count == 8);
        if (adc_fault) begin $display("FAIL: 坏帧1即 fault"); errors = errors + 1; end
        wait (frame_count == 9);
        if (!adc_fault) begin
            $display("FAIL: 连续2坏帧未置 fault"); errors = errors + 1;
        end else $display("PASS: 连续2坏帧置 fault");
        corrupt = 1'b0; angle = 21'd786432;
        wait (frame_count == 10);
        if (adc_fault) begin $display("FAIL: 好帧未清 fault"); errors = errors + 1; end
        if (adc_raw !== 16'd24576) begin
            $display("FAIL: 自愈帧 raw=%0d 期望 24576", adc_raw); errors = errors + 1;
        end else $display("PASS: 好帧自愈 raw=%0d", adc_raw);

        // E. 状态位告警：欠压 2 帧 → fault；恢复自愈
        status = 3'b100;        // UNDERVOLT
        wait (frame_count == 11);
        if (adc_fault) begin $display("FAIL: 状态告警1帧即 fault"); errors = errors + 1; end
        wait (frame_count == 12);
        if (!adc_fault) begin
            $display("FAIL: 欠压 2 帧未置 fault"); errors = errors + 1;
        end else $display("PASS: 状态位告警置 fault");
        // 健康统计：2 帧欠压告警 → status_err=2、状态码可见（bit2=欠压）、好帧=7
        if (st_status_err !== 16'd2 || st_status !== 3'b100 ||
            st_frames !== 16'd12 || st_good !== 16'd7) begin
            $display("FAIL: 状态健康 serr=%0d code=%b f=%0d g=%0d（期望 2/100/12/7）",
                     st_status_err, st_status, st_frames, st_good);
            errors = errors + 1;
        end else $display("PASS: 状态位告警计数 serr=2 code=100");
        status = 3'b000; angle = 21'd1000;
        wait (frame_count == 13);
        if (adc_fault) begin $display("FAIL: 状态恢复未清 fault"); errors = errors + 1; end
        if (adc_raw !== 16'd31) begin
            $display("FAIL: 状态恢复帧 raw=%0d 期望 31", adc_raw); errors = errors + 1;
        end else $display("PASS: 状态恢复帧 raw=%0d", adc_raw);

        // F. 断线：MISO 恒高 → 响应全 1 → CRC 错 → fault，保持末好值；恢复自愈
        online = 1'b0;
        wait (frame_count == 17);
        if (!adc_fault) begin
            $display("FAIL: 断线未置 fault"); errors = errors + 1;
        end else $display("PASS: 断线置 fault");
        if (adc_raw !== 16'd31) begin
            $display("FAIL: 断线期间 raw=%0d（应保持 31）", adc_raw); errors = errors + 1;
        end else $display("PASS: 断线期间保持末好值");
        // 断线签名：MISO 上拉恒 1 → 原始角=2^21-1、状态=7、CRC 累计 3+4=7
        if (st_angle_raw !== 21'd2097151 || st_status !== 3'b111 ||
            st_crc_err !== 16'd7 || st_frames !== 16'd17) begin
            $display("FAIL: 断线签名 raw=%0d code=%b crc=%0d f=%0d（期望 2097151/111/7/17）",
                     st_angle_raw, st_status, st_crc_err, st_frames);
            errors = errors + 1;
        end else $display("PASS: 断线签名 raw=2^21-1 code=111 crc=7");
        online = 1'b1;
        wait (frame_count == 19);
        if (adc_fault) begin
            $display("FAIL: 断线恢复未清 fault"); errors = errors + 1;
        end else $display("PASS: 断线恢复自愈");
        // 终态健康汇总：19 帧 / 10 好 / 7 CRC / 6 状态告警（2 欠压 + 4 断线帧
        // 状态位=111 同计 status_err）/ 原始角恢复 1000
        if (st_frames !== 16'd19 || st_good !== 16'd10 ||
            st_crc_err !== 16'd7 || st_status_err !== 16'd6 ||
            st_angle_raw !== 21'd1000 || st_status !== 3'd0) begin
            $display("FAIL: 终态健康 f=%0d g=%0d crc=%0d serr=%0d raw=%0d code=%b（期望 19/10/7/6/1000/000）",
                     st_frames, st_good, st_crc_err, st_status_err, st_angle_raw, st_status);
            errors = errors + 1;
        end else $display("PASS: 终态健康 f=19 g=10 crc=7 serr=6");

        if (!saw_angle_cmd) begin
            $display("FAIL: 未见 OP_ANGLE 命令字"); errors = errors + 1;
        end
        if (errors == 0) $display("TB_MT6835_SPI_IF PASS");
        else             $display("TB_MT6835_SPI_IF FAIL errors=%0d", errors);
        $finish;
    end
endmodule
