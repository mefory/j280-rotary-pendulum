// tb_mt6701_ssi_if.v — MT6701 SSI 接口行为模型测试台
// 协议模型（数据手册 Rev1.5 §6.8 + SimpleFOC 驱动交叉验证）：
//   SPI mode 2 语义：CLK 空闲高；CSN↓ 后首个 CLK↓ 为唤醒拍（哑位），
//   其后每个 CLK↑ 器件推出下一位（帧 = 哑位 + D13..D0 + Mg3..Mg0 + CRC5..CRC0），
//   主机在 CLK↓ 采样。CRC6 = x^6+x+1，MSB 先行，初值 0，覆盖 18 位 {D,Mg}。
// 覆盖：基本读数、扫角含 360° 回绕、单帧 CRC 错（保持+不置 fault）、
//       连续坏帧 fault、Mg 磁场告警、Mg[2] 推按豁免、断线 fault、自愈恢复，
//       以及协议形状（每帧 25 升/25 降沿、TL≥100ns、TH≥0.5×TCLK、CSN 高时 CLK 静止）。
`timescale 1ns / 1ps
module tb_mt6701_ssi_if;
    reg clk = 1'b0;
    reg rst_n = 1'b0;
    wire ssi_csn, ssi_clk;
    wire ssi_do;
    wire [15:0] adc_raw;
    wire adc_valid, adc_fault;
    // 链路健康诊断输出（2026-10-19）：帧/好帧/CRC 错/Mg 告警/Mg 码/原始角
    wire [15:0] st_frames, st_good, st_crc_err, st_mg_err;
    wire [2:0]  st_mg_code;
    wire [13:0] st_angle_raw;
    wire [1:0]  st_fault_cause;
    wire [15:0] st_max_no_good;
    wire        st_frame_pulse;

    mt6701_ssi_if #(
        .SSI_HALF_DIV(2),       // 半周期 40ns（周期 80ns ≥ 数据手册 TCLK 64ns）
        .TL_TICKS(4),           // 80ns（仿真加速，仍 ≥ 数据手册 TL 100ns 时按默认参数验证）
        .TH_TICKS(4),           // 80ns ≥ 0.5×TCLK
        .FRAME_GAP_CLKS(400),   // 帧周期 ~10µs
        .POWERUP_CLKS(200),
        .WATCHDOG_CLKS(50000)   // 1ms
`ifdef LATCH_MG_OFF
        // T20K 实际形态：Mg 无锁存权（数据门仍全查）——同一 tb 双配置回归
        ,.LATCH_ON_STRENGTH(1'b0), .LATCH_ON_TRACK(1'b0)
`endif
    ) dut (
        .clk(clk), .rst_n(rst_n),
        .ssi_csn(ssi_csn), .ssi_clk(ssi_clk), .ssi_do(ssi_do),
        .adc_raw(adc_raw), .angle14_o(), .adc_valid(adc_valid), .adc_fault(adc_fault),
        .st_frames(st_frames), .st_good(st_good),
        .st_crc_err(st_crc_err), .st_mg_err(st_mg_err),
        .st_mg_code(st_mg_code), .st_angle_raw(st_angle_raw),
        .st_fault_cause(st_fault_cause), .st_max_no_good(st_max_no_good),
        .st_frame_pulse(st_frame_pulse)
    );

    always #10 clk = ~clk;      // 50MHz

    integer errors = 0;

    // ---------------- MT6701 从机行为模型 ----------------
    function [5:0] crc6(input [17:0] d);
        reg [5:0] c;
        integer i;
        begin
            c = 6'd0;
            for (i = 17; i >= 0; i = i - 1) begin
                if (c[5] ^ d[i]) c = {c[4:0], 1'b0} ^ 6'h03;
                else             c = {c[4:0], 1'b0};
            end
            crc6 = c;
        end
    endfunction

    reg [13:0] angle  = 14'd1000;
    reg [3:0]  mg     = 4'b0000;
    reg        corrupt = 1'b0;   // 置位后下一帧 CRC0 位反转
    reg        online  = 1'b1;   // 0 = 模拟断线（DO 恒高，不推数据）
    reg [23:0] frame_bits;
    reg [5:0]  bit_idx;          // 已推位数（第 k 个 CLK↑ 推 frame_bits[24-k]）
    reg        do_r = 1'b1;      // 空闲=1（等效 FPGA 侧上拉）

    assign ssi_do = do_r;

    // CSN↓ 锁存本帧内容（角值在帧间更新，模型帧内稳定）
    always @(negedge ssi_csn) begin
        frame_bits = {angle, mg, crc6({angle, mg})};
        if (corrupt) frame_bits[0] = ~frame_bits[0];
        bit_idx = 6'd0;
    end

    // 第 1 个 CLK↑ 推 D13，第 24 个 CLK↑ 推 CRC0，此后保持
    always @(posedge ssi_clk) begin
        if (online && bit_idx < 6'd24) begin
            do_r    = frame_bits[23 - bit_idx];
            bit_idx = bit_idx + 6'd1;
        end else if (!online) begin
            do_r = 1'b1;
        end
    end

    // ---------------- 协议形状监视 ----------------
    // armed：复位期间 X→1 跳变不是真实沿（iverilog 视 X→1 为 posedge），
    // 首个 CSN↓ 之后才开始计数/检查。
    reg armed = 1'b0;
    always @(negedge ssi_csn) if (rst_n) armed = 1'b1;

    integer frame_count = 0;
    integer rise_cnt = 0, fall_cnt = 0;
    real    t_csn_fall, t_last_rise;

    always @(posedge ssi_csn) begin
        if (armed) begin
            frame_count = frame_count + 1;
            if (rise_cnt != 25 || fall_cnt != 25) begin
                $display("FAIL: 帧%0d 沿数 rise=%0d fall=%0d（期望 25/25）",
                         frame_count - 1, rise_cnt, fall_cnt);
                errors = errors + 1;
            end
            if (($realtime - t_last_rise) < 40.0) begin
                $display("FAIL: 帧%0d TH=%0.1fns < 0.5xTCLK", frame_count - 1,
                         $realtime - t_last_rise);
                errors = errors + 1;
            end
        end
        rise_cnt = 0;
        fall_cnt = 0;
    end

    always @(negedge ssi_csn) t_csn_fall = $realtime;

    always @(negedge ssi_clk) begin
        if (armed) begin
            if (ssi_csn) begin
                $display("FAIL: CSN 高期间 CLK 活动");
                errors = errors + 1;
            end else if (fall_cnt == 0) begin
                if (($realtime - t_csn_fall) < 80.0) begin   // TL_TICKS=4 → 160ns
                    $display("FAIL: 帧%0d TL=%0.1fns", frame_count,
                             $realtime - t_csn_fall);
                    errors = errors + 1;
                end
            end
            fall_cnt = fall_cnt + 1;
        end
    end

    always @(posedge ssi_clk) begin
        if (armed) begin
            if (ssi_csn) begin
                $display("FAIL: CSN 高期间 CLK 活动");
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

        // A. 基本读数：angle=1000 → raw=4000
        wait (frame_count == 2);
        if (adc_raw !== 16'd4000) begin
            $display("FAIL: 基本 raw=%0d 期望 4000", adc_raw); errors = errors + 1;
        end else $display("PASS: 基本帧 raw=%0d", adc_raw);
        if (adc_fault) begin $display("FAIL: 正常帧后 fault=1"); errors = errors + 1; end
        if (valid_count != 2) begin
            $display("FAIL: valid_count=%0d 期望 2", valid_count); errors = errors + 1;
        end

        // B. 扫角 + 360° 回绕
        angle = 14'd16383; wait (frame_count == 3);
        if (adc_raw !== 16'd65532) begin
            $display("FAIL: 满量程 raw=%0d 期望 65532", adc_raw); errors = errors + 1;
        end else $display("PASS: 满量程 raw=%0d", adc_raw);
        angle = 14'd0; wait (frame_count == 4);
        if (adc_raw !== 16'd0) begin
            $display("FAIL: 零点 raw=%0d 期望 0", adc_raw); errors = errors + 1;
        end else $display("PASS: 零点→满量程回绕 raw=%0d", adc_raw);
        angle = 14'd4095; wait (frame_count == 5);
        if (adc_raw !== 16'd16380) begin
            $display("FAIL: 中值 raw=%0d 期望 16380", adc_raw); errors = errors + 1;
        end else $display("PASS: 中值 raw=%0d", adc_raw);

        // C. 单帧 CRC 错：保持旧值，不置 fault
        angle = 14'd5000; corrupt = 1'b1;
        wait (frame_count == 6);
        if (adc_raw !== 16'd16380) begin
            $display("FAIL: CRC 坏帧更新了 raw=%0d（应保持 16380）", adc_raw); errors = errors + 1;
        end else $display("PASS: 单帧 CRC 错保持旧值");
        if (adc_fault) begin $display("FAIL: 单帧 CRC 错即置 fault"); errors = errors + 1; end

        // D. 恢复 + 连续两帧坏 → fault → 好帧自愈
        corrupt = 1'b0;
        wait (frame_count == 7);
        if (adc_raw !== 16'd20000) begin
            $display("FAIL: 恢复帧 raw=%0d 期望 20000", adc_raw); errors = errors + 1;
        end else $display("PASS: 恢复帧 raw=%0d", adc_raw);
        corrupt = 1'b1;
        wait (frame_count == 8);
        if (adc_fault) begin $display("FAIL: 坏帧1即 fault"); errors = errors + 1; end
        wait (frame_count == 9);
        if (!adc_fault) begin
            $display("FAIL: 连续2坏帧未置 fault"); errors = errors + 1;
        end else $display("PASS: 连续2坏帧置 fault");
        if (st_fault_cause !== 2'd1) begin
            $display("FAIL: CRC 锁存 cause=%0d 期望 1", st_fault_cause); errors = errors + 1;
        end else $display("PASS: 锁存原因=CRC");
        corrupt = 1'b0; angle = 14'd6000;
        wait (frame_count == 10);
        if (adc_fault) begin $display("FAIL: 好帧未清 fault"); errors = errors + 1; end
        if (adc_raw !== 16'd24000) begin
            $display("FAIL: 自愈帧 raw=%0d 期望 24000", adc_raw); errors = errors + 1;
        end else $display("PASS: 好帧自愈 raw=%0d", adc_raw);

        // E. Mg 磁场告警：场弱 2 帧 → 默认锁存+cause=2；LATCH_OFF → 不锁存、保持旧角
        mg = 4'b0010;         // Mg[1:0]=10 场太弱
        wait (frame_count == 11);
        if (adc_fault) begin $display("FAIL: Mg 告警1帧即 fault"); errors = errors + 1; end
        wait (frame_count == 12);
`ifdef LATCH_MG_OFF
        if (adc_fault) begin $display("FAIL: LATCH_OFF 场弱串不应 fault"); errors = errors + 1;
        end else $display("PASS: LATCH_OFF 场弱串不锁存");
        if (adc_raw !== 16'd24000) begin
            $display("FAIL: LATCH_OFF 场弱应保持 raw=%0d 期望 24000", adc_raw); errors = errors + 1;
        end else $display("PASS: LATCH_OFF 场弱帧保持旧角（数据门仍全查）");
`else
        if (!adc_fault) begin
            $display("FAIL: Mg 场弱 2 帧未置 fault"); errors = errors + 1;
        end else $display("PASS: Mg 场弱 2 帧置 fault");
        if (st_fault_cause !== 2'd2) begin
            $display("FAIL: 场强锁存 cause=%0d 期望 2", st_fault_cause); errors = errors + 1;
        end else $display("PASS: 锁存原因=场强");
`endif
        mg = 4'b0000; angle = 14'd7000;
        wait (frame_count == 13);
`ifndef LATCH_MG_OFF
        if (adc_fault) begin $display("FAIL: Mg 恢复未清 fault"); errors = errors + 1; end
`endif
        if (adc_raw !== 16'd28000) begin
            $display("FAIL: Mg 恢复帧 raw=%0d 期望 28000", adc_raw); errors = errors + 1;
        end else $display("PASS: Mg 恢复帧 raw=%0d", adc_raw);
        mg = 4'b0100;         // Mg[2]=1 推按事件，非健康告警
        wait (frame_count == 15);
        if (adc_fault) begin
            $display("FAIL: Mg[2] 推按误报 fault"); errors = errors + 1;
        end else $display("PASS: Mg[2] 推按豁免");
        if (valid_count != 10) begin
            $display("FAIL: valid_count=%0d 期望 10", valid_count); errors = errors + 1;
        end

        // E2. Mg[3] 失跟踪单独注入：默认锁存+cause=3；LATCH_OFF → 不锁存、保持旧角
        mg = 4'b1000;
        wait (frame_count == 16);
        if (adc_fault) begin $display("FAIL: 失跟踪1帧即 fault"); errors = errors + 1; end
        wait (frame_count == 17);
`ifdef LATCH_MG_OFF
        if (adc_fault) begin $display("FAIL: LATCH_OFF 失跟踪串不应 fault"); errors = errors + 1;
        end else $display("PASS: LATCH_OFF 失跟踪串不锁存");
        if (adc_raw !== 16'd28000) begin
            $display("FAIL: LATCH_OFF 失跟踪应保持 raw=%0d 期望 28000", adc_raw); errors = errors + 1;
        end else $display("PASS: LATCH_OFF 失跟踪帧保持旧角");
`else
        if (!adc_fault) begin
            $display("FAIL: 失跟踪 2 帧未置 fault"); errors = errors + 1;
        end else $display("PASS: 失跟踪 2 帧置 fault");
        if (st_fault_cause !== 2'd3) begin
            $display("FAIL: 失跟踪锁存 cause=%0d 期望 3", st_fault_cause); errors = errors + 1;
        end else $display("PASS: 锁存原因=失跟踪");
`endif
        mg = 4'b0000;
        wait (frame_count == 18);
`ifndef LATCH_MG_OFF
        if (adc_fault) begin $display("FAIL: 失跟踪恢复未清 fault"); errors = errors + 1; end
`endif

        // F. 断线：DO 恒高 → 全 1 帧 → CRC 错（优先于 Mg 位定罪）→ fault；恢复自愈
        online = 1'b0;
        wait (frame_count == 22);
        if (!adc_fault) begin
            $display("FAIL: 断线未置 fault"); errors = errors + 1;
        end else $display("PASS: 断线置 fault");
        if (st_fault_cause !== 2'd1) begin
            $display("FAIL: 断线锁存 cause=%0d 期望 1（CRC 优先）", st_fault_cause);
            errors = errors + 1;
        end else $display("PASS: 断线锁存原因=CRC（优先级正确）");
        if (adc_raw !== 16'd28000) begin
            $display("FAIL: 断线期间 raw=%0d（应保持 28000）", adc_raw); errors = errors + 1;
        end else $display("PASS: 断线期间保持末好值");
        online = 1'b1; mg = 4'b0000;
        wait (frame_count == 24);
        if (adc_fault) begin
            $display("FAIL: 断线恢复未清 fault"); errors = errors + 1;
        end else $display("PASS: 断线恢复自愈");
        if (adc_raw !== 16'd28000) begin
            $display("FAIL: 恢复帧 raw=%0d 期望 28000", adc_raw); errors = errors + 1;
        end

        // G. 链路健康计数（2026-10-19 新增输出）
        // 走到这里：好帧应 = valid_count；坏帧 = CRC 错 3 帧（C 1 帧 + D 2 帧）
        // + Mg 告警 2 帧（E）+ 失跟踪 2 帧（E2），断线 3 帧（F，全 1 → CRC 错）。
        // 断线帧同时记 crc_err 与 mg_err（全 1 的 Mg 位也是病），故分开核对下界。
        if (st_frames !== frame_count[15:0]) begin
            $display("FAIL: st_frames=%0d 期望 %0d", st_frames, frame_count);
            errors = errors + 1;
        end else $display("PASS: st_frames=%0d 与帧数一致", st_frames);
        if (st_good !== valid_count[15:0]) begin
            $display("FAIL: st_good=%0d 期望 %0d（=valid_count）", st_good, valid_count);
            errors = errors + 1;
        end else $display("PASS: st_good=%0d 与好帧数一致", st_good);
        if (st_crc_err < 16'd3) begin
            $display("FAIL: st_crc_err=%0d 期望 ≥3（CRC 错+断线）", st_crc_err);
            errors = errors + 1;
        end else $display("PASS: st_crc_err=%0d ≥3", st_crc_err);
        if (st_mg_err < 16'd2) begin
            $display("FAIL: st_mg_err=%0d 期望 ≥2（场弱告警）", st_mg_err);
            errors = errors + 1;
        end else $display("PASS: st_mg_err=%0d ≥2", st_mg_err);
        // 最长无好帧段：全程最长 = 断线段 4 连（F 段帧 19-22：online=1 在 wait(22)
        // 检查后才恢复，帧 22 也是坏的；D/E/E2 各只有 2 连）
        if (st_max_no_good !== 16'd4) begin
            $display("FAIL: st_max_no_good=%0d 期望 4（断线 4 连）", st_max_no_good);
            errors = errors + 1;
        end else $display("PASS: st_max_no_good=4（断线段最长）");
        // 当前 mg=0000、angle=7000 → mg_code=000、angle_raw=7000
        if (st_mg_code !== 3'b000) begin
            $display("FAIL: st_mg_code=%b 期望 000", st_mg_code); errors = errors + 1;
        end else $display("PASS: st_mg_code=000（场强正常）");
        if (st_angle_raw !== 14'd7000) begin
            $display("FAIL: st_angle_raw=%0d 期望 7000", st_angle_raw); errors = errors + 1;
        end else $display("PASS: st_angle_raw=%0d（原始角直读）", st_angle_raw);

        // Mg 告警时 st_mg_code 应实时反映 {lost_track, mg1, mg0}
        // （E2/F 段后时间线在 24，这里接着 25/26 走）
        mg = 4'b0010; wait (frame_count == 25);
        if (st_mg_code !== 3'b010) begin
            $display("FAIL: 场弱时 st_mg_code=%b 期望 010", st_mg_code);
            errors = errors + 1;
        end else $display("PASS: 场弱时 st_mg_code=010（实时暴露磁场告警）");
        mg = 4'b1000; wait (frame_count == 26);
        if (st_mg_code !== 3'b100) begin
            $display("FAIL: 失跟踪时 st_mg_code=%b 期望 100", st_mg_code);
            errors = errors + 1;
        end else $display("PASS: 失跟踪时 st_mg_code=100（Mg[3] 单独暴露）");
        mg = 4'b0000;

        if (errors == 0) $display("TB_MT6701_SSI_IF PASS");
        else             $display("TB_MT6701_SSI_IF FAIL errors=%0d", errors);
        $finish;
    end
endmodule
