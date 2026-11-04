// ============================================================================
// tb_angle_proc_az.v — 竖直零位自校准单元测试（2026-10-20）
// 缩短时标：AZ_EMA_SH=3（τ=8 tick）、AZ_MIN_TICKS=100。
// 驱动模型：step(m) 把"当前 offset 下应读得的 α"设为 m（物理角=raw 常数）。
// 每个会话开始时捕获 offset 为基准（会话内 offset 恒定，提交只在退出沿）。
// 用例：
//   A 基本收敛：零偏 +275 → 提交 +275；提交后同物理位置测得 α 均值归零
//   B 限幅：零偏 +700 → 只提交 +500
//   C 时长门不足（50 tick < 100）：不提交
//   D 窗外（|α|=2000 > 15° 窗）：不计入、不提交
//   E 负向零偏 −180：提交 −180
// ============================================================================
`timescale 1ns / 1ps
module tb_angle_proc_az;
    reg clk = 1'b0, rst_n = 1'b0;
    reg tick = 1'b0;
    reg [15:0] alpha_avg = 16'd0;
    reg az_en = 1'b0;
    reg calib_start = 1'b0;
    reg signed [15:0] trim_c01 = 16'sd0;
    wire signed [15:0] alpha, alpha_dot, offset_c01;
    wire calib_done;
    integer errors = 0;
    integer cur_off = 8344;     // 本会话 offset 基准（物理角参考）

    angle_proc #(
        .SCALE_Q16(16'd36000),      // T20K/MT6835 全 360° 口径
        .AZ_EMA_SH(5'd3),
        .AZ_MIN_TICKS(16'd100),
        .AZ_DELTA_MAX(11'd500),
        .AZ_ALPHA_WIN(16'd1500)
    ) dut (
        .clk(clk), .rst_n(rst_n), .tick(tick),
        .alpha_avg(alpha_avg), .calib_start(calib_start),
        .az_en(az_en),
        .trim_c01(trim_c01),
        .alpha(alpha), .alpha_dot(alpha_dot),
        .offset_c01(offset_c01), .calib_done(calib_done)
    );

    always #5 clk = ~clk;
    reg [3:0] div = 4'd0;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin div <= 4'd0; tick <= 1'b0; end
        else begin
            div <= (div == 4'd9) ? 4'd0 : div + 4'd1;
            tick <= (div == 4'd9);
        end
    end

    // m = "当前 offset 基准下应读得的 α(c01)"；同一 m ⇒ 同一物理角，
    // 提交后 dut 读数自动移 −Δ（正是待验证行为）。
    task step(input integer m);
        integer raw_c;
        begin
            raw_c = cur_off + m - 18000;
            if (raw_c < 0) raw_c = raw_c + 36000;
            // raw_c01 = alpha_avg*36000>>16 ⇒ alpha_avg ≈ raw_c*65536/36000；
            // 比例同除 4 防 32 位溢出，四舍五入（±1 LSB ≈ ±0.55 c01）
            alpha_avg = (raw_c * 16384 + 4499) / 9000;
        end
    endtask

    // 会话：n 个 tick，方波 ±osc 围绕 bias，末尾拉低 az_en 触发提交沿
    task session(input integer bias, input integer osc, input integer n, input integer en);
        integer i;
        begin
            cur_off = offset_c01;   // 会话内 offset 恒定
            az_en = (en != 0);
            for (i = 0; i < n; i = i + 1) begin
                step(bias + ((i % 8 < 4) ? osc : -osc));
                @(posedge tick);
            end
            az_en = 1'b0;
            repeat (4) @(posedge clk);
        end
    endtask

    task chk(input integer cond, input [8*44:1] name);
        begin
            if (cond) $display("  PASS  %0s", name);
            else begin $display("  FAIL  %0s", name); errors = errors + 1; end
        end
    endtask

    integer i, sum_a, mean_a, off0;
    initial begin
        rst_n = 1'b0;
        repeat (20) @(posedge clk);
        rst_n = 1'b1;
        repeat (10) @(posedge tick);          // α 链填充
        step(0); repeat (2) @(posedge tick);

        $display("== A: bias +275, 400-tick session ==");
        off0 = offset_c01;                    // 8344
        session(275, 150, 400, 1);
        $display("  offset=%0d (expect %0d+275)", offset_c01, off0);
        chk(offset_c01 >= off0 + 265 && offset_c01 <= off0 + 285, "A commit +275");
        // 提交后同物理位置（cur_off 仍为 A 基准）：测得 α 应围绕 0
        sum_a = 0;
        for (i = 0; i < 200; i = i + 1) begin
            step(275 + ((i % 8 < 4) ? 150 : -150));
            @(posedge tick);
            sum_a = sum_a + alpha;
        end
        mean_a = sum_a / 200;
        $display("  post-commit alpha mean=%0d c01 (expect |.|<=15)", mean_a);
        chk(mean_a >= -15 && mean_a <= 15, "A false-equilibrium gone");

        $display("== B: bias +700, clamp to +500 ==");
        off0 = offset_c01;
        session(700, 150, 400, 1);
        $display("  offset=%0d (expect %0d+500)", offset_c01, off0);
        chk(offset_c01 >= off0 + 498 && offset_c01 <= off0 + 502, "B clamp +500");

        $display("== C: 50 tick < gate 100, no commit ==");
        off0 = offset_c01;
        session(300, 150, 50, 1);
        $display("  offset=%0d (expect unchanged %0d)", offset_c01, off0);
        chk(offset_c01 == off0, "C duration gate");

        $display("== D: |alpha|=2000 outside window, no commit ==");
        session(2000, 0, 300, 1);
        $display("  offset=%0d (expect unchanged %0d)", offset_c01, off0);
        chk(offset_c01 == off0, "D window gate");

        $display("== E: bias -180, negative commit ==");
        session(-180, 100, 400, 1);
        $display("  offset=%0d (expect %0d-180)", offset_c01, off0);
        chk(offset_c01 >= off0 - 190 && offset_c01 <= off0 - 170, "E negative commit");

        // ---- F: 运行时 trim（参数 id13）方向与增益 ----
        // 语义：alpha = (raw - offset) - trim  ⇒  trim=+100 使 alpha 减 100 cdeg
        // 场景：物理角固定（raw 不变），只改 trim
        begin : trim_case
            integer base_a, a_plus, a_minus;
            reg [15:0] keep_raw;
            keep_raw = alpha_avg;
            trim_c01 = 16'sd0;
            repeat (3) @(posedge tick);
            base_a = alpha;
            trim_c01 = 16'sd100;              // +1°
            repeat (3) @(posedge tick);
            a_plus = alpha;
            trim_c01 = -16'sd100;             // -1°
            repeat (3) @(posedge tick);
            a_minus = alpha;
            $display("  trim=0: %0d  trim=+100: %0d  trim=-100: %0d", base_a, a_plus, a_minus);
            chk((base_a - a_plus) >= 95 && (base_a - a_plus) <= 105,
                "F trim=+100 reduces alpha by 100");
            chk((a_minus - base_a) >= 95 && (a_minus - base_a) <= 105,
                "F trim=-100 increases alpha by 100");
            trim_c01 = 16'sd0;
            alpha_avg = keep_raw;
            repeat (3) @(posedge tick);
        end

        if (errors == 0) $display("tb_angle_proc_az PASS");
        else             $display("tb_angle_proc_az FAILED: %0d errors", errors);
        $finish;
    end
endmodule
