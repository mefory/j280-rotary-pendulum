// tb_motor_pwm.v — 零点刹车行为覆盖
// 验证：①|duty|<BRAKE_TH → in1=in2=1 且 pwm 恒 0（短路制动）
//       ②|duty|≥BRAKE_TH → 01/10 方向 + 正常斩波
//       ③run_en=0 → 强制 coast（00），任何 duty 都不许有斩波
//       ④阈值边界（恰好等于/差 1）与符号对称性
// 判据约定：scan_carrier 在**对齐载波起点**后的完整周期内统计，避免上一
// case 残留方向（11/01/10）污染窗口头部；coast 的 RTL 语义=00 且无斩波。
`timescale 1ns / 1ps
module tb_motor_pwm;
    localparam CARRIER  = 100;     // 缩小载波加速仿真
    localparam BRAKE_TH = 800;
    localparam CLK_HALF = 10;      // 50MHz

    reg clk = 0, rst_n = 0, run_en = 0;
    reg signed [15:0] duty = 0;
    wire in1, in2, pwm;
    integer errors = 0;

    motor_pwm #(.CARRIER(CARRIER), .BRAKE_TH(BRAKE_TH)) dut (
        .clk(clk), .rst_n(rst_n), .run_en(run_en), .brake_en(1'b1), .duty(duty),
        .in1(in1), .in2(in2), .pwm(pwm)
    );
    always #(CLK_HALF) clk = ~clk;

    integer saw_brake, saw_dir, saw_coast, saw_pwm, i;
    // 等到下一个载波起点（cnt==0 沿）之后再扫整周期，方向寄存器必已按当前
    // duty 更新，排除上一 case 的残留方向。
    task scan_carrier;
        begin
            saw_brake = 0; saw_dir = 0; saw_coast = 0; saw_pwm = 0;
            @(posedge clk);
            while (dut.cnt != 0) @(posedge clk);
            repeat (CARRIER) begin
                @(posedge clk);
                if ({in1, in2} == 2'b11) saw_brake = 1;
                if ({in1, in2} == 2'b01 || {in1, in2} == 2'b10) saw_dir = 1;
                if ({in1, in2} == 2'b00) saw_coast = 1;
                if (pwm) saw_pwm = 1;
            end
        end
    endtask

    task check(input cond, input [127:0] name);
        if (!cond) begin
            $display("FAIL: %0s (duty=%0d run_en=%b in=%b%b pwm=%b)", name, duty, run_en, in1, in2, pwm);
            errors = errors + 1;
        end else
            $display("PASS: %0s (duty=%0d)", name, duty);
    endtask

    initial begin
        repeat (4) @(posedge clk);
        rst_n = 1;

        // ① 小信号 → 刹车：11 + 无斩波
        run_en = 1; duty = 500;  scan_carrier;
        check(saw_brake == 1 && saw_pwm == 0 && saw_dir == 0, "duty=500 刹车(11)且pwm恒0");

        duty = 0; scan_carrier;
        check(saw_brake == 1 && saw_pwm == 0 && saw_dir == 0, "duty=0 刹车");

        duty = -500; scan_carrier;
        check(saw_brake == 1 && saw_pwm == 0 && saw_dir == 0, "duty=-500 刹车");

        // ② 边界：恰好 BRAKE_TH → 正常方向输出（阈值开区间 <）
        duty = BRAKE_TH; scan_carrier;
        check(saw_dir == 1 && saw_pwm == 1 && saw_brake == 0, "duty=+800 边界走PWM");

        duty = -BRAKE_TH; scan_carrier;
        check(saw_dir == 1 && saw_pwm == 1 && saw_brake == 0, "duty=-800 边界走PWM");

        duty = BRAKE_TH - 1; scan_carrier;
        check(saw_brake == 1 && saw_pwm == 0 && saw_dir == 0, "duty=799 刹车");

        // ③ 大信号正/负 → 01/10 + 斩波
        duty = 4000;  scan_carrier;
        check(saw_dir == 1 && saw_pwm == 1 && saw_brake == 0, "duty=+4000 正常PWM");
        duty = -4000; scan_carrier;
        check(saw_dir == 1 && saw_pwm == 1 && saw_brake == 0, "duty=-4000 正常PWM");

        // ④ run_en=0 → 任何 duty 强制 coast（00 且无斩波，不许刹车）
        duty = 500;  run_en = 0; scan_carrier;
        check(saw_coast == 1 && saw_brake == 0 && saw_pwm == 0, "run_en=0 duty=500 coast");
        duty = 4000; scan_carrier;
        check(saw_coast == 1 && saw_dir == 0 && saw_pwm == 0, "run_en=0 duty=4000 coast");

        if (errors == 0) $display("TB_MOTOR_PWM PASS");
        else             $display("TB_MOTOR_PWM FAIL errors=%0d", errors);
        $finish;
    end
endmodule
