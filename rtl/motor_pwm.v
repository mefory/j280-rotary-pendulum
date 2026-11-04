// ============================================================================
// motor_pwm.v — 20kHz 载波 PWM + TB6612 方向逻辑
// duty: s16, ±10000 = ±100%；载波 2500 级 → cmp = |duty|>>2（2500/10000=1/4 整数比）
// 重写原因：官方 motor_ctrl.v 载波 9.26kHz（有啸叫）且 duty*20 上限 5100<5400
// 到不了满占空比。IN1/IN2：01/10 正反转，00 浮动停机（coast），11 短路刹车。
//
// 零点刹车（2026-10-09，方案文档 §3.4）：运行中 |duty|<BRAKE_TH 时输出 11
// 短路制动——反电动势被短接即机械阻尼，治 TB6612 coast 零点无阻尼导致的
// 底座 7.5Hz 激励（T20K 频谱定案，K4 被迫降额 25% 的根因）。
// 安全语义：run_en=0 仍强制 coast（急停/掉摆必须自由滑行，不许抱死）；
// BRAKE_TH=0 可完整退回旧行为。阈值覆盖死区（T20K 实测 700/800）：死区内
// PWM 斩波本来就推不动转子，chatter 全是无效抖动，不如换阻尼。
// ============================================================================
`include "j280_defs.vh"
module motor_pwm #(
    parameter CARRIER  = `PWM_CARRIER,   // 1250@27MHz / 2500@50MHz
    parameter DUTY_SHIFT = (`PWM_CARRIER == 1_250) ? 3 : 2,
    parameter BRAKE_TH = 16'sd800        // |duty| 低于此值切短路制动（0=退回 coast 行为）
) (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               run_en,    // 0 = 强制 coast（安全）
    input  wire               brake_en,  // 1 = 零点刹车使能（SWING 期间应拉 0：
                                         // fling 甩摆靠底座到点后的跟随冲击把杆
                                         // 荡过去，刹车会杀死"鞭甩"——2026-10-09
                                         // 实测刹车上线后自动甩摆捕获 0 命中）
    input  wire signed [15:0] duty,      // ±10000
    output reg                in1,
    output reg                in2,
    output wire               pwm
);
    reg [11:0] cnt;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)             cnt <= 12'd0;
        else if (cnt >= CARRIER-1) cnt <= 12'd0;
        else                        cnt <= cnt + 12'd1;
    end

    wire signed [15:0] mag = (duty < 0) ? -duty : duty;
    localparam [11:0] CAR = CARRIER;
    wire [15:0] mag_scaled = mag >> DUTY_SHIFT;
    wire [11:0] cmp = (mag_scaled > {4'b0000, CAR}) ? CAR : mag_scaled[11:0]; // 封顶满载波

    wire zero_brake = run_en && brake_en && (mag < BRAKE_TH);   // 运行中小信号：短路制动
    assign pwm = run_en && !zero_brake && (cnt < cmp);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)              {in1, in2} <= 2'b00;
        else if (!run_en)        {in1, in2} <= 2'b00;   // coast（安全语义不变）
        else if (cnt == 12'd0) begin                    // 载波起始更新方向，防边沿毛刺
            if (zero_brake)      {in1, in2} <= 2'b11;   // 短路制动（机械阻尼）
            else if (duty >= 0)  {in1, in2} <= 2'b01;
            else                 {in1, in2} <= 2'b10;
        end
    end
endmodule
