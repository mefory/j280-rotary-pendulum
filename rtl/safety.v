// ============================================================================
// safety.v — 独立安全通路：上电静默 / 环路看门狗 / ADC 越界 / 模式门控
// 与控制算法完全分离：任何故障直接 coast（IN1=IN2=00），恢复需人工确认。
// ============================================================================
`include "j280_defs.vh"
module safety #(
    parameter WDG_TICKS = 2*`TICK_MS_MAX   // 2 个控制周期无 tick = 环路死亡
) (
    input  wire clk,
    input  wire rst_n,
    input  wire tick,          // 控制节拍喂狗
    input  wire wdg_enable,    // 0 = 停用节拍看门狗（HIL 锁步，上位机往返 > 2ms）
    input  wire otr_fault,     // adc_if 越界粘滞
    input  wire mode_safe,     // FSM 处于 IDLE/SAFE
    input  wire fault_clr,     // 按键确认清除
    output wire run_en,        // 1 = 允许输出
    output reg  fault_latch,
    output reg  [3:0] led_fault
);
    reg [31:0] wdg_cnt;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)            wdg_cnt <= 32'd0;
        else if (tick)         wdg_cnt <= 32'd0;
        else if (wdg_cnt < WDG_TICKS) wdg_cnt <= wdg_cnt + 32'd1;
    end
    wire wdg_fault = wdg_enable & (wdg_cnt >= WDG_TICKS);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)                      fault_latch <= 1'b1;   // 上电即安全
        else if (wdg_fault || otr_fault) fault_latch <= 1'b1;
        else if (fault_clr && !wdg_fault && !otr_fault) fault_latch <= 1'b0;
    end

    assign run_en = ~fault_latch & ~mode_safe;

    always @(posedge clk) led_fault <= {otr_fault, wdg_fault, fault_latch, 1'b0};
endmodule
