// ============================================================================
// ui.v — 按键消抖/单拍脉冲 + 拨码同步 + LED 指示
// ============================================================================
// ui.v — 按键消抖/单拍脉冲 + 拨码同步 + LED 指示
// 按键共阴，按下为低电平。SW1=启动 SW2=模式切换 SW3=角度标定 SW4=急停/确认
`include "j280_defs.vh"
// ============================================================================
`include "j280_defs.vh"
module ui #(
    parameter DB_MS = 20
) (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       tick,          // 1ms
    input  wire [3:0] key_sw,        // 低有效
    input  wire [1:0] dip_sw,
    input  wire [2:0] mode,
    input  wire       fault_latch,
    input  wire       calib_done,
    output reg        key_start,     // 单拍脉冲（按 1ms 采样）
    output reg        key_mode,
    output reg        key_calib,
    output reg        key_stop,
    output wire [1:0] dip,
    output reg  [3:0] led
);
    // 消抖：连续 DB_MS 个 tick 稳定后才更新，并给出按下沿脉冲
    reg [4:0] cnt0, cnt1, cnt2, cnt3;
    reg s0, s1, s2, s3;       // 稳定态（1=按下，key_now 口径）
    reg p0, p1, p2, p3;       // 上一拍稳定态

    wire [3:0] key_now = ~key_sw;   // 转为按下=1

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt0<=0; cnt1<=0; cnt2<=0; cnt3<=0;
            s0<=0; s1<=0; s2<=0; s3<=0;      // 初值=未按下（key_now 空闲为 0）
            p0<=0; p1<=0; p2<=0; p3<=0;
            key_start<=0; key_mode<=0; key_calib<=0; key_stop<=0;
        end else if (tick) begin
            // SW1
            if (key_now[0] == s0) cnt0 <= 0;
            else if (cnt0 == DB_MS-1) begin s0 <= key_now[0]; cnt0 <= 0; end
            else cnt0 <= cnt0 + 5'd1;
            // SW2
            if (key_now[1] == s1) cnt1 <= 0;
            else if (cnt1 == DB_MS-1) begin s1 <= key_now[1]; cnt1 <= 0; end
            else cnt1 <= cnt1 + 5'd1;
            // SW3
            if (key_now[2] == s2) cnt2 <= 0;
            else if (cnt2 == DB_MS-1) begin s2 <= key_now[2]; cnt2 <= 0; end
            else cnt2 <= cnt2 + 5'd1;
            // SW4
            if (key_now[3] == s3) cnt3 <= 0;
            else if (cnt3 == DB_MS-1) begin s3 <= key_now[3]; cnt3 <= 0; end
            else cnt3 <= cnt3 + 5'd1;

            key_start <= s0 & ~p0; p0 <= s0;
            key_mode  <= s1 & ~p1; p1 <= s1;
            key_calib <= s2 & ~p2; p2 <= s2;
            key_stop  <= s3 & ~p3; p3 <= s3;
        end
    end

    assign dip = dip_sw;

    // LED：低 2 位模式色，第 3 位故障/标定完成闪烁，第 4 位心跳
    reg hb;
    reg [19:0] hb_cnt;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin hb <= 1'b0; hb_cnt <= 0; end
        else if (hb_cnt == 20'd0) begin hb <= ~hb; hb_cnt <= 20'd999999; end
        else hb_cnt <= hb_cnt - 20'd1;
    end
    always @* begin
        led[1:0] = (mode == `M_SWING)   ? 2'b01 :
                   (mode == `M_BALANCE) ? 2'b10 :
                   (mode == `M_POS)     ? 2'b11 : 2'b00;
        led[2]   = fault_latch | (calib_done & hb);
        led[3]   = hb;
    end
endmodule
