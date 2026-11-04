// ============================================================================
// loop_timer.v — 1ms 控制节拍发生器
// ============================================================================
`include "j280_defs.vh"
module loop_timer (
    input  wire clk,
    input  wire rst_n,
    output reg  tick
);
    reg [15:0] cnt;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)                              cnt  <= 16'd0;
        else if (cnt >= `TICK_MS_MAX-1)          cnt  <= 16'd0;
        else                                     cnt  <= cnt + 16'd1;
    end
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)                              tick <= 1'b0;
        else                                     tick <= (cnt >= `TICK_MS_MAX-1);
    end
endmodule
