// ============================================================================
// adc_if.v — 3PA1030 接口 + 512 次滑动窗平均
// 时序沿用官方参考例程：adc_clk = 25MHz，OE#/STBY# 常低。
// 改进：官方 3ms 才平均 16 次（333Hz），此处 20.5µs 平均 512 次（≈48.8kHz 更新），
//       噪声降 √512≈23 倍，等效分辨率 ~15bit。
// 输出 alpha_avg：u16，满量程 2V ↔ 65472（10bit 左对齐到 u16）。
// ============================================================================
`include "j280_defs.vh"
module adc_if (
    input  wire        clk,          // 50MHz
    input  wire        rst_n,
    output wire        adc_clk,      // 25MHz 给 3PA1030
    output wire        adc_oe_n,
    output wire        adc_stby_n,
    input  wire [9:0]  adc_data_in,  // D9..D0 并行
    input  wire        adc_otr,      // 量程越界指示
    output reg  [15:0] alpha_avg,
    output reg         otr_event     // 越界事件（粘滞，供 safety）
);

    reg clk_div;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) clk_div <= 1'b0;
        else        clk_div <= ~clk_div;
    end
    assign adc_clk   = clk_div;
    assign adc_oe_n   = 1'b0;
    assign adc_stby_n = 1'b0;

    // adc_clk 高电平相采样一次 = 25MSPS
    reg [9:0] d_r;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)      d_r <= 10'd0;
        else if (clk_div) d_r <= adc_data_in;
    end

    // 512 点盒式平均：sum 19bit（512*1023 = 523776 < 2^19）
    reg  [18:0] sum;
    reg  [8:0]  n;
    wire [18:0] sum_nxt = sum + {9'd0, d_r};
    wire        last = (n == 9'd511);
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sum <= 19'd0; n <= 9'd0; alpha_avg <= 16'd0;
        end else if (clk_div) begin
            if (last) begin
                alpha_avg <= sum_nxt[18:3];   // (sum>>9)<<6 = sum>>3，含本次采样
                sum <= 19'd0;
                n   <= 9'd0;
            end else begin
                sum <= sum_nxt;
                n   <= n + 9'd1;
            end
        end
    end

    // OTR 粘滞锁存（同步两拍防亚稳）
    reg [1:0] otr_sync;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            otr_sync <= 2'b00; otr_event <= 1'b0;
        end else begin
            otr_sync <= {otr_sync[0], adc_otr};
            if (otr_sync[1]) otr_event <= 1'b1;
        end
    end

endmodule
