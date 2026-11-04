// ============================================================================
// uart_rx.v — 8N1 接收（起始位 1.5 位周期对齐中点采样）
// ============================================================================
`include "j280_defs.vh"
module uart_rx #(
    parameter BAUD_DIV = `UART_BAUD_DIV
) (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       rx,
    output reg        valid,     // 单拍脉冲
    output reg  [7:0] data
);
    localparam [16:0] HALF_START = BAUD_DIV/2; // 半位检查起始位，再隔一位采样data[0]
    localparam [16:0] BMAX = BAUD_DIV - 1;
    reg [2:0]  sync;
    wire       rx_s = sync[1];
    reg [16:0] cnt;
    reg [3:0]  bit_i;    // 0..7 数据
    reg [7:0]  sh;
    reg [1:0]  st;       // 0 IDLE, 1 START, 2 BITS, 3 STOP

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) sync <= 3'b111;
        else        sync <= {sync[1:0], rx};
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            valid <= 1'b0; data <= 8'd0; cnt <= 0; bit_i <= 0; sh <= 0; st <= 2'd0;
        end else begin
            valid <= 1'b0;
            case (st)
                2'd0: if (!rx_s) begin st <= 2'd1; cnt <= 17'd0; bit_i <= 4'd0; end
                2'd1: if (cnt == HALF_START-1) begin
                          st  <= 2'd2;
                          cnt <= 17'd0;
                          if (!rx_s) bit_i <= 4'd0;     // 有效起始位
                          else       st  <= 2'd0;       // 毛刺
                      end else cnt <= cnt + 17'd1;
                2'd2: if (cnt == BMAX) begin
                          cnt <= 17'd0;
                          sh  <= {rx_s, sh[7:1]};
                          if (bit_i == 4'd7) st <= 2'd3;
                          else               bit_i <= bit_i + 4'd1;
                      end else cnt <= cnt + 17'd1;
                2'd3: if (cnt == BMAX) begin
                          st    <= 2'd0;
                          valid <= rx_s;                 // 停止位为 1 才有效
                          data  <= sh;
                      end else cnt <= cnt + 17'd1;
            endcase
        end
    end
endmodule
