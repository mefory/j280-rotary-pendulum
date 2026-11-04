// ============================================================================
// uart_tx.v — 8N1 发送，默认 1Mbaud
// ============================================================================
`include "j280_defs.vh"
module uart_tx #(
    parameter BAUD_DIV = `UART_BAUD_DIV
) (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       start,
    input  wire [7:0] data,
    output reg        busy,
    output reg        tx
);
    localparam W = $clog2(BAUD_DIV);
    localparam [W-1:0] BMAX = BAUD_DIV - 1;
    reg [W-1:0] baud_cnt;
    reg [3:0]   bit_i;      // 0 起始 + 8 数据 + 1 停止
    reg [8:0]   sh;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            busy <= 1'b1; tx <= 1'b1; baud_cnt <= 0; bit_i <= 0; sh <= 9'h1FF;
        end else begin
            if (!busy) begin
                tx <= 1'b1;
                if (start) begin
                    sh   <= {data, 1'b0};   // 数据 + 起始位在 LSB
                    busy <= 1'b1;
                    bit_i <= 4'd0;
                    baud_cnt <= 0;
                end
            end else begin
                if (baud_cnt == BMAX) begin
                    baud_cnt <= 0;
                    tx  <= sh[0];
                    sh  <= {1'b1, sh[8:1]};
                    bit_i <= bit_i + 4'd1;
                    if (bit_i == 4'd9) busy <= 1'b0;   // 停止位发完
                end else
                    baud_cnt <= baud_cnt + {{(W-1){1'b0}}, 1'b1};
            end
        end
    end
endmodule
