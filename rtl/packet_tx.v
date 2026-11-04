// One immutable frame per request. Explicit wait-for-busy handshake prevents
// the legacy telemetry sender from advancing two bytes for one UART acceptance.
module packet_tx (
    input wire clk,rst_n,request,
    input wire [7:0] kind,
    input wire [15:0] seq,length,
    // v6 遥测 88B（2026-11-02）：载荷 640→704 位、长度门 80→88。
    // 门不放宽时遥测帧被静默丢弃（上位机只见"没有遥测"）。
    input wire [703:0] payload,
    output reg busy,
    output reg byte_start,
    output reg [7:0] byte_data,
    input wire uart_busy
);
    reg [7:0] kind_r,index;
    reg [15:0] seq_r,length_r,crc;
    reg [703:0] payload_r;
    reg [1:0] phase;
    reg [7:0] next_byte;
    function [15:0] crc_byte(input [15:0] previous,input [7:0] data);
        integer b;reg [15:0] v;
        begin
            v=previous^{data,8'd0};
            for(b=0;b<8;b=b+1)v=v[15] ? (v<<1)^16'h1021 : v<<1;
            crc_byte=v;
        end
    endfunction
    always @*begin
        case(index)
            0:next_byte=8'ha5;
            1:next_byte=8'h5a;
            2:next_byte=8'd2;
            3:next_byte=kind_r;
            4:next_byte=seq_r[7:0];
            5:next_byte=seq_r[15:8];
            6:next_byte=length_r[7:0];
            7:next_byte=length_r[15:8];
            default:begin
                if(index<length_r+8)next_byte=payload_r[7:0];
                else if(index==length_r+8)next_byte=crc[7:0];
                else next_byte=crc[15:8];
            end
        endcase
    end
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            busy<=0;byte_start<=0;byte_data<=0;kind_r<=0;seq_r<=0;
            length_r<=0;payload_r<=0;crc<=16'hffff;index<=0;phase<=0;
        end else begin
            byte_start<=0;
            if(!busy)begin
                if(request && length<=88)begin
                    busy<=1;kind_r<=kind;seq_r<=seq;length_r<=length;
                    payload_r<=payload;index<=0;crc<=16'hffff;phase<=0;
                end
            end else case(phase)
                0:if(!uart_busy)begin
                    byte_data<=next_byte;byte_start<=1;phase<=1;
                    if(index>=2 && index<length_r+8)crc<=crc_byte(crc,next_byte);
                end
                1:if(uart_busy)phase<=2;
                2:if(!uart_busy)begin
                    if(index==length_r+9)busy<=0;
                    else begin
                        index<=index+1'b1;phase<=0;
                        if(index>=8 && index<length_r+8)payload_r<=payload_r>>8;
                    end
                end
                default:phase<=0;
            endcase
        end
    end
endmodule
