// V2: A5 5A VERSION TYPE SEQ16 LENGTH16 PAYLOAD CRC16.
// CRC-16/CCITT-FALSE: init FFFF, polynomial 1021, no reflection/xorout;
// covers VERSION through PAYLOAD, CRC transmitted little endian.
module packet_rx #(
    parameter TIMEOUT_CLOCKS=250000
) (
    input wire clk,rst_n,byte_valid,
    input wire [7:0] byte_data,
    output reg valid,
    output reg [7:0] kind,
    output reg [15:0] seq,length,
    output reg [511:0] payload,
    output reg [31:0] errors
);
    reg [3:0] state;
    reg [6:0] index;
    reg [15:0] crc,received_crc;
    reg [31:0] timeout_count;
    function [15:0] crc_byte(input [15:0] previous,input [7:0] data);
        integer b;reg [15:0] v;
        begin
            v=previous^{data,8'd0};
            for(b=0;b<8;b=b+1)v=v[15] ? (v<<1)^16'h1021 : v<<1;
            crc_byte=v;
        end
    endfunction
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            state<=0;index<=0;crc<=16'hffff;received_crc<=0;
            timeout_count<=0;errors<=0;valid<=0;kind<=0;seq<=0;length<=0;payload<=0;
        end else begin
            valid<=0;
            if(state!=0 && timeout_count>=TIMEOUT_CLOCKS)begin
                state<=0;timeout_count<=0;errors<=errors+1;
            end else if(byte_valid)begin
                timeout_count<=0;
                if(state>=2 && state<=8)crc<=crc_byte(crc,byte_data);
                case(state)
                    0:if(byte_data==8'ha5)state<=1;
                    1:if(byte_data==8'h5a)begin state<=2;crc<=16'hffff;payload<=0;end
                      else if(byte_data!=8'ha5)state<=0;
                    2:if(byte_data==2)state<=3;else begin state<=0;errors<=errors+1;end
                    3:begin kind<=byte_data;state<=4;end
                    4:begin seq[7:0]<=byte_data;state<=5;end
                    5:begin seq[15:8]<=byte_data;state<=6;end
                    6:begin length[7:0]<=byte_data;state<=7;end
                    7:begin
                        length[15:8]<=byte_data;index<=0;
                        if(byte_data!=0 || length[7:0]>64)begin state<=0;errors<=errors+1;end
                        else state<=length[7:0]==0 ? 9 : 8;
                    end
                    8:begin
                        payload[index*8+:8]<=byte_data;index<=index+1'b1;
                        if(index+1==length)state<=9;
                    end
                    9:begin received_crc[7:0]<=byte_data;state<=10;end
                    10:begin
                        state<=0;
                        if({byte_data,received_crc[7:0]}==crc)valid<=1;
                        else errors<=errors+1;
                    end
                    default:state<=0;
                endcase
            end else if(state!=0)timeout_count<=timeout_count+1;
        end
    end
endmodule
