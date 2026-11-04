// ============================================================================
// tb_qenc4x.v — 四倍频解码计数验证：正转/反转/混叠
// ============================================================================
`timescale 1ns / 1ps
module tb_qenc4x;
    reg clk = 0, rst_n = 0;
    reg a = 0, b = 0;
    wire signed [31:0] pos;
    integer errors = 0;

    qenc4x #(.POL_FLIP(0)) dut (.clk(clk), .rst_n(rst_n), .enc_a(a), .enc_b(b), .pos(pos));

    always #10 clk = ~clk;

    task step(input integer seq);   // seq: 0=CW(+) 1=CCW(-)
        begin
            case (seq)
                0: begin a=0; b=0; end
                1: begin a=0; b=1; end
                2: begin a=1; b=1; end
                3: begin a=1; b=0; end
            endcase
            #100;
        end
    endtask

    integer i;
    initial begin
        rst_n = 0; #100; rst_n = 1; #200;
        // 正转（A 超前 B）3 个格雷码周期 = 12 计数：00→10→11→01
        for (i = 0; i < 3; i = i + 1) begin step(3); step(2); step(1); step(0); end
        #200;
        if (pos !== 32'sd12) begin $display("FAIL CW: pos=%0d expect 12", pos); errors = errors + 1; end
        // 反转 2 周期 = -8
        for (i = 0; i < 2; i = i + 1) begin step(1); step(2); step(3); step(0); end
        #200;
        if (pos !== 32'sd4) begin $display("FAIL CCW: pos=%0d expect 4", pos); errors = errors + 1; end
        if (errors == 0) $display("tb_qenc4x PASS");
        else             $display("tb_qenc4x FAIL errors=%0d", errors);
        $finish;
    end
endmodule
