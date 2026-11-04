// ============================================================================
// tb_adc_if.v — 512 点平均验证：常量收敛 + 输出左对齐 + OTR 锁存
// ============================================================================
`timescale 1ns / 1ps
module tb_adc_if;
    reg clk = 0, rst_n = 0;
    reg [9:0] din = 10'd248;
    reg otr = 0;
    wire adc_clk, oe_n, stby_n, otr_ev;
    wire [15:0] avg;

    adc_if dut (
        .clk(clk), .rst_n(rst_n),
        .adc_clk(adc_clk), .adc_oe_n(oe_n), .adc_stby_n(stby_n),
        .adc_data_in(din), .adc_otr(otr),
        .alpha_avg(avg), .otr_event(otr_ev)
    );
    always #10 clk = ~clk;
    integer errors = 0;

    initial begin
        rst_n = 0; #100; rst_n = 1;
        assert_defaults;
        // 常量 248：512 次平均后 = 248<<6 = 15872（等满两个窗，首窗少 1 样本）
        #45000;
        if (avg !== 16'd15872) begin $display("FAIL const: avg=%0d expect 15872", avg); errors = errors+1; end
        // 阶跃 781 → 781<<6 = 49984（等窗口完全冲刷）
        din = 10'd781; #45000;
        if (avg !== 16'd49984) begin $display("FAIL step: avg=%0d expect 49984", avg); errors = errors+1; end
        // OTR 锁存
        otr = 1; #100; otr = 0; #100;
        if (!otr_ev) begin $display("FAIL otr latch"); errors = errors+1; end
        if (errors == 0) $display("tb_adc_if PASS");
        else             $display("tb_adc_if FAIL errors=%0d", errors);
        $finish;
    end
    task assert_defaults; begin
        if (oe_n !== 1'b0 || stby_n !== 1'b0) begin
            $display("FAIL oe/stby not active"); errors = errors+1;
        end
    end endtask
endmodule
