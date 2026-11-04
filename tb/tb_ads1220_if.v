// tb_ads1220_if.v — ADS1220 SPI mode 1 行为模型
// 覆盖：RESET/WREG/START、24 位下降沿采样、比例缩放和转换超时。
`timescale 1ns / 1ps
module tb_ads1220_if;
    reg clk = 1'b0;
    reg rst_n = 1'b0;
    reg ads_miso = 1'b0;
    reg ads_drdy = 1'b1;
    wire ads_cs_n, ads_sclk, ads_mosi;
    wire [15:0] adc_raw;
    wire adc_valid, adc_fault;

    ads1220_if #(
        .SPI_HALF_DIV(1),
        .POWERUP_WAIT_CLKS(3),
        .RESET_WAIT_CLKS(3),
        .WATCHDOG_CLKS(80)
    ) dut (
        .clk(clk), .rst_n(rst_n),
        .ads_miso(ads_miso), .ads_drdy(ads_drdy),
        .ads_cs_n(ads_cs_n), .ads_sclk(ads_sclk), .ads_mosi(ads_mosi),
        .adc_raw(adc_raw), .adc_valid(adc_valid), .adc_fault(adc_fault)
    );

    always #5 clk = ~clk;


    reg [39:0] cmd_shift = 40'd0;
    integer cmd_bits = 0;
    integer bit_index = 0;
    reg reading = 1'b0;
    reg saw_reset = 1'b0;
    reg saw_wreg = 1'b0;
    reg saw_start = 1'b0;
    reg [23:0] sample_code = 24'h400000;

    // DIN 在下降沿被 ADS1220 锁存；模型同时记录每个 CS 帧。
    always @(negedge ads_cs_n) begin
        cmd_shift = 40'd0;
        cmd_bits = 0;
        bit_index = 0;
        if (saw_start) begin
            reading = 1'b1;
            ads_miso = sample_code[23];
        end else begin
            reading = 1'b0;
            ads_miso = 1'b0;
        end
    end

    always @(negedge ads_sclk) begin
        if (!ads_cs_n) begin
            cmd_shift = {cmd_shift[38:0], ads_mosi};
            cmd_bits = cmd_bits + 1;
            if (reading) begin
                bit_index = bit_index + 1;
                if (bit_index < 24)
                    ads_miso = sample_code[23-bit_index];
            end
        end
    end

    // DOUT 在上升沿推出下一位，DRDY 在首个读时钟上升沿恢复高电平。
    always @(posedge ads_sclk) begin
        if (reading && !ads_cs_n)
            ads_drdy = 1'b1;
    end

    always @(posedge ads_cs_n) begin
        if (cmd_bits == 8 && cmd_shift[7:0] == 8'h06)
            saw_reset = 1'b1;
        if (cmd_bits == 40 && cmd_shift == {8'h43, 8'h81, 8'hD4, 8'h40, 8'h00})
            saw_wreg = 1'b1;
        if (cmd_bits == 8 && cmd_shift[7:0] == 8'h08) begin
            saw_start = 1'b1;
            fork
                begin
                    repeat (5) @(posedge clk);
                    ads_drdy = 1'b0;
                end
            join_none
        end
        if (cmd_bits == 24)
            reading = 1'b0;
    end

    integer errors = 0;
    initial begin
        repeat (3) @(posedge clk);
        rst_n = 1'b1;

        wait (saw_start === 1'b1);
        wait (adc_valid === 1'b1);
        if (!saw_reset) begin $display("FAIL: RESET 未发送"); errors = errors + 1; end
        if (!saw_wreg) begin $display("FAIL: WREG 配置帧错误"); errors = errors + 1; end
        if (!saw_start) begin $display("FAIL: START 未发送"); errors = errors + 1; end
        if (adc_raw !== 16'd32736) begin
            $display("FAIL: raw=%0d, 期望 32736", adc_raw);
            errors = errors + 1;
        end else begin
            $display("PASS: 24位码缩放为 raw=%0d", adc_raw);
        end
        repeat (100) @(posedge clk);
        if (!adc_fault) begin
            $display("FAIL: DRDY 超时未置 fault");
            errors = errors + 1;
        end else begin
            $display("PASS: DRDY 超时故障");
        end

        if (errors == 0) $display("TB_ADS1220_IF PASS");
        else             $display("TB_ADS1220_IF FAIL errors=%0d", errors);
        $finish;
    end
endmodule
