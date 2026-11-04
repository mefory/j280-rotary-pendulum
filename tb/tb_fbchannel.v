// tb_fbchannel.v — fb_channel 虚拟键与遥测帧快速自检
// 时序约定：50 个 clk 周期 = 1 个 tick（模拟 1ms 节拍，20 倍加速）
`timescale 1ns / 1ns
module tb_fbchannel;
    reg clk = 0, rst_n = 0, tick = 0;
    reg [15:0] alpha = 16'h1234, alpha_dot = 16'h5678, duty = 16'h8765, theta_dot = 0;
    reg [31:0] theta = 32'h00ABCDE0;
    reg [2:0] mode = 3'd4;
    reg fault = 1'b1, hil_use = 1'b0, run_en = 1'b0;
    reg [9:0] led_pio = 10'd0;
    wire [9:0] dipsw_pio;
    wire [3:0] button_pio;
    wire [3:0] virt_key_n;

    fb_channel dut (
        .clk(clk), .rst_n(rst_n), .tick(tick),
        .alpha(alpha), .alpha_dot(alpha_dot), .theta(theta), .theta_dot(theta_dot),
        .duty(duty), .mode(mode), .fault(fault), .hil_use(hil_use), .run_en(run_en),
        .led_pio(led_pio), .dipsw_pio(dipsw_pio), .button_pio(button_pio),
        .virt_key_n(virt_key_n)
    );

    always #10 clk = ~clk;  // 50MHz

    reg [5:0] tick_div = 0;
    always @(posedge clk) begin
        if (tick_div == 49) begin tick_div <= 0; tick <= 1'b1; end
        else begin tick_div <= tick_div + 1'b1; tick <= 1'b0; end
    end

    integer errors = 0;
    integer low_cnt = 0, i, j;
    reg [7:0] bytes [0:11];

    task tick_n(input integer n);
        integer k;
        begin
            for (k = 0; k < n; k = k + 1) @(posedge tick);
            @(negedge tick);
        end
    endtask

    initial begin
        repeat (5) @(posedge clk);
        rst_n = 1;
        tick_n(20);

        // HPS 写 start 命令：led_pio=1 保持 150 tick（对应守护进程 150ms）
        // 观测窗放在命令保持期内：虚拟键应在命令后 ~1-2 tick 拉低并保持 60 tick
        @(negedge tick); led_pio = 10'd1;
        low_cnt = 0;
        for (i = 0; i < 150; i = i + 1) begin
            if (!virt_key_n[0]) low_cnt = low_cnt + 1;
            tick_n(1);
        end
        led_pio = 10'd0;

        if (low_cnt >= 40 && low_cnt <= 80)
            $display("PASS: virt start key held %0d ticks", low_cnt);
        else begin
            $display("FAIL: virt start key held %0d ticks (expect ~60)", low_cnt);
            errors = errors + 1;
        end
        tick_n(80);

        // 遥测帧：等帧同步（dipsw[9]），抓 12 字节
        begin : frame
            for (j = 0; j < 80; j = j + 1) begin
                @(posedge tick);
                if (dipsw_pio[9]) begin
                    bytes[0] = dipsw_pio[7:0];
                    for (j = 1; j < 12; j = j + 1) begin
                        @(posedge tick);
                        bytes[j] = dipsw_pio[7:0];
                    end
                    if (bytes[0] == 8'hA5 && bytes[2] == 8'h12 && bytes[3] == 8'h34
                        && bytes[11][2:0] == 3'd4 && bytes[11][5] == 1'b1)
                        $display("PASS: frame sync/alpha/mode/fault ok (tail=%b)", bytes[11]);
                    else begin
                        $display("FAIL: frame bytes: %h %h %h tail=%b", bytes[0], bytes[2], bytes[3], bytes[11]);
                        errors = errors + 1;
                    end
                    disable frame;
                end
            end
        end

        if (errors == 0) $display("TB_FBCHANNEL ALL PASS");
        else $display("TB_FBCHANNEL %0d ERRORS", errors);
        $finish;
    end
endmodule
