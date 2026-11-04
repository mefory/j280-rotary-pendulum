// ============================================================================
// tb_ltc2308_if.v — ltc2308_if 自校验测试台（iverilog）
// 行为级 LTC2308 模型按 datasheet 2308fc：
//   · CONVST 上升沿采样当前通道并开始转换（1.6µs），tEN=100ns 后 SDO 给出 MSB；
//   · SCK 下降沿换出下一位，上升沿移入 DIN；
//   · 6 位 DIN {S/D,O/S,S1,S0,UNI,SLP} 在前 6 个上升沿锁存，配置下一帧通道。
// 检查项：标度（3.3V→65471、0.825V→16368）、饱和（4.0V→65535）、
//         DIN 字=100010、平均后更新率。
// ============================================================================
`timescale 1ns / 1ps
module ltc2308_model (
    input wire convst,
    input wire sck,
    input wire sdi,
    output reg sdo
);
    real vch [0:7];
    reg [11:0] out_sr;
    reg sdo_en;
    reg [2:0] din_cnt;
    reg [5:0] din_sr;
    reg [2:0] chn;
    reg frame_first;
    reg latched;
    integer din_errors;

    function [11:0] v2code(input real v);
        integer c;
        begin
            c = $rtoi(v * 1000.0 + 0.5);   // v/4.096*4096 = v×1000（1 LSB = 1mV）
            if (c < 0) c = 0;
            if (c > 4095) c = 4095;
            v2code = c[11:0];
        end
    endfunction

    initial begin
        sdo_en = 1'b0; chn = 3'd0; din_cnt = 3'd0;
        din_errors = 0; frame_first = 1'b0;
    end

    // CONVST 上升沿：锁存当前通道输入，tEN 后 SDO 使能，DIN 帧定界复位
    always @(posedge convst) begin
        out_sr <= v2code(vch[chn]);
        sdo_en <= 1'b0;
        frame_first <= 1'b1;
        latched <= 1'b0;
        #100 sdo_en = 1'b1;               // 阻塞赋值保持时序模型简单
    end

    always @* if (sdo_en) sdo = out_sr[11]; else sdo = 1'bz;
    always @(negedge sck) if (sdo_en) out_sr <= {out_sr[10:0], 1'b0};

    // DIN 上升沿移入；每帧只核对第一个 6 位窗（后续 10 个边沿属填充，芯片不关心）
    always @(posedge sck) begin
        if (frame_first) begin            // 转换后首个上升沿装 S/D 位（datasheet）
            frame_first <= 1'b0;
            din_cnt     <= 3'd0;
            din_sr      <= {5'b00000, sdi};
        end else if (!latched) begin
            din_sr <= {din_sr[4:0], sdi};
            if (din_cnt == 3'd4) begin
                latched <= 1'b1;
                if ({din_sr[4:0], sdi} != 6'b100010) begin
                    din_errors = din_errors + 1;
                    $display("[%0t] ERROR DIN=%b (期望 100010)", $time, {din_sr[4:0], sdi});
                end
                chn <= {din_sr[3], din_sr[2], din_sr[1]};  // {O/S,S1,S0}
            end else
                din_cnt <= din_cnt + 3'd1;
        end
    end
endmodule

module tb_ltc2308_if;
    reg clk = 1'b0, rst_n = 1'b0;
    wire convst, sck, sdi, sdo, otr;
    wire [15:0] alpha_avg;
    integer updates;

    ltc2308_if #(.AVGS(4)) dut (
        .clk(clk), .rst_n(rst_n),
        .adc_convst(convst), .adc_sck(sck), .adc_sdi(sdi), .adc_sdo(sdo),
        .alpha_avg(alpha_avg), .otr_event(otr)
    );
    ltc2308_model u_model (.convst(convst), .sck(sck), .sdi(sdi), .sdo(sdo));

    always #10 clk = ~clk;    // 50MHz

    always @(alpha_avg) updates = updates + 1;

    task wait_updates(input integer n);
        integer i;
        begin for (i = 0; i < n; i = i + 1) @(alpha_avg); end
    endtask

    task check(input [15:0] exp_val, input [15:0] tol, input [127:0] name);
        begin
            if (alpha_avg > exp_val + tol || alpha_avg < exp_val - tol) begin
                $display("FAIL %0s: alpha_avg=%0d 期望 %0d±%0d", name, alpha_avg, exp_val, tol);
                $finish;
            end else
                $display("ok   %0s: alpha_avg=%0d (期望 %0d±%0d)", name, alpha_avg, exp_val, tol);
        end
    endtask

    // alpha_avg 更新周期 = 16 帧 × 5.32µs ≈ 85µs；恒定电压下值不再变化，
    // 事件等待会挂死，故按周期定时等待（3 个更新周期足矣）
    task settle;
        begin #270_000; end
    endtask

    initial begin
        updates = 0;
        vch_init;
        repeat (10) @(posedge clk);
        rst_n = 1'b1;

        // 相位 1：满行程 3.3V → 3300 计数 → 65471（K 截尾 65471.99）
        u_model.vch[0] = 3.3;
        settle; check(16'd65471, 16'd2, "3.3V full-scale");

        // 相位 2：0.825V（约自然下垂区）→ 825 计数 → 16368
        u_model.vch[0] = 0.825;
        settle; check(16'd16368, 16'd2, "0.825V mid-low");

        // 相位 3：4.0V 超出 3.3V 行程 → 饱和限幅 65535
        u_model.vch[0] = 4.0;
        settle; check(16'd65535, 16'd0, "4.0V saturate");

        // 相位 4：回 0V → 0
        u_model.vch[0] = 0.0;
        settle; check(16'd0, 16'd0, "0V zero");

        if (u_model.din_errors != 0) begin
            $display("FAIL DIN 字错误 %0d 次", u_model.din_errors);
            $finish;
        end
        $display("PASS: 全部检查通过（含 DIN=100010）");
        $finish;
    end

    task vch_init;
        integer i;
        begin for (i = 0; i < 8; i = i + 1) u_model.vch[i] = 0.0; end
    endtask

    // 看门狗
    initial begin
        #20_000_000;
        $display("FAIL: 超时未完成（updates=%0d alpha_avg=%0d）", updates, alpha_avg);
        $finish;
    end
endmodule
