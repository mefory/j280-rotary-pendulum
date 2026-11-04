// ============================================================================
// tb_lqr_core.v — MAC 流水线与 TB 黄金模型逐向量比对（含饱和）
// ============================================================================
`timescale 1ns / 1ps
`include "j280_gains.vh"
module tb_lqr_core;
    reg clk = 0, rst_n = 0, start = 0;
    reg signed [15:0] alpha, adot, the, tdot;
    reg signed [15:0] k1, k2, k3, k4;
    wire signed [15:0] u;
    wire done;
    integer errors = 0;

    lqr_core dut (
        .clk(clk), .rst_n(rst_n), .start(start),
        .alpha(alpha), .alpha_dot(adot), .theta_e(the), .theta_dot(tdot),
        .k1(k1), .k2(k2), .k3(k3), .k4(k4),
        .u(u), .done(done)
    );
    always #5 clk = ~clk;

    // 黄金模型：与 RTL 相同的定点次序（Q3.13 → >>>13 → 饱和 ±9500）
    function [15:0] golden(input signed [15:0] a, ad, te, td,
                           input signed [15:0] g1, g2, g3, g4);
        reg signed [63:0] s;
        begin
            s = a*g1 + ad*g2 + te*g3 + td*g4;
            s = (s+64'sd4096) >>> 13;
            if      (s >  9500) golden = 16'sd9500;
            else if (s < -9500) golden = -16'sd9500;
            else                golden = s[15:0];
        end
    endfunction

    task vec(input signed [15:0] a, ad, te, td);
        reg [15:0] exp_u;
        begin
            @(negedge clk);
            alpha = a; adot = ad; the = te; tdot = td; start = 1;
            exp_u = golden(a, ad, te, td, k1, k2, k3, k4);
            @(negedge clk); start = 0;
            repeat (4) @(negedge clk);
            if (u !== exp_u) begin
                $display("FAIL vec a=%0d ad=%0d te=%0d td=%0d : u=%0d exp=%0d",
                         a, ad, te, td, u, exp_u);
                errors = errors + 1;
            end
        end
    endtask

    initial begin
        rst_n = 0; #40; rst_n = 1; #20;
        // 与当前 j280_gains.vh 同量纲的代表性增益
        k1 = `G_K1; k2 = `G_K2; k3 = `G_K3; k4 = `G_K4;

        vec(16'sd1000,  16'sd50,  16'sd0,   16'sd0);
        vec(-16'sd2500, -16'sd300, 16'sd800, -16'sd100);
        vec(16'sd0,     16'sd0,   16'sd0,   16'sd0);
        vec(16'sd30000, 16'sd0,   16'sd0,   16'sd0);          // 应触发饱和
        vec(-16'sd30000, 16'sd2000, -16'sd2000, 16'sd1500);
        vec(16'sd32767,16'sd32767,16'sd32767,16'sd32767);
        vec(-16'sd32768,-16'sd32768,-16'sd32768,-16'sd32768);
        k1=-16'sd32768;k2=-16'sd32768;k3=-16'sd32768;k4=-16'sd32768;
        vec(-16'sd32768,-16'sd32768,-16'sd32768,-16'sd32768);

        if (errors == 0) $display("tb_lqr_core PASS");
        else             $fatal(1,"tb_lqr_core FAIL errors=%0d", errors);
        $finish;
    end
endmodule
