// rpll_sim.v — Gowin rPLL 原语的 iverilog 行为模型
// 仅在定义 SIMULATION 宏时编译（run_sim.ps1 的 iverilog 命令行加 -DSIMULATION；
// Gowin 工程的 build.tcl 显式 add_file 列表不含本文件，综合时用工具库真原语）。
// 行为：CLKOUT 翻转频率=CLKIN（仿真无需精确 50/27 倍频比，只要求时钟树存在），
// LOCK 在 RESET 释放后数拍置位。defparam 引用的厂商参数在此不声明，工具按警告忽略。
`ifdef SIMULATION
module rPLL (
    output reg  CLKOUT,
    output reg  LOCK,
    output wire CLKOUTP,
    output wire CLKOUTD,
    output wire CLKOUTD3,
    input  wire RESET,
    input  wire RESET_P,
    input  wire CLKIN,
    input  wire CLKFB,
    input  wire [5:0] FBDSEL,
    input  wire [5:0] IDSEL,
    input  wire [5:0] ODSEL,
    input  wire [3:0] PSDA,
    input  wire [3:0] DUTYDA,
    input  wire [3:0] FDLY
);
    // 厂商参数占位：顶层 defparam 的落点（行为模型忽略其值）
    parameter FCLKIN         = "27";
    parameter IDIV_SEL       = 2;
    parameter FBDIV_SEL      = 49;
    parameter ODIV_SEL       = 9;
    parameter DYN_IDIV_SEL   = "false";
    parameter DYN_FBDIV_SEL  = "false";
    parameter DYN_ODIV_SEL   = "false";
    parameter PSDA_SEL       = "0000";
    parameter DUTYDA_SEL     = "1000";
    parameter CLKFB_SEL      = "internal";
    parameter CLKOUT_BYPASS  = "false";
    parameter CLKOUTP_BYPASS = "false";
    parameter CLKOUTD_BYPASS = "false";
    parameter CLKOUTD_SRC    = "CLKOUT";
    parameter CLKOUTD3_SRC   = "CLKOUT";
    parameter DEVICE         = "GW2A-18C";

    initial CLKOUT = 1'b0;
    always @(posedge CLKIN) CLKOUT <= ~CLKOUT;

    reg [3:0] lock_cnt;
    initial begin LOCK = 1'b0; lock_cnt = 4'd0; end
    always @(posedge CLKIN or posedge RESET) begin
        if (RESET) begin LOCK <= 1'b0; lock_cnt <= 4'd0; end
        else if (lock_cnt != 4'd15) begin
            lock_cnt <= lock_cnt + 4'd1;
            if (lock_cnt == 4'd14) LOCK <= 1'b1;
        end
    end
    assign CLKOUTP = 1'b0;
    assign CLKOUTD = 1'b0;
    assign CLKOUTD3 = 1'b0;
endmodule
`endif
