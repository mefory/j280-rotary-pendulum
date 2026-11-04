// move_blend 动力学专项探针:复现"保持段 mb=2044/k3e=112"。
// 不跑整机闭环——直接例化 ctrl_core 太重,改用【等价最小复现】:
// 用 iverilog 直接把 ctrl_core 的 move_blend always 块单独仿真,
// 但为排除"我抄错",直接 force ctrl_core 内部信号(借 tb_closedloop 的 force 手法)
// 代价高。最小方案:精确复刻块语义 + 参数,由 iverilog 裁决。
`timescale 1ns/1ps
module tb_move_blend_probe;
    reg clk=0, rst_n=0, tick=0;
    always #5 clk=~clk;
    always #1000 begin tick=1; #20; tick=0; end

    // 与 ctrl_core.v:814-830 逐行同构(155614 板上版,THDOT=80 dec=8)
    localparam signed [15:0] POS_MOVE_K3 = 16'sd360;
    localparam signed [15:0] POS_MOVE_THDOT = 16'sd80;
    reg [10:0] move_blend;
    reg pos_rehold = 0;
    reg mode_is_pos = 1;
    reg signed [15:0] theta_dot = 0;
    wire pos_moving = (mode_is_pos) &&
                      ((theta_dot > POS_MOVE_THDOT) || (theta_dot < -POS_MOVE_THDOT));
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) move_blend <= 11'd0;
        else if (tick) begin
            if (mode_is_pos != 1 || pos_rehold) move_blend <= 11'd0;
            else if (pos_moving && move_blend < 11'd1024) move_blend <= move_blend + 11'd4;
            else if (!pos_moving && move_blend > 11'd0)   move_blend <= move_blend - 11'd8;
        end
    end
    wire signed [16:0] k3m_span = $signed(POS_MOVE_K3) - $signed(600);
    wire signed [27:0] k3m_prod = k3m_span * $signed({1'b0, move_blend});
    wire signed [15:0] k3_eff_m = $signed(600) + (k3m_prod >>> 10);

    integer i;
    initial begin
        #100; rst_n = 1;
        // 场景1:θ̇ 抖动 60↔100(门 80 两侧)各 1ms,共 400ms,看 mb 峰
        for (i = 0; i < 200; i = i + 1) begin
            @(posedge tick);
            theta_dot = (i % 2) ? 16'sd100 : -16'sd100;   // 恒移动
        end
        $display("场景1 恒移动 200ms: mb=%0d k3m=%0d", move_blend, k3_eff_m);
        // 场景2:门内交替
        for (i = 0; i < 400; i = i + 1) begin
            @(posedge tick);
            theta_dot = (i % 2) ? 16'sd60 : -16'sd60;     // 恒静止侧
        end
        $display("场景2 恒静止 200ms: mb=%0d", move_blend);
        // 场景3:交替
        for (i = 0; i < 600; i = i + 1) begin
            @(posedge tick);
            theta_dot = (i % 3 == 0) ? 16'sd90 : 16'sd50; // 1/3 拍移动
        end
        $display("场景3 交替: mb=%0d", move_blend);
        $finish;
    end
endmodule
