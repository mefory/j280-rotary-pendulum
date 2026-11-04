// ============================================================================
// fb_channel.v — HPS↔fabric 调试通道（fabric 侧）
//
// 借用 FB BSP QSys 系统的三根 PIO 导出线做双向字节通道：
//   led_pio[9:0]    HPS→fabric：命令字节（写后电平保持，fabric 变化沿取指令）
//   dipsw_pio[9:0]  fabric→HPS：遥测字节流（每 tick 一字节）
//   button_pio[3:0] fabric→HPS：握手/标志（bit0=字节更新翻转、bit1=帧同步、
//                                    bit3:2={fault,mode[0]}）
//
// HPS→fabric 命令（led_pio[7:0]，写新值即触发一次，写 0x00 复位基线）：
//   0x01=启动(KEY1) 0x02=模式(KEY2) 0x03=急停/确认(KEY3) 0x04=标定(SW8)
// 触发后以 60 个 tick 的低电平注入 key_sw，经 ui 消抖形成单拍脉冲。
//
// fabric→HPS 帧（12 字节/帧 = 12ms，83Hz）：
//   [0]=0xA5 同步  [1]=帧计数  [2]=α[15:8] [3]=α[7:0]
//   [4]=duty[15:8] [5]=duty[7:0] [6]=θ[23:16] [7]=θ[15:8] [8]=θ[7:0]
//   [9]=α̇[15:8] [10]=α̇[7:0] [11]={fault,hil_use,run_en,mode[2:0]}
// ============================================================================
`include "j280_defs.vh"
module fb_channel (
    input  wire        clk,         // 50MHz
    input  wire        rst_n,
    input  wire        tick,        // 1ms 控制节拍
    // 遥测快照输入（50MHz 域，与 ctrl_core 同源）
    input  wire [15:0] alpha,
    input  wire [15:0] alpha_dot,
    input  wire [31:0] theta,
    input  wire [15:0] theta_dot,
    input  wire [15:0] duty,
    input  wire [2:0]  mode,
    input  wire        fault,
    input  wire        hil_use,
    input  wire        run_en,
    // HPS PIO 导出线
    input  wire [9:0]  led_pio,     // HPS→fabric
    output reg  [9:0]  dipsw_pio,   // fabric→HPS
    output reg  [3:0]  button_pio,  // fabric→HPS
    // 注入按键（低有效，与 key_sw 相与）
    output wire [3:0]  virt_key_n   // {stop, calib, mode, start}
);
    // ---------------- HPS→fabric 命令 ----------------
    reg [7:0] cmd_shadow;
    reg [5:0] hold_start, hold_mode, hold_stop, hold_calib;
    wire [7:0] cmd = led_pio[7:0];
    wire cmd_new = (cmd != cmd_shadow) && (cmd == 8'h01 || cmd == 8'h02 ||
                                           cmd == 8'h03 || cmd == 8'h04);
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cmd_shadow <= 8'h00;
            hold_start <= 0; hold_mode <= 0; hold_stop <= 0; hold_calib <= 0;
        end else begin
            cmd_shadow <= cmd;
            if (cmd_new) begin
                cmd_shadow    <= cmd;   // 立即记基线，防同值重触发
                case (cmd)
                    8'h01: hold_start <= 60;
                    8'h02: hold_mode  <= 60;
                    8'h03: hold_stop  <= 60;
                    8'h04: hold_calib <= 60;
                endcase
            end
            // 保持计数按 tick 递减（60 tick = 60ms，远超 20ms 消抖）
            if (tick) begin
                if (hold_start != 0) hold_start <= hold_start - 1'b1;
                if (hold_mode  != 0) hold_mode  <= hold_mode  - 1'b1;
                if (hold_stop  != 0) hold_stop  <= hold_stop  - 1'b1;
                if (hold_calib != 0) hold_calib <= hold_calib - 1'b1;
            end
        end
    end
    wire [3:0] virt_hold = {hold_stop != 0, hold_calib != 0,
                            hold_mode  != 0, hold_start != 0};
    assign virt_key_n = ~virt_hold;

    // ---------------- fabric→HPS 遥测帧 ----------------
    // 快照寄存器：tick 拍锁存，随后 12 个 tick 逐字节送出
    reg [15:0] s_alpha, s_alpha_dot, s_duty, s_theta_dot;
    reg [31:0] s_theta;
    reg [2:0]  s_mode;
    reg        s_fault, s_hil, s_run;
    reg [3:0]  byte_idx;      // 0..11
    reg [7:0]  frame_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s_alpha <= 0; s_alpha_dot <= 0; s_duty <= 0; s_theta_dot <= 0;
            s_theta <= 0; s_mode <= 0; s_fault <= 0; s_hil <= 0; s_run <= 0;
            byte_idx <= 0; frame_cnt <= 0;
        end else if (tick) begin
            byte_idx <= (byte_idx == 4'd11) ? 4'd0 : byte_idx + 1'b1;
            if (byte_idx == 4'd11) begin
                // 帧尾：锁存新快照 + 递增帧计数
                s_alpha     <= alpha;
                s_alpha_dot <= alpha_dot;
                s_theta     <= theta;
                s_theta_dot <= theta_dot;
                s_duty      <= duty;
                s_mode      <= mode;
                s_fault     <= fault;
                s_hil       <= hil_use;
                s_run       <= run_en;
                frame_cnt   <= frame_cnt + 1'b1;
            end
        end
    end

    // 当前输出字节（组合查表）
    reg [7:0] byte_out;
    always @* begin
        case (byte_idx)
            4'd0:  byte_out = 8'hA5;
            4'd1:  byte_out = frame_cnt;
            4'd2:  byte_out = s_alpha[15:8];
            4'd3:  byte_out = s_alpha[7:0];
            4'd4:  byte_out = s_duty[15:8];
            4'd5:  byte_out = s_duty[7:0];
            4'd6:  byte_out = s_theta[23:16];
            4'd7:  byte_out = s_theta[15:8];
            4'd8:  byte_out = s_theta[7:0];
            4'd9:  byte_out = s_alpha_dot[15:8];
            4'd10: byte_out = s_alpha_dot[7:0];
            // 显式 8 位（Verilog concat 实际宽度=6，隐式补零是位序坑）：
            // bit5=fault, bit4=hil, bit3=run, bit2:0=mode（与板上 debugd 解码一致）
            4'd11: byte_out = {2'b00, s_fault, s_hil, s_run, s_mode};
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dipsw_pio  <= 10'd0;
            button_pio <= 4'd0;
        end else if (tick) begin
            dipsw_pio[7:0]  <= byte_out;
            dipsw_pio[8]    <= (byte_idx == 4'd0);      // 字节有效翻转位
            dipsw_pio[9]    <= (byte_idx == 4'd0);      // 帧同步位
            button_pio[0]   <= ~button_pio[0];          // 每 tick 翻转：新字节标志
            button_pio[1]   <= (byte_idx == 4'd0);      // 帧首指示
            button_pio[3:2] <= {s_fault, s_mode[0]};    // 状态镜像
        end
    end
endmodule
