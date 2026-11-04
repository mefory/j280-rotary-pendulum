// ============================================================================
// ltc2308_if.v — DE10-Standard 板载 LTC2308 ADC 接口（替代外接 3PA1030）
//
// 协议要点（datasheet 2308fc，TIMING DIAGRAM 与 APPLICATIONS 章节）：
//   · CONVST 上升沿启动转换，内部转换时钟 tCONV typ 1.3µs / max 1.6µs；
//   · SDO 在 CONVST 上升沿后 tEN 使能并给出 MSB，之后每个 SCK 下降沿换位；
//   · DIN 6 位字 {S/D,O/S,S1,S0,UNI,SLP} 在前 6 个 SCK 上升沿移入（MSB 先行），
//     移完即开始 ≥240ns 采集，同时剩余位继续移出；
//   · 本模块固定读 CH0 单端单极性：DIN = 6'b100010；
//   · SCK 5MHz（半位 100ns×10 分频），远低于 40MHz 上限；转换期间 SCK 静止。
//
// 标度约定（与 3PA1030 的 adc_if 输出完全一致，ctrl_core/angle_proc 零改动）：
//   alpha_avg u16 满量程 65472 ↔ 摆杆电位器电气行程 345°。
//   WDD35D4 由 3.3V 激励、LTC2308 内部基准 4.096V（1 LSB = 1mV），
//   满行程 3.3V → 3300 计数，故 alpha_avg = cnt × 65472/3300（Q16 系数）。
//   3.3V 轨 ±2~3% 容差直接进入增益误差，与原板分压方案同级；如需精修，
//   调 K_Q16 即可（不动画程约定）。
//
// 吞吐：一帧 = GAP 10 + CONV 96 + SHIFT 160 ≈ 266 clk ≈ 5.3µs（≈188ksps），
//   2^AVGS 次盒式平均（默认 16）→ alpha_avg 更新率 ≈ 11.7kHz，远高于 1kHz 控制节拍。
// ============================================================================
`timescale 1ns / 1ps
module ltc2308_if #(
    parameter AVGS = 4              // 2^AVGS 次盒式平均
)(
    input  wire        clk,         // 50MHz
    input  wire        rst_n,
    output reg         adc_convst,  // → LTC2308 CONVST
    output reg         adc_sck,     // → LTC2308 SCK
    output reg         adc_sdi,     // → LTC2308 SDI（恒为 DIN 字 100010 的流水）
    input  wire        adc_sdo,     // ← LTC2308 SDO
    output reg  [15:0] alpha_avg,   // u16，满量程 65472 ↔ 345°
    output wire        otr_event    // 恒 0：LTC2308 无越界指示，0~3.3V 不会越 0~4.096V 量程
);
    // ---- 时序常量（50MHz） ----
    localparam [7:0]  CONV_CLKS = 8'd96;  // 1.92µs ≥ tCONV max 1.6µs + tEN
    localparam [3:0]  HALF      = 4'd10;  // SCK 半位 = 200ns → 5MHz
    localparam [7:0]  GAP_CLKS  = 8'd10;  // 帧间空隙
    localparam [5:0]  DIN_WORD  = 6'b100010; // 单端 CH0 / 单极性 / 正常模式

    // 标度：3300 计数(3.3V) → 65472；K = 65472/3300 = 19.84 的 Q16 定点
    localparam [20:0] K_Q16 = 21'd1300234; // round(19.84×65536)=1300234.24

    // ---- 帧状态机 ----
    localparam S_IDLE = 2'd0, S_CONV = 2'd1, S_SHIFT = 2'd2;
    reg [1:0]  state;
    reg [7:0]  wait_cnt;
    reg [3:0]  slot;       // SCK 半位计数 0..HALF-1（0..4 低半位，5..9 高半位）
    reg [3:0]  bitn;       // 帧内位计数 0..15
    reg [15:0] sdo_sr;     // MSB 先行左移入，D11..D0 = sdo_sr[15:4]

    wire smp_stb = (state == S_SHIFT) && (slot == 4'd5);           // SCK 上升沿拍
    wire frame_end = (state == S_SHIFT) && (slot == HALF-4'd1) &&
                     (bitn == 4'd15);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE; wait_cnt <= 8'd0; slot <= 4'd0; bitn <= 4'd0;
            adc_convst <= 1'b0; adc_sck <= 1'b0; adc_sdi <= 1'b0;
            sdo_sr <= 16'd0;
        end else begin
            case (state)
                S_IDLE: begin
                    adc_convst <= 1'b0;
                    if (wait_cnt == GAP_CLKS - 8'd1) begin
                        wait_cnt <= 8'd0;
                        state    <= S_CONV;
                    end else
                        wait_cnt <= wait_cnt + 8'd1;
                end
                S_CONV: begin
                    adc_convst <= 1'b1;          // 上升沿启动转换
                    if (wait_cnt == CONV_CLKS - 8'd1) begin
                        wait_cnt <= 8'd0;
                        slot     <= 4'd0;
                        bitn     <= 4'd0;
                        state    <= S_SHIFT;
                    end else
                        wait_cnt <= wait_cnt + 8'd1;
                end
                S_SHIFT: begin
                    adc_convst <= 1'b0;
                    if (slot == HALF - 4'd1) begin
                        slot <= 4'd0;
                        bitn <= bitn + 4'd1;
                        if (bitn == 4'd15)
                            state <= S_IDLE;
                    end else
                        slot <= slot + 4'd1;
                    if (smp_stb)
                        sdo_sr <= {sdo_sr[14:0], adc_sdo};  // 上升沿采样
                end
                default: state <= S_IDLE;
            endcase
            // SCK：SHIFT 态 slot 5..9 高半位（寄存一拍出片，无毛刺）
            adc_sck <= (state == S_SHIFT) && (slot >= 4'd5);
            // SDI：在前 6 个上升沿前两个半位置好字（预留 ≥40ns 建立时间）
            if (state == S_SHIFT && slot == 4'd3)
                adc_sdi <= (bitn < 4'd6) ? DIN_WORD[5-bitn[2:0]] : 1'b0;
        end
    end

    // ---- 盒式平均 + 标度 ----
    localparam [3:0] N_AVG = 4'd15;    // (1<<AVGS)-1，AVGS=4 → 16 帧
    reg [16:0] sum;                    // 16×4095 < 2^17
    reg [3:0]  navg;                   // 0..N_AVG
    wire [11:0] sample = sdo_sr[15:4]; // 末 12 次采样 = 12 位结果
    wire [16:0] sum_nxt = sum + {5'd0, sample};
    wire [11:0] avg_cnt = sum_nxt[16:AVGS]; // >>AVGS → 0..4095
    reg  [32:0] prod;
    always @* prod = avg_cnt * K_Q16;      // 4095×1300403 < 2^33
    // 结果 = prod>>16 = prod[32:16]（17 位）；仅当 ≥65536（bit32=1）才饱和
    wire [15:0] scaled = prod[32] ? 16'hFFFF : prod[31:16];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sum <= 17'd0; navg <= 4'd0; alpha_avg <= 16'd0;
        end else if (frame_end) begin
            if (navg == N_AVG) begin   // 数满 2^AVGS 帧：更新并清零
                navg      <= 4'd0;
                sum       <= 17'd0;
                alpha_avg <= scaled;
            end else begin
                navg <= navg + 4'd1;
                sum  <= sum_nxt;
            end
        end
    end

    assign otr_event = 1'b0;
endmodule
