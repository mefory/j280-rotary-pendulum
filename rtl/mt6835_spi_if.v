// ============================================================================
// mt6835_spi_if.v — MT6835 SPI 绝对角接口（双编码器方案的 α 传感前端）
//
// 协议（数据手册 Rev1.3 §7.6 + SimpleFOC 驱动双源定案，2026-10-16）：
//   SPI mode 3（CPOL=1, CPHA=1）：SCK 空闲高；MOSI/MISO 都在下降沿更新，
//   上升沿被双方采样；MISO 帧间 Hi-Z（FPGA 侧上拉读 1）。CSN↓ 启动、CSN↑ 结束。
//   TL(CSN↓→首SCK↓)≥100ns；TSCK≥60ns；半高/低≥30ns；TH(末SCK↑→CSN↑)≥0.5×TSCK；
//   TDV≤15ns；TST/THD=10ns（半周期 40ns 天然满足）。
//   实现参数（2026-10-16 新鲜度升级）：SCK 12.5MHz（TSCK 80ns）、半周期 40ns、
//   TL 120ns、TH 80ns、帧周期 ~20µs（帧体 4.1µs + 间隔 16µs）→ 轮询 ~50kHz。
//   OP_ANGLE 快速读角：单 CSN 窗口 48 拍 = MOSI 先发 24 位命令
//   {1010,12'h003,8'h00}=0xA00300，后 24 拍发 0；MISO 拍 1..16 为命令应答
//   （忽略），拍 17..48 为 32 位响应 [21 位角 | 3 位状态 | 8 位 CRC8]。
//   状态位：bit0 OVERSPEED、bit1 WEAKFIELD、bit2 UNDERVOLT。
//   CRC8 = poly 0x07（x⁸+x²+x+1），init 0x00，无最终异或，MSB 先行，
//   覆盖 {angle21, status[2:0]} 共 24 位——手册+驱动双源可查，无初值盲区。
//
// 输出口径（承 mt6701-alpha 分支的全 360° 直读定标）：
//   adc_raw = angle21[20:5] ∈ 0..65535 ↔ 0..360°（65536↔360.00°），
//   配合 ctrl_core 参数 ALPHA_SCALE_Q16=36000，控制链零改动；
//   alpha21 旁路输出 21 位原值（1 LSB=0.000172°），供观测器旁路使用。
//
// 健康链（语义对齐 ads1220_if / mt6701_ssi_if，接 otr_fault 安全链）：
//   坏帧 = CRC8 错 或 状态位≠0（超速/弱场/欠压任一）。连续 2 坏帧或 2ms 无
//   好帧 → adc_fault=1，期间 adc_raw/alpha21 保持末好值；首个好帧自清，
//   FSM 自愈回 GAP 重新发起帧。
// ============================================================================
`timescale 1ns / 1ps
module mt6835_spi_if #(
    parameter integer SPI_HALF_DIV   = 2,      // 50MHz -> 半周期 40ns（12.5MHz）
                                               // 整数分频上限；TSCK 80ns≥60ns、半≥30ns
    parameter integer TL_TICKS       = 3,      // CSN↓→首SCK↓ = 120ns ≥ 100ns（tick 对齐，确定性）
    parameter integer TH_TICKS       = 2,      // 末SCK↑→CSN↑ = 80ns ≥ 0.5×TSCK=40ns
    parameter integer FRAME_GAP_TICKS= 400,    // 帧间隔 16µs → 帧周期 ~20µs、轮询 ~50kHz
                                               // 数据在 1ms 控制拍上的年龄 ≤24µs（2026-10-16
                                               // 读角新鲜度升级：曾 100µs 间隔 0~107µs 随机
                                               // staleness 直接入观测器创新项）
                                               // （按半周期 tick 计数：GAP 出口 tick
                                               //  对齐，TL 才是确定值，见 ST_GAP）
    parameter integer POWERUP_CLKS   = (`CLK_HZ/500), // 上电等待 2ms（27MHz=54000）
    parameter integer WATCHDOG_CLKS  = (`CLK_HZ/500), // 好帧看门狗 2ms（27MHz=54000）
    parameter        CHECK_CRC       = 1'b1,   // CRC8 校验开关（排障用；勿用 bit 类型，Gowin 不支持）
    parameter        CHECK_STATUS    = 1'b1,   // 状态位计入坏帧（排障用）
    parameter [3:0]  BAD_FRAMES_LATCH = 4'd2   // 连续坏帧锁存门限（2026-10-19 加固）：
                                               //  T20K 实测三连锁存均在稳摆 duty 近零/
                                               //  换向密集区间（换向尖峰短促 EMI 打掉
                                               //  1~几帧），稳定大 duty 驱动 10s 零坏帧
                                               //  （diag_20261019_222422 复盘）。默认 2
                                               //  保持旧行为；T20K 放宽 8（8 帧 ≈160µs，
                                               //  仍 « 2ms 好帧看门狗）容忍突发，真断线/
                                               //  持续损坏仍由看门狗兜底；健康计数器
                                               //  （st_crc_err 等）不受门限影响照常计数。
)(
    input  wire        clk,
    input  wire        rst_n,
    output reg         spi_csn,
    output reg         spi_sck,
    output reg         spi_mosi,
    input  wire        spi_miso,
    output reg  [15:0] adc_raw,
    output reg  [20:0] alpha21,
    output reg         adc_valid,
    output reg         adc_fault,
    // ---- 链路健康统计（STATUS 应答导出；语义对齐 mt6701_ssi_if，2026-10-19）----
    // 计数器按协议事实累计（CRC/状态位的原始判定），不受 CHECK_CRC/
    // CHECK_STATUS 排障开关影响；16 位饱和不回绕。st_angle_raw/st_status
    // 每帧更新（含坏帧）：未接线时 MISO 上拉恒 1 → 角=2^21-1、status=7、
    // CRC 必错，是可直接读出的断线签名。
    output reg  [15:0] st_frames,      // 帧计数（含坏帧）
    output reg  [15:0] st_good,        // 好帧计数
    output reg  [15:0] st_crc_err,     // CRC8 错帧计数（饱和 0xFFFF）
    output reg  [15:0] st_status_err,  // 状态位≠0 帧计数（饱和 0xFFFF）
    output reg  [2:0]  st_status,      // 最近帧状态 {欠压,弱场,超速}（含坏帧）
    output reg  [20:0] st_angle_raw    // 最近帧原始角（未 hold，坏帧也更新）
);
    localparam [2:0] ST_POWERUP = 3'd0;
    localparam [2:0] ST_TL      = 3'd1;
    localparam [2:0] ST_BIT     = 3'd2;
    localparam [2:0] ST_TH      = 3'd3;
    localparam [2:0] ST_GAP     = 3'd4;

    // 位内相位：FALL=半周期点拉低 SCK 并推 MOSI；RISE=半周期点拉高 SCK；
    //           SAMP=上升沿后 1 拍采样 MISO；EVAL=末拍后 1 拍评估整帧
    localparam [1:0] PH_FALL = 2'd0;
    localparam [1:0] PH_RISE = 2'd1;
    localparam [1:0] PH_SAMP = 2'd2;
    localparam [1:0] PH_EVAL = 2'd3;

    // OP_ANGLE 命令：{4'b1010, 12'h003(REG_ANGLE1), 8'h00} + 24 位填充
    localparam [47:0] TX_ANGLE_CMD = {24'hA00300, 24'h000000};

    reg [2:0]  state;
    reg [1:0]  phase;
    reg [7:0]  spi_cnt;
    reg [5:0]  bit_idx;       // 已采样位数 0..47
    reg [47:0] tx_sr;
    reg [47:0] rx_sr;
    reg [7:0]  tl_cnt, th_cnt;
    reg [19:0] gap_cnt, pw_cnt, wd_cnt;
    reg [3:0]  bad_streak;

    wire spi_tick = (spi_cnt == SPI_HALF_DIV - 1);

    wire [31:0] resp_rx   = rx_sr[31:0];          // 拍 17..48 的响应
    wire [20:0] angle_rx  = resp_rx[31:11];
    wire [2:0]  status_rx = resp_rx[10:8];
    wire [7:0]  crc_calc  = crc8(resp_rx[31:8]);
    wire        crc_ok    = (crc_calc == resp_rx[7:0]);
    wire        status_ok = (status_rx == 3'b000);
    wire        frame_good = (CHECK_CRC    ? crc_ok    : 1'b1) &&
                             (CHECK_STATUS ? status_ok : 1'b1);

    // CRC-8：poly 0x07，init 0x00，无最终异或，MSB 先行（SimpleFOC 逐字节版
    // 的逐比特等价形式）
    function [7:0] crc8(input [23:0] d);
        reg [7:0] c;
        integer i;
        begin
            c = 8'd0;
            for (i = 23; i >= 0; i = i - 1) begin
                if (c[7] ^ d[i]) c = {c[6:0], 1'b0} ^ 8'h07;
                else             c = {c[6:0], 1'b0};
            end
            crc8 = c;
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= ST_POWERUP;
            phase      <= PH_FALL;
            spi_cnt    <= 8'd0;
            bit_idx    <= 6'd0;
            tx_sr      <= TX_ANGLE_CMD;
            rx_sr      <= 48'd0;
            tl_cnt     <= 8'd0;
            th_cnt     <= 8'd0;
            gap_cnt    <= 20'd0;
            pw_cnt     <= 20'd0;
            wd_cnt     <= 20'd0;
            bad_streak <= 4'd0;
            spi_csn    <= 1'b1;
            spi_sck    <= 1'b1;   // SPI mode 3：空闲高
            spi_mosi   <= 1'b0;
        adc_raw    <= 16'd0;
        alpha21    <= 21'd0;
        adc_valid  <= 1'b0;
        adc_fault  <= 1'b0;
        st_frames  <= 16'd0;
        st_good    <= 16'd0;
        st_crc_err <= 16'd0;
        st_status_err <= 16'd0;
        st_status  <= 3'd0;
        st_angle_raw <= 21'd0;
        end else begin
            adc_valid <= 1'b0;

            if (spi_tick) spi_cnt <= 8'd0;
            else          spi_cnt <= spi_cnt + 8'd1;

            case (state)
                ST_POWERUP: begin
                    spi_csn <= 1'b1;
                    spi_sck <= 1'b1;
                    if (pw_cnt >= POWERUP_CLKS - 1) begin
                        pw_cnt <= 20'd0;
                        gap_cnt<= 20'd0;
                        wd_cnt <= 20'd0;
                        state  <= ST_GAP;      // 经 tick 对齐的 GAP 进 TL，TL 才确定
                    end else begin
                        pw_cnt <= pw_cnt + 20'd1;
                    end
                end

                // TL：CSN 已低，等够 CSN↓→首SCK↓ 再发起首拍
                ST_TL: begin
                    spi_csn <= 1'b0;
                    spi_sck <= 1'b1;
                    wd_cnt  <= wd_cnt + 20'd1;
                    if (spi_tick) begin
                        if (tl_cnt >= TL_TICKS - 1) begin
                            spi_sck <= 1'b0;             // 首 SCK↓
                            spi_mosi <= tx_sr[47];         // 首个 MOSI 位
                            tx_sr   <= {tx_sr[46:0], 1'b0};
                            tl_cnt  <= 8'd0;
                            bit_idx <= 6'd0;
                            phase   <= PH_RISE;
                            state   <= ST_BIT;
                        end else begin
                            tl_cnt <= tl_cnt + 8'd1;
                        end
                    end
                end

                ST_BIT: begin
                    spi_csn <= 1'b0;
                    wd_cnt  <= wd_cnt + 20'd1;
                    case (phase)
                        PH_FALL: begin
                            if (spi_tick) begin
                                spi_sck  <= 1'b0;          // SCK↓：换下一位
                                spi_mosi <= tx_sr[47];
                                tx_sr    <= {tx_sr[46:0], 1'b0};
                                phase    <= PH_RISE;
                            end
                        end
                        PH_RISE: begin
                            if (spi_tick) begin
                                spi_sck <= 1'b1;           // SCK↑：采样点将至
                                phase   <= PH_SAMP;
                            end
                        end
                        PH_SAMP: begin
                            // SCK↑ 后 1 拍：MISO 自上一 SCK↓ 起已稳定 ≥ 半周期
                            rx_sr <= {rx_sr[46:0], spi_miso};
                            if (bit_idx == 6'd47)
                                phase <= PH_EVAL;          // 末位已入，1 拍后评估
                            else begin
                                bit_idx <= bit_idx + 6'd1;
                                phase   <= PH_FALL;
                            end
                        end
                        PH_EVAL: begin
                            // 整帧已就位（rx_sr[31:0]=响应），发布评估结果。
                            // 健康统计先于好/坏分支：坏帧也要留下原始角与状态。
                            st_frames    <= st_frames + 16'd1;
                            st_status    <= status_rx;
                            st_angle_raw <= angle_rx;
                            if (!crc_ok && st_crc_err != 16'hFFFF)
                                st_crc_err <= st_crc_err + 16'd1;
                            if (!status_ok && st_status_err != 16'hFFFF)
                                st_status_err <= st_status_err + 16'd1;
                            if (frame_good) begin
                                adc_raw   <= angle_rx[20:5];
                                alpha21   <= angle_rx;
                                adc_valid <= 1'b1;
                                adc_fault <= 1'b0;
                                bad_streak<= 4'd0;
                                wd_cnt    <= 20'd0;
                                st_good   <= st_good + 16'd1;
                            end else begin
                                bad_streak <= bad_streak + 4'd1;
                                if (bad_streak + 4'd1 >= BAD_FRAMES_LATCH)
                                    adc_fault <= 1'b1;
                            end
                            tx_sr  <= TX_ANGLE_CMD;         // 复位命令移位器
                            th_cnt <= 8'd0;
                            state  <= ST_TH;
                        end
                        default: phase <= PH_FALL;
                    endcase
                end

                ST_TH: begin
                    spi_sck <= 1'b1;
                    wd_cnt  <= wd_cnt + 20'd1;
                    if (spi_tick) begin
                        if (th_cnt >= TH_TICKS - 1) begin
                            spi_csn <= 1'b1;
                            th_cnt  <= 8'd0;
                            gap_cnt <= 20'd0;
                            state   <= ST_GAP;
                        end else begin
                            th_cnt <= th_cnt + 8'd1;
                        end
                    end
                end

                ST_GAP: begin
                    spi_csn <= 1'b1;
                    spi_sck <= 1'b1;
                    wd_cnt  <= wd_cnt + 20'd1;
                    // 按 tick 计数并只在 tick 上退出：CSN 也在退出拍同时拉低
                    // （TL = TL_TICKS×半周期，无 -1 拍缩短、无相位抖动）。
                    if (spi_tick) begin
                        if (gap_cnt >= FRAME_GAP_TICKS - 1) begin
                            gap_cnt <= 20'd0;
                            tl_cnt  <= 8'd0;
                            spi_csn <= 1'b0;   // 与退出同拍，TL 起点对齐 tick
                            state   <= ST_TL;
                        end else begin
                            gap_cnt <= gap_cnt + 20'd1;
                        end
                    end
                end

                default: state <= ST_POWERUP;
            endcase

            // 自愈：好帧看门狗超时 = 链路死（断线/卡帧），强制放弃当前帧回 GAP
            if (state != ST_POWERUP && wd_cnt >= WATCHDOG_CLKS - 1) begin
                adc_fault <= 1'b1;
                state     <= ST_GAP;
                phase     <= PH_FALL;
                bit_idx   <= 6'd0;
                tx_sr     <= TX_ANGLE_CMD;
                spi_csn   <= 1'b1;
                spi_sck   <= 1'b1;
                gap_cnt   <= 20'd0;
                wd_cnt    <= 20'd0;
            end
        end
    end
endmodule
