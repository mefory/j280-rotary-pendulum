// ============================================================================
// ads1220_if.v — ADS1220 连续转换 SPI 接口
//
// 连接约定：
//   ADS1220 SPI mode 1：SCLK 空闲低；器件在上升沿改变 DOUT，下降沿采样；
//   DRDY 下降沿表示新结果可读；连续转换模式下直接读 24 位，不发送 RDATA。
//   上电后 RESET -> WREG(4 个配置寄存器) -> START/SYNC，然后等待 DRDY。
//
// 输出保持项目现有 ADC 口径：
//   正向 24 位码 0x000000..0x7FFFFF 缩放为 0..65472；负码钳位为 0。
// ============================================================================
`timescale 1ns / 1ps
module ads1220_if #(
    parameter integer SPI_HALF_DIV      = 6,       // 50MHz -> 约 4.17MHz SCLK
    parameter integer POWERUP_WAIT_CLKS = 5000,    // 约 100us @ 50MHz
    parameter integer RESET_WAIT_CLKS   = 5000,    // >= 50us + 32*tCLK
    parameter integer WATCHDOG_CLKS     = 100000   // 约 2ms @ 50MHz
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        ads_miso,
    input  wire        ads_drdy,
    output reg         ads_cs_n,
    output reg         ads_sclk,
    output reg         ads_mosi,
    output reg  [15:0] adc_raw,
    output reg         adc_valid,
    output reg         adc_fault
);
    localparam [3:0] ST_POWERUP = 4'd0;
    localparam [3:0] ST_SEND    = 4'd1;
    localparam [3:0] ST_RESET   = 4'd2;
    localparam [3:0] ST_RUN     = 4'd3;
    localparam [3:0] ST_READ    = 4'd4;
    localparam [3:0] ST_START_GAP = 4'd5;
    localparam [3:0] ST_SEND_HOLD = 4'd6;
    localparam [3:0] ST_READ_HOLD = 4'd7;
    localparam [3:0] ST_RESYNC  = 4'd8;   // 自愈：72 个 SCLK 强制 SPI 回同步

    localparam [2:0] TX_RESET = 3'd0;
    localparam [2:0] TX_WREG  = 3'd1;
    localparam [2:0] TX_START = 3'd2;

    localparam [39:0] RESET_FRAME = {8'h06, 32'd0};
    localparam [39:0] WREG_FRAME  = {8'h43, 8'h81, 8'hD4, 8'h40, 8'h00};
    localparam [39:0] START_FRAME = {8'h08, 32'd0};

    reg [3:0]  state;
    reg [2:0]  tx_kind;
    reg [39:0] tx_sr;
    reg [5:0]  tx_bits;
    reg [23:0] rx_sr;
    reg [5:0]  rx_bits;
    reg [31:0] wait_cnt;
    reg [31:0] watchdog_cnt;
    reg [7:0]  spi_cnt;
    reg        drdy_meta;
    reg        drdy_sync;
    reg        drdy_prev;

    wire drdy_fall = drdy_prev && !drdy_sync;
    wire spi_tick  = (spi_cnt == SPI_HALF_DIV - 1);
    wire [23:0] rx_final = {rx_sr[22:0], ads_miso};
    wire [22:0] code_mag = rx_final[23] ? 23'd0 : rx_final[22:0];
    wire [39:0] scaled_product = code_mag * 17'd65472;
    wire [16:0] scaled_value = scaled_product[39:23];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            drdy_meta <= 1'b1;
            drdy_sync <= 1'b1;
            drdy_prev <= 1'b1;
        end else begin
            drdy_meta <= ads_drdy;
            drdy_sync <= drdy_meta;
            drdy_prev <= drdy_sync;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state         <= ST_POWERUP;
            tx_kind       <= TX_RESET;
            tx_sr         <= 40'd0;
            tx_bits       <= 6'd0;
            rx_sr         <= 24'd0;
            rx_bits       <= 6'd0;
            wait_cnt      <= 32'd0;
            watchdog_cnt  <= 32'd0;
            spi_cnt       <= 8'd0;
            ads_cs_n      <= 1'b1;
            ads_sclk      <= 1'b0;
            ads_mosi      <= 1'b0;
            adc_raw       <= 16'd0;
            adc_valid     <= 1'b0;
            adc_fault     <= 1'b0;
        end else begin
            adc_valid <= 1'b0;

            if (spi_tick)
                spi_cnt <= 8'd0;
            else
                spi_cnt <= spi_cnt + 8'd1;

            case (state)
                ST_POWERUP: begin
                    ads_cs_n <= 1'b1;
                    ads_sclk <= 1'b0;
                    ads_mosi <= 1'b0;
                    if (wait_cnt >= POWERUP_WAIT_CLKS - 1) begin
                        wait_cnt <= 32'd0;
                        tx_kind  <= TX_RESET;
                        tx_sr    <= RESET_FRAME;
                        tx_bits  <= 6'd8;
                        ads_cs_n <= 1'b0;
                        state    <= ST_SEND;
                    end else begin
                        wait_cnt <= wait_cnt + 32'd1;
                    end
                end

                ST_SEND: begin
                    if (spi_tick) begin
                        if (!ads_sclk) begin
                            // SPI mode 1：上升沿改变 MOSI，器件在后续下降沿锁存。
                            ads_sclk <= 1'b1;
                            ads_mosi <= tx_sr[39];
                        end else begin
                            // 下降沿结束当前 bit；最后一位后释放 CS。
                            ads_sclk <= 1'b0;
                            if (tx_bits == 6'd1) begin
                                tx_bits  <= 6'd0;
                                ads_mosi <= 1'b0;
                                state    <= ST_SEND_HOLD;
                            end else begin
                                tx_bits <= tx_bits - 6'd1;
                                tx_sr   <= {tx_sr[38:0], 1'b0};
                            end
                        end
                    end
                end

                ST_RESYNC: begin
                    // CS 低 + 72 个 SCLK（wait_cnt 计数，sclk=wait_cnt[0] 方波）
                    // ⇒ 串行界面无论失步多深都回到已知同步点，然后走 RESET 链
                    ads_cs_n <= 1'b0;
                    ads_sclk <= wait_cnt[0];
                    ads_mosi <= 1'b0;
                    if (wait_cnt >= 32'd876) begin   // 876 clk = 73 个完整脉冲 @4.17M
                        ads_sclk   <= 1'b0;
                        ads_cs_n   <= 1'b1;
                        wait_cnt   <= 32'd0;
                        tx_kind    <= TX_RESET;
                        tx_sr      <= RESET_FRAME;
                        tx_bits    <= 6'd8;
                        ads_cs_n   <= 1'b0;
                        state      <= ST_SEND;
                    end else begin
                        wait_cnt <= wait_cnt + 32'd1;
                    end
                end

                ST_SEND_HOLD: begin
                    // 让最后一个下降沿完整结束后再释放 CS。
                    ads_cs_n <= 1'b1;
                    ads_sclk <= 1'b0;
                    ads_mosi <= 1'b0;
                    if (tx_kind == TX_RESET) begin
                        wait_cnt <= 32'd0;
                        state    <= ST_RESET;
                    end else if (tx_kind == TX_WREG) begin
                        tx_kind  <= TX_START;
                        wait_cnt <= 32'd0;
                        state    <= ST_START_GAP;
                    end else begin
                        watchdog_cnt <= 32'd0;
                        state        <= ST_RUN;
                    end
                end

                ST_RESET: begin
                    ads_cs_n <= 1'b1;
                    ads_sclk <= 1'b0;
                    ads_mosi <= 1'b0;
                    if (wait_cnt >= RESET_WAIT_CLKS - 1) begin
                        wait_cnt <= 32'd0;
                        tx_kind  <= TX_WREG;
                        tx_sr    <= WREG_FRAME;
                        tx_bits  <= 6'd40;
                        ads_cs_n <= 1'b0;
                        state    <= ST_SEND;
                    end else begin
                        wait_cnt <= wait_cnt + 32'd1;
                    end
                end

                ST_START_GAP: begin
                    // WREG 与 START 之间保持一个完整的 CS 高电平周期，
                    // 让 ADS1220 复位串行接口边界。
                    ads_cs_n <= 1'b1;
                    ads_sclk <= 1'b0;
                    ads_mosi <= 1'b0;
                    if (wait_cnt == 32'd0) begin
                        wait_cnt <= 32'd1;
                    end else begin
                        tx_sr    <= START_FRAME;
                        tx_bits  <= 6'd8;
                        ads_cs_n <= 1'b0;
                        state    <= ST_SEND;
                    end
                end

                ST_RUN: begin
                    ads_cs_n <= 1'b1;
                    ads_sclk <= 1'b0;
                    ads_mosi <= 1'b0;
                    if (drdy_fall) begin
                        rx_sr    <= 24'd0;
                        rx_bits  <= 6'd0;
                        ads_cs_n <= 1'b0;
                        state    <= ST_READ;
                    end
                    if (watchdog_cnt >= WATCHDOG_CLKS - 1) begin
                        // 自愈：DRDY 2ms 无沿 = ADC 跑飞（噪声/SPI 失步/掉配置，
                        // 2026-10-09 实机复现：静置或 FPGA 重配置后 fault 锁死）。
                        // 先 ST_RESYNC 打 72 个 SCLK 强制串行界面回同步（数据手册
                        // 失步恢复法，仅 RESET 命令救不回深度失步），再 RESET→
                        // WREG→START。adc_fault 期间保持（安全链可见），首个好帧
                        // 自清（ST_READ）。
                        adc_fault    <= 1'b1;
                        watchdog_cnt <= 32'd0;
                        spi_cnt      <= 8'd0;
                        ads_cs_n     <= 1'b0;
                        ads_mosi     <= 1'b0;
                        state        <= ST_RESYNC;
                    end else begin
                        watchdog_cnt <= watchdog_cnt + 32'd1;
                    end
                end

                ST_READ: begin
                    if (spi_tick) begin
                        if (!ads_sclk) begin
                            // 器件在上升沿推出当前数据位。
                            ads_sclk <= 1'b1;
                        end else begin
                            // SPI mode 1：FPGA 在下降沿采样 DOUT/DRDY。
                            ads_sclk <= 1'b0;
                            rx_sr    <= rx_final;
                            if (rx_bits == 6'd23) begin
                                adc_raw   <= (scaled_value > 17'd65472) ?
                                             16'd65472 : scaled_value[15:0];
                                adc_valid <= 1'b1;
                                watchdog_cnt <= 32'd0;
                                adc_fault <= 1'b0;   // V2 真实模式：链路恢复即撤销超时故障
                                state     <= ST_READ_HOLD;
                            end else begin
                                rx_bits <= rx_bits + 6'd1;
                            end
                        end
                    end
                end

                ST_READ_HOLD: begin
                    ads_cs_n <= 1'b1;
                    ads_sclk <= 1'b0;
                    ads_mosi <= 1'b0;
                    state    <= ST_RUN;
                end

                default: begin
                    state    <= ST_POWERUP;
                    ads_cs_n <= 1'b1;
                    ads_sclk <= 1'b0;
                    ads_mosi <= 1'b0;
                end
            endcase
        end
    end
endmodule
