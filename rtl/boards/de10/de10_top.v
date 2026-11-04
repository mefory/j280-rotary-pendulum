// ============================================================================
// de10_top.v — DE10-Standard 板级顶层（算法验证平台，非最终硬件）
//
// 用途：手上没有 J280 硬件时，用 DE10-Standard 验证**控制算法本身**正确。
// 控制逻辑与 J280 完全同一份源码（rtl/ctrl_core.v），本文件只替换板级外壳：
//
//   J280 外壳                          DE10-Standard 外壳
//   ---------------------------------  --------------------------------------------
//   3PA1030 并行 ADC（板载）     →     **板载 LTC2308**（2026-09-22 起），或 HIL 注入
//   正交编码器（板载端子）       →     GPIO 上的 A/B 相（可选），或 HIL 注入
//   板载 4 按键 + 2 拨码 + 4 LED →     板载 KEY[3:0] + SW[9:0] + LEDR[9:0]
//   UART 经 CH340 连 FPGA        →     **GPIO 外接 USB-TTL**（板载 USB-UART 只接 HPS！）
//   TB6612 → 电机                →     GPIO 上的 PWM/DIR（仅示波器观察，需驱动板才能带电机）
//
// 【关键硬件事实】DE10-Standard 的板载 USB-UART 是 FT232R，只连 HPS 的
// HPS_UART_RX/TX，**不经过 FPGA fabric**。因此要用 UART 必须外接 USB-TTL 模块
// （CH340/CP2102/FT232 皆可）接到 GPIO 排针的 UART 引脚。对上位机而言它就是一个
// 普通串口，用法与 J280 完全一致。
//
// 两种工作模式（SW[9] 静态选择）：
//   SW[9]=1（HIL 锁步，推荐）：上位机跑被控对象模型，经 STEP 命令逐拍送传感器
//       并回读 duty。无需任何模拟器件，可验证标定→回绕→差分测速→M/T 测速→
//       模式机→起摆/LQR/位置环→死区→限幅整条链。
//   SW[9]=0：走真实传感——板载 LTC2308 读摆杆角位移传感器（0~3.3V→ADC_IN0）、
//   GPIO 读电机编码器 A/B（把 J280 机械部分接到 DE10 时用）。
//
// 构建：Quartus 工程见 de10/ 目录，引脚见 de10/DE10_Standard.qsf。
// ============================================================================
`include "j280_defs.vh"
module de10_top (
`ifdef DE10_SOC
    // ------- HPS 专用引脚（位置由 HPS 硬核决定，qsf 无需约束） -------
    // 清单复制自 Terasic DE10_Standard_FB 工程，配合 soc_system QSys 网表
    output   [14: 0]   HPS_DDR3_ADDR,
    output   [ 2: 0]   HPS_DDR3_BA,
    output             HPS_DDR3_CAS_N,
    output             HPS_DDR3_CKE,
    output             HPS_DDR3_CK_N,
    output             HPS_DDR3_CK_P,
    output             HPS_DDR3_CS_N,
    output   [ 3: 0]   HPS_DDR3_DM,
    inout    [31: 0]   HPS_DDR3_DQ,
    inout    [ 3: 0]   HPS_DDR3_DQS_N,
    inout    [ 3: 0]   HPS_DDR3_DQS_P,
    output             HPS_DDR3_ODT,
    output             HPS_DDR3_RAS_N,
    output             HPS_DDR3_RESET_N,
    input              HPS_DDR3_RZQ,
    output             HPS_DDR3_WE_N,
    output             HPS_ENET_GTX_CLK,
    inout              HPS_ENET_INT_N,
    output             HPS_ENET_MDC,
    inout              HPS_ENET_MDIO,
    input              HPS_ENET_RX_CLK,
    input    [ 3: 0]   HPS_ENET_RX_DATA,
    input              HPS_ENET_RX_DV,
    output   [ 3: 0]   HPS_ENET_TX_DATA,
    output             HPS_ENET_TX_EN,
    inout    [ 3: 0]   HPS_FLASH_DATA,
    output             HPS_FLASH_DCLK,
    output             HPS_FLASH_NCSO,
    inout              HPS_GSENSOR_INT,
    inout              HPS_I2C1_SCLK,
    inout              HPS_I2C1_SDAT,
    inout              HPS_I2C2_SCLK,
    inout              HPS_I2C2_SDAT,
    inout              HPS_I2C_CONTROL,
    inout              HPS_KEY,
    inout              HPS_LCM_BK,
    inout              HPS_LCM_D_C,
    inout              HPS_LCM_RST_N,
    output             HPS_LCM_SPIM_CLK,
    output             HPS_LCM_SPIM_MOSI,
    input              HPS_LCM_SPIM_MISO,
    output             HPS_LCM_SPIM_SS,
    inout              HPS_LED,
    inout              HPS_LTC_GPIO,
    output             HPS_SD_CLK,
    inout              HPS_SD_CMD,
    inout    [ 3: 0]   HPS_SD_DATA,
    output             HPS_SPIM_CLK,
    input              HPS_SPIM_MISO,
    inout              HPS_SPIM_MOSI,
    output             HPS_SPIM_SS,
    output             HPS_UART_TX,
    input              HPS_UART_RX,
    inout              HPS_CONV_USB_N,
    inout    [31: 0]   HPS_USB_DATA,
    output             HPS_USB_STP,
    input              HPS_USB_DIR,
    input              HPS_USB_NXT,
    input              HPS_USB_CLKOUT,
`endif
    input  wire        CLOCK_50,      // 50MHz 板载晶振

    // 人机（板载）
    input  wire [3:0]  KEY,           // KEY[0..3]，低有效（按下=0）
    input  wire [9:0]  SW,            // SW[0..9]，高有效
    output wire [9:0]  LEDR,          // 高电平点亮
    output wire [6:0]  HEX0,HEX1,HEX2,HEX3,HEX4,HEX5,  // 共阳极，低电平点亮

    // 串口（GPIO 外接 USB-TTL；板载 USB-UART 仅供 HPS）
    output wire        uart_tx,
    input  wire        uart_rx,

    // 板载 LTC2308 ADC（8ch 12bit SPI，模拟输入经板上 10-pin ADC 排针）：
    //   2026-09-22 起，摆杆角位移传感器（WDD35D4，0~3.3V）改走此路，
    //   原外接 3PA1030 并行 ADC 方案退役（排针 7-10、13-22、33 空出）
    output wire        adc_convst,
    output wire        adc_sck,
    output wire        adc_sdi,
    input  wire        adc_sdo,

    // 正交编码器（GPIO，可选）
    input  wire        enc_a,
    input  wire        enc_b,

    // 电机输出（GPIO，接 TB6612 驱动板）：标准三线
    //   AIN1/AIN2 方向真值表（10/01 正反转、00 coast、11 刹车不用）
    //   PWMA   载波斩波（20kHz，2500 级占空比）
    // 与 J280 板的 AN1/AN2/PWMA 行为逐拍一致（同一 motor_pwm 实例）。
    output wire        mot_ain1,
    output wire        mot_ain2,
    output wire        mot_pwma,

    // 调试：接逻辑分析仪观察内部节拍与模式
    output wire        dbg_tick,
    output wire [7:0]  dbg_state
);
    // ---------------- 复位与模式选择 ----------------
    wire rst_n   = KEY[0];    // KEY[0] 按住 = 复位
    wire hil_use = SW[9];     // 1 = HIL 锁步；0 = 使用真实传感（板载 LTC2308 + 编码器）
    // （历史注）SW7 曾作"未接 3PA1030"OTR 门控——2026-09-19 实机曾因悬空
    // adc_otr 采到高电平锁死 SAFE。现 ADC 换板载 LTC2308（无越界输出，且
    // 0~3.3V 传感必在 0~4.096V 量程内），OTR 整路退役，SW7 空闲。

    // ---------------- 人机映射 ----------------
    // 原 J280：SW1=启动 SW2=模式 SW3=标定 SW4=急停（均低有效按键）
    // DE10  ：KEY1=启动 KEY2=模式 KEY3=急停；标定改用 SW[8]（拨上=按下），
    //         因板载只有 4 个按键而 KEY0 已作复位。ui 内部 20ms 消抖并产生
    //         单拍脉冲，拨码的稳态电平同样适用（拨上=按一次，拨下再拨上=再按）。
    wire [3:0] key_phys = {KEY[3], ~SW[8], KEY[2], KEY[1]};
    wire [3:0] virt_key_n;   // HPS 虚拟键（低有效），经 fb_channel 60ms 保持
    wire [3:0] key_sw = key_phys & virt_key_n;
    wire [1:0] dip_sw = SW[1:0];

    // ---------------- 传感前端（板载 LTC2308 + GPIO 编码器） ----------------
    // HIL 模式下这些输入不被采用，但保持实例化以便同一比特流两用。
    wire [15:0] alpha_avg;
    wire        otr_event;
    ltc2308_if u_adc (
        .clk(CLOCK_50), .rst_n(rst_n),
        .adc_convst(adc_convst), .adc_sck(adc_sck),
        .adc_sdi(adc_sdi), .adc_sdo(adc_sdo),
        .alpha_avg(alpha_avg), .otr_event(otr_event)   // 恒 0
    );

    wire signed [31:0] enc_pos;
    qenc4x #(.POL_FLIP(`POL_ENC_FLIP)) u_enc (
        .clk(CLOCK_50), .rst_n(rst_n), .enc_a(enc_a), .enc_b(enc_b), .pos(enc_pos)
    );

    // ---------------- 控制核心（与 J280 同一份源码） ----------------
    wire signed [15:0] duty_out;
    wire               motor_enable;
    wire [2:0]         mode;
    wire signed [15:0] alpha, alpha_dot;
    wire signed [31:0] theta_c01;
    wire signed [15:0] theta_dot;
    wire               core_tick;
    wire               io_test_enable;
    wire signed [15:0] io_test_duty;

    // HIL 模式下真实 ADC 整路旁路；LTC2308 无越界指示线，0~3.3V 传感输入
    // 也必在 0~4.096V 量程内，故 OTR 保护恒为无越界（原 3PA1030 OTR 门控退役）。
    wire otr_core = 1'b0;

    ctrl_core u_core (
        .clk(CLOCK_50), .rst_n(rst_n),
        .adc_raw(alpha_avg), .otr_fault(otr_core), .enc_pos(enc_pos),
        .mt_angle14(14'd0), .th_src_mt(1'b0),   // 无电机侧 MT6701：恒霍尔源
        .mt6_frames(16'd0), .mt6_good(16'd0), .mt6_crc_err(16'd0),
        .mt6_mg_err(16'd0), .mt6_mg_code(3'd0), .mt6_angle_raw(14'd0),
        .mt6_fault(1'b0), .mt6_cause(2'd0), .mt6_max_no_good(16'd0),
        .mt68_frames(16'd0), .mt68_good(16'd0), .mt68_crc_err(16'd0),
        .mt68_status_err(16'd0), .mt68_status(3'd0), .mt68_angle_raw(21'd0),
        .mt68_fault(1'b0),
        .hil_use(hil_use),
        .key_sw(key_sw), .dip_sw(dip_sw), .led(LEDR[3:0]),
        .uart_rx(uart_rx), .uart_tx(uart_tx),
        .duty_out(duty_out), .motor_enable(motor_enable), .mode(mode),
        .alpha(alpha), .alpha_dot(alpha_dot), .theta_c01(theta_c01), .theta_dot(theta_dot),
        .io_test_enable(io_test_enable), .io_test_duty(io_test_duty),
        .tick_out(core_tick), .fault(fault_w)
    );

    // ---------------- HPS 调试通道（fabric↔Linux 经 QSys PIO） ----------------
    wire [9:0]  hps_led_pio, hps_dipsw_pio;
    wire [3:0]  hps_button_pio;
    wire        fault_w;

    fb_channel u_fbch (
        .clk(CLOCK_50), .rst_n(rst_n), .tick(core_tick),
        .alpha(alpha), .alpha_dot(alpha_dot), .theta(theta_c01),
        .theta_dot(theta_dot), .duty(duty_out), .mode(mode),
        .fault(fault_w), .hil_use(hil_use), .run_en(motor_enable),
`ifdef DE10_SOC
        .led_pio(hps_led_pio),
`else
        .led_pio(10'd0),
`endif
        .dipsw_pio(hps_dipsw_pio),
        .button_pio(hps_button_pio),
        .virt_key_n(virt_key_n)
    );

`ifdef DE10_SOC
    // Terasic DE10_Standard_FB QSys 生成系统（HPS + lw 桥 + PIO 通道）
    // 原始工程见 SystemCD；综合网表经 gcore CDN 从 Insper 镜像取得
    soc_system u_hps (
        .clk_clk(CLOCK_50),
        .reset_reset_n(1'b1),
        .memory_mem_a(HPS_DDR3_ADDR),
        .memory_mem_ba(HPS_DDR3_BA),
        .memory_mem_ck(HPS_DDR3_CK_P),
        .memory_mem_ck_n(HPS_DDR3_CK_N),
        .memory_mem_cke(HPS_DDR3_CKE),
        .memory_mem_cs_n(HPS_DDR3_CS_N),
        .memory_mem_ras_n(HPS_DDR3_RAS_N),
        .memory_mem_cas_n(HPS_DDR3_CAS_N),
        .memory_mem_we_n(HPS_DDR3_WE_N),
        .memory_mem_reset_n(HPS_DDR3_RESET_N),
        .memory_mem_dq(HPS_DDR3_DQ),
        .memory_mem_dqs(HPS_DDR3_DQS_P),
        .memory_mem_dqs_n(HPS_DDR3_DQS_N),
        .memory_mem_odt(HPS_DDR3_ODT),
        .memory_mem_dm(HPS_DDR3_DM),
        .memory_oct_rzqin(HPS_DDR3_RZQ),
        .hps_0_hps_io_hps_io_emac1_inst_TX_CLK(HPS_ENET_GTX_CLK),
        .hps_0_hps_io_hps_io_emac1_inst_TXD0(HPS_ENET_TX_DATA[0]),
        .hps_0_hps_io_hps_io_emac1_inst_TXD1(HPS_ENET_TX_DATA[1]),
        .hps_0_hps_io_hps_io_emac1_inst_TXD2(HPS_ENET_TX_DATA[2]),
        .hps_0_hps_io_hps_io_emac1_inst_TXD3(HPS_ENET_TX_DATA[3]),
        .hps_0_hps_io_hps_io_emac1_inst_RXD0(HPS_ENET_RX_DATA[0]),
        .hps_0_hps_io_hps_io_emac1_inst_MDIO(HPS_ENET_MDIO),
        .hps_0_hps_io_hps_io_emac1_inst_MDC(HPS_ENET_MDC),
        .hps_0_hps_io_hps_io_emac1_inst_RX_CTL(HPS_ENET_RX_DV),
        .hps_0_hps_io_hps_io_emac1_inst_TX_CTL(HPS_ENET_TX_EN),
        .hps_0_hps_io_hps_io_emac1_inst_RX_CLK(HPS_ENET_RX_CLK),
        .hps_0_hps_io_hps_io_emac1_inst_RXD1(HPS_ENET_RX_DATA[1]),
        .hps_0_hps_io_hps_io_emac1_inst_RXD2(HPS_ENET_RX_DATA[2]),
        .hps_0_hps_io_hps_io_emac1_inst_RXD3(HPS_ENET_RX_DATA[3]),
        .hps_0_hps_io_hps_io_qspi_inst_IO0(HPS_FLASH_DATA[0]),
        .hps_0_hps_io_hps_io_qspi_inst_IO1(HPS_FLASH_DATA[1]),
        .hps_0_hps_io_hps_io_qspi_inst_IO2(HPS_FLASH_DATA[2]),
        .hps_0_hps_io_hps_io_qspi_inst_IO3(HPS_FLASH_DATA[3]),
        .hps_0_hps_io_hps_io_qspi_inst_SS0(HPS_FLASH_NCSO),
        .hps_0_hps_io_hps_io_qspi_inst_CLK(HPS_FLASH_DCLK),
        .hps_0_hps_io_hps_io_sdio_inst_CMD(HPS_SD_CMD),
        .hps_0_hps_io_hps_io_sdio_inst_D0(HPS_SD_DATA[0]),
        .hps_0_hps_io_hps_io_sdio_inst_D1(HPS_SD_DATA[1]),
        .hps_0_hps_io_hps_io_sdio_inst_CLK(HPS_SD_CLK),
        .hps_0_hps_io_hps_io_sdio_inst_D2(HPS_SD_DATA[2]),
        .hps_0_hps_io_hps_io_sdio_inst_D3(HPS_SD_DATA[3]),
        .hps_0_hps_io_hps_io_usb1_inst_D0(HPS_USB_DATA[0]),
        .hps_0_hps_io_hps_io_usb1_inst_D1(HPS_USB_DATA[1]),
        .hps_0_hps_io_hps_io_usb1_inst_D2(HPS_USB_DATA[2]),
        .hps_0_hps_io_hps_io_usb1_inst_D3(HPS_USB_DATA[3]),
        .hps_0_hps_io_hps_io_usb1_inst_D4(HPS_USB_DATA[4]),
        .hps_0_hps_io_hps_io_usb1_inst_D5(HPS_USB_DATA[5]),
        .hps_0_hps_io_hps_io_usb1_inst_D6(HPS_USB_DATA[6]),
        .hps_0_hps_io_hps_io_usb1_inst_D7(HPS_USB_DATA[7]),
        .hps_0_hps_io_hps_io_usb1_inst_CLK(HPS_USB_CLKOUT),
        .hps_0_hps_io_hps_io_usb1_inst_STP(HPS_USB_STP),
        .hps_0_hps_io_hps_io_usb1_inst_DIR(HPS_USB_DIR),
        .hps_0_hps_io_hps_io_usb1_inst_NXT(HPS_USB_NXT),
        .hps_0_hps_io_hps_io_spim0_inst_CLK(HPS_LCM_SPIM_CLK),
        .hps_0_hps_io_hps_io_spim0_inst_MOSI(HPS_LCM_SPIM_MOSI),
        .hps_0_hps_io_hps_io_spim0_inst_MISO(HPS_LCM_SPIM_MISO),
        .hps_0_hps_io_hps_io_spim0_inst_SS0(HPS_LCM_SPIM_SS),
        .hps_0_hps_io_hps_io_spim1_inst_CLK(HPS_SPIM_CLK),
        .hps_0_hps_io_hps_io_spim1_inst_MOSI(HPS_SPIM_MOSI),
        .hps_0_hps_io_hps_io_spim1_inst_MISO(HPS_SPIM_MISO),
        .hps_0_hps_io_hps_io_spim1_inst_SS0(HPS_SPIM_SS),
        .hps_0_hps_io_hps_io_uart0_inst_RX(HPS_UART_RX),
        .hps_0_hps_io_hps_io_uart0_inst_TX(HPS_UART_TX),
        .hps_0_hps_io_hps_io_i2c0_inst_SDA(HPS_I2C1_SDAT),
        .hps_0_hps_io_hps_io_i2c0_inst_SCL(HPS_I2C1_SCLK),
        .hps_0_hps_io_hps_io_i2c1_inst_SDA(HPS_I2C2_SDAT),
        .hps_0_hps_io_hps_io_i2c1_inst_SCL(HPS_I2C2_SCLK),
        .hps_0_hps_io_hps_io_gpio_inst_GPIO09(HPS_CONV_USB_N),
        .hps_0_hps_io_hps_io_gpio_inst_GPIO35(HPS_ENET_INT_N),
        .hps_0_hps_io_hps_io_gpio_inst_GPIO37(HPS_LCM_BK),
        .hps_0_hps_io_hps_io_gpio_inst_GPIO40(HPS_LTC_GPIO),
        .hps_0_hps_io_hps_io_gpio_inst_GPIO41(HPS_LCM_D_C),
        .hps_0_hps_io_hps_io_gpio_inst_GPIO44(HPS_LCM_RST_N),
        .hps_0_hps_io_hps_io_gpio_inst_GPIO48(HPS_I2C_CONTROL),
        .hps_0_hps_io_hps_io_gpio_inst_GPIO53(HPS_LED),
        .hps_0_hps_io_hps_io_gpio_inst_GPIO54(HPS_KEY),
        .hps_0_hps_io_hps_io_gpio_inst_GPIO61(HPS_GSENSOR_INT),
        .led_pio_external_connection_export(hps_led_pio),
        .dipsw_pio_external_connection_export(hps_dipsw_pio),
        .button_pio_external_connection_export(hps_button_pio),
        .hps_0_h2f_reset_reset_n(),
        .hps_0_f2h_cold_reset_req_reset_n(1'b1),
        .hps_0_f2h_debug_reset_req_reset_n(1'b1),
        .hps_0_f2h_stm_hw_events_stm_hwevents(28'd0),
        .hps_0_f2h_warm_reset_req_reset_n(1'b1),
        .alt_vip_itc_0_clocked_video_vid_clk(CLOCK_50),
        .alt_vip_itc_0_clocked_video_vid_data(),
        .alt_vip_itc_0_clocked_video_underflow(),
        .alt_vip_itc_0_clocked_video_vid_datavalid(),
        .alt_vip_itc_0_clocked_video_vid_v_sync(),
        .alt_vip_itc_0_clocked_video_vid_h_sync(),
        .alt_vip_itc_0_clocked_video_vid_f(),
        .alt_vip_itc_0_clocked_video_vid_h(),
        .alt_vip_itc_0_clocked_video_vid_v(),
        .vga_stream_clk(CLOCK_50)
    );
`endif

    // ---------------- 电机输出 ----------------
    // 复用 rtl/motor_pwm.v，三线直出（与 J280 板同构）：AIN1/AIN2 方向 + PWMA 斩波。
    // I/O 联调只允许 SAFE/IDLE 且急停释放；其他模式仍由闭环控制接管。
    wire io_test_safe = io_test_enable &&
                        ((mode == `M_SAFE) || (mode == `M_IDLE)) &&
                        key_sw[3];
    wire motor_run = io_test_safe ? 1'b1 : motor_enable;
    wire signed [15:0] motor_duty = io_test_safe ? io_test_duty : duty_out;
    motor_pwm #(.CARRIER(`PWM_CARRIER), .BRAKE_TH(0)) u_mot (
        .clk(CLOCK_50), .rst_n(rst_n),
        .run_en(motor_run), .brake_en(1'b0), .duty(motor_duty),
        .in1(mot_ain1), .in2(mot_ain2), .pwm(mot_pwma)
    );

    // ---------------- 板载数码管（共阳极，低电平点亮） ----------------
    // HEX3..HEX0：控制器**实际使用的** alpha 原始十六进制值（s16, 0.01°）。
    //   直接读 4 位十六进制可无损核对传感链数值，不需要除法器。
    // HEX4：模式号；HEX5：H = HIL 锁步 / r = 真实传感。
    hexdigit u_h0 (.value(alpha[3:0]),   .seg(HEX0));
    hexdigit u_h1 (.value(alpha[7:4]),   .seg(HEX1));
    hexdigit u_h2 (.value(alpha[11:8]),  .seg(HEX2));
    hexdigit u_h3 (.value(alpha[15:12]), .seg(HEX3));
    hexdigit u_h4 (.value({1'b0,mode}),  .seg(HEX4));
    hexdigit u_h5 (.value(hil_use ? 4'hA : 4'hB), .seg(HEX5)); // A='H' 式标记, B

    // ---------------- 状态指示与调试引出 ----------------
    assign LEDR[9:4] = 6'd0;
    assign dbg_tick  = core_tick;
    assign dbg_state = {5'd0, mode};
endmodule

// ----------------------------------------------------------------------------
// hexdigit — 4bit → 共阳极七段段码
// DE10-Standard 的 HEX 为共阳极（低电平点亮），故内部按共阴编码后取反输出。
// 段序 {g,f,e,d,c,b,a}。
// ----------------------------------------------------------------------------
module hexdigit (
    input  wire [3:0] value,
    output wire [6:0] seg
);
    reg [6:0] seg_n;   // 共阴段码（高电平点亮）
    always @* begin
        case (value)
            4'h0: seg_n = 7'b0111111;
            4'h1: seg_n = 7'b0000110;
            4'h2: seg_n = 7'b1011011;
            4'h3: seg_n = 7'b1001111;
            4'h4: seg_n = 7'b1100110;
            4'h5: seg_n = 7'b1101101;
            4'h6: seg_n = 7'b1111101;
            4'h7: seg_n = 7'b0000111;
            4'h8: seg_n = 7'b1111111;
            4'h9: seg_n = 7'b1101111;
            4'hA: seg_n = 7'b1110111;
            4'hB: seg_n = 7'b1111100;
            4'hC: seg_n = 7'b0111001;
            4'hD: seg_n = 7'b1011110;
            4'hE: seg_n = 7'b1111001;
            4'hF: seg_n = 7'b1110001;
            default: seg_n = 7'b0000000;
        endcase
    end
    assign seg = ~seg_n;   // 共阳极：低电平点亮
endmodule
