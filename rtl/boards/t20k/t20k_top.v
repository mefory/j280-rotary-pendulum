// ============================================================================
// t20k_top.v — Tang Primer 20K 板级外壳（GW2A-LV18PG256C8/I7，算法验证平台）
//
// 定位：J280 缺货期间的低成本验证/演示板。与 DE10 外壳同一策略——板级无关的
// 控制核心（rtl/ctrl_core.v）原样复用，本文件只做板级接线：
//
//   J280 外壳                        Tang Primer 20K 外壳
//   -------------------------------  ------------------------------------------
//   50MHz 板载晶振                   → 27MHz 板载晶振(H11) + rPLL 倍频到约 50.143MHz
//                                      （27/7×13=50.143，VCO=802.29MHz，PFD=3.857MHz）
//   SW1..SW4 按键                    → Dock 用户按键 C7=启动 D7=模式 T2=标定 T3=急停
//                                      T10=复位（照 HDMI 例程习惯）
//   拨码 DIP[1:0]                    → 固定 2'b00（POS 目标默认 ±30°/±60°）
//   4×LED                            → Dock LED[3:0]（N16/N14/L14/L16），LED5/4=故障/节拍
//   CH340 USB-UART 直连 fabric       → 外接 3.3V CH340 上 PMOD2 底行：
//                                      uart_tx=B12→模块RXD、uart_rx=C12←模块TXD，
//                                      1Mbaud 与 J280/DE10 完全一致；板载 BL702 桥
//                                      （Type-C）专职 JTAG 烧录，不再走串口
//   编码器 A/B + TB6612 AIN1/AIN2/PWMA/STBY → PMOD1（T11/P11/T12/R11/M14/J14）
//                                      （MT6835 SPI 使用 PMOD2 其余脚）
//
// V2 工作模式（2026-10-09 定案）：hil_use=0 恒定——真实传感闭环：
//   位置=编码器（PMOD1 T11/P11）、电机=TB6612（PMOD1 T12/R11/M14，STBY=J14
//   常使能）。传感故障经 otr_fault 进安全链。上电即 1kHz 自由节拍，
//   上位机选「实板」后端操作；HIL 锁步验收仍由 DE10（SW[9]）承担。
// V3 传感替换（2026-10-15，experiment/mt6701-alpha 分支）：α 角改 MT6701
//   磁编码器 SSI，全 360° 直读替代 WDD35D4+ADS1220 的 345° 口径
//   （angle14<<2 + ALPHA_SCALE_Q16=36000）。
// V4 双编码器方案（2026-10-16，experiment/mt6835-gmr-observer 分支）：α 改
//   MT6835 SPI mode3（PMOD2 D14/E15/B14/A15 四线，21 位 OP_ANGLE 帧，CRC8+
//   状态健康链经 otr_fault 进安全链，语义同 V2/V3）；θ 侧 GMR 500 线与观测器
//   （OBS_USE）见 docs/MT6835_GMR_双编码器与观测器方案.md，待后续提交。
//   全 360° 直读口径（65536↔360.00°）承 V3。
// V5 电机侧第二 θ 源（2026-10-19，experiment/mt6701-theta 分支）：硬件方案更新
//   ——GMR 500 线取消（机械结构限制），电机沿用 13 线霍尔（默认 θ 源不动），
//   电机尾同轴加装 MT6701 SSI（PMOD2 B13=CSN + PMOD1 M15=SCK/J16=DO，径向
//   磁铁叠霍尔磁铁，干扰风险→霍尔链保留兜底）。MT 源选中且 MT6701 故障 →
//   otr_fault 进安全链（θ 冻结在闭环里比停机更危险，臂会带电缠绕）。
//   观测器/摆侧 MT6835 承 V4。
// V5.1 θ 源定案（2026-10-19 晚，用户拍板）：**MT6701 独占控制源 + 霍尔降级监视**。
//   依据=实机 MT 三项全胜（真尺度 0.006% vs 霍尔虚大 2.25%、分辨率 322×、
//   θ̇ 群延迟 3ms vs 5ms）；互补滤波融合被否决（尺度差烤进环里+失败语义污染，
//   有原则的融合归宿是观测器 obs_core）。TH_SRC_FORCE_MT=1（默认）恒选 MT 源，
//   A14 失效保留（置 0 恢复 V5 跳线语义供 A/B 对照）；MT6701 故障从此无条件
//   进安全链。霍尔链旁路运行仅供遥测与诊断页交叉监视（MT 磁铁打滑等驱动
//   自检盲区由双源数值对比暴露）。
// ============================================================================
`include "j280_defs.vh"
module t20k_top #(
    // V5.1 θ 源保险丝：1（默认）=MT6701 独占（A14 失效保留）；0=恢复 V5 跳线
    // 语义（悬空=霍尔/拉地=MT）供 A/B 对照实验。
    parameter TH_SRC_FORCE_MT = 1'b1,
    // OTR 宽限毫秒数（300ms 默认；仿真 override 缩短加速，语义不变）
    parameter [9:0] OTR_GRACE_MS_P = 10'd300
)(    input  wire        clk27,        // H11 板载 27MHz
    input  wire [4:0]  key_n,        // 低有效：{T3 急停, T2 标定, D7 模式, C7 启动, T10 复位}
    output wire [5:0]  led,          // Dock LED：{N16,N14,L14,L16}=控制器, {C13,A13}=诊断
    output wire        uart_tx,      // PMOD2 B12 → 外接 CH340 RXD（1Mbaud）
    input  wire        uart_rx,      // PMOD2 C12 ← 外接 CH340 TXD
    output wire        spi_csn,      // PMOD2 D14 → MT6835 CSN
    output wire        spi_sck,      // PMOD2 E15 → MT6835 SCK（空闲高，mode3）
    output wire        spi_mosi,     // PMOD2 B14 → MT6835 MOSI（OP_ANGLE 命令）
    input  wire        spi_miso,     // PMOD2 A15 ← MT6835 MISO（帧间 Hi-Z，上拉读 1）
    output wire        mt6701_csn,   // PMOD2 B13 → 电机侧 MT6701 CSN（SSI）
    output wire        mt6701_sck,   // PMOD1 M15 → MT6701 CLK（空闲高，SSI mode2）
    input  wire        mt6701_do,    // PMOD1 J16 ← MT6701 DO（内部上拉）
    input  wire        th_src_n,     // PMOD2 A14 ← V5 θ 源选择脚（V5.1 起失效保留：
                                     //   TH_SRC_FORCE_MT=1 时电平被忽略，勿再接跳线）
    // 2026-10-22：enc_a/enc_b（PMOD1 T11/P11 霍尔正交编码器）端口已删除——
    // 用户把原霍尔编码器物理拆除，θ 只剩电机侧 MT6701 一条链（TH_SRC_MT_ONLY=1）。
    // 要恢复双源：把两个端口与 cons/t20k.cst 的 enc_a/enc_b 约束加回，并把
    // TH_SRC_MT_ONLY 置 0。
    output wire        mot_ain1,     // PMOD1 T12 -> TB6612 AIN1
    output wire        mot_ain2,     // PMOD1 R11 -> TB6612 AIN2
    output wire        mot_pwma,     // PMOD1 M14 -> TB6612 PWMA
    output wire        mot_stby      // PMOD1 J14 -> TB6612 STBY（常使能）
);
    // ---------------- 时钟：27MHz 晶振直驱（2026-10-27 降档）----------------
    // 原 rPLL 倍频 50.143MHz 已移除：约束恰骑在实测 Fmax 分布（49.17-51.62MHz）
    // 中间，每次 PnR 都是掷骰；27MHz 直驱留约 85% 时序余量。控制环全在 1kHz
    // tick 域，主频只影响计数器派生常数（见 j280_defs.vh 的 CLK_HZ 体系）。
    // 保留 clk50 网名以免全文件改动：它现在就是 27MHz。
    wire clk50 = clk27;

    // PLL lock 复位改为上电复位计数器：16bit @27MHz ≈ 2.43ms（与原 lock 同量级）。
    // T10 按住时计数器清零、rst_n 保持低，释放语义与原来一致。
    reg [15:0] por_cnt;
    always @(posedge clk50 or negedge key_n[4]) begin
        if (!key_n[4])      por_cnt <= 16'd0;
        else if (!por_cnt[15]) por_cnt <= por_cnt + 16'd1;
    end
    wire rst_n = por_cnt[15] & key_n[4];   // T10 = 复位

    // ---------------- 人机映射 ----------------
    // J280：SW1=启动 SW2=模式 SW3=标定 SW4=急停（低有效）——同序映射到 Dock 按键
    wire [3:0] key_sw = key_n[3:0];         // {T3 急停, T2 标定, D7 模式, C7 启动}
    wire [1:0] dip_sw = 2'b00;              // V1 无拨码：POS 目标默认 ±30°/±60°

    // ---------------- MT6835 SPI 传感接口（V4：控制核心真实输入） ----------------
    wire [15:0] mt_raw;
    wire [20:0] mt_alpha21;
    wire        mt_valid;
    wire        mt_fault;
    wire [15:0] mt68_frames, mt68_good, mt68_crc_err, mt68_status_err;
    wire [2:0]  mt68_status;
    wire [20:0] mt68_angle_raw;
    mt6835_spi_if #(
        .BAD_FRAMES_LATCH(4'd8)      // 2026-10-19 加固：稳摆 duty 近零/换向尖峰的
                                      // 短促 EMI 不再锁停（三连 1s 内锁存实测）；
                                      // 真断线仍由 2ms 好帧看门狗兜底
    ) u_mt6835 (
        .clk(clk50), .rst_n(rst_n),
        .spi_csn(spi_csn), .spi_sck(spi_sck),
        .spi_mosi(spi_mosi), .spi_miso(spi_miso),
        .adc_raw(mt_raw), .alpha21(mt_alpha21),
        .adc_valid(mt_valid), .adc_fault(mt_fault),
        .st_frames(mt68_frames), .st_good(mt68_good),
        .st_crc_err(mt68_crc_err), .st_status_err(mt68_status_err),
        .st_status(mt68_status), .st_angle_raw(mt68_angle_raw)
    );

    // 真实角度源：0..65535 ↔ 360°（angle21>>5）；换算口径在 ctrl_core
    // （ALPHA_SCALE_Q16=36000），angle_proc 标定/回绕原样。alpha21 旁路 21 位
    // 原值（1 LSB=0.000172°）留给观测器，当前未消费计入 unused_ok。
    wire [15:0] alpha_avg = mt_raw;
    // V4：CRC8/状态位连续坏帧或 2ms 无好帧 → 安全链 otr_fault（失联即锁 SAFE）。
    // V5.1：MT6701 独占 θ 源（TH_SRC_FORCE_MT=1 恒 th_src_mt=1）——其故障**无条件**
    // 进安全链：θ 冻结在闭环里比停机危险（臂带电缠绕）。
    wire        mt6701_fault_w;
    wire        th_src_mt;
    // 2026-10-22：SWING 期间 MT6701 故障不进安全链。
    // 依据：用户现场确认 MT6701 的径向磁铁就装在电机霍尔磁铁旁边，电机大电流加速时
    // 磁场互相干扰，MT6701 会报失追踪/场差异常。而快速甩摆的本质就是满舵大电流，
    // 两者互斥——今天两版实测都是甩到 3.0-3.6s 被安全链锁 SAFE（fault=3），甩摆永远
    // 走不完。SWING 内 θ 只用于行程计时与保持相（保持相权限已夹 4000 且有停滞看门狗），
    // 不构成闭环位置回路，冻结/丢弃都不会造成危险驱动；α 侧 MT6835（mt_fault）仍无条件
    // 进安全链，捕获/掉摆判定不受影响。BALANCE/POS/IDLE 语义完全不变。
    wire        swing_drive = (mode == 3'd1);   // `M_SWING
    wire        otr_event = mt_fault | (th_src_mt & mt6701_fault_w & ~swing_drive);
    // ---- OTR 宽限保持（自 audit_20261030_143029 移植，2026-10-31 三冠融合版）----
    // 坏帧期间接口保持末好值、好帧自清——"数据基本正确"按构造成立；旧策略
    // OT 第一拍锁 SAFE 毁掉长稳摆。持续 ≥300ms 才锁（真断线缠绕保护保留）。
    // 门控必须用毫秒拍：第一版直接数 clk50（300×20ns=6µs）形同没有宽限，
    // 板上 fall 瞬间的 ms 级 OT 脉冲照样锁 SAFE（pa_round1/3 复现）。
    // ★本树修正：源快照分频写死 49999（按 50MHz 设计），而 10-27 起实际
    // 27MHz——照搬会得到 555ms 假宽限。改为按 `J280_CLK_HZ 推导，任何主频
    // 下都是真 300ms（j280_defs.vh 已在文件头 include，构建脚本补丁 27MHz）。
    // 连续 300 个 ms 采样为高才放行；任一 ms 采样为低即归零。
    localparam [15:0] OTR_MS_DIV   = (`J280_CLK_HZ/1000) - 1;  // 27MHz→26999 / 50MHz→49999
    reg [15:0] grace_div;
    reg [9:0]  grace_ms;
    reg        otr_grace_full;
    always @(posedge clk50 or negedge rst_n) begin
        if (!rst_n) begin
            grace_div <= 16'd0; grace_ms <= 10'd0; otr_grace_full <= 1'b0;
        end else if (grace_div == OTR_MS_DIV) begin
            grace_div <= 16'd0;
            if (!otr_event) begin
                grace_ms <= 10'd0;
                otr_grace_full <= 1'b0;
            end else if (!otr_grace_full) begin
                if (grace_ms == OTR_GRACE_MS_P - 10'd1)
                    otr_grace_full <= 1'b1;
                else
                    grace_ms <= grace_ms + 10'd1;
            end
        end else begin
            grace_div <= grace_div + 16'd1;
        end
    end
    wire otr_for_safety = otr_grace_full;
    wire signed [31:0] enc_pos_hw;
    // 2026-10-22：霍尔编码器已拆除，qenc4x 不再实例化。`enc_pos`（ctrl_core 的
    // 输入，同时是 STATUS/遥测里的 `encoder` 字段与 diag_encoder）改投**MT6701
    // 原始 14 位角**（声明见下方 MT6701 段之后，避免用未声明的隐式 1 位网）：
    //   · 解除了一个悬空/带上拉的无关输入，也不再让 qenc 计数漂着进诊断量；
    //   · 上位机因此能拿"θ 多圈累加器"和"原始绝对角"做一致性交叉核对——
    //     这是拆掉霍尔之后唯一还能自检的冗余（能抓累加器丢步/回绕 bug，
    //     抓不到磁场掉线，因为掉线时两者一起冻结）。
    // 注意：TH_SRC_MT_ONLY=1 时 ctrl_core 不使用 enc_pos 做控制，只是透传诊断。
    wire signed [31:0] enc_pos;

    // ---------------- 电机侧 MT6701 SSI（V5 第二 θ 源） ----------------
    // 帧间隔 1000clk ≈ 20µs → 轮询 ~40kHz：Δ/事件 ≤ ~19 LSB（44 电机转/s 峰值），
    // 采样年龄 ≤ 25µs，与 MT6835 α 同级新鲜度。CRC6 恒查；Mg[1:0] 场强 22:47
    // 计数器定案后关闭（换向线圈场扰的"意见"位，角度已证有效），Mg[3] 失跟踪恒拦。
    wire [13:0] mt6_angle14;
    wire        mt6_valid_unused;
    wire [15:0] mt6_frames, mt6_good, mt6_crc_err, mt6_mg_err;
    wire [2:0]  mt6_mg_code;
    wire [13:0] mt6_angle_raw;
    wire [1:0]  mt6_cause;
    wire [15:0] mt6_max_no_good;
    mt6701_ssi_if #(
        .FRAME_GAP_CLKS(1000),        // 默认 5000(~9.6kHz) 压到 1000(~40kHz)
        .BAD_FRAMES_LATCH(4'd8),      // 同 MT6835：容忍换向突发 EMI，看门狗兜底断线
        .LATCH_ON_STRENGTH(1'b0),     // 23:29 cause=3 终审定案：Mg 场强/失跟踪在
        .LATCH_ON_TRACK(1'b0),        // 电机旁都是"意见"（换向线圈场扰瞬时越限/
                                       // 丢锁成串，Mg +207、CRC 恒 0、角度事后有
                                       // 效），双关锁存权只留 CRC。
        // 数据门：Mg 告警帧的角度也采纳（2026-10-23 用户定案）。
        // 实测满舵期冻结 10 段/平均 7.2ms，冻结时原始角也不动（传感器在保持），
        // 解冻跳变 2/3 与累积转角一致（<0.6°）⇒ 告警帧基本可用。打开后 θ 连续、
        // θ̇ 不再假性塌 0，刹车/捕获才有可能改回闭环判据。
        .ACCEPT_MG_WARN(1'b1),
        .MG_WARN_MAX(2000),           // 连续 200ms 告警才判故障（原 200=20ms）。
            // 2026-10-23 14:52 实测：捕获瞬间 duty 0→-6000，MT6701 场强/失跟踪告警
            // 必然连续超过 20ms，于是 otr_fault 立刻锁死，BALANCE 只活了 30ms
            // （遥测 fault=3，bit1=otr_fault；α/θ 数据本身完全正常）。告警帧的
            // 角度已验证可用（冻结消失、跳变 <0.6°），20ms 就把"可用但告警"当成
            // 断线是错的；真断线由无帧看门狗（WATCHDOG_CLKS=20ms）兜底。
        .WATCHDOG_CLKS(1000000)       // 12:40 cause=0：看门狗路径发火=存在 ms 级连续
                                       // 告警段（≥2ms 全帧无好）。2ms→20ms 放宽（真
                                       // 断线 20ms 兜底；θ 期间保持旧角，控制容忍）。
                                       // st_max_no_good 实测段长后再校此值
    ) u_mt6701_m (
        .clk(clk50), .rst_n(rst_n),
        .ssi_csn(mt6701_csn), .ssi_clk(mt6701_sck), .ssi_do(mt6701_do),
        .adc_raw(), .angle14_o(mt6_angle14),
        .adc_valid(mt6_valid_unused), .adc_fault(mt6701_fault_w),
        .st_frames(mt6_frames), .st_good(mt6_good),
        .st_crc_err(mt6_crc_err), .st_mg_err(mt6_mg_err),
        .st_mg_code(mt6_mg_code), .st_angle_raw(mt6_angle_raw),
        .st_fault_cause(mt6_cause), .st_max_no_good(mt6_max_no_good),
        .st_frame_pulse()
    );

    // `encoder` 诊断字段 = MT6701 原始 14 位绝对角（霍尔拆除后的替代口径）。
    // 上位机用它和 θ 多圈累加器做一致性核对（见上面 enc_pos 声明处注释）。
    assign enc_pos = {18'sd0, mt6_angle14};

    // θ 源选择：V5 跳线两级同步（防亚稳）。V5.1 定案 MT 独占——TH_SRC_FORCE_MT=1
    // （默认）恒选 MT 源，A14 电平被忽略（常数旁路，同步链被综合裁掉）；置 0 恢复
    // 跳线语义（悬空=霍尔/拉地=MT，两级同步防亚稳）供 A/B 对照。两链同一复位起算，
    // 切换瞬态仅为 2.2% 尺度差×累计角（量级无害，见 theta_mt_path.v 头注释）。
    reg [1:0] th_src_sync;
    always @(posedge clk50 or negedge rst_n) begin
        if (!rst_n) th_src_sync <= 2'b11;   // 复位期默认霍尔
        else       th_src_sync <= {th_src_sync[0], th_src_n};
    end
    assign th_src_mt = TH_SRC_FORCE_MT ? 1'b1 : ~th_src_sync[1];

    // ---------------- 控制核心（与 J280/DE10 同一份源码） ----------------
    wire signed [15:0] duty_out;
    wire               motor_enable;
    wire [2:0]         mode;
    wire               swing_brake;
    wire signed [15:0] alpha, alpha_dot, theta_dot;
    wire signed [31:0] theta_c01;
    wire               core_tick, fault_w;
    wire signed [31:0] diag_encoder;
    wire [15:0]        diag_adc;
    wire signed [15:0] diag_pos_trim, diag_alpha_pre, diag_k3_eff, diag_k4_eff, diag_duty_pre_dz;
    wire               diag_obs_active;
    // v4 遥测：θe + 分项（term 恒 0，上位机重建）
    wire signed [15:0] diag_theta_e, diag_term_k1, diag_term_k2, diag_term_k3, diag_term_k4;
    wire               io_test_enable;
    wire signed [15:0] io_test_duty;

    ctrl_core #(
        // 霍尔编码器已物理拆除（2026-10-22）：θ 只用电机侧 MT6701，arm_vel/
        // cnt2deg 整条链不实例化，arm_stalled 由 theta_mt_path 的角度事件判停提供。
        .TH_SRC_MT_ONLY(1'b1),
        // 换向守卫（2026-10-22 用户澄清：不是"不能快"，是"不能高频"）。真正治
        // 高频的是相位机本身：HOLD 相只短路制动不出力 ⇒ 一杆只剩 1 次变号，
        // 补甩 3s 一杆 ⇒ ≤0.33 次/秒。本守卫只作兜底，取 120ms（=8.3 次/秒
        // 上限）是为了不误伤鞭甩——实测加速相只有 260ms 量级，取 300 会把刹车
        // 命令挡掉 40ms，鞭甩就软了。
        .SWING_REV_GUARD_MS(10'd120),
        .THDOT_SLEW_LIM(16'sd1000),  // θ̇ 斜率限制：掐断 K4×T法×刹车 20-60ms 短脉冲自激
        .THDOT_PIPELINE_ON_TICK(0), // 两级流水改按 FPGA 时钟更新；EMA/限斜率仍为 1kHz
        .POS_ACC_MAX(1),             // POS S 曲线：0→10°/s 用 256ms（起停不激励摆杆）
        // 捕获增强：K4 ×1（2026-10-10 实测停用）、K2 ×1（2026-10-21 实测停用）。
        // K4 的停用理由见下；K2 的理由是同一天 6 轮自动捕获里定量的：捕获瞬间
        // |α̇| 实测 79~367°/s，K2=10000 未增强时 K2·α̇ 已 −2400~−4500 duty，
        // ×2.67 后 −6400~−12000 直接顶到 ±9500；臂被驱动到 ±500°/s 飞转，
        // K4·θ̇ 再叠加彻底饱和，摆杆被反应扭矩甩过顶。6 轮分界干净：
        // |α̇|入窗 ≤171°/s 的 3 轮全存活、≥218°/s 的 3 轮全掉（1.1/1.3/1.8s）。
        // 增强相原本是为 480°/s 热捕获补阻尼，现在捕获门已收到 <40°/s 量级，
        // 增强只剩副作用。要复活时把 NUM/SH 配回 8/3。
        // 捕获窗内摆杆通道 K2 ×3（2026-10-23）。入场逐帧：捕获后 +40ms duty 从
        // +1446 翻成 −1219，而此时摆杆正以 +499°/s 冲向顶 —— 分解出力：
        //   臂项 K4·θ̇=−660 + K3·θe=−119，摆杆项 K1·α=−500 + K2·α̇=+609
        // ⇒ **臂的残余速度项压过摆杆项，把摆杆顶过顶再摔下来**（+240ms α 已到 −111°）。
        // 抬 K2（不抬 K4，K4 增强 10-10 已否决）让捕获瞬间摆杆通道占优。
        .CATCH_K2_NUM(1), .CATCH_K2_SH(0),
        // ---- 外摆阻尼反馈 DFB / 甩摆轨 9900：**本树不移植**（2026-11-01 用户定案：
        // DFB 增强通道是稳摆泵能源头，BALANCE 稳态律严格保持 10-20 简单栈；
        // 输出轨保持 9500）。
        // K4 增强 ×4 已停用（2026-10-10 实测）：制动期本就 duty 钉 ±9500 饱和，
        // 增强对制动零贡献，只在恢复期延长 rail——捕获 θ̇=-336°/s 时 K4×4·θ̇=-71M
        // 把舵钉死 200ms，臂积蓄 -600°/s 飞转（K4·θ̇ 正反馈），α 稳在 ±2.5°
        // 也被反应扭矩掀翻（swing_gate_log 62567-62825）。增强留 DE10，T20K 用基础 K4。
        // 捕获窗内降 K4（×1/4）已撤销（2026-10-23 复盘）：当初"入场 +40ms duty
        // 被 K4·θ̇ 压过"的逐帧结论取自**未做下垂标定**的日志（α 偏 68.6°），
        // 捕获本身不成立，故该归因无效。恢复 10-23 12:41 构建（单杆捕获 +
        // 稳摆 12s）的 K4_SH=0。
        // 捕获窗内 K4 ×1/4 **已实测否决**（2026-10-23 15:29，已标定）：入场确实被
        // 接得很软（α 从 −12.5° 一路降到 +2°、α̇ 只剩 20-40°/s），但臂失去阻尼后
        // 漂移，800ms 窗口结束时 K4 恢复的阶跃把摆杆一冲就满舵（α 20°→107° 掉摆）。
        // 入场被臂速项压过的机理成立（见下方 CATCH_K3 注释与 150133 逐拍），但
        // "整窗静音"的代价大于收益 —— 要做只能做成 150-200ms 短窗，而那条路
        // 加 mux 后 Fmax 48.5-49.9MHz 连过不了门禁，暂缓。
        .CATCH_K4_NUM(1), .CATCH_K4_SH(0),
        // 捕获窗内把 K3 渐变抬到 POS 档（2026-10-22）：臂带甩摆残余速度进 BALANCE，
        // 而 K3=300 要 175° 臂误差才越死区、臂环没有权限消速度；抬到 2064 后门槛
        // 降到 25°，能在 800ms 窗内把臂拽回锚点。这是唯一对准"臂残余动能"的修法。
        // 2026-11-01 注：这是起摆冠 100% 捕获配置的组成部分，800ms 窗后恢复
        // K3=600（良好参数）。若板验发现稳摆早期异常，第一步把它配 0 回纯律。
        // 2026-11-01 极点扫描后改成对化：K3 单抬到 2064（K4 不动）窗满裕度仅
        // -0.286（基础 -0.748 的一半不到），叠 K2 增强即翻正。CATCH_PAIR_EN=1 后
        // 窗目标=运行时 POS 对（id11=1200/id12=12000 成对），裕度 -1.57。
        // CATCH_K3 保留非零只为使能通路（PAIR_EN=1 时值不被使用）。
        .CATCH_K3(16'sd2064),
        .CATCH_PAIR_EN(1),
        // ---- 输出级整形（2026-11-02 P3b，golden-base 板验定案移植）----
        // 全幅死区补偿（板上实加=参数值）消除摩擦阻尼 ⇒ 继电器极限环 + 臂漂移
        // （221908/统计 12 轮实证）。软区 1024+速度自适应 200：过零连续、静止全
        // 补偿（保稳摆穿透力）、运动中按 θ̇ 缩放（恢复部分摩擦阻尼）。滞环仅 POS。
        .OUT_DZ_SOFT_TH(16'sd1024),
        // ★2026-11-02 启用速度自适应（dz(θ) 配套）：LUT 全额常加在移动时与轨迹
        // 推力同向叠加 → move1 甩飞 3/3（此前同配置 0/3）。速度项把"LUT 全额"
        // 约束到静止态：|θ̇|=100°/s 时降至 1/5——移动中补偿≈LUT/5，静止保持时
        // 全额穿透摩擦坡。静（位置表）/动（速度项）分离。
        // ★2026-11-03 夜 P5（184835 板测定谳）：REF=1000(100°/s) 正好压在 POS
        // 常规臂速（|θ̇| 均值 72、中位 88）上 ⇒ 补偿实际只有 20-42%，乘 LUT/2 后
        // 120-550 duty，而 LUT 自记静突破 1200-2600 ⇒ 摩擦坡穿不过、te 直流
        // +16~23°。REF 抬到 2500(250°/s)：72°/s→73%、88°/s→72%、150°/s→55%，
        // 高速段仍降到 1/5 保留"移动防甩飞"保护。
        .OUT_DZ_THDOT_REF(16'sd2500),
        // ★2026-11-02 板测判别：THDOT_REF=200（速度自适应全模式生效）时起摆→稳摆
        // 链全灭（NO_BAL 6/6——甩摆/刹车期补偿被 θ̇ 缩没、力矩顶不动真死区）；
        // 硬死区构建同日 3/3 SUCCESS。改 0=关速度自适应，仅保留过零软区（治保持
        // 段继电器抖动），运动期恢复全额补偿。
        // ★2026-11-03 夜 P8：400→0。实测 POS 段 K4·θ̇ 项 |<400| 占比 49%，
        // 而滞环恰好把"反向且 <400"的指令整段挡成 0——摆动折返点（θ̇≈0）
        // 的阻尼被切掉，是 ±30-50° 等幅慢摆的候选泵源。滞环本为降"咔咔"，
        // 但那是在旧 rehold/blend 结构下调的；现结构下先验证控制代价。
        .OUT_REV_HYST(16'sd0),
        // dz(θ) 位置查表（2026-11-02）：LUT=自动扫描摩擦地图（静突破 1200-2000
        // 随位置起伏），仅 POS 生效。定量依据：K3=1200 @25° 误差 731+dz640=1371
        // ≈静突破 1400 ⇒ 漂移 ±25-30°；查表后位置力在全域"穿透"摩擦坡。
        // 2026-11-03 夜 P9 判别实验已做（DZ_TH_EN=0，audit_20261103_201800）：
        // LUT 关掉后退回均匀 640/740 → α 峰峰 16.3→36.4°、te 峰峰 75→88、
        // duty std 1277→1724，全面变差 ⇒ **LUT 不是泵源，是在帮忙**（稳住摆杆）。
        // 恢复 1，LUT 保留。
        .DZ_TH_EN(1),
        .OUT_BACKLASH_BOOST(16'sd0), .OUT_BACKLASH_MS(10'd30),
        .FLING_RETHROW_MS(3000),     // 甩摆自动补甩：单发相位敏感，3s 一杆
        // 捕获门限（单位 0.1°/s，注意不是 0.01°/s——旧注释按 cdeg 写，导致
        // 实际门限比标称松 10 倍）。2026-10-22 用 20 次真实捕获做了门限扫描
        // （α̇ 门槛 200..∞ × θ̇ 门槛 200..∞ 全组合）：
        //   α̇<200 存活率 83% 但只捕获 6/20 → 总体成功率 25%；
        //   α̇<400 捕获 19/20、存活 12 → 总体成功率 60%（最优）；
        //   门限全开也是 20/20 捕获、12 存活 = 60%。
        // 即"收紧门限"只是把掉摆换成不捕获，总成功率不升——瓶颈不在捕获
        // 判定，而在 BALANCE 入场的接管过程。故 θ̇ 门保持宽松、只留 α̇<400
        // 挡掉 420-760°/s 的极端热杆（真会 0.4s 内直接翻过去）。
        // ★2026-11-01 本树改判（用户多轮日志 20261101_164419）：上面的"θ̇ 门
        // 零贡献"结论出自 DFB 全开的树（DFB_K4_ATT 压着 K4·θ̇），本树是 10-20
        // 简单栈、K4·θ̇ 无压制，4 轮交接入场臂速 455/454/496/454°/s 全部挤在
        // 隐藏硬门 ±500°/s（mode_fsm 硬编码 5000）边缘——3 死 1 平该区间全灭，
        // seg3 的 0.78s 甩飞与 222416 三连实测同签名（K4·θ̇ 顶满舵反作用鞭打
        // 慢速摆杆）。配 3500（350°/s）：450-500 死区全拒、等 HOLD 相摩擦把臂
        // 速衰减（实测甩摆停泵后 ~2.5s 臂停）再抓。
        // ★2026-11-01 二次收紧 3500→1300（日志 165804，TD=350 已烧板实测）：
        //   门内 pass 3 死 1 活——seg1/seg5 臂速 326 入场仍被 K4·θ̇ 泵到
        //   406-559°/s 反作用甩杆（0.8-1.4s 满轨）；seg3 臂停（21°/s）但快杆
        //   穿零（α̇=381）动量转移臂-摆互摆发散。合并两日志 8 次交接：
        //   臂速 ≤117 的 1/1 活（10.66s），130-350 的 0/3 活。故 3500→1300。
        //   代价=甩摆等待更长（见 BPUMP/补甩侧"全圈旋转维持"问题，另案处理）。
        // ★2026-11-01 三改 1300→5000（用户手测日志 194141，4/4 首杆过顶被拒）：
        //   首杆在 t≈2.0s 把摆杆干净抛进 15° 窗（内向 160-460°/s）——最理想的
        //   一次过顶 pass——但臂仍在 450-590°/s，1300 门全拒 ⇒ duty=0 滑行，
        //   摆杆到顶无人接管即掉（用户实感"到顶没力度"）。1300 的依据（455-496
        //   入场 3 死）出自旧单抬窗（裕度 -0.286、K4 停 5300 无刹臂权限）；成对窗
        //   K4 171ms 内抬到 12000 即刹臂权限，前提已变。放回 5000=mode_fsm 隐藏
        //   硬门值（champion 首杆捕获正是靠该门+8ms 驻留在臂刚降到 500 以下时抓）。
        .CATCH_TD_LIM(16'sd5000),
        .CATCH_AD_LIM(16'sd4000),    // 摆杆 <400°/s（收紧到 300 实测 0/3 捕获，退回）
        .CATCH_DWELL_MS(6'd8),       // 捕获去抖 8ms（原 20ms）。2026-10-23 14:47 逐拍
            // 实测：首杆以 α̇=344°/s 干净过顶，但门限里的硬性 |θ̇|<5000 直到过零点前
            // 10ms 才放开（臂 494°/s），"内向"半窗只剩 ~10 拍 < 20 拍 ⇒ 不捕获，
            // 眼看它过顶转一圈。8 拍仍能挡单拍坏帧，又够 350°/s 过顶的窄窗。
        .CATCH_AD_MIN(16'sd200),     // 捕获动量下限 20°/s（10-24 定案：100 太低）
            // 2026-10-23 试过抬到 200（按"预测过顶速度≥200°/s"的门，10 次已标定
            // 捕获里正好挡住 5/5 失败），实测 4 轮 **0/4 存活**，比基线 4/8 更差——
            // 说明入场状态预测不了存活（同一状态 198°/s 一死一活），系统就卡在稳定
            // 边界上。回退到 100，不再用进场门筛。
        .CATCH_DEAD_BAND(16'sd500),  // ±5° 近直立豁免：准静止态不设动量下限
        .CATCH_DEAD_AD(16'sd300),    // 豁免必须同时 |α̇|<30°/s。2026-10-22 实测：
            // 唯一"向外高速过顶"（α=+1.0° 但 α̇=+246°/s）的捕获被无条件的
            // ±5° 豁免放行，0.8s 就翻到 -180°；同批其余 5 次向内过顶全存活。
            // 豁免的本意是"准静止"，不是"恰好在零附近"，补上摆速条件。
        .FALL_DWELL_MS(6'd30),       // 掉摆门去抖 30ms：拒端区腐败假掉摆
        .AUTO_RESWING_EN(1),         // 掉摆自动重甩：硬件级自愈，演示保底
        .RETHROW_ALPHA_MIN(16'sd9000), // 补甩相位门：摆杆下半区(|α|>90°)才泵力
        .RETHROW_ADOT_MAX(16'sd2000), // 补甩摆速平静门：|α̇|<200°/s 才锁目标
        // ★2026-11-01 板测定谳（181359 日志，BPUMP 关闭后出力仍 ±8800×738ms×45 段）：
        // 转圈维持的真泵源=补甩本体。旋转摆每次过顶 α̇ 瞬间跌破 200°/s，80ms
        // 假平静窗被钻空子 ⇒ 每 ~5s 补一杆续转圈。拉到 250ms（8 位上限 255）：
        // 顶点假平静撑不满 ⇒ 旋转时停补甩 ⇒ 摩擦自由衰减（实测 1183→142°/s 约 4s）
        // ⇒ 真平静再补甩。治 46-68s 干等。
        .RETHROW_CALM_MS(8'd250),
        .ADAPTIVE_RETHROW_EN(1),     // 自适应甩距：同参数杆间能量方差 ±40%（apex
            // 0.1-92° 散布）会让定值甩距饿死捕获窗；按每杆实测 apex 阶梯调补甩
            // 距离，2-3 杆收敛进 15° 窗。一次快速甩摆 + 自适应补甩是起摆线定案路径。
        // 起摆 = 甩摆（ACCEL + 反向拉回 + HOLD 短路）。能量泵(PUMP_MODE=4) 已于
        // 2026-10-23 15:57 重验后放弃：用户判定该方向不可行，回到时间制基线。
        .PUMP_MODE(0),
            // 2026-10-23 能量账定案：这是唯一能把底部能量堆到 239~269 的机制
            // （过顶线 262），泵最高只到 197。泵(PUMP_MODE=4)与混合(交棒)代码
            // 都留档，实测都撞在 85° 天花板 —— 详见文档 §九/§十。
        // ★2026-11-03 夜回滚：曾收到 3000（300°/s），实测**一次补甩都发不出**——
        // 旋转摆反作用把臂一直带到 ~900°/s，门永不满足 ⇒ SWING 136s、622 次过顶、
        // duty<100 占 98%（213137 日志）。恢复 30000；甩摆能量改用运行时 sw_amp
        // 控制（8800→7000 板测 5/5 捕获，见 212732 日志）。
        .RETHROW_TD_MAX(16'sd30000),  // 补甩门：底座 <3000°/s 才补力
        // SWING_DEAD 实测否决（2026-10-22）：本意是掐掉纯比例伺服在 target 附近
        // 的 ±8800 自激振荡，但撤力后臂几乎不动（θ 只转 2°）、能量传不给摆杆，
        // 甩击次数反而从 21 次/24.5s 增到 42 次/0.2s 间隔，摆杆 |α|min 从
        // 21~30° 恶化到 30.4°。甩摆需要臂持续加速才有动量，不能"到点就停"。
        // 保持 0（旧行为），起摆能量问题改由三相位甩摆解决（见下）。
        .SWING_DEAD(32'sd0),
        // 三相位甩摆（2026-10-22）：加速→刹车→阻尼保持，取代纯比例伺服的甩法。
        // 依据（130025/135008/135847 逐帧核算）：一杆真正把摆杆抛过顶的是刹车相
        // （130025 t=6020ms：满舵加速 111°/260ms → 反向满舵刹车 220ms → 摆杆从
        // 120° 冲过顶留 433°/s，0.74s 捕获）。坏在刹车后比例伺服立刻反打满舵，
        // 形成 ±100°/≈1Hz 等幅振荡，其对摆杆做功的相位随机：130025 泄能（6.7s
        // 捕获），135008 泵能（24s 内摆杆从未进 15° 窗）——这才是"有时一杆就起、
        // 有时 40s 起不来"的真因，与捕获门无关。旧路径 FLING_BRAKE_EN=0 可随时回退。
        .FLING_BRAKE_EN(1),
        // 刹车时长 = 峰值臂速>>4 + 50ms，**夹在 [400, 470]ms**。
        // 2026-10-23 14:41 实测（已标定，offset=15280）：峰速项被满舵期 MT6701 电流
        // 注入的 θ̇ 假尖峰（856°/s）顶到上限 588ms，首杆被刹过头 —— 摆杆只到 α=33.9°
        // 就回落，臂反向飞到 −590°/s（飞轮储能），后续每杆能量在 33°/0.3° 之间乱跳。
        // 同一台机器同一初态（静止下垂）、只差刹车时长：
        //   438ms（12:41 构建实测）⇒ 首杆过顶 280°/s 被捕获，稳摆 12.03s；
        //   588ms（14:41 实测）     ⇒ 首杆 apex 33.9°，失败。
        // 所以把公式夹到 438±35ms：既能跟上电池电压/杆间差异，又不让假峰速把刹车
        // 拉长到破坏相位。下限 400 同时挡掉"没刹住"（14:06 那版 225ms 过顶 419°/s）。
        .FLING_BRAKE_SH(4), .FLING_BRAKE_EXTRA_MS(10'd50),
        .FLING_BRAKE_MIN_MS(10'd440), .FLING_BRAKE_MS(10'd470),
        // 刹车结束的摆杆速度门：**实测未跑赢基线，关闭**（2026-10-23 15:44）。
        // 试过的三种释放判据：按时间（基线，过顶 133-344°/s，4/8 稳 10-30s）、
        // 按总能量 E≥15000（门在跨水平瞬间就触发，根本没削能量，过顶 420°/s）、
        // 按摆杆速度 |α̇|≤600（3 轮里 2 轮摆杆爬到 34° 就停住、1 轮过顶 355°/s）。
        // 真因：释放→过顶这段的损耗随臂的状态在 900~2700 能量单位间变（臂在拖摆杆），
        // 所以任何"单变量释放判据"都定不住落点。管路保留（VEND_EN=0 时行为与
        // 改动前逐位一致），要做只能做"释放时刻 + 臂状态"的二维判据。
        .FLING_VEND_EN(0), .FLING_VEND_V(16'sd600), .FLING_VEND_MIN_MS(10'd150),
        // 换向斜率：0=不限（鞭甩最强）。用户澄清后不需要限速，只需限**频率**：
        // 每杆 1 次换向、补甩周期 3s ⇒ ≤0.33 次/秒，另有 rev_guard 300ms 硬闸。
        .FLING_SLEW_STEP(16'd0),
        // HOLD 相：**臂速闭环制动已实测否决**（2026-10-23 15:35）：改成
        // duty=−KV·θ̇（KP=0、MAX=4500、DZ=800）后，入场确实变干净（θ̇=+52°/s，
        // 稳 10s），但代价是甩摆能量被一起刹掉——捕获从 1.0s 变成 16s/4s，
        // 且 DZ 阈值在峰值附近反复穿越，出力段从 1 段变成 78/19 段（变相 bang-bang，
        // 违背"不许高频换向"）。物理上臂停不下来是因为"刹得够早"和"留够能量"
        // 互相矛盾：捕获前 50ms 内 4500 duty 只能刹掉 ~80°/s。
        // 恢复短路制动（弱阻尼、但不抢能量）。
        .FLING_HOLD_BRAKE(1),
        // 相位驱动刹车（2026-10-23）：用户实感"有时过顶转一圈像没刹车、有时刹
        // 太早能量不够"——根因是刹车靠 f_peak>>4 估时长 + |θ̇| 判停，两者都与
        // 摆杆相位无关（满舵期 θ 冻结还会造成假"停住"提前撤刹）。改成：
        //   ACCEL(下半区上升) → COAST(整个上半区，拉回在这里是卸能)
        //   → BRAKE(下半区下降，拉回在这里做正功) → COAST(重新过水平，撒手)。
        .FLING_PHASE_BRAKE(0),         // 相位驱动刹车实测否决（2026-10-23，见文档 §九）
        .FLING_BRAKE_STOP(16'sd1500),  // |α|<15° 收手，交给捕获窗
        .FLING_BRAKE_FLIP(16'sd300),   // 上升/下降判据 30°/s
        .FLING_COAST_MS(10'd900),      // 滑行相兜底
        // 相位+能量门补甩（2026-10-23）：单杆能量账只有 10% 余量且被臂飞轮吃掉
        // 一半 ⇒ 必须多杆累积。旧门是"3s 定时 + 摆杆平静"，每杆都从静止重来；
        // 改成跟摆杆节奏：在底部（|α|≥150°）、有方向（|α̇|>30°/s）、且底部速度
        // 还低于过顶所需（<520°/s；过顶理论 367°/s，含衰减余量）就立刻补一杆。
        .RETHROW_PHASE_EN(0),          // 相位补甩随 fling 一起停用（当前用 PUMP_MODE=4）
        .RETHROW_PHASE_V(16'sd5200),
        .RETHROW_PHASE_ADOT(16'sd300),
        .RETHROW_PHASE_ALPHA(16'sd15000),
        .RETHROW_PHASE_GAP(10'd500),
        // 双向能量泵参数：**满舵幅度**（2026-10-23 实测 4000=49%占空时 apex 卡在
        // 101° 爬不上去——用户"幅度太小和不猛，起不来"）。相位正确的前提下每半周
        // 推满，能量才追得上损耗；|α̇|>600°/s 仍会滑行收手，所以不会一直猛。
        .BPUMP_AMP(16'sd8800),
        .BPUMP_ADOT_MIN(16'sd300),
        .BPUMP_V_STOP(16'sd9500),     // 目标：底部 ~877°/s 是过顶门槛，950 留 8% 余量
        .BPUMP_E_TARGET(32'sd12000),  // 目标总能量 0.01(rad/s)²（过顶需 11740）
        .BPUMP_E_HYST(32'sd1500),     // 迟滞带：带内滑行，避免加/泄能来回抖
        // 泵→甩摆混合：泵把摆杆推到 apex≈85°（泵的天花板），再打一杆收尾
        // ★2026-11-01 板测定谳（180906 日志）：转圈维持的真泵源=BPUMP 能量伺服——
        // 34 段出力全部 ±8800（BPUMP_AMP 签名）、738ms/段、每 5.2s 一段，
        // 把系统锚在 E_target≈过顶线=永远转圈。它本是 PUMP_MODE=4 混合交棒
        // 的参数，PUMP_MODE=0 下应关。关后 SWING 纯三相位+补甩，能量可衰减。
        .BPUMP_HYBRID_EN(0),
        .BPUMP_APEX_TRIG(16'sd9000),  // apex<90° 即达标
        .BPUMP_HYBRID_GAP(12'd1200),  // 两次收尾最小间隔
        .BPUMP_DWELL_MS(10'd40),
        // 刹车门限必须 ≥ θ̇ 的量化台阶（MT6701 1 raw LSB/ms = 931 单位）。
        // VSTOP_CNT=0（提前退出现在已在上面关掉），VSTOP 仅保留代码兼容。
        .FLING_VSTOP(16'sd1000), .FLING_VSTOP_CNT(6'd40),
        // 摆杆过水平即停加速（卸能边界，功率 ∝ cosα）
        // ★2026-11-01 板测 180127：首杆满舵抛到 |α̇|≈1240°/s 后摆杆进入低摩擦
        // 自由旋转（转圈段 90% 时间 pdz=0，每 30s 才一杆补甩被平静门放行），
        // 168s 不衰减——捕获只能等它擦出边缘 pass。撤力角 90°→70° 降低首杆
        // 注入能量，宁要"补甩堆能"不要"首杆转圈"。
        .FLING_ALPHA_END(16'sd7000), .FLING_ALPHA_MIN_MS(10'd60),
        // KV 必须小：931 台阶×6 = 5586 duty 纯量化噪声 ⇒ 实测 45 次极性翻转/秒
        .FLING_HOLD_KP(16'sd2),       // 保持相：2 duty/cdeg
        .FLING_HOLD_KV(16'sd2),       // 阻尼：2 duty per 0.1°/s
        // 保持相 = 短路制动、不出力（2026-10-23 15:35 复核：纯滑行会让臂的残余
        // 动量继续泵能、第一杆直接抛过顶；主动臂速制动又会刹掉甩摆能量并抖出
        // 78 段出力）。位置环是高频换向的源头，保持关闭。
        .FLING_HOLD_MAX(16'sd0),
        .FLING_HOLD_DZ(16'sd250),
        // 两段式起摆 stage2：底部相位微冲（机理见 swingup_core.v 同名参数段）。
        // 单杆物理上限 = 41.3 单位 vs 过顶需要 41.0 ⇒ 必然时中时不中；这一记
        // 在底部按 sign(α̇) 补 +8 单位，把 apex 22.5° 抬到过窗 175°/s 的温柔 pass。
        .TRIM_EN(0),                  // stage2 微冲默认关（实测未改善：见文档 §九）
        .TRIM_DANGLE(12'd1500),       // 转过 15° 即收手（Δsin=0.26 ⇒ 稳定 +6 单位）
        .TRIM_MS(10'd80),             // 兜底时长（摆杆在底部很慢时）
        .TRIM_ALPHA(16'sd16000),      // 只在底部 ±20° 窗口内补
        .TRIM_APEX_MAX(16'sd1200),    // 上一 apex 差于 12° 才补
        .TRIM_ADOT_MIN(16'sd500),     // |α̇|≥50°/s 才认方向
        .TRIM_MAX(4'd2),              // 每杆最多 2 记
        .TRIM_GAP_MS(10'd900),        // 两记之间 ≥900ms（≈一个摆杆半周期）
        // 相位泵参数（PUMP_MODE=3）：起摆能量按半周注入，节奏 ≈1 次/周期。
        .PUMP_GATE_ALPHA(16'sd9000),  // |α|≥90° 才算泵能半周（cosα<0）
        .PUMP_PHASE_MIN(16'sd200),    // α̇ 相位门 20°/s（4ms 差分 1LSB=2.5°/s）
        .PUMP_PULSE_MS(10'd200),      // 单次出力上限 200ms ⇒ 臂速 ≈380°/s
        .PUMP_PULSE_MIN_MS(10'd60),   // 相位翻面后至少推完 60ms
        .PUMP_REST_MS(10'd120),       // 强制静默 120ms ⇒ 出力节奏 ≤3.1 次/秒
        .PUMP_TD_CAP(16'sd4000),      // 臂速 >400°/s 不再加速（改由短路制动泄速）
        .PUMP_TD_STOP(16'sd800),      // |θ̇|<80°/s 视为已停（撤短路制动）
        .PUMP_KICK_ALPHA(16'sd15000), // 悬垂（|α|>150°）静止 ⇒ 起振脉冲
        .PUMP_KICK_ADOT(16'sd400),    // 起振判据 |α̇|<40°/s
        .PUMP_KICK_HOLD_MS(10'd200),  // 且连续静止 200ms（防慢速抖振）
        .PUMP_ADOT_CAP(16'sd5000),    // 摆杆 |α̇|>500°/s 已够过顶（底部过顶需 ≈370°/s）
                                      // ⇒ 停止加能，让摩擦把能量收回来，防旋转极限环
        .ALPHA_SCALE_Q16(16'd36000),  // MT6835 全 360° 口径：65536↔360.00°（承 V3）
        // 观测器接管（2026-10-20 第 1 步）：先用现有增益拿真机基准，验回放
        // 结论（α̇ 降噪 0.48× 但滞后 +24ms @qa=3）能否在板重现。BALANCE/POS
        // 且 |α|<30° 时 α/α̇/θ/θ̇ 全部由 obs_core 给出，越窗/换模式自动回退
        // 差分链（从动模式，切换零瞬态）。
        .OBS_USE(1'b1),   // 恢复 12:41 构建（观测器在板，那份配置单杆捕获+稳摆 12s）。
                          // 10-23 13:xx 的"关掉验证"是在无标定日志上做的，结论无效。
        .AZ_ENABLE(1'b0)              // 自动自校准关闭（2026-10-20）：实测过冲严重
            // ——序列 15241→15534→15578→15196→15426→…→15214→15586 在
            // 15200-15600 间震荡不收敛，且把"臂运动导致的表观 α 偏移"当零偏
            // 修掉（185628 那轮卡在 θe=-222° 的死点，αRMS 假性 0.88°）。
            // 改为：上位机按全段静态标定结果写参数 id13 (alpha_trim)，
            // 每轮开始前设定；CAL 仍可打底。AZ 代码保留（AZ_ENABLE=1 可复活）。
    ) u_core (
        .clk(clk50), .rst_n(rst_n),
        // 2026-10-31 三冠融合：经 300ms 毫秒门控宽限再进安全链（ otr_event 直连
        // 会在 OT 第一拍锁 SAFE，毁掉捕获瞬间的电流瞬变，见上方宽限块注释）。
        .adc_raw(alpha_avg), .otr_fault(otr_for_safety), .enc_pos(enc_pos),
        .mt_angle14(mt6_angle14), .th_src_mt(th_src_mt),
        .mt6_frames(mt6_frames), .mt6_good(mt6_good),
        .mt6_crc_err(mt6_crc_err), .mt6_mg_err(mt6_mg_err),
        .mt6_mg_code(mt6_mg_code), .mt6_angle_raw(mt6_angle_raw),
        .mt6_fault(mt6701_fault_w),
        .mt6_cause(mt6_cause), .mt6_max_no_good(mt6_max_no_good),
        .mt68_frames(mt68_frames), .mt68_good(mt68_good),
        .mt68_crc_err(mt68_crc_err), .mt68_status_err(mt68_status_err),
        .mt68_status(mt68_status), .mt68_angle_raw(mt68_angle_raw),
        .mt68_fault(mt_fault),
        .hil_use(1'b0),          // V2：真实传感闭环（1kHz 自由节拍 + 看门狗使能）
        .key_sw(key_sw), .dip_sw(dip_sw), .led(led[3:0]),
        .uart_rx(uart_rx), .uart_tx(uart_tx),
        .duty_out(duty_out), .motor_enable(motor_enable), .mode(mode),
        .alpha(alpha), .alpha_dot(alpha_dot), .theta_c01(theta_c01), .theta_dot(theta_dot),
        .diag_encoder(diag_encoder), .diag_adc(diag_adc),
        .diag_pos_trim(diag_pos_trim), .diag_alpha_pre(diag_alpha_pre),
        .diag_k3_eff(diag_k3_eff), .diag_k4_eff(diag_k4_eff),
        .diag_duty_pre_dz(diag_duty_pre_dz), .diag_obs_active(diag_obs_active),
        .diag_theta_e(diag_theta_e),
        .diag_term_k1(diag_term_k1), .diag_term_k2(diag_term_k2),
        .diag_term_k3(diag_term_k3), .diag_term_k4(diag_term_k4),
        .io_test_enable(io_test_enable), .io_test_duty(io_test_duty),
        .tick_out(core_tick), .fault(fault_w), .swing_brake(swing_brake)
    );

    // 电机输出：闭环 duty_out/motor_enable 与 IO_TEST 联调共用一个 motor_pwm，
    // 联调请求优先（与 DE10 板同构）。联调仍只允许 SAFE/IDLE + 急停未按。
    wire io_test_safe = io_test_enable && (mode == 3'd0 || mode == 3'd4) && key_n[3];
    wire motor_run    = io_test_safe ? 1'b1 : motor_enable;
    wire signed [15:0] motor_duty = io_test_safe ? io_test_duty : duty_out;
    // 零点刹车：BALANCE/POS 全程使能（治起点无阻尼的 7.5Hz 激励）。
    // 2026-10-22 起 SWING 的 HOLD 相也拉高（duty 恒 0 ⇒ motor_pwm 的 zero_brake
    // 生效 = 纯短路制动）。注意 motor_pwm 的 brake_en 只在 |duty|<BRAKE_TH 时
    // 才切 11，所以它不会盖掉加速相/刹车相的驱动 —— 鞭甩（满舵反向拉回）保留。
    // 注意与 2026-10-09 那次否决的区别：那次是 brake 在 SWING **全程**使能，
    // 连加速相都被刹住；这里只在 HOLD 相使能，且此时 duty 本来就是 0。
    // IDLE/SAFE 由 run_en 强制 coast。
    wire brake_active = (mode == 3'd2) || (mode == 3'd3) ||
                        ((mode == 3'd1) && swing_brake);
    motor_pwm u_motor_pwm (
        .clk(clk50), .rst_n(rst_n), .run_en(motor_run), .brake_en(brake_active),
        .duty(motor_duty),
        .in1(mot_ain1), .in2(mot_ain2), .pwm(mot_pwma)
    );
    // TB6612 STBY 常使能：安全截断靠 IN1/IN2=coast + PWM 停斩，不靠 STBY 掉电。
    assign mot_stby = 1'b1;

    // ---------------- 诊断 LED ----------------
    // LED4：慢闪与 ADC 原始字 bit15 异或，便于确认 SPI 输入确实在变化；
    // LED5：控制故障或传感链故障。V4 下 mt_fault 同时进安全链。
    reg [25:0] blink_cnt;
    reg        mt_half;
    always @(posedge clk50 or negedge rst_n) begin
        if (!rst_n) begin
            blink_cnt <= 26'd0;
            mt_half   <= 1'b0;
        end else begin
            blink_cnt <= blink_cnt + 26'd1;
            if (mt_valid) mt_half <= mt_raw[15];
        end
    end
    assign led[4] = blink_cnt[25] ^ mt_half;
    // V5.1：控制/α 传感故障 + θ 源（恒 MT）的电机侧 MT6701 故障
    assign led[5] = fault_w | mt_fault | (th_src_mt & mt6701_fault_w);

    wire unused_ok = &{1'b0, theta_c01, theta_dot,
                       diag_encoder, diag_adc, enc_pos_hw, mt_raw, mt_valid,
                       mt_alpha21, mt6_valid_unused,
                       io_test_enable, io_test_duty, core_tick, 1'b0};

endmodule
