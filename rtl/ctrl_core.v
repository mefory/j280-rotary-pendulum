// ============================================================================
// ctrl_core.v — 旋转倒立摆控制核心（板级无关）
//
// 由原 j280_top.v 抽出，控制逻辑逐行保持原样。只有三处结构变化：
//   1) 传感器前端移到板级外壳：本模块接收已解码的 adc_raw[15:0]（adc_if 平均字）
//      与 enc_pos[31:0]（qenc4x 计数），以及 adc_if 的 otr_fault；输出 duty_out
//      与 motor_enable，由外壳驱动 motor_pwm。
//   2) 原顶层内部信号 tick/mode_safe 重建为同名内部信号（语义不变）。
//   3) 传感器前端 mux：inject_use=1 时用 inject_adc/inject_enc 替代真实传感器，
//      注入点在**原始字**层，因此标定/回绕/差分/EMA/M-T 测速整条链仍被真实覆盖。
//      j280 外壳将 inject_use 恒接 0，行为与原顶层逐比特一致。
//
// 板级外壳（引脚、ADC 串行时序、正交解码、电机输出、按键 LED）：
//   rtl/boards/j280/j280_top.v  — GW2A-55 + 3PA1030 + TB6612（现行硬件）
//   rtl/boards/de10/de10_top.v  — DE10-Nano（无 J280 硬件时的算法验证平台）
//
// 数据流（单时钟域，1ms 节拍闭环）：
//   adc_raw→angle_proc(标定/换算/差分)→α,α̇ ┐
//   enc_pos→arm_vel(测速/换角)→θ,θ̇ ────────┤→模式选择
//   SWING→swingup_core；BALANCE/POS→lqr_core →out_stage(死区/限幅)
//   调试: UART V2 CRC帧；500Hz 完整位置遥测，停机原子参数事务
// ============================================================================
`include "j280_defs.vh"
`include "j280_gains.vh"
module ctrl_core #(
    // θ̇ 斜率限制（0.1°/s/拍，0=禁用）。板级按需覆盖：T20K 零点刹车配置 1000，
    // 掐断 K4×T法×刹车的 20-60ms 短脉冲自激（arm_vel.v 注释详述）。DE10 默认 0。
    parameter THDOT_SLEW_LIM = 0,
    // 臂速流水寄存器更新粒度：默认保持控制拍，T20K 用 FPGA 时钟减少测速延迟。
    parameter THDOT_PIPELINE_ON_TICK = 1,
    // POS 参考轨迹加速度限（Q8 c01/tick²）。256=无限加速度（旧纯限速斜坡，
    // DE10 默认，行为零变化）；T20K 配 1 ⇒ 0→10°/s 用 256ms（≈39°/s²，温和起停）。
    parameter POS_ACC_MAX = 256,
    // 捕获增强 K4 剂量 = K4×NUM>>>SH（800ms 窗口）。默认 8/3≈×2.67；
    // T20K 16/2=×4（K4=5300 → 峰值 21200，补回热捕获阻尼）。
    parameter CATCH_K4_NUM = 8,
    parameter CATCH_K4_SH  = 3,
    // 甩摆自动补甩周期（ms，0=单发旧行为）。T20K 3000：单发相位敏感，
    // 每 3s 从当前 θ 重锁目标再甩一杆，杆过顶即被捕获窗接走。
    parameter FLING_RETHROW_MS = 0,
    // 捕获底座速度门（0.1°/s，透传 mode_fsm）。32767=禁用（DE10）；
    // T20K 20000：底座飞转过顶时不捕获。
    parameter CATCH_TD_LIM = 16'sd32767,
    // 捕获摆杆角速度门（0.01°/s，透传 mode_fsm）。32767=禁用（DE10）；
    // T20K 4000（40°/s）：热杆必掉、冷杆可稳（swing_gate 杆1/杆2 对照）。
    parameter CATCH_AD_LIM = 16'sd32767,
    // 捕获驻留去抖 ms（透传 mode_fsm）。0=旧行为；T20K 20：拒端区腐败单拍。
    parameter CATCH_DWELL_MS = 6'd0,
    // 捕获动量下限/近直立豁免（透传 mode_fsm）。0=旧行为；T20K 1000/500。
    parameter CATCH_AD_MIN    = 16'sd0,
    parameter CATCH_DEAD_BAND = 16'sd0,
    // 近直立豁免要求的摆速上限（0.1°/s，0=不要求）。见 mode_fsm 注释。
    parameter CATCH_DEAD_AD   = 16'sd0,
    // 掉摆门驻留去抖 ms + 掉摆自动重甩（透传 mode_fsm）。T20K 30/1。
    parameter FALL_DWELL_MS   = 6'd0,
    parameter AUTO_RESWING_EN = `AUTO_RESWING,
    // 补甩摆杆相位门（透传 swingup_core）。0=无门；T20K 9000（|α|>90° 才补甩）。
    parameter RETHROW_ALPHA_MIN = 16'sd0,
    // 补甩/入场摆速平静门（0=旧行为；T20K 200°/s，见 swingup_core 注释）。
    parameter RETHROW_ADOT_MAX = 16'sd0,
    // 补甩放行要求的连续平静时长（ms，仅 ADOT_MAX≠0 生效）。T20K 80。
    parameter [7:0] RETHROW_CALM_MS = 8'd80,
    // 自适应甩距（0=旧行为；T20K=1，按每杆 apex 阶梯调补甩距离）。
    parameter ADAPTIVE_RETHROW_EN = 0,
    // 摆角质量门 / 共振驱动整形（仅 PUMP_MODE=2 用，MT6835 全 360° 下恒关）。
    parameter ALPHA_QUALITY_EN = 0,
    parameter RES_DRIVE_SHAPING_EN = 0,
    parameter signed [15:0] RES_RISE_STEP = 16'sd150,
    // 甩摆到达撤力半窗（0.01°，0=旧行为）。见 swingup_core 同名参数。
    parameter signed [31:0] SWING_DEAD = 32'sd0,
    // 三相位甩摆（透传 swingup_core）。0=旧行为（纯比例伺服）；T20K 1=加速→刹车
    // →阻尼保持。机理：比例伺服甩后 ±100°/1Hz 等幅振荡会随机对摆杆做功，是
    // "有时一杆就起、有时 40s 起不来"的来源。详见 swingup_core 实现处注释。
    parameter FLING_BRAKE_EN = 0,
    parameter signed [15:0] FLING_VSTOP = 16'sd200,
    parameter [5:0] FLING_VSTOP_CNT = 6'd40,
    parameter FLING_BRAKE_SH = 4,
    parameter [9:0] FLING_BRAKE_MS = 10'd600,
    parameter [9:0] FLING_BRAKE_MIN_MS = 10'd120,
    parameter [9:0] FLING_BRAKE_EXTRA_MS = 10'd0,
    // 刹车结束的摆杆速度门（透传 swingup_core，0=关闭）。T20K 开启。
    parameter FLING_VEND_EN = 0,
    parameter signed [15:0] FLING_VEND_V = 16'sd600,
    parameter [9:0] FLING_VEND_MIN_MS = 10'd150,
    parameter signed [15:0] FLING_ALPHA_END = 16'sd6000,
    parameter [9:0] FLING_ALPHA_MIN_MS = 10'd60,
    parameter signed [15:0] FLING_HOLD_KP = 16'sd4,
    parameter signed [15:0] FLING_HOLD_KV = 16'sd6,
    parameter signed [15:0] FLING_HOLD_MAX = 16'sd4000,
    parameter signed [15:0] FLING_HOLD_DZ = 16'sd250,
    // 换向斜率限幅（duty/tick，0=不限=旧行为）与 HOLD 相只短路制动
    parameter [15:0] FLING_SLEW_STEP = 16'd0,
    parameter FLING_HOLD_BRAKE = 0,
    // 相位驱动刹车（1=新）：ACCEL→COAST→BRAKE→COAST，全部按摆杆相位切换
    parameter FLING_PHASE_BRAKE = 0,
    parameter signed [15:0] FLING_BRAKE_STOP = 16'sd1500,
    parameter signed [15:0] FLING_BRAKE_FLIP = 16'sd300,
    parameter [9:0] FLING_COAST_MS = 10'd900,
    // 相位 + 能量门补甩（1=新）：底部速度还没到过顶所需就立刻补一杆
    parameter RETHROW_PHASE_EN = 0,
    parameter signed [15:0] RETHROW_PHASE_V = 16'sd5200,
    parameter signed [15:0] RETHROW_PHASE_ADOT = 16'sd300,
    parameter signed [15:0] RETHROW_PHASE_ALPHA = 16'sd15000,
    parameter [9:0] RETHROW_PHASE_GAP = 10'd500,
    // 滑行相微剂量能量伺服（透传 swingup_core，0=关闭）。T20K 开启。
    parameter COAST_DOSE_EN = 0,
    parameter [31:0] COAST_E_TARGET = 32'd12600,
    parameter [31:0] COAST_E_HYST   = 32'd800,
    parameter signed [15:0] COAST_AMP = 16'sd2500,
    parameter [9:0] COAST_DOSE_MS = 10'd120,
    parameter [9:0] COAST_GAP_MS  = 10'd180,
    parameter [1:0] COAST_MAX     = 2'd2,
    // 双向能量泵（PUMP_MODE=4）：方向取 sign(α̇·cosα) 取反，幅度固定，防抖保持
    parameter signed [15:0] BPUMP_AMP = 16'sd4000,
    parameter signed [15:0] BPUMP_ADOT_MIN = 16'sd300,
    parameter signed [15:0] BPUMP_V_STOP = 16'sd9500,
    parameter signed [15:0] BPUMP_V_SOFT = 16'sd7800,
    parameter signed [15:0] BPUMP_AMP_SOFT = 16'sd2500,
    parameter signed [31:0] BPUMP_E_TARGET = 32'sd12000,
    parameter signed [31:0] BPUMP_E_HYST   = 32'sd1500,
    parameter BPUMP_HYBRID_EN = 0,
    parameter signed [15:0] BPUMP_APEX_TRIG = 16'sd6000,
    parameter [11:0] BPUMP_HYBRID_GAP = 12'd1200,
    parameter [9:0] BPUMP_DWELL_MS = 10'd40,
    // 两段式起摆 stage2：底部相位微冲（透传 swingup_core，机理见该文件）
    parameter TRIM_EN = 0,
    parameter [11:0] TRIM_DANGLE = 12'd1500,
    parameter [9:0] TRIM_MS = 10'd80,
    parameter signed [15:0] TRIM_ALPHA = 16'sd16000,
    parameter signed [15:0] TRIM_APEX_MAX = 16'sd1200,
    parameter signed [15:0] TRIM_ADOT_MIN = 16'sd500,
    parameter [3:0] TRIM_MAX = 4'd2,
    parameter [9:0] TRIM_GAP_MS = 10'd900,
    // ---- 单极性相位泵（PUMP_MODE=3，2026-10-22）----
    // duty 只取 0/+amplitude、制动只做短路制动 ⇒ 结构性零电源换向。
    // 机理与实板符号依据见 swingup_core.v 同名参数段。
    parameter signed [15:0] PUMP_GATE_ALPHA = 16'sd6000,
    parameter signed [15:0] PUMP_PHASE_MIN = 16'sd200,
    parameter [9:0] PUMP_PULSE_MS = 10'd200,
    parameter [9:0] PUMP_PULSE_MIN_MS = 10'd60,
    parameter [9:0] PUMP_REST_MS = 10'd120,
    parameter signed [15:0] PUMP_TD_CAP = 16'sd6000,
    parameter signed [15:0] PUMP_TD_STOP = 16'sd800,
    parameter signed [15:0] PUMP_KICK_ALPHA = 16'sd15000,
    parameter signed [15:0] PUMP_KICK_ADOT = 16'sd400,
    parameter [9:0] PUMP_KICK_HOLD_MS = 10'd200,
    parameter signed [15:0] PUMP_ADOT_CAP = 16'sd7000,
    // 能量泵模式/增益/目标（透传 swingup_core）。DE10 默认 0=fling 行为不变；
    // T20K 1=能量伺服泵（K_E Q8=384、E_target=758 ⇔ apex≈22°）。
    parameter PUMP_MODE = 0,
    parameter signed [15:0] PUMP_KE = 16'sd384,
    parameter signed [31:0] PUMP_E_TARGET = 32'sd758,
    // 补甩底座速度门（透传 swingup_core）。T20K 30000（300°/s）。
    parameter RETHROW_TD_MAX = 16'sd32767,
    // SWING 换向驻留守卫（ms，0=禁用=行为不变）。齿轮箱不允许快速换向：
    // 任何一次极性变化后该窗口内禁止再变号，需要反向时输出 0 滑行等待。
    // T20K 配 120 ⇒ 翻转率硬性 ≤8.3 次/秒。见 rev_guard.v。
    parameter [9:0] SWING_REV_GUARD_MS = 10'd0,
    // 捕获增强相剂量：K2×NUM>>>SH、K4×NUM>>>SH（800ms 窗口）。K2/K4 分开
    // 参数化（2026-10-21：K2 增强在高入场摆速下是掉摆主因，T20K 配 ×1）。
    // NUM==SH 常数下限时乘法链常量折叠，与参数化前综合结果一致。
    parameter CATCH_K2_NUM = 8,
    parameter CATCH_K2_SH  = 3,
    // 捕获窗内抬臂位置增益目标值（0=禁用，行为与改动前逐位一致）。
    // T20K 配 2064（≈POS 档）：K3 从 600 渐变到 2064，捕获窗内主动消臂残余速度。
    parameter signed [15:0] CATCH_K3 = 16'sd0,
    // 捕获窗目标成对化（2026-11-01 极点扫描定案）：1=窗目标走运行时 POS 对
    // pos_k3/pos_k4（K3/K4 同 blend 成对抬，路径全中间态稳定）；0=旧 CATCH_K3
    // 常数单抬 K3（K4 不动，窗满裕度仅基础点一半，留档对照）。
    parameter CATCH_PAIR_EN = 0,
    // α 满量程口径（透传 angle_proc，Q16：counts→0.01°）。默认 34534 =
    // 65472↔345°（LTC2308/ADS1220 电位器链，DE10/J280 不变）；T20K+MT6701
    // 全 360° 直读传 36000（65536↔360.00°）。仅换算常数，控制律零改动。
    parameter [15:0] ALPHA_SCALE_Q16 = 16'd34534,
    // 观测器状态源开关（2026-10-16 双编码器方案）。0=默认，差分链直通，
    // 行为与原实现逐比特一致；1=BALANCE/POS 且 |α|<30° 由 obs_core 接管。
    parameter        OBS_USE = 1'b0,
    // 竖直零位自校准门（2026-10-20）：稳摆物理上只平衡在真竖直，BALANCE 期间
    // 测得 α 的慢 EMA=传感器零偏，退出 BALANCE 沿一次性提交进 offset（会话内
    // 控制行为零变化）。默认 0：j280/de10 行为零变化；T20K 覆盖 1。
    parameter        AZ_ENABLE = 1'b0,
    // 电机侧 MT6701 θ 链方向取反（experiment/mt6701-theta）。实机方向核对后
    // 定案：MT6701 角增方向与 qenc4x 计数方向相反时置 1。
    parameter        TH_MT_DIR_FLIP = 1'b0,
    // 只用 MT6701 一条 θ 链（2026-10-22，T20K 霍尔编码器已物理拆除）。
    // 1=不实例化 arm_vel（qenc4x→cnt2deg 整条链，含 28 周期 T 法除法器）与
    // 双源 mux；arm_stalled 由 theta_mt_path 的角度事件判停提供。
    // 0=保留双源语义（J280/DE10 板仍有霍尔编码器，行为逐位不变）。
    parameter        TH_SRC_MT_ONLY = 1'b0,
    // ---- 输出级整形（2026-11-02 P3b，移植 golden-base 板验配置，见 out_stage 例化处）----
    parameter signed [15:0] OUT_SLEW_MAX     = 16'sd3000,  // SWING 档每 tick 最大变化
    parameter signed [15:0] OUT_SLEW_SOFT    = 16'sd3000,  // BALANCE/POS 档（golden 实测 700 退化回 3000）
    parameter signed [15:0] OUT_DZ_SOFT_TH   = 16'sd0,     // 死区软区阈值（0=旧行为）
    parameter signed [15:0] OUT_DZ_THDOT_REF = 16'sd0,     // 死区速度自适应参考（0=关）
    parameter signed [15:0] OUT_BACKLASH_BOOST = 16'sd0,   // 齿隙补偿幅值（0=关）
    parameter [9:0]         OUT_BACKLASH_MS    = 10'd30,   // 齿隙补偿时长
    parameter signed [15:0] OUT_REV_HYST       = 16'sd0,   // 换向滞环阈值（0=关）
    parameter DZ_TH_EN = 0                                  // dz(θ) 位置查表使能（out_stage）
) (
    input  wire        clk,
    input  wire        rst_n,

    // 传感器（来自板级外壳的传感前端）
    input  wire [15:0] adc_raw,       // adc_if 平均字
    input  wire        otr_fault,     // adc_if 越界粘滞
    input  wire signed [31:0] enc_pos,
    // 电机侧 MT6701 绝对角（experiment/mt6701-theta 第二 θ 源；无该传感器的
    // 板级外壳恒接 14'd0，th_src_mt 恒 0 → 本链不参与，行为与原实现一致）
    input  wire [13:0] mt_angle14,
    input  wire        th_src_mt,     // 1=θ/θ̇ 用 MT6701 链（板级外壳同步后送入）
    // 电机侧 MT6701 链路健康（STATUS 应答导出；无该传感器的外壳恒 0）
    input  wire [15:0] mt6_frames, input wire [15:0] mt6_good,
    input  wire [15:0] mt6_crc_err, input wire [15:0] mt6_mg_err,
    input  wire [2:0]  mt6_mg_code, input wire [13:0] mt6_angle_raw,
    input  wire        mt6_fault,     // 驱动故障电平（已含选中语义由外壳组合）
    input  wire [1:0]  mt6_cause,     // MT6701 最近锁存原因（0 看门狗/1 CRC/2 场强/3 失跟踪）
    input  wire [15:0] mt6_max_no_good, // MT6701 最长连续无好帧段（帧数，×~25µs）
    // 摆侧 MT6835 链路健康（STATUS 应答导出；无该传感器的板卡恒 0）
    input  wire [15:0] mt68_frames, input wire [15:0] mt68_good,
    input  wire [15:0] mt68_crc_err, input wire [15:0] mt68_status_err,
    input  wire [2:0]  mt68_status, input wire [20:0] mt68_angle_raw,
    input  wire        mt68_fault,    // 驱动故障电平（通常即安全链 otr_fault 源）

    // HIL 锁步协同仿真（DE10 无硬件验证用；j280 外壳恒 0 → 行为与原顶层完全一致）
    //   hil_use=1：传感器字改由上位机（被控对象模型）经 STEP 命令送入，控制
    //   节拍也由 STEP 驱动，每个命令恰好推进一个仿真毫秒。上位机每拍回读遥测
    //   取得 duty 后再送下一拍，节拍严格乒乓同步，故轨迹与 1kHz 实跑一致。
    //   看门狗在该模式下停用（上位机往返远超 2ms，见 safety 实例）。
    input  wire        hil_use,

    // 人机
    input  wire [3:0]  key_sw,        // 低有效
    input  wire [1:0]  dip_sw,
    output wire [3:0]  led,

    // 串口
    input  wire        uart_rx,
    output wire        uart_tx,

    // 输出
    output wire signed [15:0] duty_out,
    output wire        motor_enable,  // run_en & ~remote_stop & 模式门控
    output wire [2:0]  mode,          // 仅供仿真观察/板级 LED 复用
    output wire signed [15:0] alpha,
    output wire signed [15:0] alpha_dot,
    output wire signed [31:0] theta_c01,
    output wire signed [15:0] theta_dot,
    output wire signed [31:0] diag_encoder,
    output wire [15:0] diag_adc,
    // ---- 遥测诊断量（2026-10-21 记录格式 v3）----
    //   pos_trim      POS 积分器实际输出的 duty（臂到底靠什么力矩站住）
    //   alpha_pre     trim 之前的 α（当轮真实零偏，免去 offset−trim 反推）
    //   k3_eff/k4_eff 渐变态之后的实际增益（把入场瞬态与渐变分离）
    //   duty_pre_dz   死区补偿前的指令（640/740 到底加了多少，可直接核对）
    //   obs_active    观测器是否接管（OBS_USE=1 后必须能确认走的是哪条链）
    output wire signed [15:0] diag_pos_trim,
    output wire signed [15:0] diag_alpha_pre,
    output wire signed [15:0] diag_k3_eff,
    output wire signed [15:0] diag_k4_eff,
    output wire signed [15:0] diag_duty_pre_dz,
    output wire        diag_obs_active,
    // v4 遥测（2026-10-27）：θe + 四个分项。term 恒 0（不新增乘积链），
    // 由上位机按 k_eff 与 wa 权重重建；θe 是 POS 诊断的关键量。
    output wire signed [15:0] diag_theta_e,
    output wire signed [15:0] diag_term_k1,
    output wire signed [15:0] diag_term_k2,
    output wire signed [15:0] diag_term_k3,
    output wire signed [15:0] diag_term_k4,
    output wire io_test_enable,
    output wire signed [15:0] io_test_duty,
    output wire        tick_out,      // 控制节拍（板级调试观察用）
    output wire        fault,         // 故障锁存原始电平（DE10 调试桥遥测用）
    output wire        swing_brake,   // 1 = SWING 刹车相：板级据此开短路制动
    // ---- v6 遥测（2026-11-02）：级联/降额/稳定窗/LUT 内部状态 ----
    output wire signed [15:0] diag_pos_bias,   // 级联倾斜偏置（注入 LQR α 的实际值）
    output wire [15:0]        diag_move_blend, // 移动降额 blend（0=满额 1024=降额）
    output wire [15:0]        diag_settle_ms,  // 入场稳定窗计数
    output wire [3:0]         diag_dz_idx      // dz(θ) LUT 格号
);
    // ---------------- 传感器源选择 ----------------
    // hil_tick/hil_adc/hil_enc 由 debug_link 的 STEP 命令驱动（见下方实例）。
    wire        hil_tick;
    wire [15:0] hil_adc;
    wire signed [31:0] hil_enc;
    wire [15:0] adc_src = hil_use ? hil_adc : adc_raw;
    wire signed [31:0] enc_src = hil_use ? hil_enc : enc_pos;

    // ---------------- 节拍 ----------------
    wire tick_free;
    wire tick = hil_use ? hil_tick : tick_free;
    wire        fault_latch;
    assign fault = fault_latch;   // DE10 调试桥遥测
    assign diag_encoder = enc_pos;
    assign diag_adc = adc_raw;
    wire        run_en;
    wire remote_start,remote_stop,remote_clear,remote_mode,remote_calib;
    wire remote_force_pos;   // FORCE_POS：绕过 pos_ready 强制进 M_POS（2026-11-03）
    wire vel_demo_en;        // VEL_DEMO：速度曲线演示（固件内生三角波 v_ref）
    wire signed [15:0] vel_demo_v, vel_demo_a;
    wire move_pending,config_lock;
    wire signed [31:0] move_delta;
    wire [31:0] config_revision,rx_errors;
    reg [11:0] calibration_wait;
    wire calibration_allowed=(mode==`M_SAFE || mode==`M_IDLE) && !config_lock;
    wire calibration_request=(key_calib || remote_calib) && calibration_allowed;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)calibration_wait<=0;
        else if(tick)begin
            if(calibration_request)calibration_wait<=12'd2100;
            else if(calibration_wait!=0)calibration_wait<=calibration_wait-1'b1;
        end
    end
    loop_timer u_tick (.clk(clk), .rst_n(rst_n), .tick(tick_free));

    // ---------------- 传感 ----------------
    // θ 源（2026-10-22 定案）：T20K 上**霍尔编码器已物理拆除**，只剩电机侧
    // MT6701 绝对角→theta_mt_path（÷20.45，1ms 差分测速）。TH_SRC_MT_ONLY=1
    // 时整条霍尔链（qenc4x 计数→arm_vel→cnt2deg，含 28 周期的 T 法除法器）
    // 被常量折叠掉，同时 arm_stalled 改由 theta_mt_path 提供（那边按角度事件
    // 判停，语义与 arm_vel 的 v_t_zero 一致）。
    // TH_SRC_MT_ONLY=0 保留双源语义（J280/DE10 板仍有霍尔编码器，行为逐位不变）。
    wire signed [31:0] theta_c01_i;
    wire signed [15:0] theta_dot_i;
    wire arm_stalled;
    wire signed [31:0] theta_mt_c01;
    wire signed [15:0] theta_mt_dot;
    wire               mt_stalled;
    theta_mt_path #(.SLEW_LIM(THDOT_SLEW_LIM),
                    .DIR_FLIP(TH_MT_DIR_FLIP)) u_thmt (
        .clk(clk), .rst_n(rst_n), .tick(tick), .angle14(mt_angle14),
        .theta_c01(theta_mt_c01), .theta_dot(theta_mt_dot),
        .arm_stalled(mt_stalled)
    );
    generate if (TH_SRC_MT_ONLY) begin : g_th_mt
        assign theta_c01_i = theta_mt_c01;
        assign theta_dot_i = theta_mt_dot;
        assign arm_stalled = mt_stalled;
    end else begin : g_th_dual
        wire signed [31:0] theta_hall_c01;
        wire signed [15:0] theta_hall_dot;
        arm_vel #(.SLEW_LIM(THDOT_SLEW_LIM),
                  .PIPELINE_ON_TICK(THDOT_PIPELINE_ON_TICK)) u_armvel (
            .clk(clk), .rst_n(rst_n), .tick(tick),
            .pos(enc_src), .theta_c01(theta_hall_c01), .theta_dot(theta_hall_dot),
            .arm_stalled(arm_stalled)
        );
        assign theta_c01_i = th_src_mt ? theta_mt_c01 : theta_hall_c01;
        assign theta_dot_i = th_src_mt ? theta_mt_dot : theta_hall_dot;
    end endgenerate

    wire        key_calib, calib_done;

    // ================= EC 偏心前馈（P1，移植 rom-baseline-20261030 11-01）=================
    // trim(θ) = EC_SIN*sinθ + EC_COS*cosθ：嵌得（偏心）造成的 α 零位随 θ 正弦变化，
    // 恒定 trim 只能补一个角度——臂转到别处偏心变了就是持续推臂的净偏置。
    // 本机标定（两次静态点动，记忆 theta-dependent-alpha-zero-offset）：
    //   A≈1.10°, φ≈−89° ⇒ EC_SIN=A·cosφ≈+2 cdeg, EC_COS=A·sinφ≈−110 cdeg
    // EC_EN=0 时 ec_trim 恒 0，行为逐位等于旧版。
    localparam        EC_EN   = 1'b1;
    localparam signed [15:0] EC_BIAS = 16'sd0;
    localparam signed [15:0] EC_SIN  = 16'sd2;     // = A·cos(φ)
    localparam signed [15:0] EC_COS  = -16'sd110;  // = A·sin(φ)
    reg signed [15:0] ec_trim;         // trim(θ) 偏心补偿量（EC_EN=0 时恒 0）
function signed [12:0] sin_lut;
    input [6:0] idx;   // 0..90 => 0..90deg
    begin case (idx)
        7'd0 : sin_lut = 13'sd0;
        7'd1 : sin_lut = 13'sd71;
        7'd2 : sin_lut = 13'sd143;
        7'd3 : sin_lut = 13'sd214;
        7'd4 : sin_lut = 13'sd286;
        7'd5 : sin_lut = 13'sd357;
        7'd6 : sin_lut = 13'sd428;
        7'd7 : sin_lut = 13'sd499;
        7'd8 : sin_lut = 13'sd570;
        7'd9 : sin_lut = 13'sd641;
        7'd10: sin_lut = 13'sd711;
        7'd11: sin_lut = 13'sd782;
        7'd12: sin_lut = 13'sd852;
        7'd13: sin_lut = 13'sd921;
        7'd14: sin_lut = 13'sd991;
        7'd15: sin_lut = 13'sd1060;
        7'd16: sin_lut = 13'sd1129;
        7'd17: sin_lut = 13'sd1198;
        7'd18: sin_lut = 13'sd1266;
        7'd19: sin_lut = 13'sd1334;
        7'd20: sin_lut = 13'sd1401;
        7'd21: sin_lut = 13'sd1468;
        7'd22: sin_lut = 13'sd1534;
        7'd23: sin_lut = 13'sd1600;
        7'd24: sin_lut = 13'sd1666;
        7'd25: sin_lut = 13'sd1731;
        7'd26: sin_lut = 13'sd1796;
        7'd27: sin_lut = 13'sd1860;
        7'd28: sin_lut = 13'sd1923;
        7'd29: sin_lut = 13'sd1986;
        7'd30: sin_lut = 13'sd2048;
        7'd31: sin_lut = 13'sd2110;
        7'd32: sin_lut = 13'sd2171;
        7'd33: sin_lut = 13'sd2231;
        7'd34: sin_lut = 13'sd2290;
        7'd35: sin_lut = 13'sd2349;
        7'd36: sin_lut = 13'sd2408;
        7'd37: sin_lut = 13'sd2465;
        7'd38: sin_lut = 13'sd2522;
        7'd39: sin_lut = 13'sd2578;
        7'd40: sin_lut = 13'sd2633;
        7'd41: sin_lut = 13'sd2687;
        7'd42: sin_lut = 13'sd2741;
        7'd43: sin_lut = 13'sd2793;
        7'd44: sin_lut = 13'sd2845;
        7'd45: sin_lut = 13'sd2896;
        7'd46: sin_lut = 13'sd2946;
        7'd47: sin_lut = 13'sd2996;
        7'd48: sin_lut = 13'sd3044;
        7'd49: sin_lut = 13'sd3091;
        7'd50: sin_lut = 13'sd3138;
        7'd51: sin_lut = 13'sd3183;
        7'd52: sin_lut = 13'sd3228;
        7'd53: sin_lut = 13'sd3271;
        7'd54: sin_lut = 13'sd3314;
        7'd55: sin_lut = 13'sd3355;
        7'd56: sin_lut = 13'sd3396;
        7'd57: sin_lut = 13'sd3435;
        7'd58: sin_lut = 13'sd3474;
        7'd59: sin_lut = 13'sd3511;
        7'd60: sin_lut = 13'sd3547;
        7'd61: sin_lut = 13'sd3582;
        7'd62: sin_lut = 13'sd3617;
        7'd63: sin_lut = 13'sd3650;
        7'd64: sin_lut = 13'sd3681;
        7'd65: sin_lut = 13'sd3712;
        7'd66: sin_lut = 13'sd3742;
        7'd67: sin_lut = 13'sd3770;
        7'd68: sin_lut = 13'sd3798;
        7'd69: sin_lut = 13'sd3824;
        7'd70: sin_lut = 13'sd3849;
        7'd71: sin_lut = 13'sd3873;
        7'd72: sin_lut = 13'sd3896;
        7'd73: sin_lut = 13'sd3917;
        7'd74: sin_lut = 13'sd3937;
        7'd75: sin_lut = 13'sd3956;
        7'd76: sin_lut = 13'sd3974;
        7'd77: sin_lut = 13'sd3991;
        7'd78: sin_lut = 13'sd4006;
        7'd79: sin_lut = 13'sd4021;
        7'd80: sin_lut = 13'sd4034;
        7'd81: sin_lut = 13'sd4046;
        7'd82: sin_lut = 13'sd4056;
        7'd83: sin_lut = 13'sd4065;
        7'd84: sin_lut = 13'sd4074;
        7'd85: sin_lut = 13'sd4080;
        7'd86: sin_lut = 13'sd4086;
        7'd87: sin_lut = 13'sd4090;
        7'd88: sin_lut = 13'sd4094;
        7'd89: sin_lut = 13'sd4095;
        7'd90: sin_lut = 13'sd4096;
        default: sin_lut = 13'sd4096;
    endcase end
endfunction
    generate if (EC_EN) begin : g_ec
        reg [20:0] th_deg_r;
        reg [1:0]  quad_r;
        reg [8:0]  a_deg_r;
        reg signed [12:0] s_a_r, s_b_r;
        reg        sin_neg_r, cos_neg_r;
        reg signed [27:0] acc_ec_r;
        wire signed [31:0] th_in = theta_c01;
        wire signed [32:0] th_n1 = (th_in >= 32'sd36000) ? (th_in - 32'sd36000) :
                                    (th_in <  32'sd0)     ? (th_in + 32'sd36000) : th_in;
        wire [31:0] th_norm = (th_n1[32]) ? 32'd0 :
                              (th_n1 >= 33'sd36000) ? (th_n1[31:0] - 32'sd36000) : th_n1[31:0];
        always @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin
                th_deg_r <= 21'd0; quad_r <= 2'd0; a_deg_r <= 9'd0;
                s_a_r <= 13'sd0; s_b_r <= 13'sd0; sin_neg_r <= 1'b0; cos_neg_r <= 1'b0;
                acc_ec_r <= 28'sd0;
            end else if (tick) begin
                th_deg_r <= (th_norm * 21'd655) >> 16;
                quad_r   <= (th_deg_r[20:0] >= 21'd270) ? 2'd3 :
                            (th_deg_r[20:0] >= 21'd180) ? 2'd2 :
                            (th_deg_r[20:0] >= 21'd90)  ? 2'd1 : 2'd0;
                a_deg_r  <= (th_deg_r[20:0] >= 21'd270) ? (th_deg_r[8:0] - 9'd270) :
                            (th_deg_r[20:0] >= 21'd180) ? (th_deg_r[8:0] - 9'd180) :
                            (th_deg_r[20:0] >= 21'd90)  ? (th_deg_r[7:0] - 8'd90)  : th_deg_r[8:0];
                s_a_r     <= sin_lut(a_deg_r[6:0]);
                s_b_r     <= sin_lut(7'd90 - a_deg_r[6:0]);
                sin_neg_r <= (quad_r == 2'd2) || (quad_r == 2'd3);
                cos_neg_r <= (quad_r == 2'd1) || (quad_r == 2'd2);
                acc_ec_r  <= $signed(EC_SIN) *
                             (sin_neg_r ? -$signed({1'b0, quad_r[0] ? s_b_r : s_a_r})
                                        :  $signed({1'b0, quad_r[0] ? s_b_r : s_a_r}))
                           + $signed(EC_COS) *
                             (cos_neg_r ? -$signed({1'b0, quad_r[0] ? s_a_r : s_b_r})
                                        :  $signed({1'b0, quad_r[0] ? s_a_r : s_b_r}));
            end
        end
        always @(posedge clk or negedge rst_n) begin
            if (!rst_n) ec_trim <= 16'sd0;
            else if (tick) ec_trim <= sat16_ec((acc_ec_r >>> 12) + $signed(EC_BIAS));
        end
    end else begin : g_noec
        always @(posedge clk or negedge rst_n) begin
            if (!rst_n) ec_trim <= 16'sd0;
            else if (tick) ec_trim <= 16'sd0;
        end
    end endgenerate
    function signed [15:0] sat16_ec(input signed [31:0] v);
        sat16_ec = (v > 32'sd1600) ? 16'sd1600 :
                   (v < -32'sd1600) ? -16'sd1600 : v[15:0];
    endfunction
    // ================= EC 结束 =================
        wire signed [15:0] alpha_i, alpha_dot_i, offset_c01;
    wire signed [15:0] alpha_pre_w;       // trim 前的 α（遥测诊断用）
    // α 零位：offset_c01 来自下垂标定（CAL），alpha_trim 来自运行时参数 id13
    // （2026-10-20：自动自校准过冲严重——序列 15241→15534→15578→15196→…在
    // 15200-15600 间震荡，且会把"臂运动导致的表观 α 偏移"当零偏修。改为
    // 关掉 AZ、由上位机按全段标定结果写入 id13，每轮开始前手动设定）。
    // 两者相加：CAL 打底（测下垂点），trim 做标定后微调。
    wire signed [15:0] az_en = 1'b0;   // AZ 关闭（AZ_ENABLE 参数保留作门控，见下）
    angle_proc #(
        .SCALE_Q16(ALPHA_SCALE_Q16),
        .AZ_TRIM_EN(1'b0)              // 禁用 az_en 相关逻辑（保留代码路径）
    ) u_ang (
        .clk(clk), .rst_n(rst_n), .tick(tick),
        .alpha_avg(adc_src),
        .calib_start(calibration_request),
        .az_en(AZ_ENABLE && az_en),
        .trim_c01(alpha_trim + ec_trim),   // 常数 trim + 偏心补偿（EC_EN=0 时后者恒 0）
        .alpha(alpha_i), .alpha_pre(alpha_pre_w), .alpha_dot(alpha_dot_i),
        .offset_c01(offset_c01), .calib_done(calib_done)
    );

    // ---------------- 观测器状态源（OBS_USE，2026-10-16 双编码器方案） ----------------
    // 4 状态稳态 Kalman（j280_obs_gains.vh，matlab/obs_to_rtl.m 生成）。使能时
    // BALANCE/POS 且 |α|<30° 由观测器接管 α/α̇/θ/θ̇；SWING/SAFE/IDLE 及越窗
    // 自动回退现行差分链——obs_core 从动模式保证切换零瞬态。OBS_USE=0（默认）
    // 时 obs_active 恒 0，四信号逐比特等于差分链，行为与原实现完全一致。
    wire obs_en = OBS_USE && (mode == `M_BALANCE || mode == `M_POS);
    wire        obs_act;
    wire signed [31:0] obs_theta_w;
    wire signed [15:0] obs_theta_dot_w, obs_alpha_w, obs_alpha_dot_w;
    reg  signed [15:0] obs_duty_q;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)          obs_duty_q <= 16'sd0;
        else if (tick)       obs_duty_q <= duty_out;   // 观测器输入：上一拍 duty
    end
    obs_core u_obs (
        .clk(clk), .rst_n(rst_n), .tick(tick), .obs_enable(obs_en),
        .theta_meas(theta_c01_i), .theta_dot_meas(theta_dot_i),
        .alpha_meas(alpha_i), .alpha_dot_meas(alpha_dot_i),
        .duty_prev(obs_duty_q),
        .obs_active(obs_act),
        .obs_theta(obs_theta_w), .obs_theta_dot(obs_theta_dot_w),
        .obs_alpha(obs_alpha_w), .obs_alpha_dot(obs_alpha_dot_w)
    );
    assign theta_c01 = obs_act ? obs_theta_w     : theta_c01_i;
    assign theta_dot = obs_act ? obs_theta_dot_w : theta_dot_i;
    assign alpha     = obs_act ? obs_alpha_w     : alpha_i;
    assign alpha_dot = obs_act ? obs_alpha_dot_w : alpha_dot_i;

    // ---------------- 人机 ----------------
    wire key_start, key_mode, key_stop;
    wire [1:0] dip;
    ui u_ui (
        .clk(clk), .rst_n(rst_n), .tick(tick),
        .key_sw(key_sw), .dip_sw(dip_sw),
        .mode(mode), .fault_latch(fault_latch), .calib_done(calib_done),
        .key_start(key_start), .key_mode(key_mode),
        .key_calib(key_calib), .key_stop(key_stop),
        .dip(dip), .led(led)
    );

    // ---------------- 参数 ----------------
    wire signed [15:0] k1, k2, k3, k4, dz_pos, dz_neg, sw_amp,
                       catch_ang, catch_ang_fall, theta_ref_lo, fling_delta,
                       pos_k3, pos_k4, alpha_trim;
    // 滑行补能运行时参数（id14/15）在 14 参 schema 中不存在：接 0。
    // COAST_DOSE_EN=0 时 swingup_core 不消费这两个值，仅满足端口连接。
    wire signed [15:0] coast_amp_w = 16'sd0, coast_margin_w = 16'sd0;
    reg  [2:0] mode_prev;

    // ---------------- 模式 ----------------
    wire mode_safe = (mode == `M_IDLE) || (mode == `M_SAFE);
    mode_fsm #(.CATCH_TD_LIM(CATCH_TD_LIM), .CATCH_AD_LIM(CATCH_AD_LIM),
               .CATCH_DWELL_MS(CATCH_DWELL_MS), .CATCH_AD_MIN(CATCH_AD_MIN),
               .CATCH_DEAD_BAND(CATCH_DEAD_BAND), .CATCH_DEAD_AD(CATCH_DEAD_AD),
               .FALL_DWELL_MS(FALL_DWELL_MS),
               .AUTO_RESWING_EN(AUTO_RESWING_EN)) u_fsm (
        .clk(clk), .rst_n(rst_n), .tick(tick),
        .key_start((key_start || remote_start) && !config_lock &&
                   calibration_wait==0 && !calibration_request),
        .key_mode(key_mode || remote_mode), .key_stop(key_stop || remote_stop || remote_clear),
        .force_pos(remote_force_pos),
        .fault(fault_latch),
        .alpha(alpha), .alpha_dot(alpha_dot),
        .theta_dot(theta_dot),
        .arm_stalled(arm_stalled),
        .catch_ang(catch_ang), .catch_ang_fall(catch_ang_fall),
        .mode(mode)
    );

    safety u_safe (
        .clk(clk), .rst_n(rst_n),
        .tick(tick), .otr_fault(otr_fault), .mode_safe(mode_safe),
        .wdg_enable(~hil_use),   // HIL 锁步下上位机往返远超 2ms，停用节拍看门狗
        .fault_clr(key_stop || remote_clear),
        .run_en(run_en), .fault_latch(fault_latch), .led_fault()
    );

    // ---------------- 控制律 ----------------
    // SWING: fling swing-up（甩摆位置伺服）. BALANCE/POS: generated full-state feedback.
    wire entering_swing=(mode==`M_SWING && mode_prev!=`M_SWING);
    wire signed [15:0] duty_swing;
    swingup_core #(
        .RETHROW_MS(FLING_RETHROW_MS), .RETHROW_TD_MAX(RETHROW_TD_MAX),
        .RETHROW_ALPHA_MIN(RETHROW_ALPHA_MIN),
        .PUMP_MODE(PUMP_MODE), .PUMP_KE(PUMP_KE), .PUMP_E_TARGET(PUMP_E_TARGET),
        // 2026-10-22 从起摆线移植（自 adaptive-fling 起）：补甩摆速平静门 +
        // 自适应甩距 + 三相位甩摆 + 双向能量泵混合。本树（2026-11-01）起摆
        // 链取起摆冠配置（共振激振实测不行），BALANCE 稳态律仍为 10-20 简单栈。
        .RETHROW_ADOT_MAX(RETHROW_ADOT_MAX),
        .RETHROW_CALM_MS(RETHROW_CALM_MS),
        .ADAPTIVE_RETHROW_EN(ADAPTIVE_RETHROW_EN),
        .ALPHA_QUALITY_EN(ALPHA_QUALITY_EN),
        .RES_DRIVE_SHAPING_EN(RES_DRIVE_SHAPING_EN), .RES_RISE_STEP(RES_RISE_STEP),
        .SWING_DEAD(SWING_DEAD),
        .FLING_BRAKE_EN(FLING_BRAKE_EN), .FLING_VSTOP(FLING_VSTOP),
        .FLING_VSTOP_CNT(FLING_VSTOP_CNT),
        .FLING_BRAKE_SH(FLING_BRAKE_SH), .FLING_BRAKE_MS(FLING_BRAKE_MS),
        .FLING_BRAKE_MIN_MS(FLING_BRAKE_MIN_MS),
        .FLING_BRAKE_EXTRA_MS(FLING_BRAKE_EXTRA_MS),
        .FLING_VEND_EN(FLING_VEND_EN), .FLING_VEND_V(FLING_VEND_V),
        .FLING_VEND_MIN_MS(FLING_VEND_MIN_MS),
        .FLING_ALPHA_END(FLING_ALPHA_END), .FLING_ALPHA_MIN_MS(FLING_ALPHA_MIN_MS),
        .FLING_HOLD_KP(FLING_HOLD_KP), .FLING_HOLD_KV(FLING_HOLD_KV),
        .FLING_HOLD_MAX(FLING_HOLD_MAX), .FLING_HOLD_DZ(FLING_HOLD_DZ),
        .FLING_SLEW_STEP(FLING_SLEW_STEP), .FLING_HOLD_BRAKE(FLING_HOLD_BRAKE),
        .FLING_PHASE_BRAKE(FLING_PHASE_BRAKE), .FLING_BRAKE_STOP(FLING_BRAKE_STOP),
        .FLING_BRAKE_FLIP(FLING_BRAKE_FLIP), .FLING_COAST_MS(FLING_COAST_MS),
        .RETHROW_PHASE_EN(RETHROW_PHASE_EN), .RETHROW_PHASE_V(RETHROW_PHASE_V),
        .RETHROW_PHASE_ADOT(RETHROW_PHASE_ADOT),
        .RETHROW_PHASE_ALPHA(RETHROW_PHASE_ALPHA),
        .RETHROW_PHASE_GAP(RETHROW_PHASE_GAP),
        .COAST_DOSE_EN(COAST_DOSE_EN), .COAST_E_TARGET(COAST_E_TARGET),
        .COAST_E_HYST(COAST_E_HYST), .COAST_AMP(COAST_AMP),
        .COAST_DOSE_MS(COAST_DOSE_MS), .COAST_GAP_MS(COAST_GAP_MS),
        .COAST_MAX(COAST_MAX),
        .BPUMP_AMP(BPUMP_AMP), .BPUMP_ADOT_MIN(BPUMP_ADOT_MIN),
        .BPUMP_V_STOP(BPUMP_V_STOP), .BPUMP_DWELL_MS(BPUMP_DWELL_MS),
        .BPUMP_V_SOFT(BPUMP_V_SOFT), .BPUMP_AMP_SOFT(BPUMP_AMP_SOFT),
        .BPUMP_E_TARGET(BPUMP_E_TARGET), .BPUMP_E_HYST(BPUMP_E_HYST),
        .BPUMP_HYBRID_EN(BPUMP_HYBRID_EN), .BPUMP_APEX_TRIG(BPUMP_APEX_TRIG),
        .BPUMP_HYBRID_GAP(BPUMP_HYBRID_GAP),
        .TRIM_EN(TRIM_EN), .TRIM_MS(TRIM_MS), .TRIM_DANGLE(TRIM_DANGLE),
        .TRIM_ALPHA(TRIM_ALPHA),
        .TRIM_APEX_MAX(TRIM_APEX_MAX), .TRIM_ADOT_MIN(TRIM_ADOT_MIN),
        .TRIM_MAX(TRIM_MAX), .TRIM_GAP_MS(TRIM_GAP_MS),
        .PUMP_GATE_ALPHA(PUMP_GATE_ALPHA), .PUMP_PHASE_MIN(PUMP_PHASE_MIN),
        .PUMP_PULSE_MS(PUMP_PULSE_MS), .PUMP_PULSE_MIN_MS(PUMP_PULSE_MIN_MS),
        .PUMP_REST_MS(PUMP_REST_MS),
        .PUMP_TD_CAP(PUMP_TD_CAP), .PUMP_TD_STOP(PUMP_TD_STOP),
        .PUMP_KICK_ALPHA(PUMP_KICK_ALPHA), .PUMP_KICK_ADOT(PUMP_KICK_ADOT),
        .PUMP_KICK_HOLD_MS(PUMP_KICK_HOLD_MS),
        .PUMP_ADOT_CAP(PUMP_ADOT_CAP)
    ) u_swing (
        .clk(clk), .rst_n(rst_n), .tick(tick), .start(entering_swing),
        .theta(theta_c01), .theta_dot(theta_dot), .alpha(alpha),
        .alpha_dot(alpha_dot),
        .delta(fling_delta), .amplitude(sw_amp), .duty(duty_swing),
        .catch_ang(catch_ang),
        .coast_amp(coast_amp_w), .coast_margin(coast_margin_w),  // 14 参无 id14/15，恒 0
        .brake_req(swing_brake),
        // MT6835 SPI 全 360° 直读，无电位器盲区；端口保留恒 1（同起摆线）。
        .alpha_valid(1'b1), .swing_active(mode == `M_SWING)
    );
    // All references use continuous 0.01deg. DIP selects +d,-d,+2d,-2d.
    // Freeze BOTH anchor and requested offset on entry into POS.
    wire signed [31:0] requested = {{16{theta_ref_lo[15]}},theta_ref_lo};
    wire signed [31:0] scaled = dip[1] ? (requested <<< 1) : requested;
    wire signed [31:0] delta_pos = move_pending ? move_delta : (dip[0] ? -scaled : scaled);
    reg signed [31:0] pos_tgt, pos_hold;
    reg signed [31:0] pos_cmd_delta;   // MOVE 位移命令锁存（settle 结束后从当前臂位兑现）
    // 目标缓速分解（2026-11-03 晚，用户定案）：目标不跳变，固件内以恒速
    // POS_TGT_SLEW c01/tick 逐拍逼近——位置误差始终小、rehold 不再吃命令，
    // 并且这条匀速拉升就是可观测的位置/速度曲线（拓展2）。
    localparam signed [31:0] POS_TGT_SLEW = 32'sd4;   // 4 c01 每 POS_SLEW_DIV 拍
    // 位置控制要"最稳最快到位"：默认不细分频 = 4 c01/ms = 40°/s（历史长期值，
    // demo 步进能到位）。4°/s 的慢速曲线留给后续"速度曲线"扩展再单独开关。
    localparam integer       POS_SLEW_DIV = 2;        // 4c01/2ms = 20°/s（慢速旋转档）
    // ★2026-11-03 夜单位修正：原实现"每拍 +4 c01" = 4×0.01°/1ms = **40°/s**
    // （注释写的 4°/s 错了 10 倍；历史"缓速分解"实际一直是 40°/s，55° 步进
    // 1.4s 走完像跳变）。加 ÷10 分频后才是真正的 4°/s 速度曲线：30° 需 7.5s。
    reg [3:0] slew_div;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) slew_div <= 4'd0;
        else if (tick) slew_div <= (slew_div >= POS_SLEW_DIV-1) ? 4'd0 : slew_div + 4'd1;
    end
    reg signed [31:0] pos_tgt_cmd;      // 最终目标（滑坡的终点）
    wire entering_pos=(mode==`M_POS && mode_prev!=`M_POS);
    // ---- ★轮3-② 微步蠕行（末段突发式进锁，2026-11-04 晨）----
    // 依据：K3=1200 弹簧太软（0.5° 阶跃只出 7 duty，顶不开 640 死区），纯目标
    // 微步蠕不动臂。改"突发式"：末段 5° 内每 400ms 发一发 120ms 突发——目标
    // 跳 3° + 弹簧 ×8（≈351 duty + dz 补偿 → 可突破），突发结束弹簧回落，臂被
    // 静摩擦当场钉住（粘滑棘轮机制），动量不累积 ⇒ 无超调、极限环不被激发，
    // 到位即低能量 → FRICTION_LOCK 秒开；齿轮箱换向次数同步大降。
    // CREEP_EN=0 → 逐位回退 20°/s 恒斜坡。BURST/PERIOD 以 slew 分支执行周期
    // （POS_SLEW_DIV=2 → 2ms/次）计：60 次=120ms、200 次=400ms。
    localparam CREEP_EN          = 1'b1;   // ★R4v2：蠕行启用（R3v2 位流=CREEP_EN=0）
    localparam integer CREEP_ZONE   = 500; // 末段 5° 进蠕行
    localparam integer CREEP_STEP   = 300; // 每发目标跳 3°
    localparam integer CREEP_BURST  = 60;  // ×2ms = 120ms 突发
    localparam integer CREEP_PERIOD = 200; // ×2ms = 400ms 周期
    localparam integer CREEP_K3X    = 8;   // 突发期弹簧倍率
    reg        burst_on;
    reg  [9:0] creep_ms;
    wire signed [31:0] creep_rem = pos_tgt_cmd - pos_tgt;
    wire signed [31:0] creep_rem_abs = (creep_rem >= 32'sd0) ? creep_rem : -creep_rem;
    wire creep_zone = CREEP_EN && (mode == `M_POS) && !entering_pos &&
                      (creep_rem > -CREEP_ZONE) && (creep_rem < CREEP_ZONE);
    // POS 内连续定位（2026-11-01 P1，移植 champion 2026-10-20）：已在 POS 时收到
    // 新 MOVE（move_pending 上升沿且非入场拍），把 pos_tgt 直接改成 θ_当前+delta——
    // traj_gen 从当前 ref 平滑追到新目标，不重启轨迹、不重走 K3/K4 渐变。
    reg move_pending_d;
    wire move_in_pos = move_pending && !move_pending_d && (mode==`M_POS) && !entering_pos;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            pos_tgt<=0;pos_hold<=0;mode_prev<=`M_SAFE;move_pending_d<=1'b0;
        end else if(tick) begin
            mode_prev<=mode;
            move_pending_d<=move_pending;
            // 2026-09-30 回退"锚点随动"：随动删掉了 theta_e 饱和提供的回拉力
            // （K3 饱和 ~412 duty > 死区 375，是把臂游走拉回有界范围的唯一机制），
            // 实测臂自由漂移每轮 +5→+15 圈失控缠绕。恢复冻结锚点；POS 入场走
            // mode_fsm/debug_link 的 stall-or-慢速门（折返点判定），不依赖随动。
            if(mode==`M_SWING || mode==`M_IDLE || mode==`M_SAFE ||
               (mode==`M_BALANCE && mode_prev==`M_POS))
                pos_hold<=theta_c01;
            // BAL 锚点牵引绳（2026-11-03 晚，治"臂持续单向游走"观感）：
            // BAL 期间 arm 是平衡执行器，会持续被摆杆反作用推着走；锚点完全
            // 冻结时 θe 一路涨到饱和、弹簧变成恒定向一边的偏置——臂看起来在
            // 匀速转圈（160141 BAL θ̇ 中位 17.8°/s 单向）。牵引绳：偏离锚点
            // 超 ±120° 就把锚点跟到当前位置，臂被限制在锚点 ±120° 的圈内
            // 往返（历史上"锚点随动致缠绕"是【无牵引绳全随动】的老案例，
            // 本设计只在越界时跳一次，弹簧力度不变）。
            else if(mode==`M_BALANCE &&
                    ((theta_c01 - pos_hold) > 32'sd12000 ||
                     (theta_c01 - pos_hold) < -32'sd12000))
                pos_hold<=theta_c01;
            if(entering_pos)begin pos_tgt_cmd<=theta_c01+delta_pos; pos_cmd_delta<=delta_pos; end
            else if(move_in_pos && move_is_abs)begin
                // 绝对目标（MOVE_ABS）：直接置 cmd，不再加执行拍 θ
                pos_tgt_cmd<=move_delta;
                pos_cmd_delta<=move_delta-theta_c01;
            end
            else if(move_in_pos)begin pos_tgt_cmd<=theta_c01+move_delta; pos_cmd_delta<=move_delta; end
            // 速度演示：目标跟臂（K3≈0），位置环不跟速度跟踪打架
            else if(vel_demo_en && mode==`M_POS)begin pos_tgt<=theta_c01; pos_tgt_cmd<=theta_c01; end
            // rehold 重锚（2026-11-02 晚 214617 五轮"POS 快速无限旋转"定谳）：
            // |θe|>60° 触发 rehold 后 settle 清 0 → traj clear=1 → ref 追 θ，
            // 但 pos_tgt 留在原目标 → tgt/ref 永久分叉，臂带惯性转圈时 ref 被
            // 拖着走、ref 判据（|ref−tgt|>10°）永远读"移动中"→ 降额死锁
            // （k3e 360）→ 刚度消失 → 无限旋转（四段 POS 全部 th span 430-500°）。
            // rehold 拍把 target 也重锚到当前 θ：臂停哪目标就在哪，稳定窗
            // （1.5s）结束后重新收敛，不再有幽灵目标。
            else if(pos_rehold)begin pos_tgt<=theta_c01; pos_tgt_cmd<=theta_c01;
                                               pos_cmd_delta<=32'sd0; end
            // ★稳定窗内 tgt 跟踪 θ（2026-11-03 晚 160743 逐帧定谳）：
            // 修八把 θe 改成绝对目标（tgt−θ）后，稳定窗"ref 随动 ⇒ θe≈0"的
            // 保护失效——θe 不再看 ref，看冻结的 tgt。臂带速入场溜走 200+°/s
            // ⇒ 弹簧一路追到 60° ⇒ rehold 重锚 ⇒ 臂带速继续冲 ⇒ 再追 60°
            // ⇒ 每 0.3-0.5s 一循环泵摆，2.1-2.6s 掉（duty 顶轨 9628、α→105°）。
            // 稳定窗本意就是"先让臂静下来再交控制权"：窗内 tgt:=θ 连续跟踪，
            // 弹簧力≈0（K3 渐变期间也不泵），窗满后 tgt 自然停在臂当时位置，
            // 由宿主 MOVE（move_in_pos）重新赋目标——入场 MOVE 的 delta 不再
            // 用于拉开位移（演示的移动全部走显式 MOVE，语义更干净）。
            // settle 过渡拍：稳定窗结束，从【当前臂位】兑现锁存的 MOVE 位移
            // （入场时臂还在漂，从入场点起算的旧目标已失效；语义=「先稳住，
            // 再从前稳后的位置走 delta」）。无 MOVE 时 delta=0，就地保持。
            // settle 过渡拍：稳定窗结束——从【当前臂位】重算最终目标
            // （入场时臂还在漂，入场点起算的旧目标已失效），并从当前位置开始滑坡。
            else if(mode==`M_POS && pos_settle_ms == POS_SETTLE_MS-1)begin
                pos_tgt_cmd<=theta_c01+pos_cmd_delta;
                pos_tgt<=theta_c01;
            end
            // 稳定窗内：tgt 跟踪 θ（弹簧中性）
            else if(mode==`M_POS && pos_settle_ms < POS_SETTLE_MS-1)pos_tgt<=theta_c01;
            // ★目标缓速分解：窗外每 10 拍向最终目标逼近 4 c01（=4°/s），
            // 每拍位置误差夹在滑坡步长内（K3弹簧始终小且平稳）。
            else if(mode==`M_POS && slew_div == POS_SLEW_DIV-1)begin
                // ★轮3-②：蠕行区（末段 5°）用突发式推进；蠕行区外保持 20°/s。
                if(creep_zone)begin
                    if(creep_rem_abs <= CREEP_STEP)begin
                        pos_tgt<=pos_tgt_cmd; burst_on<=1'b0; creep_ms<=10'd0;
                    end else if(!burst_on)begin
                        // 周期满且臂静止（|θ̇|≤15°/s、|α|≤4°）才发起下一发；
                        // 未静止重新计满窗（宁慢勿泵）
                        if(creep_ms >= CREEP_PERIOD-1)begin
                            if(theta_dot <= 16'sd150 && theta_dot >= -16'sd150 &&
                               alpha <= 16'sd400 && alpha >= -16'sd400)begin
                                pos_tgt <= pos_tgt + ((pos_tgt_cmd > pos_tgt) ?
                                                      32'sd300 : -32'sd300);
                                burst_on <= 1'b1; creep_ms <= 10'd0;
                            end else creep_ms <= 10'd0;
                        end else creep_ms <= creep_ms + 10'd1;
                    end else begin
                        // 突发窗：到时或 |α|>6°（泵摆自检）即收
                        if(creep_ms >= CREEP_BURST-1 || alpha > 16'sd600 || alpha < -16'sd600)begin
                            burst_on <= 1'b0; creep_ms <= 10'd0;
                        end else creep_ms <= creep_ms + 10'd1;
                    end
                end else begin
                    burst_on <= 1'b0; creep_ms <= 10'd0;
                    // 20°/s 恒定（DIV=2）：连续 MOVE 连发即慢速旋转。
                    if((pos_tgt_cmd-pos_tgt) > 32'sd4)       pos_tgt<=pos_tgt+32'sd4;
                    else if((pos_tgt_cmd-pos_tgt) < -32'sd4) pos_tgt<=pos_tgt-32'sd4;
                    else                                     pos_tgt<=pos_tgt_cmd;
                end
            end
        end
    end
    wire signed [31:0] pos_ref_traj;
    wire signed [15:0] pos_v_ref;
    // ---- P2''：入场稳定窗（2026-11-01，P2' 板验 224420 两轮甩飞逐帧定谳）----
    // 甩飞链：捕获残余（α̇ −95°/s）经 K2 推臂倒退 → ref 已在拉 + ⇒ θe 长大
    // （实测 400ms 到 −35°、1s 到 −73°）→ K3·θe 同向加推（正 K3 在本机 θ 约定
    // 下对跟踪是正反馈，级联轮对照：K3=600 时甩飞消失但也没位置力）→ 臂蓄能
    // 40-70° → 猛烈回甩抛飞摆杆。修法：入场后 1.5s 内保持 traj_gen clear=1
    // （ref 跟随实际 θ，θe≈0，K3 无从加推），让残余阻尼掉、K3/K4 渐变走完，
    // 再放行轨迹。渐变（1024ms）恰在窗内完成。
    localparam integer POS_SETTLE_MS = 1500;
    reg  [10:0] pos_settle_ms;
    // θe 看门狗（2026-11-02 v6 遥测定谳 153630）：臂坡区锁死 → θe 蓄到 −200° →
    // 降额退出瞬间 K3 恢复力爆发（−2942 duty）抛飞摆杆。|θe|>60° 即重新进稳定窗
    // （ref 拉回实际 θ，轨迹作废重锚），把弹簧在 60° 就剪断。move_blend 同拍清零
    // （锁死态下保持降额无意义，恢复满额让 dz(θ) LUT 有最大权限解锁）。
    // ★2026-11-03 定谳：rehold 必须用【未钳位】的 theta_e_raw！θe 钳位（±45°）
    // 曾把 60° 看门狗变成死代码——|te| 永远到不了 60° → 安全阀失效 → 臂在 POS
    // 里游走 5463°（15 圈）后摆杆被泵掉（155433 seg1 逐帧：te 恒 −45、duty 顶轨
    // ±9600、α →−70.8°）。看门狗看真实误差，钳位只服务 LQR 输入。
    // 2026-11-03 晚：rehold 门 60°→90°（板测目标切换实验：60° 时 +10° 位移
    // 命令在 0.5s 内就被重锚吃掉，位置环来不及兑现；90° 给命令更多空间，
    // 仍远小于失控阈值——K3=1200 在 90° 时弹簧 879→1319 duty，低于泵摆临界）。
    // P4（2026-11-03 夜，板测定谳后用）：门 40°→120°。40° 时 K3 项
    // 1200*4000/8192 = 586 duty 恰在死区 640 线下 → 位置环零权限，臂自由漂，
    // 2.3s 撑到门 → rehold 抹掉 MOVE（184108 实测：+8° 只走了 2° 就被重锚）。
    // 120° 时 K3 项 = 1758 duty 足以刹车；rehold 退化为安全阀。
    // ★2026-11-04 夜轮2：加 pos_tgt==pos_tgt_cmd（保持态）限定。过渡期
    // （slew 未爬到 cmd）θe_raw 大是预期，041239 实测 −91.4° 请求被臂振推过
    // 120° 门瞬间吃掉；保持态 |θe|>120° 仍是异常漂移/外力的安全阀。
    wire pos_rehold = (mode == `M_POS) && (pos_tgt == pos_tgt_cmd) &&
                      ((theta_e_raw > 16'sd12000) || (theta_e_raw < -16'sd12000));
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) pos_settle_ms <= 11'd0;
        else if (tick) begin
            if (mode != `M_POS || entering_pos || pos_rehold) pos_settle_ms <= 11'd0;
            // 显式 MOVE 立即释放稳定窗（2026-11-03 晚）：稳定窗内 tgt 跟踪 θ，
            // 若窗口期间宿主发 MOVE，move_in_pos 设的 tgt 会在下一拍被跟踪分支
            // 覆盖（161738 demo 实测：+30° 一步"瞬间到位"——位移被稳态跟踪吃掉）。
            // MOVE = 明确的位置意图，权重高于入场保护：释放 settle，让 tgt 扎根。
            else if (move_in_pos) pos_settle_ms <= POS_SETTLE_MS;
            else if (pos_settle_ms < POS_SETTLE_MS) pos_settle_ms <= pos_settle_ms + 1'b1;
        end
    end
    traj_gen #(.A_MAX(POS_ACC_MAX)) u_traj (
        .clk(clk), .rst_n(rst_n), .tick(tick),
        .clear(mode != `M_POS || entering_pos || pos_settle_ms < POS_SETTLE_MS),
        .pos_actual(theta_c01),
        .pos_target(pos_tgt),
        .pos_ref(pos_ref_traj),
        .v_ref(pos_v_ref)
    );
    // 修八（2026-11-02 晚 222036 五轮定谳，用户批准的架构级简化）：绝对目标直驱。
    // 旧链 = θe 基于 ref（随动/轨迹混合体），三个机制互相打架：
    //   ① 稳定窗内 ref 随动（clear=1 防入场瞬态）→ K3·θe≡0，位置环被结构性
    //      断开——满刚度是假象，臂溜坡无回拉力（逐帧：settle 0→1500 期间
    //      θ −573→−806°、θ̇ 0→−736°/s，duty 与 θ̇ 同向=LQR 在加速臂）；
    //   ② rehold 重锚 tgt（修六）+ 轨迹从被拖走的位置重启（clear 撤除后
    //      traj_gen 从当前 ref 向老 tgt 推进）→ 每次都制造 180° 级大移动；
    //   ③ 降额判据看 ref 推进 → ref 被臂拖走时误判移动 → K3 被砍 → 更快溜。
    // 新语义：θe 恒用 tgt−θ（误差对真实目标有效，臂一溜 K3 立刻回拉）；
    // ref 仅作【轨迹前馈的载体】保留：稳定窗内 ref=θ（随动，θe 由 tgt 直出），
    // 窗满后 traj_gen 从 ref 位置向 tgt 推进——但 rehold 时 tgt/ref 同锚（修六），
    // 消灭 180° 幽灵移动。轨迹速度曲线（拓展2）后续由上位机小步进 tgt 实现。
    wire signed [31:0] pos_ref = (mode == `M_POS) ? pos_ref_traj : pos_hold;
    wire signed [32:0] the32 = $signed({theta_c01[31],theta_c01}) -
                               $signed({pos_tgt[31],pos_tgt});
    // θe 钳位（2026-11-03 秒死定谳后的修复）：BAL 内锚点弹簧封顶 ±45°。
    // 病理（132913 逐帧分解）：捕获窗内 K4 成对渐变抬向 12000 + K3·θe 弹簧
    // （锚点=pos_hold 冻结在入窗点）随臂漂移线性增长，θe 原钳 ±327.68° 使
    // 弹簧顶到 2400 duty 恒定偏置，与 K4·θ̇(万级) 叠加 → duty 满轨 bang-bang
    // → 0.3-0.9s 秒死。钳 ±45°（4500 c01）后弹簧上限 330 duty：稳摆期 θe
    // 均值 −0.58°（10-20 基线）远低于门，行为不变。
    // ★2026-11-03 晚修正：钳位改 ±120°（12000 c01）。45° 只给 330 duty，低于
    // 死区（~800-1000）——BAL 里锚点弹簧形同虚设，臂自由漂 18°/s（160141
    // 实测），既让观感像"无限旋转"又堵死 POS 入场 θ̇ 门。120° → 879 duty
    // 刚过死区：有弱牵引防漂、又只有原 2400 饱和偏置的 37%（132913 秒死
    // 的 K3 贡献大幅缩水）。POS 用 raw 不受影响（te 由 rehold 门 60° 封顶）。
    wire signed [15:0] theta_e_raw = the32 >  32'sd32767 ? 16'sd32767 :
                                     the32 < -32'sd32768 ? -16'sd32768 : the32[15:0];
    wire signed [15:0] theta_e = (mode == `M_POS) ? theta_e_raw :
                                 (theta_e_raw >  16'sd12000) ? 16'sd12000 :
                                 (theta_e_raw < -16'sd12000) ? -16'sd12000 :
                                 theta_e_raw;
    // LQR：tick 后一拍启动，采样到的是本拍新状态（3 级流水）
    reg start_d;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) start_d <= 1'b0;
        else        start_d <= tick;
    end
    wire signed [15:0] duty_lqr;
    // ---------------- 定位积分 trim（2026-09-23 位置环重设计）----------------
    // 门保持原样（斜坡完成+近目标+摆杆直立+θ̇ 小+输出未饱和）。其中 |θ̇|<200 这条
    // 在 M 法低速量化（单计数台阶 865）下永远不成立——T 法（arm_vel USE_T=1）给出
    // 干净的爬行 θ̇（20~50）之后才真正可用。
    // 增益 2^20→2^11、限幅 ±200→±600：旧参数下 E=200cdeg 累积 1000 拍只出 0.2 duty，
    // 而执行器死区 350 ⇒ 积分器从未起过作用；末段逼近因此只能靠 M 法 θ̇ 噪声经 K4 的
    // "抖动整流"偶然顶过死区——实机上同一机制被放大成 ±9000 duty 的输出抖动
    // （实测 duty 峰峰 14770、换向 34.5 次/s、机械咔咔共振），且 K4 不可砍
    // （砍到 8000 捕获后立不住）。T 法 + 本积分器改为确定性推出：300cdeg 残差
    // ~0.7s 顶过死区。模型四方向验收：late 0.80~0.92°、σ 0.13~0.22、hold 全过；
    // 增益扫描：2^10 起变差（−3000 late 1.5°），2^9 极限环（late 6~11.7°）。
    localparam integer       IQ_SH    = 11;
    localparam signed [31:0] IQ_LIM   = 32'sd600;                 // duty
    localparam signed [31:0] IQ_LIM_Q = IQ_LIM <<< IQ_SH;          // = 1_228_800
    // ================= 静摩擦锁定保持（P7，2026-11-03 夜）=================
    // 现象（200015 板测实况）：POS 到位后臂被静摩擦卡住、摆杆在 ±1° 内被轴系
    // 摩擦顶住、duty=0，磁编码器 raw 完全恒定 ⇒ 真静止 10s+。这是旋转倒立摆
    // 的天然特性（机械死区刚好卡死），可作为"零扭矩保持"扩展功能。
    // 但现版是【偶然】锁死：pos_trim 顶饱和 600 + REV_HYST=400 挡反向 + cascade
    // 反向积分，三者凑巧把输出锁 0；唤醒要等 α 漂到 ~2° 且积分清零后才有力。
    // 本状态机把它确定性化：安静且到位 → 主动断输出（真零扭矩）→ 摆杆一歪立刻
    // 接管。入锁条件全部要求"目标已停走"，MOVE 期不会误锁。
    // FRICTION_LOCK_EN=0 → 逐位回退旧行为。
    localparam FRICTION_LOCK_EN = 1'b1;
    reg        fl_lock;     // 1 = 锁定保持中（duty 强制 0）
    reg [9:0]  fl_cnt;      // 入锁去抖（200ms）
    reg [9:0]  fl_wake;     // 唤醒去抖（20ms）
    wire fl_tgt_still = (pos_tgt == pos_tgt_cmd) && (pos_settle_ms >= POS_SETTLE_MS);
    // ★2026-11-04 夜 v2：旧门 (5°/2°) 在 K4=11000 慢摆下几乎开不了（|te| 均值
    // 9-12°）⇒ 放宽到 te 8°/α 2.4°、驻留 100→250ms；唤醒门同步对齐（te 10°→12°
    // 给 4° 蠕滑余量；α 3.5°=机械死区经验值、α̇ 80°/s 不变）。
    wire fl_enter = (mode == `M_POS) && !entering_pos && fl_tgt_still &&
                    (theta_e   <= 16'sd800 && theta_e   >= -16'sd800) &&   // |te|≤8°
                    (alpha     <= 16'sd240 && alpha     >= -16'sd240) &&   // |α|≤2.4°
                    (theta_dot <= 16'sd500 && theta_dot >= -16'sd500) &&   // |θ̇|≤50°/s
                    (alpha_dot <= 16'sd300 && alpha_dot >= -16'sd300);     // |α̇|≤30°/s
    wire fl_wake_ev = (alpha > 16'sd350) || (alpha < -16'sd350) ||         // |α|>3.5°
                      (alpha_dot > 16'sd800) || (alpha_dot < -16'sd800) || // |α̇|>80°/s
                      (theta_e > 16'sd1200) || (theta_e < -16'sd1200) ||   // |te|>12°
                      move_in_pos;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fl_lock <= 1'b0; fl_cnt <= 10'd0; fl_wake <= 10'd0;
        end else if (tick) begin
            if (mode != `M_POS || entering_pos) begin
                fl_lock <= 1'b0; fl_cnt <= 10'd0; fl_wake <= 10'd0;
            end else if (!fl_lock) begin
                if (fl_enter) begin
                    if (fl_cnt >= 10'd250) begin fl_lock <= 1'b1; fl_cnt <= 10'd0; end  // 250ms 连续安静
                    else fl_cnt <= fl_cnt + 10'd1;
                end else fl_cnt <= 10'd0;
            end else begin
                if (fl_wake_ev) begin
                    if (fl_wake >= 10'd20) begin fl_lock <= 1'b0; fl_wake <= 10'd0; end
                    else fl_wake <= fl_wake + 10'd1;
                end else fl_wake <= 10'd0;
            end
        end
    end
    wire fl_hold = FRICTION_LOCK_EN && fl_lock;   // 锁定期 duty 强制 0
    // ---- ★轮3-① 锁前镇静（速度门控 K4 增量制动，2026-11-04 晨 v2 修正）----
    // v1 教训（10:15-10:20 四段 jsonl）：×1 增量在 POS 入场瞬态（臂带捕获能量，
    // |θ̇|>300 常态）触发 → 等效 K4 22000 越过满电 16000 悬崖 → POS 1-4s 泵摆
    // 掉落（duty 顶 9900 满轨）。v2：a) 增量降到 0.25×（>>>15，有效 ≤13.7k
    // 甜点内）b) 钳 ±500 c) 只在【已建立的保持态】出手（fl_tgt_still：稳定窗
    // 满+目标停走），入场瞬态永不触发。滞回 30/15°/s；|α|>6° 退出（泵摆自检）。
    localparam SB_EN = 1'b1;
    // ★R6 坏区阻尼器（11-4 用户实测 110656 定谳）：掉摆带=60-105°——dz(θ) LUT
    // 高格区（45-112.5°，静突破 2200-2600 全臂摩擦最大），补偿过推+继电器泵能
    // 在此最强，运动穿越时振荡增长→摔（掉前 |α| 已 36.6°、duty 6754）。
    // 修法：该区内只要杆大幅摆动（|α̇|>30°/s）就注入 0.25×K4 制动（同域同号），
    // 不要求保持态——运动穿越也有阻尼。保持态原逻辑不变（fl_tgt_still 分支）。
    // ★时序修复（R6 首版 26.65MHz FAIL 教训）：dz_idx_o 是 out_stage 内部组合
    // 输出，直接回馈 duty 链会形成穿越输出级的长路径 → 先过一级 clk 寄存器
    // 打断（延迟 1 clk≈1ms @tick 域无关紧要）。
    reg  [3:0] dz_idx_d;
    always @(posedge clk) dz_idx_d <= diag_dz_idx;
    wire sb_zone = (dz_idx_d >= 4'd2) && (dz_idx_d <= 4'd4);
    reg sb_on;
    wire sb_gate = (mode == `M_POS) && !entering_pos && !fl_hold && !vel_demo_en &&
                   !burst_on &&
                   (alpha <= 16'sd400) && (alpha >= -16'sd400) &&
                   ( (fl_tgt_still && (theta_e <= 32'sd2000) && (theta_e >= -32'sd2000) &&
                      (theta_dot > 16'sd300 || theta_dot < -16'sd300))
                     ||
                     (sb_zone && (alpha_dot > 16'sd300 || alpha_dot < -16'sd300)) );
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) sb_on <= 1'b0;
        else if (tick) begin
            if (mode != `M_POS || entering_pos) sb_on <= 1'b0;
            else if (!sb_on)
                sb_on <= SB_EN && sb_gate;
            else
                sb_on <= sb_gate ||
                         (sb_zone && (alpha_dot > 16'sd150 || alpha_dot < -16'sd150) &&
                          (alpha <= 16'sd600) && (alpha >= -16'sd600));
        end
    end
    wire signed [31:0] sb_prod = $signed(theta_dot) * $signed(pos_k4);
    wire signed [31:0] sb_sh   = sb_prod >>> 15;      // 0.25×K4 增量
    wire signed [31:0] sb_brake = (SB_EN && sb_on) ?
        ((sb_sh > 32'sd500)  ? 32'sd500 :
         (sb_sh < -32'sd500) ? -32'sd500 : sb_sh) : 32'sd0;
    // 蠕行突发期弹簧增强（×8 后钳 ±2000；与 LQR p3=θe·K3 同号同域）
    wire signed [31:0] cp_prod = $signed(theta_e) * $signed(pos_k3);
    wire signed [31:0] cp_sh   = cp_prod >>> 10;      // ÷8192×8
    wire signed [31:0] creep_push = (CREEP_EN && burst_on) ?
        ((cp_sh > 32'sd2000)  ? 32'sd2000 :
         (cp_sh < -32'sd2000) ? -32'sd2000 : cp_sh) : 32'sd0;
    reg signed [31:0] pos_i;
    // 2026-10-20 POS 实测：|θe| 门 500(5°) 太窄——POS 稳定段 |θe| 中位 3.8°、
    // 90 分位 10.9°、max 22.1°，门只满足 62% 时间，积分器在误差最大的时候
    // 反而被冻结，导致 θ 慢漂（0.07Hz、std 6.3°）收不掉。放宽到 2000(20°)
    // 覆盖实测漂移区间。安全性：积分限幅 IQ_LIM=±600 duty 不变，下方 α/θ̇/
    // duty 三道门仍全查，放宽 θe 不会让积分器在危险状态累积。
    wire trim_enable=(pos_ref_traj==pos_tgt) && (theta_e>=-2000 && theta_e<=2000) &&
                     (alpha > -500 && alpha < 500) &&
                     // 2026-09-30：θ̇ 门 200→1500。原门假设臂近静止（09-23 设计），
                     // 实机稳摆臂晃 θ̇ 中位 ~1780 单位，门几乎不开 → 定位残差靠
                     // K3 顶死区（k3=1191 需 52° 误差才够 375 死区）永远收不掉。
                     // 积分器限幅 ±600 duty，放宽门不影响安全。
                     (theta_dot > -1500 && theta_dot < 1500) &&
                     (duty_out > -9000 && duty_out < 9000);
    wire signed [32:0] i_next=$signed({pos_i[31],pos_i})+
                              $signed({{17{theta_e[15]}},theta_e});
    wire signed [31:0] pos_trim=(pos_i+ (32'sd1 <<< (IQ_SH-1))) >>> IQ_SH;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n)pos_i<=0;
        else if(tick) begin
            if(mode!=`M_POS || entering_pos || fl_hold)pos_i<=0;
            else if(trim_enable)
                pos_i <= i_next>$signed({1'b0,IQ_LIM_Q}) ? IQ_LIM_Q :
                         (i_next < -$signed({1'b0,IQ_LIM_Q}) ? -IQ_LIM_Q : i_next[31:0]);
        end
    end
    wire signed [31:0] duty_with_trim=$signed(duty_lqr)+((mode==`M_POS) ? pos_trim : 32'sd0)
                                      +((mode==`M_POS) ? sb_brake : 32'sd0)
                                      +((mode==`M_POS) ? creep_push : 32'sd0);
    wire signed [15:0] duty_bal=(duty_with_trim>9500) ? 16'sd9500 :
                                ((duty_with_trim < -9500) ? -16'sd9500 : duty_with_trim[15:0]);
    wire lqr_done;
    // POS 专用臂环增益（2026-09-27）：稳摆增益回退 q_theta=0.03 后 K3 无位置权限
    // （tb_closedloop 末段误差 36.8°），仅在 M_POS 模式把臂环 K3/K4 成对抬到
    // 定位档；BALANCE/SWING 的稳摆行为零影响。
    // 2026-09-28 改运行时参数 id11/id12（默认=2064/13166）：该对 sim 通过但实机
    // 掉摆（k3 实机不稳定主导），扫描定档需逐档上机，运行时可调免去每档重编译。
    // 编译期兜底宏 G_K3_POS/G_K4_POS 保留在 j280_gains.vh，不再被引用。
    // 2026-09-30 硬切换→1024ms 线性渐变：进入瞬间 K3 266→1191、K4 10297→13830
    // 一次性移交位置权限，而稳摆极限环（臂晃 θ̇ 中位 ~1780）仍在——伺服猛拽臂
    // 扰动摆杆即掉摆（模式干涉，换任何档位都治不了）。渐变期位置权限逐毫秒
    // 移交，摆杆稳定通道 K1/K2 全程不变；斜坡是状态量，POS/BALANCE 往返
    // （creep 步进、掉摆重甩）全程数值连续无跳变。增益域 [0,32767]
    // （j280_debug_params.vh 强制），凸组合恒在 16 位有符号内，无溢出；
    // blend=0/1024 端点精确回到 k3/k4 与 pos_k3/pos_k4。
    reg [10:0] pos_blend;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) pos_blend <= 11'd0;
        else if (tick) begin
            if (mode == `M_POS) begin
                if (pos_blend < 11'd1024) pos_blend <= pos_blend + 1'b1;
            end else if (pos_blend != 11'd0) begin
                pos_blend <= pos_blend - 1'b1;
            end
        end
    end
    wire signed [16:0] k3_span = $signed(pos_k3) - $signed(k3);
    wire signed [27:0] k3_prod = k3_span * $signed({1'b0, pos_blend});
    wire signed [15:0] k3_base = $signed(k3) + (k3_prod >>> 10);
    // 捕获增强相（2026-09-28）：甩摆以 ~480°/s 入窗（无门限捕获），软稳摆组
    // 接不住——杆过零时臂已 ±590°/s 晃动，动量带崩系统（实测捕获后 1.1-1.4s
    // 必掉）。SWING→BALANCE 交接后 800ms 内把 K2/K4 提到 13/8 倍（杆阻尼+臂
    // 阻尼），实测存活时间 1.1-1.4s → 10-23s（10 倍）。
    // 剂量教训（同日实测）：4s 长剂量反而 0.5-0.9s 崩（增强增益持续放大 α̇
    // 噪声）；k4 运行时抬到 9000/11000（有效 >14k）0.5s 内崩——K4 只能限时
    // 增强，不能长抬。参数区写锁只允许 IDLE/SAFE，增强只能做在 RTL。
    reg [10:0] catch_ms;
    reg        catch_boost;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            catch_ms <= 11'd0; catch_boost <= 1'b0;
        end else if (tick) begin
            if (mode == `M_BALANCE && mode_prev == `M_SWING) begin
                catch_ms <= 11'd0; catch_boost <= 1'b1;
            end else if (catch_boost && catch_ms < 11'd800) begin
                catch_ms <= catch_ms + 1'b1;
            end else if (mode != `M_BALANCE) begin
                catch_boost <= 1'b0;
            end else if (catch_ms >= 11'd800) begin
                catch_boost <= 1'b0;
            end
        end
    end
    // ---- 捕获窗内抬臂位置增益（2026-10-22，起摆冠 100% 捕获配置的组成部分）----
    // 机理（28 次实板捕获 + 逐帧核算）：K3 低档时臂环要上百°误差才越过死区，
    // 而甩摆把臂以残余速度带进 BALANCE，K4 对臂是负阻尼，臂环没有权限消这个
    // 速度。CATCH_K3>0 时捕获窗内把 K3 渐变抬到该值（200ms 升 / 32ms 降，
    // 渐变而非硬切）。**本树配 2064**（起摆冠值）；配 0 时整条通路常量折叠
    // （行为与 10-20 简单栈逐位一致）。
    // 位宽纪律：blend 满量程是 1024，必须 11 位。写成 [9:0] 时 1024 会被截成 0
    // （2026-10-22 实测踩过：k3_eff 全程不变，CATCH_K3 完全没生效）。
    reg  [10:0] catch_k3_blend;      // 0..1024
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) catch_k3_blend <= 11'd0;
        else if (tick) begin
            if (catch_boost) begin
                // 200 拍升满（+6/拍 ⇒ 1024/6≈171 拍）
                if (catch_k3_blend < 11'd1024)
                    catch_k3_blend <= (catch_k3_blend > 11'd1018) ? 11'd1024
                                                                 : catch_k3_blend + 11'd6;
            end else if (catch_k3_blend != 11'd0) begin
                // 退窗 32→3/拍（~341ms，2026-11-01）：800ms 窗末 K3 2064→600 在 32ms 内
                // 骤降，臂残余速度反弹把刚稳住的摆杆打掉（164419 seg12 θ̇ -20→+584、
                // 用户实感"最后能稳住的一瞬间掉摆"）。缓降让臂环权限平滑移交。
                catch_k3_blend <= (catch_k3_blend > 11'd3) ? catch_k3_blend - 11'd3 : 11'd0;
            end
        end
    end
    // ---- 捕获窗目标（2026-11-01 闭环极点扫描定案，tmp/ramp_stability_scan.py）----
    // 用户指出"渐变时的 LQR 参数组可能进入不稳定范围"——扫描证实：
    //   · 现役路径（K3→2064 单抬，K4=5300）名义稳定但窗满裕度仅 -0.286
    //     （基础点 -0.748 的一半都不到），实机死区/延迟下等于不稳——
    //     "最后能稳住的一瞬间掉摆"=摆杆在不稳定增益组里待了几百 ms；
    //   · K2×2.5 + K3=2064 + K4=5300 直接翻正（+0.41）；
    //   · **成对**渐变到 POS 对（K3=1200, K4=12000）+ K2 全程 ×2.5：
    //     全中间态 0 违例，窗满裕度 -1.57 = 基础点两倍。
    // CATCH_PAIR_EN=1：窗目标=运行时 pos_k3/pos_k4（id11/id12，免重编译调）。
    // =0：走旧 CATCH_K3 常数路径（K4 不抬，留档对照）。
    wire signed [16:0] k3_cap_tgt = CATCH_PAIR_EN ? $signed(pos_k3) : $signed(CATCH_K3);
    wire signed [16:0] k3_cap_span = k3_cap_tgt - $signed(k3_base);
    wire signed [27:0] k3_cap_prod = k3_cap_span * $signed({1'b0, catch_k3_blend});
    wire signed [15:0] k3_eff  = (CATCH_K3 == 16'sd0) ? k3_base
                              : ($signed(k3_base) + (k3_cap_prod >>> 10));
    // 捕获增强剂量分子/移位（×NUM>>>SH）。K2/K4 分开参数化（2026-10-21：
    // K2 增强在高入场摆速下是掉摆主因——捕获瞬间 |α̇| 200~370°/s 时 K2·α̇
    // 未增强就有 −2400~−4500 duty，×2.67 直接顶到 ±9500。T20K 现配 ×1）。
    // ×1 时整条乘法链要靠常量折叠掉，否则参数化后的变量乘法会把 Fmax 拖垮
    // （实测）；boost_clip 上限 30000 与历史行为一致。
    function signed [31:0] boost_clip(input signed [31:0] v);
        boost_clip = (v > 32'sd30000) ? 32'sd30000 : (v < -32'sd30000) ? -32'sd30000 : v;
    endfunction
    wire signed [31:0] k4_boost = catch_boost ? boost_clip($signed({{16{k4[15]}}, k4}) * CATCH_K4_NUM >>> CATCH_K4_SH)
                                              : $signed({{16{k4[15]}}, k4});
    // ---- 捕获窗摆杆通道加权增强（借鉴 champion DFB 对称形式，仅窗内）----
    // 用户定案"捕获相不够强"+"增强要考虑切换问题"。形制=DFB_K2 反泵终版：
    // wa=clamp((|α|−250)/500) Q10 权重，K2×(1+1.5·wa)——满权重在 |α|≥7.5°，
    // 过零点自动归零（K2·α̇ 顶轨风险最大处恰在零点，权重自己退场）。
    // 三重防阶跃/防不稳：①梯形窗包络 k2_env（入 128ms/满 544ms/出 128ms，
    // Q8 0..256）；②窗目标走已验证稳定路径（上注扫描）；③稳态零改动
    // （窗后 env=0 ⇒ 精确回基础 K2，无 DFB 常开的稳态泵能问题）。
    reg signed [15:0] ca_abs;
    reg        ca_abs_q;    // 延迟一拍的标志（ca_abs 首拍未更新）
    reg [15:0] ca_wa;
    reg signed [31:0] ca_prod;   // α·α̇（>0 外摆）
    reg [15:0] ca_wo;            // 外摆权重 Q10
    reg [15:0] ca_w;             // min(wa, wo)
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ca_abs <= 16'sd0; ca_abs_q <= 1'b0; ca_wa <= 16'd0;
            ca_prod <= 32'sd0; ca_wo <= 16'd0; ca_w <= 16'd0;
        end else if (tick) begin
            ca_abs   <= alpha[15] ? -$signed(alpha) : $signed(alpha);
            ca_prod  <= $signed({{16{alpha[15]}}, alpha}) *
                        $signed({{16{alpha_dot[15]}}, alpha_dot});
            ca_abs_q <= 1'b1;
            // wa 用上一拍 ca_abs（1ms 延迟，窗动态 ~100ms 量级，无害）
            ca_wa    <= !ca_abs_q ? 16'd0 :
                        (ca_abs <= 16'sd250)  ? 16'd0 :
                        (ca_abs >= 16'sd750)  ? 16'd1024 :
                        ({16'd0, ca_abs - 16'sd250} <<< 10) >>> 9;
            // ---- 外摆门（2026-11-01 用户手测 195412 差分定谳）----
            // 对称 |α| 权重会在**回摆冲程**（α·α̇<0，摆杆自己正往零点回）给
            // K2×2.5：为刹一根本来就在回来的杆，底座反向猛推 ⇒ 臂负向加速
            // ⇒ K4·θ̇（对臂是正反馈）把臂推到 -709°/s 顶轨 ⇒ 底座跑离摆杆。
            // seg7(死) vs seg12(活) 同入场，分岔就在 180ms 折返拍：回摆 -164
            // vs -124°/s，K2 项 -5004 vs -3783，前者臂失控。改 champion DFB
            // 原始形式 min(wa, wo)：只刹外摆，回摆交给基础 K2。常开 DFB 改对称
            // 是为治跨周期泵能，本增强只活 800ms（~2 个周期）无此虑。
            // wo=clamp(α·α̇ >>> 4, 0, 1024)：α=12°×α̇=1.3°/s 即满权，实质是
            // 带微小线性区的符号门——切换点 α̇≈0 处 K2·α̇ 本身≈0，无输出阶跃。
            ca_wo    <= (ca_prod <= 32'sd0)      ? 16'd0 :
                        (ca_prod >= 32'sd16384)  ? 16'd1024 :
                        ca_prod[13:4];
            ca_w     <= (ca_wa < ca_wo) ? ca_wa : ca_wo;
        end
    end
    reg [8:0] k2_env;    // Q8 0..256
    wire [10:0] k2_out_ms = 11'd800 - catch_ms;   // 距窗末拍数
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) k2_env <= 9'd0;
        else if (tick) begin
            if (!catch_boost)            k2_env <= 9'd0;
            else if (catch_ms < 11'd128) k2_env <= {2'b00, catch_ms[6:0], 1'b0}; // ×2: 0..254
            else if (k2_out_ms > 11'd128) k2_env <= 9'd256;
            else                          k2_env <= {2'b00, k2_out_ms[6:0], 1'b0}; // 出坡 672..800 ×2
        end
    end
    wire signed [31:0] k2_lin = $signed({{16{k2[15]}}, k2});
    wire signed [31:0] k2_gain1 = (k2_lin * 32'sd768) >>> 9;                    // k2×1.5
    wire signed [31:0] k2_weighted = k2_gain1 * $signed({6'd0, ca_w});           // ×min(wa,wo) (0..1024)
    wire signed [31:0] k2_boost2 = (k2_weighted >>> 10) * $signed({1'b0, k2_env}); // ×env (0..256)
    wire signed [31:0] k2_catch = k2_lin + (k2_boost2 >>> 8);
    wire signed [31:0] k2_eff32 = boost_clip(k2_catch);
    wire signed [16:0] k4_span = $signed(pos_k4) - $signed(k4);
    wire signed [27:0] k4_prod = k4_span * $signed({1'b0, pos_blend});
    wire signed [15:0] k4_pos  = $signed(k4) + (k4_prod >>> 10);
    // CATCH_PAIR_EN=1 时 K4 随 K3 同一 blend 成对抬到 pos_k4（K4=k4_pos=基值 in
    // BALANCE，因 pos_blend=0；凸组合恒 ≤ pos_k4 不越界）
    wire signed [16:0] k4_cap_span = $signed(pos_k4) - $signed(k4_pos);
    wire signed [27:0] k4_cap_prod = k4_cap_span * $signed({1'b0, catch_k3_blend});
    wire signed [15:0] k4_cap  = $signed(k4_pos) + (k4_cap_prod >>> 10);
    wire signed [15:0] k2_eff = k2_eff32[15:0];
    wire signed [15:0] k4_eff = CATCH_PAIR_EN ? k4_cap
                              : (catch_boost ? k4_boost[15:0] : k4_pos);
    // ---- P2''''：POS 移动降额（2026-11-02，dz(θ) 战役后的最后一刀）----
    // move1 甩飞机理（225029/232657/统计 12 轮）：轨迹推力+K3·θe 滞后推力同向
    // （臂落后 ref 40-66° ⇒ θe −54° ⇒ K3 项 −1360 duty 追加推动）⇒ 臂冲过快
    // ⇒ 摆杆抛飞。dz(θ) LUT 同向叠加使 P2'' 的 0/3 退化为 3/3。修法：|θ̇| 大
    // （移动中）时成对降额到 (360,4000)——K3×0.3 削滞后推力 70%，K4×0.5 保臂
    // 阻尼（极点扫描：K4<4000 会翻正，(360,4000) 与路径全稳）；停稳渐回满额
    // （dz(θ) 在静止保持时全额发力）。blend 斜率 +4/-8 每拍（256ms 入/128ms 出）防阶跃。
    localparam signed [15:0] POS_MOVE_K3 = 16'sd360;
    localparam signed [15:0] POS_MOVE_K4 = 16'sd9000;   // ★轮3修正：移动期K4不再软化到4000（满电下运动中掉摆主因）
    localparam signed [15:0] POS_MOVE_THDOT = 16'sd80;   // |θ̇|>8°/s 即降额。
    // 门限史：500（145240 到场太晚）→150（155314 保持段 θ̇ −6..−11°/s 卡死在
    // 降额态 k3e=360 不退——位置力 88 duty 推不过摩擦坡，θ 漂 −80° 甩飞）→
    // 8°/s：高于稳摆微调 6°/s，保持段（θ̇ 中位 ~3°/s）满额、任何移动降额。
    reg  [10:0] move_blend;   // 0=满额 1024=降额
    wire pos_moving = (mode == `M_POS) &&
                      ((theta_dot > POS_MOVE_THDOT) || (theta_dot < -POS_MOVE_THDOT));
    // 显式饱和（2026-11-02 板测定谳后重写）：旧式 "mb<1024 才 +4" 有两处洞——
    //   ① 钳位查旧值，1023+4=1027 越过满量程（外推立刻反向过冲）；
    //   ② 板上实测（155816 hold_test 三轮）mb 冲到 2044、k3e 外推成 −471/112——
    //      iverilog 全链仿真同 RTL 峰值精确钳 1024（tb_pos_hold_mb），板上却越界，
    //      判定为 Gowin 对 11 位边界比较的综合异常（本库已有 signed 乘法/条件折叠
    //      先例）。防御式改写：升坡算术结果先钳再存，且任何 ≥1024 的存量值
    //      （无论来源）同拍强制回 1024，外推侧再夹一层，三道防线不依赖找根因。
    wire [10:0] mb_up   = move_blend + 11'd4;
    wire [10:0] mb_up_s = (|mb_up[10:0]) &&
                          (mb_up[10:0] > 11'd1024) ? 11'd1024 : mb_up[10:0];
    wire [10:0] mb_dn   = (move_blend > 11'd8) ? (move_blend - 11'd8) : 11'd0;
    // 保持期禁降额（2026-11-02 dip2 闭环仿真定谳）：|θ̇|>8°/s 单一门在保持段
    // 构成正反馈——臂微摆超 8°/s ⇒ K3/K4 齐砍（刚度+阻尼消失）⇒ 摆幅更大
    // ⇒ 更常超门（tb_closedloop dip2：保持段 θ 在 145↔244° 间发散摆动，
    // late_error 59°；门 150 的 155314 板上卡死同签名）。降额只服务【真移动】
    // ——保持期臂速再大也只是扰动，位置刚度必须全程在场。
    // 移动判据两版板测（2026-11-02 晚 hold_test 三轮定谳）：
    //   tgt_err 版（|target−θ|>10°，215310 日志）：臂被摩擦坡卡住时 tgt_err
    //   大≠移动，是"该动没动"→ 降额死锁（k3e 卡 360）→ 60° rehold 循环甩飞，
    //   轮0/1 两轮同签名；轮2 臂恰好跟上 → 完美保持（span 0.0°）。
    //   ref 版（现行）：|pos_ref_traj−pos_tgt|>10° = 轨迹发生器还在推进才是
    //   真移动；ref 已吸附 target 而 θ 没到 = 保持失误，满额刚度顶摩擦。
    //   pos_ref_traj 在 pos_tgt 声明之后（本块前向引用，显式 wire 合法）。
    wire signed [31:0] traj_run_err = $signed(pos_tgt) - $signed(pos_ref_traj);
    wire pos_traj_running = (traj_run_err > 32'sd1000) || (traj_run_err < -32'sd1000);
    // 修七（2026-11-02 晚 220307 五轮定谳）：入场稳定窗（settle<1500）内
    // ref 随动（clear=1，设计来防入场瞬态），臂被稳摆通道带动中速溜坡时
    // ref 被拖着走 → |ref−tgt| 被动越 10° → 降额判据误触发 → K3 渐变爬升
    // 被拦腰砍断（360）→ 溜得更快 → 1.5s 窗满时臂已 700°/s → 满轨 slam
    // （逐帧：settle 99→1499 期间 θ −418→−806°、θ̇ 0→−736°/s、mb 0→1024、
    // k3e 892→360 同步反向）。窗内本就无位置权限，降额毫无意义——
    // 升坡再加一道 settle 满 1.5s 的门，窗内 K3/K4 渐变全力爬升。
    wire pos_blend_up = pos_moving && pos_traj_running &&
                        (pos_settle_ms >= POS_SETTLE_MS);
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) move_blend <= 11'd0;
        else if (tick) begin
            if (mode != `M_POS || pos_rehold) move_blend <= 11'd0;
            else if (pos_blend_up) move_blend <= mb_up_s;
            else if (move_blend > 11'd1024) move_blend <= 11'd1024;  // 越界存量强制回
            else if (move_blend != 11'd0)   move_blend <= mb_dn;
        end
    end
    // 第三道防线（外推侧夹紧）：混合公式对 blend∈[0,1024] 才是凸组合，
    // blend 一旦越界（无论什么来源），外推值越过两个端点——K3 被推成负值
    // 即"保持段位置刚度消失"的直接机制。此处把参与外推的 blend 硬夹在
    // [0,1024]，K3/K4 输出数学上保证落在 [min,max] 端点内。
    wire [10:0] mb_sat = (move_blend > 11'd1024) ? 11'd1024 : move_blend;
    wire signed [16:0] k3m_span = $signed(POS_MOVE_K3) - $signed(k3_eff);
    wire signed [27:0] k3m_prod = k3m_span * $signed({1'b0, mb_sat});
    wire signed [15:0] k3_eff_m = $signed(k3_eff) + (k3m_prod >>> 10);
    wire signed [16:0] k4m_span = $signed(POS_MOVE_K4) - $signed(k4_eff);
    wire signed [27:0] k4m_prod = k4m_span * $signed({1'b0, mb_sat});
    wire signed [15:0] k4_eff_m = $signed(k4_eff) + (k4m_prod >>> 10);
    // ---- P2'：级联倾斜指令 lean-to-move（2026-11-01，日志 221908 定谳后设计）----
    // K3·θe 通道对实际出力的协方差占比 −1.5%（死通道）：稳摆项占 duty 的 67%，
    // 位置精度靠 K3 提是被淹没的。改为让位置误差穿过**唯一有权限的稳摆通道**：
    // θe 积分 → α 偏置（饱和 ±1.5°）注入 LQR 的 α 输入。物理：想让臂向 − 方向
    // 回（θe>0），就让摆杆向 − 微倾，平衡环自己驱臂追赶（Segway"倾斜即加速"
    // 同构）；稳态 θe→0 → bias→0 → 摆杆回直立，无残差。
    // 符号（θe>0 时需要 τ<0 ⇒ LQR 的 α 输入须为负）：pos_bias = −sat(∫θe>>>13)。
    // SH=15（2026-11-03 竞赛日镇静版，原 13）：θe=10° 恒差时 bias 速率 0.03cdeg/ms
    // ≈0.3°/s。放慢 4 倍+饱和 100（原 150）：级联在互摆振荡（θe ±44°、1.5-2s 周期）
    // 中构成慢速正反馈（半周 te 正→bias 满→持续驱臂→加泵），是 POS 有界振荡
    // FELL（a_max 110-123°）的最后未动激励源。它的本职只是近目标精调——
    // 慢 4 倍+幅度 2/3 后远处几乎不泵、近处仍能收静态误差。
    // LQR 本体零改动（参考偏置不进极点）；稳摆模式 pos_bias≡0 逐位不变。
    reg signed [31:0] pos_bias_acc;
    // 轨迹运动检测（2026-11-02 v6 遥测定谳）：ref 在变 = 移动期，级联积分器
    // 冻结并回零——级联职责是"保持"，移动交给轨迹+K3。首版 移动全程 bias
    // 钉 −150 饱和与轨迹反向对抗（θe 涨到 90° 臂不动，摆杆被甩飞 117°）。
    // 判据用 |target−θ|>10°（命令位置差）而非 ref 速度：稳态窗内 target=θ±，
    // 一发 MOVE target 即跳开。
    wire signed [31:0] tgt_err = $signed(pos_tgt) - $signed(theta_c01);
    wire pos_traj_moving = (mode == `M_POS) &&
                           ((tgt_err > 32'sd1000) || (tgt_err < -32'sd1000));
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) pos_bias_acc <= 32'sd0;
        else if (tick) begin
            if (mode != `M_POS || entering_pos || pos_traj_moving || fl_hold) pos_bias_acc <= 32'sd0;
            else                                                   pos_bias_acc <= pos_bias_acc + theta_e;
        end
    end
    // 2026-11-03 晚：搬运用（用户思路——用稳摆通道的旋驱力把 theta 送到目标）。
    // 镇静版 SH15/±100 太慢太小（4x 慢 + 2/3 幅，位置误差推不动臂）；
    // 恢复到 SH13/±150：te=10° 恒差时 bias 速率 ≈0.12cdeg/ms（1.2°/s），
    // 30° 误差约 2s 建立 ±1.5° 倾角，稳摆环以 100-200°/s 输送臂——
    // 数据证明这是唯一能稳定推动臂的通道（K3 直驱在 2064 就泵摆）。
    wire signed [15:0] pos_bias_raw = pos_bias_acc >>> 15;
    wire signed [15:0] pos_bias_sat = (pos_bias_raw > 16'sd100)  ? 16'sd100 :
                                      (pos_bias_raw < -16'sd100) ? -16'sd100 :
                                      pos_bias_raw;
    // ================= PD 倾斜级联（P6，2026-11-03 夜）=================
    // 动机（184835/190223 板测）：POS 的主误差是臂绕目标 ±30~50° 的 0.25Hz 慢摆，
    // 不是直流。K3 直驱在死区线下（te=20° 只有 293 duty < 640-2600 静突破），
    // K4 是甜点型（5300/4000 摔、12000/18000 恶化）；上面这版【积分器】级联
    // 相位滞后 90° 且 |te|>10° 就冻结回零，摆动期间根本不工作。
    // 本版换成 PD：α_ref = −sat(kp·θe + kd·θ̇)，走稳摆环（K1 增益 17751，K3 的
    // 15 倍权限）驱动臂——P 同相（回复力）、D 超前（阻尼），稳态 θe=0 ⇒ 摆杆直立。
    // kp=0.05：θe=20° → 1.0° 倾角；kd=0.02：θ̇=200°/s → 0.4° 反倾（阻尼）。
    // 饱和 ±200 cdeg(2°)；乘积先打拍（不进组合关键路径，保 Fmax）。
    // PD_CASCADE_EN=0 时逐位回退到积分器版（pos_bias 原路径）。
    localparam PD_CASCADE_EN = 1'b0;   // 2026-11-04 晨板测：倾斜驱动泵摆（保持段 |te|43°/pp151°）→ 关
                                       // （|te| 24-27° / 路径 74°/s vs 5.5-24° / 8-58），
                                       // 保留代码作对照，默认关=走原积分器级联。
    localparam signed [15:0] PD_KP_Q10 = 16'sd123;   // 0.12：θe=10° → 1.2° 倾角（驱动）
    localparam signed [15:0] PD_KD_Q10 = 16'sd51;    // 0.05：θ̇=100°/s → 0.5° 反倾（刹车）
    localparam signed [15:0] PD_BIAS_LIM = 16'sd220; // ±2.2° 倾角上限
    reg signed [31:0] pd_p, pd_d;
    always @(posedge clk) begin
        pd_p <= $signed(theta_e)   * $signed(PD_KP_Q10);
        pd_d <= $signed(theta_dot) * $signed(PD_KD_Q10);
    end
    wire signed [31:0] pd_acc = pd_p + pd_d;
    wire signed [15:0] pd_raw = pd_acc >>> 10;
    wire signed [15:0] pd_sat = (pd_raw >  PD_BIAS_LIM) ?  PD_BIAS_LIM :
                                (pd_raw < -PD_BIAS_LIM) ? -PD_BIAS_LIM : pd_raw;
    // 入场/稳定窗内 te 无意义（tgt 跟踪 θ），PD 置 0；窗满后全程生效。
    wire pos_pd_active = (mode == `M_POS) && !entering_pos && (pos_settle_ms >= POS_SETTLE_MS);
    wire signed [15:0] pos_bias_pd = pos_pd_active ? (-pd_sat) : 16'sd0;
    // ★VEL_DEMO 速度环（2026-11-04 晨 v6）：速度误差 → 倾斜（不是 duty）。
    // 物理：tilt>0 → 摆杆前倾 → 平衡环（K1/K2，权限 15x）把臂加速 → 速度跟上；
    // 稳态 v=v_ref ⇒ tilt=0 ⇒ 摆杆回直立。这是唯一有权限的速度通道。
    // kv=0.5 cdeg 倾角 per (0.1°/s) 误差；限幅 ±200（2°）。
    // v6e（2026-11-04 晨，用户定案）：不追速度闭环，只要"臂在转、摆杆不掉、
    // v_ref 曲线准确"。kv=0.5 版（v6a）实测 20s 存活、α 1-8°、臂稳定在 ~190°/s
    // 自转（倾角常饱和 2°）；kv=0.05 反而 2s 掉摆。用 0.5。
    localparam signed [15:0] VEL_TILT_KV_Q8 = 16'sd38;    // 0.15（v6f：0.05 启动太弱/0.5 跑飞，取中）
    localparam signed [15:0] VEL_TILT_LIM    = 16'sd200;
    wire signed [15:0] vel_verr = v_ref - theta_dot;   // v6c：回到 v6a 符号
    wire signed [31:0] vel_tilt_p = vel_verr * VEL_TILT_KV_Q8;
    wire signed [15:0] vel_tilt_raw = vel_tilt_p >>> 8;
    wire signed [15:0] vel_tilt = (vel_tilt_raw >  VEL_TILT_LIM) ?  VEL_TILT_LIM :
                                  (vel_tilt_raw < -VEL_TILT_LIM) ? -VEL_TILT_LIM : vel_tilt_raw;
    wire signed [15:0] pos_bias = vel_demo_en ? vel_tilt :
                                  PD_CASCADE_EN ? pos_bias_pd :
                                  ((mode == `M_POS) ? (-pos_bias_sat) : 16'sd0);
    wire signed [15:0] alpha_lqr = alpha + pos_bias;   // 仅 POS 注入；遥测/门控仍用真 α
    // ================= 速度曲线演示（VEL_DEMO，2026-11-03 夜）=================
    // 用户方向：不需要绝对定位，只要"定速"。摆杆直立、臂按固件内生的三角波
    // 速度曲线旋转：v_ref 0→V（斜率 A）→0 循环；LQR 的 θ̇ 项从 K4·θ̇ 改成
    // K4·(θ̇−v_ref)，位置环同时把 pos_tgt 锚在臂上（K3≈0），两者不打架。
    // 匀速旋转对摆杆无惯性扰动（只有轴摩擦），加减速段由 LQR 吸收。
    // 遥测：借用 term_k1 字段回传 v_ref（该字段原是 0 桩）。
    // 状态机：SETTLE（v_ref=0，PI 先把臂刹停，等臂速/摆杆平静 300ms）
    // → RUN（三角波 V→0，到底停 500ms 再爬升）。演示关掉即全部复位，
    // 与正常控制完全隔离（K4 输入置 0 + PI 只在本模式生效）。
    localparam VEL_ST_SETTLE = 2'd0, VEL_ST_RUN = 2'd1;
    reg signed [15:0] v_ref;
    reg               vel_up;
    reg [1:0]         vel_state;
    reg [9:0]         vel_calm_ms;
    reg [9:0]         vel_dwell_ms;
    reg [3:0]         vel_div;
    wire vel_calm = (theta_dot < 16'sd800) && (theta_dot > -16'sd800) &&
                    (alpha < 16'sd600) && (alpha > -16'sd600);
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) vel_div <= 4'd0;
        else if (tick) vel_div <= (vel_div >= 4'd9) ? 4'd0 : vel_div + 4'd1;
    end
    // A 的单位 = 0.1°/s 每 10ms ⇒ A=1 即 10°/s²
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            v_ref <= 16'sd0; vel_up <= 1'b1; vel_state <= VEL_ST_SETTLE;
            vel_calm_ms <= 10'd0; vel_dwell_ms <= 10'd0;
        end else if (tick) begin
            if (!vel_demo_en) begin
                v_ref <= 16'sd0; vel_up <= 1'b1; vel_state <= VEL_ST_SETTLE;
                vel_calm_ms <= 10'd0; vel_dwell_ms <= 10'd0;
            end else if (vel_state == VEL_ST_SETTLE) begin
                // v6d：不再要求"平静"（臂已在转时永远等不到，v_ref 卡 0）。
                // 固定 800ms 缓冲后直接进 RUN——倾斜环自己把速度拉到目标。
                v_ref <= 16'sd0; vel_up <= 1'b1;
                if (vel_calm_ms >= 10'd800) begin
                    vel_state <= VEL_ST_RUN; vel_dwell_ms <= 10'd0;
                end else vel_calm_ms <= vel_calm_ms + 10'd1;
            end else if (vel_div == 4'd9) begin
                if (vel_dwell_ms != 10'd0) begin
                    vel_dwell_ms <= vel_dwell_ms - 10'd1;
                end else if (vel_up) begin
                    if (v_ref + vel_demo_a >= vel_demo_v) begin v_ref <= vel_demo_v; vel_up <= 1'b0; end
                    else v_ref <= v_ref + vel_demo_a;
                end else begin
                    if (v_ref - vel_demo_a <= 16'sd0) begin
                        v_ref <= 16'sd0; vel_up <= 1'b1; vel_dwell_ms <= 10'd500;
                    end else v_ref <= v_ref - vel_demo_a;
                end
            end
        end
    end
    // v5：K4 保留（它是唯一的臂阻尼，置 0 会让臂自由飞——v4 实测 987°/s）。
    // 速度环用独立 PI 压过 K4 的反阻尼分量（K4≈9.8 duty/(°/s)，Kp 取 ~30）。
    wire signed [15:0] theta_dot_lqr = theta_dot;
    // ---- 速度演示专用臂速 PI（2026-11-03 夜 v2）----
    // 实测平衡点：duty +1691 ↔ θ̇ +338°/s ⇒ 正 duty 推正速度。
    // Kp=1 duty/(0.1°/s)（20°/s 误差给 200 duty）；Ki 满 1000ms 在 20°/s
    // 误差下积到 ~780 duty，足以越过死区；积分限幅 ±2000 duty。
    localparam signed [15:0] VEL_KP_Q8 = 16'sd768;   // 3 duty/(0.1°/s)=30 duty/(°/s)，压过 K4
    localparam integer       VEL_I_SH  = 8;
    localparam signed [31:0] VEL_I_LIM = 32'sd1200 <<< VEL_I_SH;
    reg  signed [31:0] vel_i_acc;
    wire signed [15:0] vel_err = vel_demo_en ? (v_ref - theta_dot) : 16'sd0;
    wire signed [31:0] vel_p   = vel_err * VEL_KP_Q8;
    wire signed [31:0] vel_u_raw = (vel_p >>> 8) + (vel_i_acc >>> VEL_I_SH);
    // RUN 限 ±1500（够越死区又不甩摆）；SETTLE 只给 ±400 的温柔刹车
    // 注意：条件依赖运行时 reg（vel_state），必须用 wire 而非 localparam
    wire signed [31:0] VEL_U_LIM = (vel_state == VEL_ST_RUN) ? 32'sd1500 : 32'sd400;
    wire signed [15:0] vel_u = (vel_u_raw >  VEL_U_LIM) ?  VEL_U_LIM[15:0] :
                               (vel_u_raw < -VEL_U_LIM) ? -VEL_U_LIM[15:0] : vel_u_raw[15:0];
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) vel_i_acc <= 32'sd0;
        else if (tick) begin
            if (!vel_demo_en) vel_i_acc <= 32'sd0;
            else if (vel_i_acc + vel_err > VEL_I_LIM)  vel_i_acc <= VEL_I_LIM;
            else if (vel_i_acc + vel_err < -VEL_I_LIM) vel_i_acc <= -VEL_I_LIM;
            else vel_i_acc <= vel_i_acc + vel_err;
        end
    end
    lqr_core u_lqr (
        .clk(clk), .rst_n(rst_n), .start(start_d),
        .alpha(alpha_lqr), .alpha_dot(alpha_dot),
        .theta_e(theta_e), .theta_dot(theta_dot_lqr),
        .k1(k1), .k2(k2_eff), .k3(k3_eff_m), .k4(k4_eff_m),
        .u(duty_lqr), .done(lqr_done)
    );

    // 模式选择 → 寄存 1 拍（切断 mode 长组合链）→ 换向守卫 → 输出级
    // Leave zero commands in inactive modes so stale slew state cannot reappear.
    // 换向守卫（2026-10-22）：齿轮箱不允许快速换向，SWING 侧硬性限死（见 rev_guard.v）。
    wire signed [15:0] swing_req = (mode == `M_SWING) ? duty_swing : 16'sd0;
    wire signed [15:0] swing_gated;
    rev_guard #(.GUARD_MS(SWING_REV_GUARD_MS)) u_revguard (
        .clk(clk), .rst_n(rst_n), .tick(tick),
        .duty_in(swing_req), .duty_out(swing_gated)
    );
    reg signed [15:0] duty_sel;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) duty_sel <= 16'sd0;
        else        duty_sel <= fl_hold ? 16'sd0 :
                                (mode == `M_SWING) ? swing_gated :
                                ((mode == `M_BALANCE || mode == `M_POS) ? duty_bal : 16'sd0);
    end
    out_stage #(
        // 甩摆轨抬升（2026-11-03，借鉴 champion-merge 板验值 9500→9900）：
        // 低电量补偿 +4.9%——电池 19% 衰减即引发捕获秒死率飙升（11-3 实测）。
        // BAL/POS 不受影响：duty_bal 在上游（本文件 ~626 行）已自钳 ±9500，
        // out_stage 轨 9900 只对 SWING 的满轨需求生效（champion 同构）。
        .DUTY_RAIL(16'sd9900),
        // golden-base 板验配置（2026-10-28 降噪定案，2026-11-02 P3b 移植）：
        // POS 实测 duty 跨零 10.3/s×硬跳 690 ⇒ 齿轮箱被反复砸、底座共振。
        //   DZ_SOFT_TH=1024：|u|<1024 时补偿按 |u|/1024 线性给出，过零连续无硬跳
        //   DZ_THDOT_REF=200：速度自适应，θ̇=100°/s 时补偿降半（θ̇ 中位 88°/s ⇒ ~53%）
        //   REV_HYST=400：换向滞环仅 POS；SLEW 700 实测退化（teRMS 12.4→25.9）故 3000
        .SLEW_MAX(OUT_SLEW_MAX),
        .SLEW_MAX_SOFT(OUT_SLEW_SOFT),
        .DZ_SOFT_TH(OUT_DZ_SOFT_TH),
        .DZ_THDOT_REF(OUT_DZ_THDOT_REF),
        .BACKLASH_BOOST(OUT_BACKLASH_BOOST),
        .BACKLASH_MS(OUT_BACKLASH_MS),
        .REV_HYST(OUT_REV_HYST)
    ) u_out (
        .clk(clk), .rst_n(rst_n), .tick(tick),
        // SWING 用硬档（保证甩摆/刹车换向速度），其余模式软档（降噪）
        .slew_soft(mode != `M_SWING),
        .hyst_en(mode == `M_POS),   // 滞环仅 POS（BAL 稳摆靠高频微调）
        .thdot_in(theta_dot),
        // dz(θ) 位置查表仅 POS（摩擦-位置地图：BAL 稳摆高频微调依赖均匀补偿，
        // 位置表会在坡区造成过度补偿；SWING 全速不需要）
        // ★位宽纪律：theta_c01 是 32 位多圈量，out_stage 端口只收 16 位（其内部
        // 把 theta_in[15] 当符号位、[14:11] 当格号）。直接接 32 位会被隐式截断，
        // 多圈负角的高位丢失后符号位判错 → dz_idx 出 X（tb_top_smoke 实测）。
        // 显式先 mod 360°（65 536 c01）再取 16 位，与 out_stage 内部语义一致。
        .theta_in(theta_c01[15:0]),
        .dz_th_en(mode == `M_POS),
        .bypass(1'b0),
        .dz_pos(dz_pos), .dz_neg(dz_neg),
        .duty_in(duty_sel), .duty_out(duty_out),
        .dz_idx_o(diag_dz_idx)
    );

    // 板级外壳用：允许输出（含远程停机与模式门控）
    assign motor_enable = run_en & ~remote_stop &
                          ((mode == `M_SWING) || (mode == `M_BALANCE) || (mode == `M_POS));
    assign tick_out = tick;

    // ---- 遥测诊断量（记录格式 v3，2026-10-21）----
    // 全部在 tick 上打一拍再送出：一是切断"增益渐变乘法器 → 串口"的跨模块
    // 组合路径（Fmax 门禁），二是让四个量属于同一拍、和 duty 对得上。
    reg signed [15:0] k3_eff_d, k4_eff_d, duty_sel_d;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            k3_eff_d <= 16'sd0; k4_eff_d <= 16'sd0; duty_sel_d <= 16'sd0;
        end else if (tick) begin
            k3_eff_d   <= k3_eff_m;   // v6 修正：LQR 真输入（原 k3_eff 与降额后脱节）
            k4_eff_d   <= k4_eff_m;
            duty_sel_d <= duty_sel;
        end
    end
    // pos_trim 只在 POS 生效，其余模式归 0（与信号用途一致，避免误读）。
    assign diag_pos_trim    = (mode == `M_POS) ? pos_trim[15:0] : 16'sd0;
    assign diag_alpha_pre   = alpha_pre_w;
    assign diag_k3_eff      = k3_eff_d;
    assign diag_k4_eff      = k4_eff_d;
    assign diag_duty_pre_dz = duty_sel_d;
    assign diag_obs_active  = obs_act;
    // v4：θe 直接引出；分项乘积 RTL 置 0（上位机重建）。
    // 遥测报【未钳位】真实误差（2026-11-03）：钳位值会让 v6 遥测看不到
    // 臂游走真值（155433 曾见 te 恒 ±45 的假象，掩盖了 5463° 的跑飞）。
    assign diag_theta_e     = theta_e_raw;
    // v6：级联/降额/稳定窗内部状态（打拍与 k3_eff_d 同域）。
    reg [15:0] move_blend_d, settle_ms_d;
    reg signed [15:0] pos_bias_d;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            move_blend_d <= 16'd0; settle_ms_d <= 16'd0; pos_bias_d <= 16'sd0;
        end else if (tick) begin
            move_blend_d <= {5'd0, move_blend};
            settle_ms_d  <= {6'd0, pos_settle_ms};
            pos_bias_d   <= pos_bias;
        end
    end
    assign diag_pos_bias   = pos_bias_d;
    assign diag_move_blend = move_blend_d;
    assign diag_settle_ms  = settle_ms_d;
    assign diag_term_k1     = vel_demo_en ? v_ref : {12'd0, sb_on, sb_zone, burst_on, creep_zone}; // 轮3探针: bit2镇静 bit1突发 bit0蠕行区
    // 诊断（2026-11-04 晨）：term_k2/k3 原是恒 0 桩，借来做 MOVE 链路探针。
    //   term_k2 = move_delta（上位机下发的位移，c01，±6000 内）
    //   term_k3 = pos_tgt_cmd − θ（目标还需走多少，锯齿状=正常爬升；恒 0=命令丢了）
    wire signed [31:0] tgt_rem_diag = $signed(pos_tgt_cmd) - $signed(theta_c01);
    assign diag_term_k2     = move_delta[15:0];
    assign diag_term_k3     = (tgt_rem_diag >  32'sd32767) ?  16'sd32767 :
                              (tgt_rem_diag < -32'sd32768) ? -16'sd32768 :
                                                              tgt_rem_diag[15:0];
    // term_k4 = {14'b0, move_pending, move_pending_d}（探针：看 MOVE 握手有没有上升沿）
    assign diag_term_k4     = {14'd0, move_pending, move_pending_d};

    // ---------------- 串口：遥测 + 参数 ----------------
    wire tel_start, tx_busy;
    wire [7:0] tel_byte;
    wire rx_valid;
    wire [7:0] rx_byte;
    debug_link u_debug (
        .clk(clk), .rst_n(rst_n), .tick(tick),
        .hil_mode(hil_use),      // HIL 每拍都要回遥测供上位机积分，不能沿用 2ms 抽取
        .hil_tick(hil_tick), .hil_adc(hil_adc), .hil_enc(hil_enc),
        .rx_valid(rx_valid),.rx_byte(rx_byte),.tx_start(tel_start),.tx_byte(tel_byte),.tx_busy(tx_busy),
        .mode(mode), .fault(fault_latch),.otr_fault(otr_fault),.run_en(run_en),
        .calib_done(calib_done),.offset_c01(offset_c01),.adc(adc_src),.encoder(enc_src),
        .diag_encoder(diag_encoder),.diag_adc(diag_adc),
        .diag_pos_trim(diag_pos_trim),.diag_alpha_pre(diag_alpha_pre),
        .diag_k3_eff(diag_k3_eff),.diag_k4_eff(diag_k4_eff),
        .diag_duty_pre_dz(diag_duty_pre_dz),.diag_obs_active(diag_obs_active),
        .diag_theta_e(diag_theta_e),.diag_term_k1(diag_term_k1),
        .diag_term_k2(diag_term_k2),.diag_term_k3(diag_term_k3),
        .diag_term_k4(diag_term_k4),
        .diag_pos_bias(diag_pos_bias),.diag_move_blend(diag_move_blend),
        .diag_settle_ms(diag_settle_ms),.diag_dz_idx(diag_dz_idx),
        // v5 遥测（2026-10-31）：原始量测链（观测器 mux 之前）+ 精确增益 +
        // 积分器分量。本树无 DFB，k1_eff=k1（裸参数）、k2_eff=k2_eff；
        // pos_i_trim=pos_trim（无 AT 分离结构，差值恒 0）。
        .diag_theta_raw(theta_c01_i),.diag_alpha_raw(alpha_i),
        .diag_theta_dot_raw(theta_dot_i),.diag_alpha_dot_raw(alpha_dot_i),
        .diag_k1_eff(k1),.diag_k2_eff(k2_eff),
        .diag_pos_i_trim(pos_trim[15:0]),
        .alpha(alpha), .alpha_dot(alpha_dot),
        .theta(theta_c01), .theta_dot(theta_dot), .duty(duty_out),
        .reference(pos_ref),.target(pos_tgt),
        .local_stop(key_stop),.local_start(key_start || calibration_wait!=0 || calibration_request),
        .k1(k1),.k2(k2),.k3(k3),.k4(k4),.dz_pos(dz_pos),.dz_neg(dz_neg),
        .sw_amp(sw_amp),.catch_ang(catch_ang),.fall_ang(catch_ang_fall),.relative_target(theta_ref_lo),
        .fling_delta(fling_delta),
        .pos_k3(pos_k3),.pos_k4(pos_k4),
        .alpha_trim(alpha_trim),
        .remote_start(remote_start),.remote_stop(remote_stop),.remote_clear(remote_clear),
        .remote_mode(remote_mode),.remote_calib(remote_calib),
        .remote_force_pos(remote_force_pos),
        .vel_demo_en(vel_demo_en), .vel_demo_v(vel_demo_v), .vel_demo_a(vel_demo_a),
        .arm_stalled(arm_stalled),
        .io_test_enable(io_test_enable),.io_test_duty(io_test_duty),
        .move_pending(move_pending),.move_delta(move_delta),
        .move_is_abs(move_is_abs),.config_lock(config_lock),
        .revision(config_revision),.rx_errors(rx_errors),
        .mt6_frames(mt6_frames),.mt6_good(mt6_good),
        .mt6_crc_err(mt6_crc_err),.mt6_mg_err(mt6_mg_err),
        .mt6_mg_code(mt6_mg_code),.mt6_angle_raw(mt6_angle_raw),
        .mt6_fault(mt6_fault),.mt6_sel(th_src_mt),.mt6_cause(mt6_cause),
        .mt6_max_no_good(mt6_max_no_good),
        .mt68_frames(mt68_frames),.mt68_good(mt68_good),
        .mt68_crc_err(mt68_crc_err),.mt68_status_err(mt68_status_err),
        .mt68_status(mt68_status),.mt68_angle_raw(mt68_angle_raw),
        .mt68_fault(mt68_fault)
    );
    uart_tx #(.BAUD_DIV(`UART_BAUD_DIV)) u_tx (
        .clk(clk), .rst_n(rst_n),
        .start(tel_start), .data(tel_byte), .busy(tx_busy), .tx(uart_tx)
    );

    uart_rx #(.BAUD_DIV(`UART_BAUD_DIV)) u_rx (
        .clk(clk), .rst_n(rst_n), .rx(uart_rx),
        .valid(rx_valid), .data(rx_byte)
    );
endmodule
