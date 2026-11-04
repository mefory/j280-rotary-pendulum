// ============================================================================
// mode_fsm.v — 模式状态机（1ms 节拍采样）
// IDLE --KEY_START--> SWING --捕获域(滞回)--> BALANCE <--KEY_MODE--> POS
// IDLE --KEY_MODE且手扶就绪--> BALANCE（调参用直接稳摆，不经过甩摆输出）
//  BALANCE/POS 中 |α|>掉摆阈值 → 默认 SAFE（单次起摆）；可由 AUTO_RESWING 恢复自动重甩摆
//  任意态 --KEY_STOP 或安全故障--> SAFE --KEY_START--> IDLE
// 捕获滞回：进 30°(臂速 <500°/s)/出 90°，防阈值抖动切换。
// 2026-09-28 实测定案：α̇ 门限（无论 50 或 150°/s）在本机不可用——甩摆后杆过顶
// 旋转 ~350-580°/s（p50≈420），低摩擦下 25s 内几乎不衰减，有限门限永不满足；
// 150°/s 门三轮 0 捕获。回到 09-23 实机验证版（47aad1d）：无 α̇ 条件，首次
// 过顶即捕获，LQR 速度反馈兜住（数值镇定域至 1560°/s，simulink/fling_check）。
// ============================================================================
`include "j280_defs.vh"
module mode_fsm #(
    // 捕获底座速度门（0.1°/s）。32767=禁用（DE10 默认，无门限捕获）；
    // T20K 配 20000（200°/s）：甩摆把底座甩到 3000°/s 级飞转，过顶瞬间底座
    // 仍在狂飞则 K4·θ̇ 顶满舵、捕获后 0.2-0.4s 必甩飞（222416 三次实测），
    // 等摩擦衰减到温和过圈再抓。
    parameter signed [15:0] CATCH_TD_LIM = 16'sd32767,
    // 摆杆角速度门（0.01°/s）。32767=禁用（DE10 默认）。T20K 配 4000（40°/s，
    // 2026-10-10）：15° 窗下回落 pass 的 α̇ 实测 28-60°/s，59.5°/s 热杆制动
    // 越顶摆到 -91° 必掉（swing_gate 杆1），27.7°/s 冷杆 9s 收敛可稳——拒抓
    // 热杆交给 3s 补甩重试，不赌 LQR 的制动极限。
    parameter signed [15:0] CATCH_AD_LIM = 16'sd32767,
    // 捕获驻留去抖（ms）。0=旧行为（DE10）。T20K 配 20（2026-10-10）：摆杆甩进
    // 电位器轨道端区（显示角 ±78~94°）时雨刷跳抬，α/adc 单拍乱跳 ±100°
    // （swing_gate 杆2 adc 0↔54943、杆4 7852↔-992），腐败样本恰好单拍落进
    // 捕获门→假捕获→下拍真值 |α|>90° 立即掉摆。真实 pass 以 ≤40°/s 穿 30°
    // 窗需 ~750ms，20ms 驻留零代价；端区腐败只撑 1-2 拍，被拒。
    parameter [5:0] CATCH_DWELL_MS = 6'd0,
    // 捕获动量下限（0.01°/s）。0=禁用（DE10 默认，旧行为）。T20K 配 1000
    // （10°/s，2026-10-10）：死杆 pass（折返点附近准静止，swing_gate 杆1
    // α=13.35°/α̇=-5.8°/s）物理上接不住——满舵臂加速度折算 0.25 m/s²，13°
    // 静止摆需 2.2 m/s²，饱和下增益无感；只收内向动量 pass（|α̇|≥下限）。
    // 被拒的摆荡回去由补甩再泵，等下一个好 pass。
    parameter signed [15:0] CATCH_AD_MIN = 16'sd0,
    // 近直立豁免半宽（0.01°）。0=无豁免（DE10 默认）。T20K 配 500（±5°）：
    // 手扶等价的小角度准静止态在臂预算内（需求 ∝ sinα），不设动量下限。
    // 2026-10-22 修正：豁免必须同时要求摆速小（|α̇|<CATCH_DEAD_AD），否则
    // 高速过顶（|α|=1° 但 α̇=246°/s）会被放行——实测该情形 0.8s 必掉。
    // 豁免的本意只是"准静止"，不是"恰好在零附近"。
    parameter signed [15:0] CATCH_DEAD_BAND = 16'sd0,
    parameter signed [15:0] CATCH_DEAD_AD   = 16'sd0,
    // 掉摆门驻留去抖（ms）。0=旧行为（DE10）。T20K 配 30：电位器轨道端区
    // 腐败（顶轨=显示角 93.6°>90° 掉摆门）可单拍假掉摆；真实掉摆从窗内
    // 滑到 90° 需数百 ms，30ms 驻留零代价。
    parameter [5:0] FALL_DWELL_MS = 6'd0,
    // 掉摆自动重甩（BALANCE/POS out_fall → SWING）。默认随 `AUTO_RESWING
    // （DE10 单次起摆语义不变）；T20K 覆盖 1=硬件级自愈重试（演示保底）。
    parameter AUTO_RESWING_EN = `AUTO_RESWING,
    // 甩摆有界：单次 START 内不自动重甩，25s 超时强制 SAFE（捕获依赖衰减尾，
    // 实测 ~14s，2s 窗口会掐死整个衰减捕获路径，2026-09-27 实测定案）。
    // 参数化仅为 tb_control_path 缩短仿真（默认 25000 与定案一致）。
    parameter [15:0] SWING_TIMEOUT_MS = 16'd40000,
    // POS 最小驻留（ms，2026-11-01 P1 移植 champion 10-21）：进入 POS 后该窗口内
    // 任何 key_mode 都不退出，兜住命令/按键的采样竞争（champion 实测 POS 83 拍→
    // BAL 88 拍→POS 846 拍三连跳，每次都清积分器、重走 K3/K4 渐变）。
    parameter [15:0] POS_MIN_DWELL_MS = 16'd300
) (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               tick,
    input  wire               key_start,    // 单拍脉冲
    input  wire               key_mode,     // 单拍脉冲
    input  wire               key_stop,     // 单拍脉冲
    input  wire               fault,        // safety 故障（OTR/看门狗）
    input  wire signed [15:0] alpha,
    input  wire signed [15:0] alpha_dot,
    input  wire signed [15:0] theta_dot,
    input  wire               arm_stalled,  // 臂真停（arm_vel 300ms 无边沿判停，2026-09-30）
    input  wire               force_pos,    // 强制 BAL→POS（绕过 pos_ready，仅 |α|<15° 由上位机把关）
    input  wire signed [15:0] catch_ang,    // 进入阈值（正幅值）
    input  wire signed [15:0] catch_ang_fall,
    output reg  [2:0]         mode
);
    wire base_win = (alpha > -catch_ang)      && (alpha < catch_ang) &&
                    (theta_dot > -16'sd5000) && (theta_dot < 16'sd5000) &&
                    (theta_dot > -CATCH_TD_LIM) && (theta_dot < CATCH_TD_LIM) &&
                    (alpha_dot > -CATCH_AD_LIM) && (alpha_dot < CATCH_AD_LIM);
    // 内向 = 朝零运动（α 与 α̇ 反号；α=0 视为内向）。
    wire inward   = (alpha > 0) ? (alpha_dot < 0) :
                    (alpha < 0) ? (alpha_dot > 0) : 1'b1;
    wire ad_lo    = (alpha_dot <= -CATCH_AD_MIN) || (alpha_dot >= CATCH_AD_MIN);
    wire momentum_ok = (CATCH_AD_MIN == 16'sd0) ? 1'b1 : (inward && ad_lo);
    wire dead_ok  = (CATCH_DEAD_BAND == 16'sd0) ? 1'b0 :
                    ((alpha > -CATCH_DEAD_BAND) && (alpha < CATCH_DEAD_BAND) &&
                     ((CATCH_DEAD_AD == 16'sd0) ||
                      ((alpha_dot > -CATCH_DEAD_AD) && (alpha_dot < CATCH_DEAD_AD))));
    wire in_catch = base_win && (dead_ok || momentum_ok);
    wire out_fall  = (alpha >  catch_ang_fall) || (alpha < -catch_ang_fall);
    reg [15:0] swing_ms;
    reg [15:0] pos_ms;      // POS 驻留计时（饱和，离开 POS 归零）
    reg [5:0]  catch_dw;
    reg [5:0]  fall_dw;
    // 手扶直立直接稳摆使用更严格的固定门限，避免把自动甩摆的宽捕获域用于人工接管。
    // 单位：alpha 0.01°；alpha_dot/theta_dot 0.1°/s。
    // 2026-10-20 实测收紧 |α| 门限 5°→1.5°：连续 5 次同配置（K3=600/trim=50）
    // 稳摆实验显示入场 α 是存活决定因素——α=+0.85°/+1.51° 均活过 99s；
    // α=+1.72°/+1.96°/+3.74° 分别 3.3/3.7/1.0s 崩。机理=K1 正反馈：
    // duty=K1·α+死区补偿，α 偏差越大初始推力越大 → 臂被推 → 枢轴继续把
    // 摆杆外推 → α 更大。判据：入场后 500ms 的 θ̇ 超 ~90°/s 必崩。
    // 1.5° 门限恰好分开已知样本；手扶场景本就要求摆杆稳住再接管。
    wire manual_balance_ready = (alpha > -16'sd150) && (alpha < 16'sd150) &&
                                (alpha_dot > -16'sd300) && (alpha_dot < 16'sd300) &&
                                (theta_dot > -16'sd200) && (theta_dot < 16'sd200);
    // 与模型一致：摆杆接近直立、摆臂低速时才接收定位请求。
    // 2026-09-29 实机：|θ̇|<200（20°/s）在稳摆期近乎常闭——健康稳摆 θ̇RMS 就有
    // 106-209°/s（臂大幅补偿摆杆）+ 静止粘滑 T 法尖峰，实测 30s/750 发零命中；
    // 而捕获后稳摆存活窗仅 10-23s（ctrl_core 增强相剂量教训），POS 入场在结构上
    // 不可能。放宽到 600（60°/s）。
    // 2026-09-30 再进一步：θ̇ 阈值判不住"物理静止"（T 法读数在静止粘滑下
    // 0↔400°/s 尖峰），POS 入场改为要求 arm_stalled——但实机复测：稳摆本身
    // 要求臂持续微调（平衡是臂的工作），臂以 ~6°/s 匀漂、永不停，stall 门
    // 永不打开。改为 stall 或慢速（<15°/s）：软臂环下 10°/s 入场实测 POS
    // 稳到 αRMS 0.4°，慢速入场足够安全。
    // 2026-11-03 晚：内层慢速门 15→40°/s。θe 钳位后 BAL 臂以 ~18°/s 匀漂
    // （160141 实测中位 17.8°/s），15°/s 门通过率仅 1.2% → MOVE 虽排队但
    // mode_fsm 等不到同拍 pos_ready → 六轮全 NO_POS。40°/s 让门常开；入场
    // 瞬态由 1.5s 稳定窗（traj clear+K3/K4 渐变）吸收，与 09-30 慢速入场
    // 同语义（当时 15°/s 的基准是 6°/s 漂移）。
    wire pos_ready = (alpha > -16'sd500) && (alpha < 16'sd500) &&
                     (theta_dot > -16'sd600) && (theta_dot < 16'sd600) &&
                     (arm_stalled || (theta_dot > -16'sd400 && theta_dot < 16'sd400));

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mode <= `M_SAFE;
            swing_ms <= 16'd0;
            pos_ms <= 16'd0;
            catch_dw <= 6'd0;
            fall_dw  <= 6'd0;
        end else if (tick) begin
            if (mode == `M_SWING && swing_ms < SWING_TIMEOUT_MS)
                swing_ms <= swing_ms + 1'b1;
            else if (mode != `M_SWING)
                swing_ms <= 16'd0;
            if (mode == `M_POS && pos_ms < POS_MIN_DWELL_MS)
                pos_ms <= pos_ms + 1'b1;
            else if (mode != `M_POS)
                pos_ms <= 16'd0;
            // 驻留计数默认每拍清零，仅对应分支覆盖递增——离开该模式自动归零
            catch_dw <= 6'd0;
            fall_dw  <= 6'd0;
            case (mode)
                `M_IDLE:    if (key_mode && !fault && manual_balance_ready) mode <= `M_BALANCE;
                            else if (key_start)                    mode <= `M_SWING;
                `M_SWING:   if (key_stop || fault || swing_ms >= SWING_TIMEOUT_MS) mode <= `M_SAFE;
                            else if (in_catch && catch_dw >= CATCH_DWELL_MS) mode <= `M_BALANCE;
                            else if (in_catch)         catch_dw <= catch_dw + 6'd1;
                `M_BALANCE: if (key_stop || fault)     mode <= `M_SAFE;
                            else if (out_fall && fall_dw >= FALL_DWELL_MS)
                                                       mode <= AUTO_RESWING_EN ? `M_SWING : `M_SAFE;
                            else if (out_fall)         fall_dw <= fall_dw + 6'd1;
                            else if (key_mode && (pos_ready || force_pos)) mode <= `M_POS;
                `M_POS:     if (key_stop || fault)     mode <= `M_SAFE;
                            else if (out_fall && fall_dw >= FALL_DWELL_MS)
                                                       mode <= AUTO_RESWING_EN ? `M_SWING : `M_SAFE;
                            else if (out_fall)         fall_dw <= fall_dw + 6'd1;
                            else if (key_mode && pos_ms >= POS_MIN_DWELL_MS) mode <= `M_BALANCE;
                `M_SAFE:    if ((key_start || key_stop) && !fault) mode <= `M_IDLE;
                default:    mode <= `M_SAFE;
            endcase
        end
    end
endmodule
