// ============================================================================
// out_stage.v — 输出级：死区补偿 + 压摆率限制 + 限幅
// 死区补偿 u' = u + sign(u)·dz/2（dz 由斜坡法实测，param_regs 在线整定），
// 是消除平衡点慢速极限环的关键非线性补偿（架构文档 §3.4）。
// ============================================================================
`include "j280_defs.vh"
module out_stage #(
    parameter SLEW_MAX = 16'sd3000,  // 每 tick 最大变化 30%（0.01%/ms）
    // 输出轨（2026-10-24 参数化）：默认 9500（历史约定，留 5% 驱动余量）。
    // T20K 提到 9900 = 甩摆/刹车的低电量补偿（用户提议"同样 duty 动能小就提高
    // duty"——ACCEL 输出 9440 已贴 9500 轨，唯一空间是抬轨本身；TB6612 99%
    // 占空安全，稳摆/POS 在 ctrl_core 另有 9500 自钳不受影响）。等效电压 +4.9%。
    parameter signed [15:0] DUTY_RAIL = 16'sd9500,
    // 软死区补偿阈值（2026-10-28）。0 = 旧行为（全额半死区补偿，硬跳）。
    // 板级实测（POS 45s）：duty 跨零 10.3 次/秒，每次跨零实际输出硬跳
    // mean 1858 / max 3790，其中死区补偿一项就贡献 ±690。齿轮箱在齿隙里被
    // 这样反复砸 ⇒ "咔咔"声 + 底座共振。
    // 非零时（必须是正的 2 次幂）：|duty_in| < TH 的软区内，补偿量按
    // |duty_in|/TH 线性缩放，过零连续，不再有 ±dz/2 的阶跃。
    parameter signed [15:0] DZ_SOFT_TH = 16'sd0,
    // 模式相关压摆（2026-10-28）：SWING 需要快速换向/刹车泄能（甩摆半周期~500ms、
    // 刹车相要 6ms 级反转），不能限太狠；BALANCE/POS 的咔咔声来自跨零时的高速
    // 力矩反转，需要压。slew_sel=1 时用 SLEW_MAX_SOFT，0 时用 SLEW_MAX。
    parameter signed [15:0] SLEW_MAX_SOFT = 16'sd3000,
    // ---- 速度自适应死区补偿（2026-10-28，用户定案）----
    // 640/740 是"从静止启动克服静摩擦"的力；POS 运行中臂 55.5% 时间在动
    // （|θ̇| 中位 88°/s），动摩擦只需其一小部分。全额补偿每拍恒加 673 duty
    // （占控制量 14%），且每次跨零固定跳 690 ⇒ 每秒往齿隙里砸 10.3 次，
    // 这就是"咔咔 + 底座共振"的主因。
    // 补偿缩放 = REF/(REF+|θ̇|)：静止全额、|θ̇|=REF 时半额、更快则更小。
    // THDOT_SEL 单位与 thdot_in 相同（0.1°/s）；0 = 关闭（旧全额行为）。
    parameter signed [15:0] DZ_THDOT_REF = 16'sd0,
    // ---- 过零反向齿隙补偿（2026-10-28，用户提出）----
    // 齿轮箱换向时有一段空程（齿隙），duty 反向后要"空转"一小段才真正咬合、
    // 才有力传到臂上。这期间控制律以为已经出力，实际是零力矩 ⇒ 相位滞后 +
    // 臂漂。POS 实测 duty 换向 10.3 次/秒，每次都吃这个亏。
    // 做法：检测 duty 符号翻转的瞬间，在随后 BACKLASH_MS 内把输出幅值额外
    // 抬高 BACKLASH_BOOST（叠加在正常指令上，方向跟随新符号），帮它尽快
    // 跨过空程。与"静摩擦补偿"是两件不同的事：
    //   静摩擦补偿：恒定幅值，从静止启动用（运行中可减 —— 见 DZ_THDOT_REF）
    //   齿隙补偿  ：**只在换向后短时施加**，因为只有换向才遇到空程
    parameter signed [15:0] BACKLASH_BOOST = 16'sd0,  // 额外幅值（0=关）
    parameter [9:0]         BACKLASH_MS    = 10'd30,  // 补偿持续时间
    // ---- 换向滞环（2026-10-29 单变量①：仿真 σ×3 改善，实机验证机械代价）----
    // POS 静摆换向 19.7-21.5 次/s（每秒 0.3-0.6s 在齿隙空程）。hyst_en 仅 POS。
    parameter signed [15:0] REV_HYST = 16'sd0,
    // ---- 位置相关死区补偿 dz(θ)（2026-11-02，摩擦-位置地图驱动）----
    // 自动扫描地图（123724_friction，154 事件）：静摩擦突破 1200-2000 duty
    // 随位置起伏 ±40%（θ mod 360 多峰）。K3=1200 位置环 @25° 误差 731+dz640
    // =1371 ≈ 静突破 1400 ⇒ 保持漂移 ±25-30° 的物理根源；±5° 保持需
    // K3≥12450（线性稳定域外）——唯一出路=补偿跟着位置走。
    // 16 格 LUT（每格 22.5°，最近邻足够：摩擦坡宽 30-60°）。值=该格静突破
    // duty（板级实测，见 tmp/fit 后地图）；半幅生效由下方 dz_hp 语义统一处理：
    // 即 LUT 给"该格等效 dz_pos"（全幅口径），输出路径不变。
    // DZ_TH_EN=0 时整块旁路（行为与旧版逐位一致）。
    parameter DZ_TH_EN = 0,
    parameter signed [15:0] DZ_TH_0  = 16'sd1400,  // θ∈[0,22.5)
    parameter signed [15:0] DZ_TH_1  = 16'sd1500,  // [22.5,45)
    parameter signed [15:0] DZ_TH_2  = 16'sd2200,  // [45,67.5) 坡区×1.4（150826 实证 LUT 低估→臂锁死）
    parameter signed [15:0] DZ_TH_3  = 16'sd2600,  // [67.5,90) 坡区×1.3
    parameter signed [15:0] DZ_TH_4  = 16'sd2200,  // [90,112.5) 坡区×1.4
    parameter signed [15:0] DZ_TH_5  = 16'sd1400,  // [112.5,135)
    parameter signed [15:0] DZ_TH_6  = 16'sd1200,  // [135,157.5)
    parameter signed [15:0] DZ_TH_7  = 16'sd1000,  // [157.5,180)
    parameter signed [15:0] DZ_TH_8  = 16'sd1200,  // [180,202.5)
    parameter signed [15:0] DZ_TH_9  = 16'sd1400,  // [202.5,225)
    parameter signed [15:0] DZ_TH_10 = 16'sd1600,  // [225,247.5)
    parameter signed [15:0] DZ_TH_11 = 16'sd2400,  // [247.5,270) 坡区×1.33
    parameter signed [15:0] DZ_TH_12 = 16'sd2600,  // [270,292.5) 坡区×1.3
    parameter signed [15:0] DZ_TH_13 = 16'sd2400,  // [292.5,315) 坡区×1.33
    parameter signed [15:0] DZ_TH_14 = 16'sd1600,  // [315,337.5)
    parameter signed [15:0] DZ_TH_15 = 16'sd1400   // [337.5,360)
) (
    input  wire               slew_soft,  // 1 = 用 SLEW_MAX_SOFT（BALANCE/POS）
    input  wire               hyst_en,    // 1 = 换向滞环生效（仅 POS；BAL 稳摆
                                          // 依赖高频微调，滞环会拖死纠正速度）
    input  wire signed [15:0] thdot_in,   // 臂速（0.1°/s），用于补偿自适应
    input  wire signed [15:0] theta_in,   // 臂角 c01（多圈，下方 mod 360）
    input  wire               dz_th_en,   // 1 = dz(θ) 查表生效（ctrl_core 仅 POS 置 1）
    input  wire               clk,
    input  wire               rst_n,
    input  wire               tick,
    input  wire               bypass,    // 1 = 不补偿不限幅（调试直通）
    input  wire signed [15:0] dz_pos,    // 正向死区 0.01%
    input  wire signed [15:0] dz_neg,    // 反向死区 0.01%
    input  wire signed [15:0] duty_in,
    output reg  signed [15:0] duty_out,
    output wire [3:0]         dz_idx_o    // dz(θ) LUT 格号（v6 遥测）
);
    // 半死区补偿：dz_pos/dz_neg（运行时）或 dz(θ) LUT（DZ_TH_EN）二选一，
    // 见下方 dz_pos_eff/dz_neg_eff。
    // ---- dz(θ) 查表（DZ_TH_EN=1 时用位置格值替换运行时 dz 的幅值）----
    // θ c01 → 16 格：idx = (θ mod 360)/22.5，>>11 提取（36000/16=2250 c01/格，
    // 取 >>11≈2048 近似格宽 20.5°——格间误差小于摩擦坡宽度，可接受）。
    wire [15:0] th_pos = theta_in[15] ? (16'd0 - theta_in) : theta_in[14:0];
    wire [3:0]  dz_idx = theta_in[15] ? (4'd0 - th_pos[14:11])   // 负角（多圈负）取补
                                      :        th_pos[14:11];
    assign dz_idx_o = dz_idx;
    wire signed [15:0] dz_lut = (dz_idx == 4'd0)  ? DZ_TH_0  :
                                (dz_idx == 4'd1)  ? DZ_TH_1  :
                                (dz_idx == 4'd2)  ? DZ_TH_2  :
                                (dz_idx == 4'd3)  ? DZ_TH_3  :
                                (dz_idx == 4'd4)  ? DZ_TH_4  :
                                (dz_idx == 4'd5)  ? DZ_TH_5  :
                                (dz_idx == 4'd6)  ? DZ_TH_6  :
                                (dz_idx == 4'd7)  ? DZ_TH_7  :
                                (dz_idx == 4'd8)  ? DZ_TH_8  :
                                (dz_idx == 4'd9)  ? DZ_TH_9  :
                                (dz_idx == 4'd10) ? DZ_TH_10 :
                                (dz_idx == 4'd11) ? DZ_TH_11 :
                                (dz_idx == 4'd12) ? DZ_TH_12 :
                                (dz_idx == 4'd13) ? DZ_TH_13 :
                                (dz_idx == 4'd14) ? DZ_TH_14 : DZ_TH_15;
    // 生效条件：DZ_TH_EN 且 ctrl_core 置 dz_th_en（仅 POS）。LUT 值为该位置
    // 静突破 duty（全幅口径）——直接替换 dz_pos/dz_neg 的角色，过半幅路径。
    wire signed [15:0] dz_pos_eff = (DZ_TH_EN && dz_th_en) ? dz_lut : dz_pos;
    wire signed [15:0] dz_neg_eff = (DZ_TH_EN && dz_th_en) ? dz_lut : dz_neg;
    wire signed [15:0] dz_hp = dz_pos_eff >>> 1;
    wire signed [15:0] dz_hn = dz_neg_eff >>> 1;
    wire signed [31:0] dz    = (duty_in > 16'sd0) ? {16'b0, dz_hp} :
                               (duty_in < 16'sd0) ? (32'sd0 - {16'b0, dz_hn}) : 32'sd0;
    // 软死区：|duty_in| < TH 时把补偿按比例缩小 ⇒ 过零连续（无 ±dz/2 阶跃）。
    // 符号只由 duty_in 携带，幅值恒非负（旧版负×负得正补偿的坑见 pos-control 树）。
    wire signed [15:0] dz_mag = (duty_in > 16'sd0) ? dz_hp :
                                (duty_in < 16'sd0) ? dz_hn : 16'sd0;
    wire [15:0] u_abs = (duty_in < 16'sd0) ? -duty_in : {1'b0, duty_in[14:0]};
    localparam integer DZ_SH = $clog2((DZ_SOFT_TH > 0) ? DZ_SOFT_TH : 2);
    wire signed [31:0] dz_soft = ($signed({1'b0, u_abs}) * $signed({{16{dz_mag[15]}}, dz_mag})) >>> DZ_SH;
    wire signed [31:0] dz_eff  = (DZ_SOFT_TH != 16'sd0 && u_abs < DZ_SOFT_TH[15:0])
                                 ? dz_soft : dz;
    // 速度自适应：scale ≈ REF/(REF+|θ̇|)，Q10。
    // ★ 不能用真正的除法器：首版用 32 位 `/` 直接把 Fmax 从 38.9MHz 压到
    //   16.6MHz（47 处 setup 违例）。改为**分段线性**近似，只用移位/加法：
    //     r = |θ̇| / REF   （用移位比较实现整数比值档）
    //     scale = 1024 / (1 + r) 在 r = 0,1,2,3,≥4 处取 1024,512,341,256,205，
    //     档内线性插值（用移位近似，误差 < 5%），足够控制用。
    // 效果：静止全额、|θ̇|=REF 半额、|θ̇|=3·REF 时约 1/4。
    wire [15:0] thd_abs = (thdot_in < 16'sd0) ? -thdot_in : {1'b0, thdot_in[14:0]};
    // 连续线性缩放（无除法、无阶跃）：scale = clamp(1024 - |θ̇|·K, MIN, 1024)
    //   K 由 REF 决定：|θ̇| = REF 时约降到 MIN。取 K = (1024-MIN)/REF，
    //   用「先乘常数再右移」实现——REF 是编译期常数，K 也是常数，无除法器。
    //   为保持可调且不引入除法，这里用倒数常数：K_Q16 = (1024-MIN)*65536/REF
    //   在 localparam 里由综合器预计算（常数折叠，不进硬件）。
    localparam integer REF_I = (DZ_THDOT_REF > 0) ? DZ_THDOT_REF : 1;
    localparam integer DZ_MIN = 205;                       // 下限 1/5 全额
    localparam integer K_Q16  = ((1024 - DZ_MIN) * 65536) / REF_I;
    localparam integer K_Q10  = (K_Q16 + 32) >> 6;         // 转 Q10（乘法用）
    // 速度自适应作用域（2026-11-02）：仅 slew_soft（BAL/POS）生效——SWING 甩摆/
    // 刹车期 θ̇ 恒大，速度项会把补偿缩没、力矩顶不动真死区（NO_BAL 6/6 实证）。
    // BALANCE 侧实测也与 golden 时代"全家 NO_BAL"同签名——进一步仅 POS：
    wire thdot_adapt_en = slew_soft && hyst_en;   // hyst_en 即 POS（ctrl_core 接线）
    wire [31:0] thd_e   = {16'd0, thd_abs};
    wire [31:0] dec     = (thd_e * K_Q10) >>> 10;          // 常数乘 + 移位
    wire [10:0] dz_scale_r = (DZ_THDOT_REF == 16'sd0 || !thdot_adapt_en) ? 11'd1024 :
                             (dec >= 11'd819)             ? 11'd205  :
                             (11'd1024 - dec[10:0]);
    wire signed [31:0] dz_vel =
        ($signed(dz_eff) * $signed({1'b0, dz_scale_r})) >>> 10;
    wire signed [31:0] dz_use = (DZ_THDOT_REF == 16'sd0) ? dz_eff : dz_vel;
    // 齿隙补偿：duty 符号翻转后 BACKLASH_MS 内额外抬幅
    reg        bl_arm;                 // 补偿窗口激活
    reg [9:0]  bl_cnt;
    reg signed [15:0] du_prev;
    wire       flip = (duty_in > 16'sd0 && du_prev < 16'sd0) ||
                      (duty_in < 16'sd0 && du_prev > 16'sd0);
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bl_arm <= 1'b0; bl_cnt <= 10'd0; du_prev <= 16'sd0;
        end else if (tick) begin
            du_prev <= duty_in;
            if (BACKLASH_BOOST != 16'sd0 && flip) begin
                bl_arm <= 1'b1; bl_cnt <= 10'd0;
            end else if (bl_arm) begin
                if (bl_cnt >= BACKLASH_MS - 1) bl_arm <= 1'b0;
                else                           bl_cnt <= bl_cnt + 10'd1;
            end
        end
    end
    // 补偿方向跟随当前 duty 符号；duty=0 时不加（否则会自己造出符号）
    wire signed [31:0] bl_amt =
        (BACKLASH_BOOST == 16'sd0 || !bl_arm) ? 32'sd0 :
        (duty_in > 16'sd0) ?  {16'd0, BACKLASH_BOOST[15:0]} :
        (duty_in < 16'sd0) ? -{16'd0, BACKLASH_BOOST[15:0]} : 32'sd0;
    wire signed [31:0] comp   = bypass ? duty_in : (duty_in + dz_use + bl_amt);
    // 两级流水：第 1 拍寄存死区补偿+饱和结果，压短 mode→mux→dz 长组合链
    reg signed [15:0] target_r;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) target_r <= 16'sd0;
        else        target_r <= sat16(comp);
    end

    // 第 2 级：压摆率限制（tick 节拍）
    localparam signed [31:0] SLEW_HARD = SLEW_MAX;       // SWING 档
    localparam signed [31:0] SLEW_SOFTQ = SLEW_MAX_SOFT; // BALANCE/POS 档
    wire signed [31:0] SLEW = slew_soft ? SLEW_SOFTQ : SLEW_HARD;
    // 换向滞环：极性 = 最近一次非零输出；反向小信号滑行（极性保持）
    reg  [1:0] pol_out;   // 0=从未 1=+ 2=−
    wire [1:0] req_pol = (target_r > 16'sd0) ? 2'd1 :
                         (target_r < 16'sd0) ? 2'd2 : 2'd0;
    wire       rev_small = (REV_HYST != 16'sd0) && slew_soft && hyst_en &&
                           (req_pol != 2'd0) && (pol_out != 2'd0) && (req_pol != pol_out) &&
                           (u_abs < REV_HYST[15:0]);
    wire signed [31:0] target_sel = rev_small ? 32'sd0 : $signed(target_r);
    // ★2026-11-04 修复（组合环）：原实现在 always 块外定义
    //     diff = target_sel - duty_out;  step = clamp(diff, ±SLEW);
    //   而在块内 duty_out <= sat16(duty_out + step)。
    //   duty_out(reg) ← step(wire) ← duty_out 构成【组合环】：iverilog 在
    //   time-0（duty_out 尚为 X）解析环得 X，X 又沿 sat16 自反馈永久锁死
    //   （实测 duty_out=X → PWMA=X，其余中间量全正常）。
    //   Gowin 综合因采样语义恰好"能用"，但仿真不可判定，必须断环。
    //   修法：把减法/限幅直接内联进非阻塞赋值的右值表达式，并让它读
    //   duty_out 的"下一拍寄存器副本"——这样右值只依赖已寄存的量，无环。
    reg signed [15:0] duty_q;          // duty_out 的同步副本（断环用）
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            duty_out <= 16'sd0; pol_out <= 2'd0; duty_q <= 16'sd0;
        end else if (tick) begin
            duty_q <= duty_out;
            // 用 duty_q（上一拍）而非 duty_out 计算，右值不含环
            duty_out <= sat16(duty_q +
                              ((target_sel - $signed(duty_q) >  SLEW) ? SLEW :
                               (target_sel - $signed(duty_q) < -SLEW) ? -SLEW :
                               (target_sel - $signed(duty_q))));
            if (!rev_small && req_pol != 2'd0) pol_out <= req_pol;
        end
    end

    function signed [15:0] sat16(input signed [31:0] v);
        sat16 = (v >  $signed({16'd0, DUTY_RAIL}))   ? DUTY_RAIL  :
                (v < -$signed({16'd0, DUTY_RAIL}))   ? -DUTY_RAIL : v[15:0];
    endfunction
endmodule
