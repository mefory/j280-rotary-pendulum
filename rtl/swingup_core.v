// ============================================================================
// swingup_core.v — 甩摆（fling）自动起摆：位置伺服把臂甩到 进入SWING时锁存的
// theta0+fling_delta，靠反作用把摆杆荡上倒立点；捕获窗命中后由 mode_fsm 切
// BALANCE，LQR 接管前还须通过 mode_fsm 的 |α̇|<50°/s 捕获门限；
// 旧数值模型声称的高速到达可恢复，不代表现行 RTL 会在高速时切入稳摆。
// 取代旧有界能量泵：WHEELTEC 先验参数下泵律收敛到过顶旋转极限环（2026-09-18
// pump_trace 扫描 15 组参数不收敛），甩摆为同类实机出厂方案。
// u = sat(FLING_KP*(theta0+delta-theta), amplitude)，FLING_KP=9 duty/cdeg
// （10° 误差即饱和，WHEELTEC 式 bang 甩）；delta/amplitude 为 V2 运行时参数。
//
// 自动补甩（2026-10-09，RETHROW_MS）：单发甩相位敏感——发力瞬间臂的相位
// 决定能否过顶，实测连续 3 次 SWING 超时无捕获（一次坐在目标点 4.8s 杆荡
// 而不过）。RETHROW_MS>0 时，SWING 内每该毫秒数从**当前 θ** 重新锁存
// target（=多次甩泵：每次甩从当前相位再 +delta，杆过顶即被捕获窗接走；
// 未过顶则回摆衰减，下一发再甩）。0=旧行为（单发，DE10 默认）。
//
// 能量泵模式（2026-10-11，PUMP_MODE=1）：实时能量伺服取代盲补甩——
//   E = ½ω² + (g/l_eff)(1-cosα)，u = sat(K_E·(E−E_target)·sign(ω·cosα), amp)
// 只在摆杆下半区（|α|<90°, cosα>0）打正功（荡秋千）；上半区切弱回中伺服
// （位置环 KP/3 限 amp/3 拉回 θ0）防臂漂走。E_target 对应 apex≈22°，每圈
// 生成落在捕获动量带 [10,40]°/s 的 pass，捕获门挑最好一拍接走——泵+门互补，
// 当年 pump_trace 单泵不收敛（目标=过顶能量→旋转极限环）的问题不存在：
// 目标能量远低于过顶（7.6/207），极限环就是"绕 apex22° 的持续可收 pass 流"。
// 定点：E 单位 = 0.01(rad/s)²。kin = α̇²·1523>>>30（α̇[0.01°/s]→rad/s² 换算
// π/18000≈1.745e-4，½ω²=α̇²·1.523e-8，×100=1.523e-6≈1523/2^30）；pot = ROM
// [1° 步长]×10400>>>8（(g/l_eff)≈104 (rad/s)²，1-cosα∈[0,1]→8bit ROM）。
// 90 项 ROM 覆盖 |α|∈[0,90°)，|α|≥90° 视为满能量（不泵）。
// ============================================================================
module swingup_core #(
    parameter RETHROW_MS    = 0,     // 补甩周期（拍）；0=单发（旧行为）
    parameter RETHROW_TD_MAX = 16'sd32767,  // 补甩的底座速度门（0.1°/s）：
                                           // 底座狂飞时不再补甩加力
    // 补甩摆杆相位门（0.01°）。0=无门（旧行为，DE10 默认）。T20K 配 9000
    // （|α|>90° 摆杆在下半区才补甩，2026-10-10）：定时盲甩会搅黄正在成形的
    // 窗口 pass——只在摆杆垂下时泵力（荡秋千式），临近窗口的 pass 不打扰。
    parameter RETHROW_ALPHA_MIN = 16'sd0,
    // 补甩/入场摆速平静门 |α̇| 上限（0.1°/s）。0=旧行为（不查摆速，DE10
    // 默认）。T20K 2000=200°/s：掉摆重甩入场或摆杆尚在甩时不锁目标，待平静。
    parameter RETHROW_ADOT_MAX = 16'sd0,
    // 补甩放行要求的连续平静时长（ms）。仅 ADOT_MAX≠0 时生效；80ms 约为
    // 下垂摆半个周期的量级，防大幅摆动折返点（瞬时低速）误放行。
    parameter [7:0] RETHROW_CALM_MS = 8'd80,
    // 自适应甩距（0=恒用参数 delta，DE10 默认）。1=按每杆实测 apex 阶梯
    // 调整补甩甩距，收敛进捕获带（详见实现处注释）。
    parameter ADAPTIVE_RETHROW_EN = 0,
    // 起摆模式。0=fling+盲补甩（DE10 默认，行为不变）；1=能量伺服泵（实机
    // 已否决，勿用）；2=共振激振（2026-10-12，用户手动数据定案：sign(α̇)
    // 方波驱动频率自锁摆固有频率，幅值 amp/8，包络 |α̇| 峰 220°/s 停推）。
    parameter PUMP_MODE = 0,
    // 泵增益 Q8 [duty / 0.01(rad/s)²]：u = K_E×(E−E_target)>>>8。
    // 默认 256=1.0：E 误差 9000（≈满舵量级）即饱和 amp。
    parameter signed [15:0] PUMP_KE = 16'sd256,
    // 泵目标能量 0.01(rad/s)²。默认 758 ⇔ apex≈22°（104×(1−cos22°)×100）。
    parameter signed [31:0] PUMP_E_TARGET = 32'sd758,
    parameter ALPHA_QUALITY_EN = 0,
    // 模式 2：每 1ms 推力幅值最多增加 1.5%，撤力/减力不延迟。
    parameter RES_DRIVE_SHAPING_EN = 0,
    parameter RES_RISE_STEP = 150,
    // 甩摆到达撤力半窗（0.01°，0=旧行为）。T20K 配 800（8°）：臂进目标 8° 内撤力
    // 滑行，避免纯比例伺服在 target 附近 ±8800 自激振荡（详见实现处注释）。
    parameter signed [31:0] SWING_DEAD = 32'sd0,
    // ---- 三相位甩摆（2026-10-22，FLING_BRAKE_EN）----
    // 0=旧行为（纯比例伺服，逐位一致，tb_control_path 依赖）；1=加速→刹车→阻尼保持。
    // 机理与实板依据见实现处注释（比例伺服甩后 ±100°/1Hz 等幅振荡会随机对摆杆
    // 做功，这是"有时一杆就起、有时 40s 起不来"的真正来源）。
    parameter FLING_BRAKE_EN = 0,
    // 刹车相**按时间**结束，不按臂速结束。原因是实测缺陷：满舵期间 MT6701 θ 链
    // 会冻 10-30ms（2026-10-22 实测：满舵段 θ 停滞密度是低占空段的 20 倍，279
    // 次/整场），θ̇ 的 M-T 估计在空档里指数衰减到 0——按 |θ̇|<VSTOP 判推出会让
    // 刹车在真实臂速还有 500°/s 时提前退出，残余速度全砸进保持相。
    // 刹车时长由 ACCEL 期间实测峰值臂速算出：t = v/a，a≈1800°/s² ⇒ v[0.1°/s]>>4 ms
    // （除法改移位：真实除法器会吃掉 Fmax）。
    parameter FLING_BRAKE_SH = 4,
    parameter [9:0] FLING_BRAKE_MS = 10'd600,      // 刹车时长上限（兜底）
    parameter [9:0] FLING_BRAKE_MIN_MS = 10'd120,  // 刹车时长下限
    // 刹车时长延长（2026-10-23 能量账定标）：实测第一杆底部能量 239~251，而捕获线
    // 是 E_bottom≥262（第二杆 262~269 就捕获了）——只差 4.4%。刹车时长是最直接的
    // 加能量口：多刹这一段，摆杆从"反向拉回"里拿到的功就多一分。
    parameter [9:0] FLING_BRAKE_EXTRA_MS = 10'd0,
    // 提前撤刹车的臂速门（0.1°/s）+ 需连续满足的拍数。设 0 关闭提前退出。
    // 40 拍远长于 30ms 的 θ 掉线，真停住才放行，掉线骗不过去。
    // ★ 门限必须 ≥ θ̇ 的量化台阶：MT6701 1 raw LSB/ms = 9.3064 c01/ms = 931（0.1°/s），
    // 即 θ̇ 静止时就在 0/±931 之间跳。设 200（20°/s）物理上不可达 ⇒ 提前退出永不触发，
    // 刹车只能靠时间兜底 ⇒ 必然过刹、反向，实测把摆杆抛上来的功又抵掉一半。
    parameter signed [15:0] FLING_VSTOP = 16'sd1000,
    parameter [5:0] FLING_VSTOP_CNT = 6'd40,
    // 刹车结束的摆杆速度门（0=关闭）。见实现处注释：越过水平后等 |α̇| 降到
    // FLING_VEND_V 才撒手；MIN_MS 保证鞭甩已建立。
    parameter FLING_VEND_EN = 0,
    parameter signed [15:0] FLING_VEND_V = 16'sd600,
    parameter [9:0] FLING_VEND_MIN_MS = 10'd150,
    // ACCEL 时间上限（ms）。θ 掉线时行程判据冻住，没有时间兜底会一直顶满舵。
    parameter [9:0] FLING_ACCEL_MS = 10'd450,
    // 摆杆过水平即停加速（0.01°，0=禁用）。机理：臂对摆杆做功的功率 ∝ cosα
    // （α 从竖直向上算），摆杆一旦越过水平（|α|<90°），继续同向加速就在**卸能**。
    // 起摆只在 |α|>90° 区间加速才有效，之后交给刹车/滑行把摆杆带过顶。
    parameter signed [15:0] FLING_ALPHA_END = 16'sd9000,
    parameter [9:0] FLING_ALPHA_MIN_MS = 10'd60,   // 加速相至少跑这么久才认 α 判据
    // 保持相 θ 停滞看门狗（ms）：θ 采样连续这么多拍不变就归零输出。
    // 真的停住时归零无代价（摩擦自己停），传感器冻结时防止恒定指令把臂慢慢
    // 推起来（θ 冻着看不见，只有这个看门狗能兜住）。
    parameter [6:0] FLING_STILL_MS = 7'd100,
    // 保持相阻尼位置环：duty = KP*(hold−θ) − KV*θ̇_f，KP 单位 duty/cdeg，
    // KV 单位 duty per (0.1°/s)。θ̇_f 是 EMA 后的臂速。
    // ★ KV 不能大：θ̇ 的量化台阶本身就是 931（0.1°/s，MT6701 1 raw LSB/ms）。
    // 旧值 6 ⇒ 931×6 = 5586 duty 的纯量化噪声，臂几乎不动而 H 桥在 ±4500 之间
    // 满幅翻向 —— 2026-10-22 实板实测 **45 次极性翻转/秒**（用户听成"高速换相
    // 共振"）。KV=2 配 EMA(>>>3) 后噪声压到 ~660 duty，低于死区补偿 640 量级。
    parameter signed [15:0] FLING_HOLD_KP = 16'sd2,
    parameter signed [15:0] FLING_HOLD_KV = 16'sd2,
    // 保持相权限上限（duty）。位置环在 ~30° 误差时就够顶到 8800 满舵，那就等于
    // 把"甩后振荡"原样搬进保持相。夹到 4000：够顶死区 640 的 6 倍、够抗摆杆
    // 反作用，但顶不到满舵 ⇒ 臂不会再被甩起来。
    parameter signed [15:0] FLING_HOLD_MAX = 16'sd4000,
    // 保持相指令死区（duty）。|duty|<该值时输出 0，让 out_stage 的
    // sign(duty)·dz 半死区补偿整段旁路——否则 ±50 的保持指令会被放大成
    // ±690 的继电器，臂在保持点来回弹（旧实现 duty=0 段之所以真能静止，
    // 正是因为指令恰好为 0）。
    parameter signed [15:0] FLING_HOLD_DZ = 16'sd250,
    // ---- 换向频率（2026-10-22 用户澄清：不是"不能快"，是"不能高频"）----
    // 策略保持原样：先正向加速、再反向拉回（鞭甩），换向本身可以快。
    // 要消灭的是**高频**换向——实板 45 次极性翻转/秒来自 HOLD 相的定量位置环
    // （duty 在 ±4500 之间随 θ̇ 的 931 量化台阶翻向）。结构性处置：
    //   · HOLD 相只短路制动、不出力（FLING_HOLD_BRAKE=1）⇒ 每杆只剩 1 次换向
    //     （ACCEL + → BRAKE −），补甩周期 3s ⇒ ≤0.33 次/秒；
    //   · ctrl_core 的 rev_guard 驻留窗（T20K 300ms）是第二道硬闸；
    //   · 本参数只留作机械保护的可选斜率限幅（0=不限，鞭甩最强）。
    parameter [15:0] FLING_SLEW_STEP = 16'd0,
    // HOLD 相改为"只短路制动、不出力"（2026-10-22）：刹车后臂已接近停住，
    // 位置环再出力只会制造"低速小幅换向"（实测 45 次极性翻转/秒，用户听成
    // 高速换相共振）。1=HOLD 只发短路制动 + duty 恒 0。0=旧位置环（留档回退）。
    parameter FLING_HOLD_BRAKE = 0,
    // ---- 相位驱动的刹车（2026-10-23 用户实感定案）----
    // 用户原话："有时候过顶旋转一圈感觉没有刹车，有时候刹太早了，能量不够。"
    // 旧实现按 f_peak>>4 估时长、或用 |θ̇| 判停——两者都与摆杆相位无关：
    //   · 满舵期间 MT6701 的 θ 会冻 10-30ms，θ̇ 塌到 0 ⇒ 假"停住" ⇒ 刹车提前撤，
    //     臂带着 700°/s 继续飞（"没刹车，过顶转一圈"）；
    //   · 峰值臂速估出来的时长与实际需要不成比例 ⇒ 有时刹不够（"能量不够"）。
    // 泵能律只认相位：a>0 在 |α|>90° 且上升时做正功；a<0（反向拉回）在
    // |α|>90° 且**下降**时做正功。于是把一杆拆成：
    //   ACCEL（|α|>90 上升）→ COAST（整个上半区，刹车在这里是卸能，什么都不做）
    //   → BRAKE（|α|>90 下降，反向拉回做正功）→ COAST（摆杆升到水平以上，撒手）。
    // 全部相位判据，不再有"猜时长"和"用 θ̇ 判停"。
    parameter signed [15:0] FLING_BRAKE_STOP = 16'sd1500,  // |α|<15° 收手交给捕获
    parameter signed [15:0] FLING_BRAKE_FLIP = 16'sd300,  // 判"上升/下降"的 α̇ 门（30°/s）
    parameter [9:0] FLING_COAST_MS = 10'd900,             // 滑行相的兜底时长
    parameter FLING_PHASE_BRAKE = 0,                      // 1=相位驱动刹车（新）
    // ---- 滑行相微剂量能量伺服（COAST_DOSE_EN，2026-10-24 用户定方向）----
    // 依据（114351/114419/145812 对照）：首杆 apex 停在刀锋上——刹车 436ms⇒apex 0.1°
    // （过顶、入场好、稳 30s）、470ms⇒17.9°（差一口气、擦窗入场约半数死），能量
    // 灵敏度 0.9 单位/ms ⇒ 定时不可能跨电池站在刀锋上。本伺服在刹车后的滑行相里
    // 用**小剂量**（AMP×DOSE_MS，约满舵鞭甩的 1/20 分辨率）按需补最后一口气：
    // 摆杆在内向半程（30°<|α|<100°，耦合 |cos|>0.17）且总能量低于目标-迟滞时，
    // 顺着泵能律方向（duty 与 α̇·cosα 异号做正功）注入一记；能量到线即停。
    // 目标 E_TARGET 取刚过顶（过顶静止=11730，+3.5% ⇒ 顶上 ~85°/s 温柔穿窗）。
    parameter COAST_DOSE_EN = 0,
    parameter [31:0] COAST_E_TARGET = 32'd12600,   // 目标总能量（0.01(rad/s)²）
    parameter [31:0] COAST_E_HYST   = 32'd800,     // 迟滞：低于目标-迟滞才起剂
    parameter signed [15:0] COAST_AMP = 16'sd2500, // 剂量幅度（duty）
    parameter [9:0] COAST_DOSE_MS = 10'd120,       // 单剂时长
    parameter [9:0] COAST_GAP_MS  = 10'd180,       // 两剂最小间隔
    parameter [1:0] COAST_MAX     = 2'd2,          // 每杆最多剂量数
    // ---- 相位 + 能量门补甩（RETHROW_PHASE_EN，2026-10-23）----
    // 依据（文档 §九 的能量账）：单杆做功上限 ≈45 单位 vs 过顶需 41，还要被臂
    // （飞轮）吃掉一半以上 ⇒ 单杆必然时中时不中，**必须多杆累积**。旧补甩是
    // "3s 定时 + 摆杆平静（|α̇|<200°/s）才放行"，等于每杆都从静止重来，把上一杆
    // 挣到的能量白扔。本门改成跟摆杆节奏走：
    //   摆杆在底部（|α|≥150°）、有明确摆动方向（|α̇|>30°/s）、
    //   且**底部速度还低于过顶所需**（|α̇|<RETHROW_PHASE_V）时立刻补一杆。
    // 过顶判据：从底部到过顶要 ω=sqrt(2·(g/l))=sqrt(41)≈6.4 rad/s=367°/s；
    // 实测回归 (g/l)=20.5 ⇒ 取 520°/s 作门（含 22.5° apex 衰减后的 508°/s 仍放行，
    // 而已经能过顶的 600°/s 以上不再补）。纯速度判据，省掉 cos 查表与宽乘法。
    parameter RETHROW_PHASE_EN = 0,
    parameter signed [15:0] RETHROW_PHASE_V = 16'sd5200,      // 底部 |α̇| 上限（0.1°/s）
    parameter signed [15:0] RETHROW_PHASE_ADOT = 16'sd300,    // 要有明确方向
    parameter signed [15:0] RETHROW_PHASE_ALPHA = 16'sd15000, // |α|≥150° 才算底部
    parameter [9:0] RETHROW_PHASE_GAP = 10'd500,              // 距上次起杆最少 ms
    // ---- 双向能量泵（PUMP_MODE=4，2026-10-23 用户定案）----
    // 用户原话："改为双向能量泵，就是之前我自己手动模拟起摆的那样，向左向右反复
    // 晃荡到合适的时机注入能量，这样起摆后的状态可以控制，也不需要一边的很大的 duty。"
    // 泵能律（用实测回归符号 α̈ = 0.1176sinα − 1.37e-5·duty·cosα 定标）：
    // 臂对摆杆做功 P ∝ −duty·α̇·cosα ⇒ 只有在 duty 与 α̇·cosα **异号**时做正功。
    // 所以：
    //   duty 方向 = sign(α̇·cosα) 取反 —— 摆杆朝哪边摆、在上半区还是下半区，
    //   决定这一拍该往哪边推；这就是"到合适的时机注入"。
    //   幅度固定 BPUMP_AMP（≈42% 占空，不再是满舵），每次注入的能量小 ⇒ 末态可控。
    //   能量到顶（|α̇| > BPUMP_V_STOP）就滑行，不再加。
    // 防抖：|α̇| < BPUMP_ADOT_MIN 时方向不可信（量化噪声 25），**保持上一方向**，
    // 于是换向只在 α̇ 真正穿过 ±30°/s 时发生（≈每半周期一次，0.4~0.5s），
    // 不可能出现旧泵 28Hz 那种抖振；rev_guard(120ms) 仍是第二道闸。
    parameter signed [15:0] BPUMP_AMP = 16'sd4000,       // 出力幅度（duty）
    parameter signed [15:0] BPUMP_ADOT_MIN = 16'sd300,   // 低于此速度保持方向（0.1°/s）
    parameter signed [15:0] BPUMP_V_STOP = 16'sd6000,    // 到达此摆速即滑行（0.1°/s）
    // 两档幅度（2026-10-23 实测定标）：摆杆底部要 ~877°/s 才能过顶（周期 822ms
    // ⇒ g/l≈58.7、(1+cos) 差 2 ⇒ ω=sqrt(2·2·58.7)=15.3rad/s）。
    // 只用一档满舵会"要么冲不过、要么直接转圈"——实测 600°/s 门→apex 84° 爬不上，
    // 1200°/s 门→飞起来转圈（臂峰 1399°/s）。所以在接近目标速度时换小幅度细调：
    //   |α̇| < V_SOFT      → 满幅度（差距大，猛推）
    //   V_SOFT ≤ |α̇| < V_STOP → 小幅度（每半周只补一点，末态可控）
    //   ≥ V_STOP          → 滑行
    parameter signed [15:0] BPUMP_V_SOFT = 16'sd7800,   // 0.1°/s
    parameter signed [15:0] BPUMP_AMP_SOFT = 16'sd2500, // 细调幅度
    // ---- 能量伺服（2026-10-23 用户："还是会旋转，可以好好考虑刹车吗"）----
    // 只用 |α̇| 判停是错的：泵在半高处收手后，摆杆继续把势能变动能，底部冲到
    // 1652°/s（过顶只需 877°/s）⇒ 转圈。正确做法是**用总能量判**：
    //   E = ½α̇² + (g/l)(1+cosα)，(g/l)=58.7（实测周期 822ms ⇒ ω0=7.64rad/s）
    //   E < E_TARGET − HYST → 按泵能方向推（加能）
    //   E > E_TARGET + HYST → **反推泄能**（把多余能量刹掉，这正是缺的刹车）
    //   带内 → 滑行，等捕获窗
    // 单位 0.01(rad/s)²。E_TARGET：过顶需 2·(g/l)=117.4；取 120 ⇒ 过 15° 窗时
    // |α̇|≈150°/s，正好落在捕获动量带 100~400 的中间。
    parameter signed [31:0] BPUMP_E_TARGET = 32'sd12000,
    parameter signed [31:0] BPUMP_E_HYST   = 32'sd1500,
    parameter BPUMP_ESERVO = 0,   // 1=能量伺服（含泄能刹车）
    // ---- 泵 + 甩摆混合（BPUMP_HYBRID_EN，2026-10-23 用户定案）----
    // 实测定标：双向泵在 apex≈85° 撞天花板，甩摆单杆 ACCEL 也停在 84.4°——
    // 同一个墙（耦合 ∝cosα 在水平位置归零，底部堆的能量在爬升途中被臂吃掉）。
    // 唯一破过这堵墙的是反向拉回刹车（实板 apex 22.5°）。所以分工：
    //   泵负责 180°→~85°（这一段泵效率最高、末态可控）；
    //   apex 好在 TRIG 以内且摆杆在底部有明确方向时**交棒给甩摆一杆**
    //   （ACCEL + 反向拉回），打完回 HOLD 再由泵接着评估。
    // 每轮仍只有 1 次换向，泵的连续晃荡变成"准备动作"。
    parameter BPUMP_HYBRID_EN = 0,
    parameter signed [15:0] BPUMP_APEX_TRIG = 16'sd9000,  // apex<90° 就交棒
    parameter [11:0] BPUMP_HYBRID_GAP = 12'd1200,          // 两次交棒最小间隔
    // 方向更新的最小驻留（ms）：即使相位判据跳变也不许更快换向。配合下面
    // "|α̇| 太小 / 快过水平"时直接不出力的做法，换向被限制在每半周期一次。
    parameter [9:0] BPUMP_DWELL_MS = 10'd150,
    // ---- 两段式起摆 stage2：底部相位微冲（TRIM_EN，2026-10-22 夜）----    // 实板数据（20261022_222059）把"为什么时不时一次起不来"量化了：第一杆把
    // 摆杆打到 |α|_min=22.5°，刚好差在捕获窗（|α|<15° 且 |α̇|∈[100,400]°/s）
    // 外面；随后 apex 22.5°→55.3°→70.0°→89.8°，能量每周期衰减。
    // 能量账（用本文件实测回归系数）：加速相在 α:180°→90° 做功 +22.6 单位、
    // 反向刹车在 90°→180° 又 +18.7 单位，合计 41.3，而"从底部到过顶"需要
    // 41.0 —— 单杆的物理上限就只有 ~100%，没有任何余量。所以单杆必然时中时不中。
    // 修法不是加大单杆（已经顶到满舵），而是**在底部按相位补一记小冲**：
    //   · 只在上一杆 apex 差于 TRIM_APEX_MAX 时补；
    //   · 只在摆杆过底部（|α|≥TRIM_ALPHA）且 |α̇| 足够分辨方向时补；
    //   · 方向取 sign(α̇)（与泵能律同向），脉冲长度 TRIM_MS 决定补多少能量
    //     （底部 α̇≈340°/s 时 60ms ≈ +8 单位，正好把 22.5° 抬到 45 单位
    //      ⇒ 过 15° 窗时 |α̇|≈175°/s，正落在捕获动量带中间，入场更温柔）。
    // 这一记冲的臂速增量只有 ~110°/s，制动相也不参与，机械上是小动作。
    parameter TRIM_EN = 0,
    // 微冲按**转角**结束，不按时间：做功 = |b2|·duty·Δsinα，底部 Δsin 由转过的
    // 角度决定而与速度无关；按时间会在摆杆底部很快时（实测 α̇≈1250°/s）一记
    // 送出 21 单位（应是 6），直接把摆杆打过顶 → 捕获变成 350°/s 的飞越，
    // LQR 接不住（3/3 捕获但 0.6~1.6s 全掉）。TRIM_DANGLE=15° 对应 Δsin=0.26
    // ⇒ 稳定给 +6 单位，与底部速度解耦；TRIM_MS 只做兜底。
    parameter [11:0] TRIM_DANGLE = 12'd1500,
    parameter [9:0]  TRIM_MS = 10'd80,
    parameter signed [15:0] TRIM_ALPHA = 16'sd16000,    // 底部窗口 |α|≥160°
    parameter signed [15:0] TRIM_APEX_MAX = 16'sd1200,  // 上次 apex 差于 12° 才补
    parameter signed [15:0] TRIM_ADOT_MIN = 16'sd500,   // |α̇|≥50°/s 才认相位
    parameter [3:0]  TRIM_MAX = 4'd2,                   // 每杆最多补几次
    // 两次微冲的最小间隔（ms）。同时起到"必须先看到一杆自己的 apex"的作用：
    // 摆杆半周期 ≈800ms，900ms 内不允许补冲 ⇒ 不可能在加速相刚结束的底部
    // 立刻叠一记（那是 ACCEL 的活，不是 stage2 的活）。
    parameter [9:0]  TRIM_GAP_MS = 10'd900,
    // ---- 单极性相位泵（PUMP_MODE=3，2026-10-22）----
    // 泵能半周判据：|α|≥该值（0.01°）才算"下半区"。摆杆对臂加速的做功
    // 功率 ∝ cosα，|α|<90° 时代数为卸能，一律不出力。
    parameter signed [15:0] PUMP_GATE_ALPHA = 16'sd9000,
    // 相位判据的 α̇ 门（0.1°/s）。α̇ 来自 4ms 差分（1 LSB/4ms = 25），
    // EMA 后再留 8 倍裕量：200=20°/s。
    parameter signed [15:0] PUMP_PHASE_MIN = 16'sd200,
    // 单次出力时长（ms）：上限 / 下限。上限决定这一次给臂多少速度
    // （1900°/s² × 200ms ≈ 380°/s），下限保证起振脉冲真的推出去。
    parameter [9:0] PUMP_PULSE_MS     = 10'd200,
    parameter [9:0] PUMP_PULSE_MIN_MS = 10'd60,
    // 两次出力之间的强制静默（ms）。结构性节奏上限 ≈1/(PULSE+REST)，
    // 且 duty 只有 0/+amp 两态 ⇒ 不存在任何电源换向。
    parameter [9:0] PUMP_REST_MS = 10'd120,
    // 臂速上限/停住判据（0.1°/s）。超过上限不再加速（改由短路制动泄速），
    // 停住判据用于判定短路制动是否可以撤掉。
    parameter signed [15:0] PUMP_TD_CAP  = 16'sd6000,
    parameter signed [15:0] PUMP_TD_STOP = 16'sd800,
    // 起振脉冲：摆杆悬垂静止时没有相位可用（α̇≈0），必须给一记脉冲才起摆。
    // 条件=|α|>KICK_ALPHA、|α̇|<KICK_ADOT、臂已停、且静止累计 KICK_HOLD_MS。
    parameter signed [15:0] PUMP_KICK_ALPHA = 16'sd15000,
    parameter signed [15:0] PUMP_KICK_ADOT  = 16'sd400,
    parameter [9:0] PUMP_KICK_HOLD_MS = 10'd200,
    // 摆杆动能上限（0.1°/s）：超过它说明已经能过顶（甚至已在转圈），
    // 停止加速让摩擦把能量收回来，防止旋转极限环。
    parameter signed [15:0] PUMP_ADOT_CAP = 16'sd7000
) (
    input wire clk, rst_n,
    input wire tick,
    input wire start,              // 进入 SWING 的沿（ctrl_core 模式沿，含 tick 门控）
    input wire signed [31:0] theta,      // 连续 0.01°
    input wire signed [15:0] theta_dot,  // 底座速度（补甩门控用，2026-10-09）
    input wire signed [15:0] alpha,      // 摆杆显示角（补甩相位门控用，2026-10-10）
    input wire signed [15:0] alpha_dot,  // 摆杆角速度（能量泵用，2026-10-11）
    input wire signed [15:0] delta,      // 甩摆距离 0.01°（运行时参数）
    input wire signed [15:0] catch_ang,  // 捕获窗（0.01°，自适应甩距升档门槛跟随它）
    input wire signed [15:0] coast_amp,   // 滑行补能幅度（id14，0=关；运行时）
    input wire signed [15:0] coast_margin,// 滑行补能余量（id15，kin×2<def×M；运行时）
    input wire signed [15:0] amplitude,  // duty 限幅（运行时参数）
    output reg signed [15:0] duty,
    output wire brake_req,      // 1 = 刹车相：请求板级短路制动（TB6612 IN1=IN2=1）
    input wire alpha_valid,
    input wire swing_active
);
    localparam signed [31:0] KP = 32'sd9;   // duty/cdeg
    reg signed [31:0] target;
    reg [15:0] rethrow_cnt;
    wire alpha_ok = !ALPHA_QUALITY_EN || alpha_valid;
    wire phase_ok = alpha_ok && ((RETHROW_ALPHA_MIN == 16'sd0) ||
                    (alpha > RETHROW_ALPHA_MIN) || (alpha < -RETHROW_ALPHA_MIN));
    // 摆速平静门（2026-10-21）：掉摆自动重甩入场时摆杆 α̇ 可达 ~900°/s，
    // 旧逻辑 start 脉冲立即锁 target=θ+Δ，臂伺服在摆杆大幅甩动中就开始追——
    // 用户实测"没静止就又起摆"，且运动中泵能不可控（穿窗速度 267-640°/s 的
    // 方差来源之一）。未平静时 target=θ（伺服零输出待命），由补甩门
    // （含本门）在 |α̇| 平静后 3s 节奏接管。0=旧行为不查摆速。
    // 入场（start）用瞬时判定（新鲜 START 摆杆下垂静止，无需等待）；
    // 补甩用连续 RETHROW_CALM_MS ms 平静——瞬时采样会被大幅摆动的折返点
    // （每周期两次 |α̇| 短暂 <门限）溜过；连续平静≈摆动真正衰减完，
    // 让每杆都回到准"第一杆"的确定性出发状态。
    wire adot_now = (RETHROW_ADOT_MAX == 16'sd0) ||
                    (alpha_dot > -RETHROW_ADOT_MAX && alpha_dot < RETHROW_ADOT_MAX);
    reg  [7:0] calm_ms;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)                      calm_ms <= 8'd0;
        else if (!swing_active || start) calm_ms <= 8'd0;
        else if (tick)
            calm_ms <= adot_now ? ((calm_ms == 8'd255) ? 8'd255 : calm_ms + 8'd1)
                                : 8'd0;
    end
    wire adot_calm = adot_now && (calm_ms >= RETHROW_CALM_MS);
    // ---------------- 自适应甩距（2026-10-21，ADAPTIVE_RETHROW_EN）----------------
    // 根因：同参数下杆与杆能量方差 ±40%（实测 apex 0.1-92° 散布，臂起始位置/
    // 摆杆静止取向微差所致），apex 极少恰好落进捕获上限带 → 40s 窗也会
    // 零捕获（firstfling 105° a2 实测）。修法：每杆记录摆杆最近点
    // (min|α|, 该点 |α̇|)，补甩瞬间按阶梯调整下一杆甩距——
    //   穿窗过热（|α̇|@最近点 >250°/s）→ ×0.75；
    //   apex 超出捕获窗 5° 以上才升档（×1.125/×1.25/×1.5）；限幅 [50°,150°]。
    // ★2026-10-23 22:27 门槛改跟随 catch_ang（原固定 3/15/40° 是 2.5° 窗时代的
    // 死数）：弱电池实测第一杆(118°)apex 17.6-18.0 本已在 20° 窗边，旧门槛
    // "apex>15° 就 ×1.25"把甩距推到 150°，apex 反而掉到 27-35°——甩得越远
    // 摆杆跟着臂走越久、能量被臂飞轮吃掉越多（150° 行程实测能量不升反降）。
    // 现在只有 apex > 窗+5° 才升档：窗 20° ⇒ apex>25° 才加，17-18° 的杆保持
    // 118° 重来一次直接被窗接走。满电时 apex 0-7° 本就不升档，行为不变。
    reg  signed [15:0] ad_delta;    // 当前自适应甩距（c01）
    reg  signed [15:0] apex_min;    // 本杆 min|α|（c01）
    reg  signed [15:0] apex_adot;   // 该点 α̇（0.1°/s，带号）
    wire signed [15:0] alpha_abs16 = alpha[15] ? -alpha : alpha;
    wire signed [15:0] apex_adot_abs = apex_adot[15] ? -apex_adot : apex_adot;
    wire hot_pass = apex_adot_abs > 16'sd2500;  // 最近点 |α̇|>250°/s = 穿窗而过
    // 升档门槛相对捕获窗（c01）：窗+5° 起小步 ×1.125，窗+25° ×1.25，窗+45° ×1.5
    wire [1:0] up_step = (apex_min > catch_ang + 16'sd4500) ? 2'd3 :
                         (apex_min > catch_ang + 16'sd2500) ? 2'd2 :
                         (apex_min > catch_ang + 16'sd500)  ? 2'd1 : 2'd0;
    wire signed [31:0] ad_cur   = {{16{ad_delta[15]}}, ad_delta};
    wire signed [31:0] ad_up_v  = (up_step == 2'd3) ? ad_cur + (ad_cur >>> 1) :
                                  (up_step == 2'd2) ? ad_cur + (ad_cur >>> 2) :
                                  (up_step == 2'd1) ? ad_cur + (ad_cur >>> 3) : ad_cur;
    wire signed [31:0] ad_raw   = hot_pass ? (ad_cur - (ad_cur >>> 2)) : ad_up_v;
    wire signed [31:0] ad_next  = (ad_raw > 32'sd15000) ? 32'sd15000 :
                                  (ad_raw <  32'sd5000) ? 32'sd5000 : ad_raw;
    wire signed [31:0] delta_eff = ADAPTIVE_RETHROW_EN ? ad_cur :
                                                   {{16{delta[15]}}, delta};
    wire signed [31:0] delta_eff_next = ADAPTIVE_RETHROW_EN ? ad_next :
                                                       {{16{delta[15]}}, delta};
    reg apex_clear;   // start/补甩放行拍置 1（单拍脉冲），清除 apex 跟踪
    // stage2 微冲的计数寄存器与复位脉冲（声明放早：apex 跟踪块要用 trim_clear）
    reg [9:0] trim_ms;     // 本次微冲已持续
    reg [3:0] trim_cnt;    // 本杆已补次数
    reg [9:0] trim_gap;    // 距上次微冲
    reg       trim_clear;  // 微冲结束的 apex 复位脉冲（与主路径 apex_clear 分开，
                           // 否则同一 reg 被两个 always 块驱动 → 综合报多驱动）
    // 补甩授权（2026-10-22 抽出成 wire）：旧路径直接用它锁 target，三相位路径
    // 用它触发下一杆 ACCEL。抽出来后两条路径共用同一套门，避免两处判据漂移。
    wire rethrow_gate = (theta_dot > -RETHROW_TD_MAX) && (theta_dot < RETHROW_TD_MAX) &&
                        phase_ok && adot_calm;
    wire rethrow_due  = (RETHROW_MS != 0) && (rethrow_cnt >= RETHROW_MS[15:0] - 1);
    wire rethrow_fire = rethrow_due && rethrow_gate;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            target <= 32'sd0; rethrow_cnt <= 16'd0;
            ad_delta <= 16'sd0; apex_clear <= 1'b0;
        end else begin
            apex_clear <= 1'b0;
            if (tick && start) begin
                // 首杆用运行时参数 delta，不读自适应寄存器：ad_delta 复位值是 0，
                // 旧写法 target=θ+ad_cur 会让上电后第一杆变成"0° 甩距"——实板
                // 130025 实测 START 后整整 3s 臂一动不动（duty 恒 0），靠 3s 补甩
                // 才真正起摆。ad_delta 仍在同一拍写 delta，供后续补甩递推。
                target <= adot_now ? (theta + delta) : theta;
                rethrow_cnt <= 16'd0;
                ad_delta <= delta;
                apex_clear <= 1'b1;
            end else if (tick && RETHROW_MS != 0) begin
                if (rethrow_cnt >= RETHROW_MS[15:0]-1) begin
                    rethrow_cnt <= 16'd0;
                    // 底座慢下来才补甩：狂飞中加力只会叠速度（见 RETHROW_TD_MAX）；
                    // 摆杆垂在下半区才补甩：临近窗口的 pass 不被盲甩搅黄；
                    // 摆速连续平静才补甩：运动中泵能不可控（见 RETHROW_ADOT_MAX）
                    if (rethrow_gate) begin
                        target <= theta + delta_eff_next;
                        ad_delta <= delta_eff_next[15:0];
                        apex_clear <= 1'b1;
                    end
                end else begin
                    rethrow_cnt <= rethrow_cnt + 16'd1;
                end
            end
        end
    end
    // 摆杆最近点跟踪（独立块——不能挂在 target 块的 else 上，否则被
    // RETHROW_MS≠0 的每拍分支遮蔽）：记录本杆 (min|α|, 该点 α̇)，
    // 平静门保证补甩时摆杆已静止，apex_min 即上一杆全部摆动的最高点。
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            apex_min <= 16'sd32767; apex_adot <= 16'sd0;
        end else if (apex_clear || trim_clear) begin
            apex_min <= 16'sd32767; apex_adot <= 16'sd0;
        end else if (tick && swing_active) begin
            if (alpha_abs16 < apex_min) begin
                apex_min <= alpha_abs16;
                apex_adot <= alpha_dot;
            end
        end
    end

    // ---------------- 三相位甩摆（FLING_BRAKE_EN=1，2026-10-22）----------------
    // 实板依据（130025 / 135008 / 135847 三轮逐帧核算）：
    //   · 一次甩击真正把摆杆抛过顶的是「刹车相」。130025 t=6020ms 那杆：满舵
    //     加速 111°/260ms → 反向满舵刹车 220ms → 摆杆从 120° 一路冲过顶、留
    //     433°/s 余速，0.74s 就被捕获窗接走。
    //   · 坏在刹车之后：duty=KP*(target−θ) 的比例伺服在臂冲过 target 后立刻
    //     反打满舵，形成 ±100°/≈1Hz 的等幅振荡（无明显阻尼）。这个振荡相对
    //     摆杆转动的相位是随机的，于是随机地对摆杆做功 —— 130025 里它泄能
    //     （α̇_top 1150→350°/s，6.7s 后捕获），135008 里它泵能（24s 内
    //     α̇_top 恒 >800°/s、摆杆从未进入 15° 窗，一次都捕获不到）。
    //     这就是"有时一杆就起、有时 40s 起不来"的真正来源，与捕获门无关。
    // 改法：把一杆切成三相位，刹车后不再回追。
    //   ACCEL：满舵朝 dir 方向加速，行程到 f_delta 结束（f_delta 首杆取运行时
    //          参数 delta，补甩取自适应值）；
    //   BRAKE：满舵反 θ̇ 刹车，|θ̇| 落进 VSTOP 或超时结束；
    //   HOLD ：阻尼位置环 KP*(hold−θ)−KV*θ̇，臂真正停住 ⇒ 摆杆能量只由摩擦
    //          单调衰减，捕获时刻可预测，且不再有无规律泵能。
    // dir 取进杆瞬间的 sign(α̇)：摆杆在底部朝哪边摆，就往哪边甩，保证这一杆
    // 是加能而不是反向抵能（滑环允许任意方向，无缠线约束）。
    localparam [2:0] F_IDLE = 3'd0, F_ACCEL = 3'd1, F_BRAKE = 3'd2, F_HOLD = 3'd3,
                     F_TRIM = 3'd4, F_COAST = 3'd5;
    reg [2:0]         fph;
    reg               f_dir;        // 1=+θ 方向甩
    reg signed [31:0] f_theta0;     // 本杆 ACCEL 起点
    reg signed [31:0] f_hold;       // HOLD 目标
    reg signed [31:0] f_delta;      // 本杆甩距
    reg [9:0]         f_brake_ms;
    reg [9:0]         f_accel_ms;
    reg [9:0]         f_brake_tgt;   // 本杆刹车时长（由峰值臂速算出）
    reg signed [15:0] f_peak;        // ACCEL 期间 |θ̇| 峰值
    reg [5:0]         f_slow_cnt;    // |θ̇|<VSTOP 的连续拍数
    reg               f_below;       // ACCEL 起点摆杆在下半区（α 卸能判据的使能）
    reg signed [31:0] f_theta_prev;
    reg [6:0]         f_still_cnt;   // θ 连续不变的拍数
    wire signed [31:0] delta_rt = {{16{delta[15]}}, delta};
    wire signed [31:0] f_travel = f_dir ? (theta - f_theta0) : (f_theta0 - theta);
    wire signed [15:0] alpha_abs16b = alpha[15] ? -alpha : alpha;
    // 加速结束：行程到位 / 摆杆过水平（卸能边界）/ 时间兜底。
    // α 判据只在"起杆时摆杆确实在下半区"才使能：若进杆时摆杆已在水平以上
    // （近顶慢 pass），按 α 判据会立刻结束加速——那是捕获场景，不该当起摆处理，
    // 限时兜底仍会把它走完。
    wire f_alpha_end = (FLING_ALPHA_END != 16'sd0) && f_below &&
                       (f_accel_ms >= FLING_ALPHA_MIN_MS) &&
                       (alpha_abs16b < FLING_ALPHA_END);
    wire f_accel_done = (f_travel >= f_delta) || f_alpha_end || (f_accel_ms >= FLING_ACCEL_MS);
    wire signed [15:0] tdot_abs = theta_dot[15] ? -theta_dot : theta_dot;
    // 保持相阻尼用的臂速：EMA(>>>3, τ≈8ms) 压 MT6701 测速的 931 台阶量化噪声
    reg signed [15:0] f_tdot_f;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) f_tdot_f <= 16'sd0;
        else if (tick) f_tdot_f <= f_tdot_f + ((theta_dot - f_tdot_f) >>> 3);
    end
    wire signed [15:0] tdot_abs_f = f_tdot_f[15] ? -f_tdot_f : f_tdot_f;
    // 峰值臂速 [0.1°/s] >> SH = ms；上下限兜住低压/误测。EXTRA 是可调的加能量口。
    wire [9:0] f_peak_ms    = f_peak[15:0] >> FLING_BRAKE_SH;
    wire [10:0] f_peak_ext  = {1'b0, f_peak_ms} + {1'b0, FLING_BRAKE_EXTRA_MS};
    wire [9:0] f_brake_calc = (f_peak_ext > {1'b0, FLING_BRAKE_MS})     ? FLING_BRAKE_MS :
                              (f_peak_ext < {1'b0, FLING_BRAKE_MIN_MS}) ? FLING_BRAKE_MIN_MS :
                              f_peak_ext[9:0];
    // 提前撤刹车：真停住（连续 VSTOP_CNT 拍都慢）才放行，θ 掉线骗不过去
    // 刹车结束的**摆杆速度门**（FLING_VEND_EN，0=关闭，行为与改动前逐位一致）：
    // 摆杆越过水平后，等刹车把它减速到 |α̇| ≤ FLING_VEND_V 就撒手。
    // 依据（6 个已标定日志，刹车释放瞬间 |α̇| 与最终过顶速度单调对应）：
    //   535→133°/s、631→202°/s（稳 30s）、740→344、815→280、827→420，
    // 而原先"按时间刹"的释放速度在 535~841 之间漂 ⇒ 落点必然散。取 600 复现
    // 唯一稳定那次的释放条件。只用 α/α̇（ADC），没有 MT6701 电流注入的假尖峰。
    // 对照：先试过用总能量 E 当门限，但 E 在越过水平那一刻已超目标，门在
    // 跨水平时立刻触发、根本没削能量（154046 实测过顶 420°/s）——能量在该
    // 时刻不是单调量，速度才是。
    wire f_vend_ok = (FLING_VEND_EN != 0) &&
                     (f_brake_ms >= FLING_VEND_MIN_MS) &&
                     (alpha_abs16b < 16'sd9000) &&      // 只在上半区认（爬升段）
                     (adot_abs16 <= FLING_VEND_V);
    // 提前撤刹车：真停住（连续 VSTOP_CNT 拍都慢）才放行，θ 掉线骗不过去
    wire f_brake_done = (f_brake_ms >= f_brake_tgt) || f_vend_ok ||
                        ((FLING_VSTOP_CNT != 6'd0) && (f_slow_cnt >= FLING_VSTOP_CNT));
    // ---- stage2 微冲的门（放在相位机之前声明，避免前向引用）----
    wire signed [15:0] adot_abs16 = alpha_dot[15] ? -alpha_dot : alpha_dot;
    // ---- 相位判据（FLING_PHASE_BRAKE）----
    wire f_upper   = (alpha_abs16b < FLING_ALPHA_END);           // |α|<90°：上半区
    wire f_rising  = f_dir ? (alpha_dot >  FLING_BRAKE_FLIP)
                           : (alpha_dot < -FLING_BRAKE_FLIP);    // 摆杆还在朝"上"走
    wire f_falling = f_dir ? (alpha_dot < -FLING_BRAKE_FLIP)
                           : (alpha_dot >  FLING_BRAKE_FLIP);
    wire f_top     = (FLING_BRAKE_STOP != 16'sd0) &&
                     (alpha_abs16b < FLING_BRAKE_STOP);          // 进捕获窗门口
    // 相位+能量门补甩
    reg [9:0] rph_ms;   // 距上次起杆（ms）
    wire rethrow_phase_c = RETHROW_PHASE_EN && (alpha_abs16b >= RETHROW_PHASE_ALPHA) &&
                           (adot_abs16 > RETHROW_PHASE_ADOT) &&
                           (adot_abs16 < RETHROW_PHASE_V) &&
                           (rph_ms >= RETHROW_PHASE_GAP);
    // 判据寄存一拍再进状态机（2026-10-24 时序教训：EN=0 时常量折叠、一启用
    // 组合比较链直接进 fph/duty 选择 ⇒ Fmax 47.9-48.6 三连败；1ms 延迟无害）。
    reg rethrow_phase;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)             rethrow_phase <= 1'b0;
        else if (tick)          rethrow_phase <= rethrow_phase_c;
    end
    wire trim_need  = (apex_min > TRIM_APEX_MAX);
    wire trim_phase = (alpha_abs16b > TRIM_ALPHA) && (adot_abs16 > TRIM_ADOT_MIN);
    wire trim_fire  = TRIM_EN && (trim_cnt < TRIM_MAX) && trim_need && trim_phase &&
                      (trim_gap >= TRIM_GAP_MS);
    // ---- 泵→甩摆交棒（BPUMP_HYBRID_EN）----
    reg [11:0] hyb_ms;      // 距上次交棒
    reg       hyb_fling;   // 1 = 正在打这一杆甩摆
    wire hyb_fire = BPUMP_HYBRID_EN && (PUMP_MODE == 4) && !hyb_fling &&
                    (apex_min < BPUMP_APEX_TRIG) &&                 // apex 已达标
                    (alpha_abs16b >= FLING_ALPHA_END) &&            // 摆杆在底部
                    (adot_abs16 > 16'sd1500) &&                     // 有明确方向
                    (hyb_ms >= BPUMP_HYBRID_GAP);
    reg signed [15:0] trim_a0;    // 微冲起点角
    wire signed [16:0] trim_da = $signed({alpha[15], alpha}) - $signed({trim_a0[15], trim_a0});
    wire signed [16:0] trim_da_w = (trim_da > 17'sd18000)  ? trim_da - 17'sd36000 :
                                   (trim_da <= -17'sd18000) ? trim_da + 17'sd36000 : trim_da;
    wire [16:0] trim_da_abs = trim_da_w[16] ? -trim_da_w : trim_da_w;
    wire trim_done = (trim_da_abs >= {5'd0, TRIM_DANGLE}) || (trim_ms >= TRIM_MS);
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fph <= F_IDLE; f_dir <= 1'b1; f_theta0 <= 32'sd0; f_hold <= 32'sd0;
            f_delta <= 32'sd0; f_brake_ms <= 10'd0; f_accel_ms <= 10'd0;
            f_brake_tgt <= 10'd0;
            f_peak <= 16'sd0; f_slow_cnt <= 6'd0; f_below <= 1'b0;
            f_theta_prev <= 32'sd0; f_still_cnt <= 7'd0;
            trim_ms <= 10'd0; trim_cnt <= 4'd0; trim_gap <= 10'd0;
            rph_ms <= 10'd0; hyb_ms <= 10'd0; hyb_fling <= 1'b0;
        end else if (!swing_active) begin
            fph <= F_IDLE; f_brake_ms <= 10'd0; f_accel_ms <= 10'd0;
            f_slow_cnt <= 6'd0; f_still_cnt <= 7'd0;
            trim_ms <= 10'd0; trim_cnt <= 4'd0; trim_gap <= 10'd0;
            rph_ms <= 10'd0; hyb_ms <= 10'd0; hyb_fling <= 1'b0;
        end else if (tick) begin
            // θ 停滞看门狗（与相位无关，所有相位共用）
            if (theta == f_theta_prev)
                f_still_cnt <= (f_still_cnt == 7'd127) ? 7'd127 : f_still_cnt + 7'd1;
            else
                f_still_cnt <= 7'd0;
            f_theta_prev <= theta;
            trim_clear <= 1'b0;
            if (trim_gap != 10'd1023) trim_gap <= trim_gap + 10'd1;
            if (rph_ms != 10'd1023)   rph_ms   <= rph_ms + 10'd1;
            if (hyb_ms != 12'd1023)   hyb_ms   <= hyb_ms + 12'd1;
            case (fph)
            F_IDLE: if (start) begin
                // 入场摆杆不平静时只能保持：运动中泵能不可控（同 adot_now 语义）
                f_dir    <= (alpha_dot < 0) ? 1'b0 : 1'b1;
                f_theta0 <= theta;
                f_delta  <= delta_rt;
                f_hold   <= theta;
                f_accel_ms <= 10'd0;
                f_peak   <= 16'sd0;
                f_below  <= (alpha_abs16b >= FLING_ALPHA_END);
                trim_cnt <= 4'd0; trim_gap <= 10'd0;
                fph      <= adot_now ? F_ACCEL : F_HOLD;
            end
            F_ACCEL: begin
                f_accel_ms <= f_accel_ms + 10'd1;
                if (tdot_abs > f_peak) f_peak <= tdot_abs;
                if (f_accel_done) begin
                    if (FLING_PHASE_BRAKE) begin
                        // 新：加速相结束先进滑行相，等摆杆自己到顶掉头再反向拉回
                        fph <= F_COAST; f_brake_ms <= 10'd0; f_slow_cnt <= 6'd0;
                    end else begin
                        fph <= F_BRAKE; f_brake_ms <= 10'd0;
                        f_brake_tgt <= f_brake_calc; f_slow_cnt <= 6'd0;
                    end
                end
            end
            F_COAST: begin
                // 上半区什么都不做（此时反向拉回是卸能）；等摆杆掉头落到水平以下
                // 再拉回，那一半才是反向加速做正功的相位。
                if (f_top) begin
                    fph <= F_HOLD; f_hold <= theta;      // 已到捕获窗门口，撒手
                end else if (!f_upper && f_falling) begin
                    fph <= F_BRAKE; f_brake_ms <= 10'd0; f_slow_cnt <= 6'd0;
                end else if (f_brake_ms >= FLING_COAST_MS) begin
                    fph <= F_HOLD; f_hold <= theta;      // 兜底：滑行超时
                end else
                    f_brake_ms <= f_brake_ms + 10'd1;
            end
            F_BRAKE: begin
                f_slow_cnt <= (tdot_abs_f < FLING_VSTOP) ?
                              ((f_slow_cnt == 6'd63) ? 6'd63 : f_slow_cnt + 6'd1) : 6'd0;
                if (FLING_PHASE_BRAKE) begin
                    // 新：摆杆重新升到水平以上（这一半反向拉回已不再做正功）或
                    // 已进捕获窗门口就收手；纯相位判据，不用 θ̇ 判停。
                    if (f_top || (f_upper && f_rising) || (f_brake_ms >= FLING_BRAKE_MS)) begin
                        fph <= F_HOLD; f_hold <= theta;
                    end else
                        f_brake_ms <= f_brake_ms + 10'd1;
                end else if (f_brake_done) begin
                    fph <= F_HOLD; f_hold <= theta;
                end else
                    f_brake_ms <= f_brake_ms + 10'd1;
            end
            F_HOLD: if (hyb_fling) begin
                // 这一杆甩摆打完，交回泵继续评估（泵在 duty 多路器里接管）
                hyb_fling <= 1'b0;
            end else if (hyb_fire) begin
                // 泵已把摆杆推到 apex≈85°（泵的天花板），交棒给甩摆顶最后一程
                f_dir    <= (alpha_dot < 0) ? 1'b0 : 1'b1;
                f_theta0 <= theta;
                f_delta  <= delta_rt;
                f_peak   <= 16'sd0;
                f_accel_ms <= 10'd0;
                f_below  <= (alpha_abs16b >= FLING_ALPHA_END);
                hyb_fling <= 1'b1;
                hyb_ms    <= 10'd0;
                trim_clear <= 1'b1;      // 重新跟踪 apex
                fph      <= F_ACCEL;
            end else if (rethrow_fire || rethrow_phase) begin
                f_dir    <= (alpha_dot < 0) ? 1'b0 : 1'b1;
                f_theta0 <= theta;
                f_delta  <= ad_next;
                f_peak   <= 16'sd0;
                f_accel_ms <= 10'd0;
                f_below  <= (alpha_abs16b >= FLING_ALPHA_END);
                rph_ms   <= 10'd0;
                fph      <= F_ACCEL;
            end else if (trim_fire) begin
                // stage2 微冲：方向取摆杆此刻的摆动方向（泵能同向）
                f_dir    <= (alpha_dot < 0) ? 1'b0 : 1'b1;
                trim_ms  <= 10'd0;
                trim_a0  <= alpha;
                fph      <= F_TRIM;
            end
            F_TRIM: begin
                trim_ms <= trim_ms + 10'd1;
                if (trim_done) begin
                    fph        <= F_HOLD;
                    trim_cnt   <= trim_cnt + 4'd1;
                    trim_gap   <= 10'd0;
                    trim_clear <= 1'b1;      // 重新跟踪 apex，判断这一记够不够
                end
            end
            default: fph <= F_IDLE;
            endcase
            // 新一轮 SWING（含掉摆自动重甩）无条件复位 stage2 计数：start 可能
            // 落在 F_HOLD（初判不平静直接保持）而不经过 F_IDLE。
            if (start) begin trim_cnt <= 4'd0; trim_gap <= 10'd0; rph_ms <= 10'd0; end
        end
    end
    wire signed [15:0] f_amp = $signed({1'b0, amplitude});
    wire signed [15:0] duty_accel = f_dir ? f_amp : -f_amp;
    // 刹车相：恢复"反向拉回"（2026-10-22 用户定案：先正向加速、再反向移动），
    // 但**不再满幅硬切**——下面 duty_phase_s 的斜率限幅把 +amp→−amp 铺成
    // 2×amp/STEP 毫秒的斜坡，换向次数由 rev_guard 驻留窗封顶。
    // （上一版曾改成"只短路制动、零反压"，实测鞭甩不够：apex 只到 84.4°、
    //  14s 零捕获。反向拉回才够，问题只在换向速度。）
    wire signed [15:0] duty_brake = f_dir ? -f_amp : f_amp;
    // brake_req 见文件下方（泵/甩摆两路合并后统一驱动）
    wire signed [31:0] hold_err  = f_hold - theta;
    wire signed [31:0] hold_p    = hold_err * $signed(FLING_HOLD_KP);
    wire signed [31:0] hold_d    = $signed({{16{f_tdot_f[15]}}, f_tdot_f}) *
                                   $signed(FLING_HOLD_KV);
    wire signed [31:0] hold_raw  = hold_p - hold_d;
    wire signed [15:0] hold_max_p = FLING_HOLD_MAX;
    wire signed [15:0] hold_max_n = -FLING_HOLD_MAX;
    wire signed [15:0] hold_lim  =
        (hold_raw > $signed(hold_max_p)) ? hold_max_p :
        (hold_raw < $signed(hold_max_n)) ? hold_max_n : hold_raw[15:0];
    wire signed [15:0] hold_out  =
        ((hold_lim <= FLING_HOLD_DZ) && (hold_lim >= -FLING_HOLD_DZ)) ? 16'sd0 :
        (f_still_cnt >= FLING_STILL_MS) ? 16'sd0 : hold_lim;
    wire signed [15:0] duty_phase = (fph == F_ACCEL) ? duty_accel :
                                    (fph == F_BRAKE) ? duty_brake :
                                    (fph == F_TRIM)  ? duty_accel :
                                    (fph == F_HOLD)  ?
                                        (dose_run ? dose_out :
                                         (FLING_HOLD_BRAKE ? 16'sd0 : hold_out)) : 16'sd0;
    // 斜率限幅（FLING_SLEW_STEP>0）：任何满幅换向至少 2×amp/STEP 毫秒。
    // 这正是"先正向加速、再反向移动"能安全落地的前提——策略不变，只把硬切
    // 铺成斜坡。0=旁路（旧行为逐位一致，tb_control_path/tb_swingup_fling 依赖）。
    reg signed [15:0] f_slew;
    wire signed [15:0] slew_step = $signed(FLING_SLEW_STEP);
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)            f_slew <= 16'sd0;
        else if (!swing_active) f_slew <= 16'sd0;
        else if (tick) begin
            if (FLING_SLEW_STEP == 16'd0)
                f_slew <= duty_phase;
            else if (duty_phase > f_slew + slew_step)
                f_slew <= f_slew + slew_step;
            else if (duty_phase < f_slew - slew_step)
                f_slew <= f_slew - slew_step;
            else
                f_slew <= duty_phase;
        end
    end
    wire signed [15:0] duty_phase_s = f_slew;

    // 双向能量泵（PUMP_MODE=4）实体在 oneminuscos ROM 之后定义（要用到查表）


    // 用户硬约束：齿轮箱不允许快速换向。本模式把"不换向"做成结构性质而非整定值：
    //   ① duty 只可能取 0 或 +amplitude —— 永不为负，H 桥极性从不翻转；
    //   ② 制动相只请求 TB6612 短路制动（IN1=IN2=1，反电动势短接），不加反压；
    //   ③ 每次出力后强制静默 PUMP_REST_MS，节奏上限 = 1/(PULSE+REST) ≈ 3/s。
    // 泵能律：臂对摆杆做功的功率 P = −m·l·a·α̇·cosα（a=臂切线加速度 ∝ duty，
    // α 从竖直向上算）。a>0 只在 α̇·cosα<0 的半周做正功；a<0（短路制动的纯
    // 阻尼）在相反半周做正功。于是本模式按相位分工：
    //   上升半周（|α|>90° 且 α̇>0）→ 满舵加速；
    //   下降半周（|α|>90° 且 α̇<0）→ 短路制动，把臂速在"制动也做正功"的半周里
    //                                泄掉；
    //   |α|<90°（水平以上）        → 两相都不出力（继续同向出力是卸能）。
    // 这正是短路制动版甩摆失败的原因与修法：原来 ACCEL 后**立刻**刹车，刹车落在
    // 同一个半周，把刚加的能量原样卸回去（实板 apex 只到 84.4°）。相位泵把制动
    // 搬到相反半周，两次都做正功，每周期净加能。
    // 符号依据（实板 20261022_212422 SWING 段 7023 样本最小二乘）：
    //   α̈ = +0.1176·sinα − 1.37e-5·duty·cosα
    // b1>0 符合倒立摆不稳定极点（≈20.5 rad/s² ⇒ l≈0.48m，与机构吻合）；b2<0
    // ⇒ 正 duty 在 α̇·cosα<0 时做正功。同轮满舵段 α:180°→84° 全程 α̇>0、|α|>90°，
    // 正是该半周，故下面 gate_a 取 (|α|>90° 且 α̇>+门限)。
    reg signed [15:0] p_adot_f;              // α̇ 的 EMA（压 4ms 差分噪声）
    reg               p_drive;               // 1=正在出力
    reg [9:0]         p_ms;                  // 本次出力已持续 ms
    reg [9:0]         p_since_ms;            // 距上次出力结束 ms（饱和）
    reg [9:0]         p_quiet_ms;            // 摆杆+臂静止累计 ms
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) p_adot_f <= 16'sd0;
        else if (tick) p_adot_f <= p_adot_f + ((alpha_dot - p_adot_f) >>> 3);
    end
    wire signed [15:0] p_adot_abs = p_adot_f[15] ? -p_adot_f : p_adot_f;
    wire p_lower      = (alpha_abs16b >= PUMP_GATE_ALPHA);
    wire p_gate_a     = p_lower && (p_adot_f >  PUMP_PHASE_MIN);   // 加速半周
    wire p_gate_b     = p_lower && (p_adot_f < -PUMP_PHASE_MIN);   // 制动半周
    wire p_arm_moving = (tdot_abs_f > PUMP_TD_STOP);
    wire p_arm_over   = (tdot_abs_f > PUMP_TD_CAP);
    wire p_over_e     = (p_adot_abs > PUMP_ADOT_CAP);              // 已能过顶
    wire p_cool_ok    = (p_since_ms >= PUMP_REST_MS);
    // 悬垂静止 ⇒ 没有相位可用，靠一记起振脉冲打破僵局（否则永不起摆）。
    // 静止累计门防止"推一下→回落→再推"变成慢速抖振。
    wire p_quiet      = !p_arm_moving && (p_adot_abs < PUMP_KICK_ADOT) &&
                        (alpha_abs16b > PUMP_KICK_ALPHA);
    wire p_kick       = p_quiet && (p_quiet_ms >= PUMP_KICK_HOLD_MS);
    wire p_drive_req  = (p_gate_a || p_kick) && p_cool_ok && !p_arm_over && !p_over_e;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            p_drive <= 1'b0; p_ms <= 10'd0; p_since_ms <= 10'd1023;
            p_quiet_ms <= 10'd0;
        end else if (!swing_active) begin
            p_drive <= 1'b0; p_ms <= 10'd0; p_since_ms <= 10'd1023;
            p_quiet_ms <= 10'd0;
        end else if (tick) begin
            if (p_since_ms != 10'd1023)
                p_since_ms <= p_since_ms + 10'd1;
            if (p_quiet) begin
                if (p_quiet_ms != 10'd1023)
                    p_quiet_ms <= p_quiet_ms + 10'd1;
            end else
                p_quiet_ms <= 10'd0;
            if (p_drive) begin
                p_ms <= p_ms + 10'd1;
                // 退出：时长到顶 / 越过水平（硬边界）/ 相位翻面（且已过下限）/
                // 臂速超顶。相位翻面与过水平都必须立刻退出——留着就在卸能。
                if ((p_ms + 10'd1 >= PUMP_PULSE_MS) || p_arm_over || !p_lower ||
                    (!p_gate_a && (p_ms + 10'd1 >= PUMP_PULSE_MIN_MS))) begin
                    p_drive <= 1'b0; p_ms <= 10'd0; p_since_ms <= 10'd0;
                end
            end else if (p_drive_req) begin
                p_drive <= 1'b1; p_ms <= 10'd1; p_since_ms <= 10'd0;
                p_quiet_ms <= 10'd0;
            end
        end
    end
    // 出力恒为正（幅度=运行时 amplitude，与甩摆同一限幅口）
    wire signed [15:0] duty_ratchet = p_drive ? $signed({1'b0, amplitude}) : 16'sd0;
    // 制动：只在"制动也做正功"的半周（或臂速超顶必须泄速时）短路制动
    wire p_brake = !p_drive && p_arm_moving && (p_gate_b || p_arm_over);
    // 两路制动请求合并：
    //  · 甩摆 F_BRAKE 相**不再请求短路制动** —— 这一相就是"反向拉回"，靠 duty
    //    斜坡出力（短路制动会盖掉驱动，鞭甩就没了，实测 apex 只到 84.4°）。
    //  · FLING_HOLD_BRAKE=1 时 HOLD 相只短路制动、不出力（刹车后臂已近停，
    //    位置环再出力只会制造低速小幅反复换向）。
    //  · 相位泵（PUMP_MODE=3）走 p_brake。
    assign brake_req = (PUMP_MODE == 3) ? p_brake :
                       (PUMP_MODE == 4) ? (hyb_fling && FLING_HOLD_BRAKE &&
                                           (fph == F_HOLD) && !dose_run) :
                       (FLING_HOLD_BRAKE && (fph == F_HOLD) && !dose_run);

    // ---------------- 能量泵（PUMP_MODE=1）----------------
    // 时序纪律：乘法链全部寄存器化（tick 域两拍流水），1ms 拍算法多 1-2ms
    // 延迟无感（ Energies 时间尺度 600ms）。不加流水 36.5MHz 不可布线。
    // 流水级 0：输入寄存（alpha/alpha_dot/theta_dot 采样 + 静态求值）。
    // 符号约定严格照抄 pump_trace.m 数值验证版：
    //   signpump = (av·cosα > 0) ? -1 : +1（av=0 取 +1）
    //   u = sat(k_E·(E − E_target)·signpump, amp)
    // 1-cosα 查表：0.01°→1° 索引（|α|×41>>>12 ≈ ÷100），90 项，
    // 8bit = round((1-cos)×255)。
    function [7:0] oneminuscos;
        input [6:0] idx;   // 0..89
        begin
            case (idx)
                7'd0:  oneminuscos = 8'd0;   7'd1:  oneminuscos = 8'd0;
                7'd2:  oneminuscos = 8'd0;   7'd3:  oneminuscos = 8'd0;
                7'd4:  oneminuscos = 8'd1;   7'd5:  oneminuscos = 8'd1;
                7'd6:  oneminuscos = 8'd1;   7'd7:  oneminuscos = 8'd2;
                7'd8:  oneminuscos = 8'd2;   7'd9:  oneminuscos = 8'd3;
                7'd10: oneminuscos = 8'd4;   7'd11: oneminuscos = 8'd5;
                7'd12: oneminuscos = 8'd6;   7'd13: oneminuscos = 8'd7;
                7'd14: oneminuscos = 8'd8;   7'd15: oneminuscos = 8'd9;
                7'd16: oneminuscos = 8'd10;  7'd17: oneminuscos = 8'd11;
                7'd18: oneminuscos = 8'd12;  7'd19: oneminuscos = 8'd14;
                7'd20: oneminuscos = 8'd15;  7'd21: oneminuscos = 8'd17;
                7'd22: oneminuscos = 8'd19;  7'd23: oneminuscos = 8'd20;
                7'd24: oneminuscos = 8'd22;  7'd25: oneminuscos = 8'd24;
                7'd26: oneminuscos = 8'd26;  7'd27: oneminuscos = 8'd28;
                7'd28: oneminuscos = 8'd30;  7'd29: oneminuscos = 8'd32;
                7'd30: oneminuscos = 8'd34;  7'd31: oneminuscos = 8'd36;
                7'd32: oneminuscos = 8'd39;  7'd33: oneminuscos = 8'd41;
                7'd34: oneminuscos = 8'd44;  7'd35: oneminuscos = 8'd46;
                7'd36: oneminuscos = 8'd49;  7'd37: oneminuscos = 8'd51;
                7'd38: oneminuscos = 8'd54;  7'd39: oneminuscos = 8'd57;
                7'd40: oneminuscos = 8'd60;  7'd41: oneminuscos = 8'd63;
                7'd42: oneminuscos = 8'd65;  7'd43: oneminuscos = 8'd69;
                7'd44: oneminuscos = 8'd72;  7'd45: oneminuscos = 8'd75;
                7'd46: oneminuscos = 8'd78;  7'd47: oneminuscos = 8'd81;
                7'd48: oneminuscos = 8'd84;  7'd49: oneminuscos = 8'd88;
                7'd50: oneminuscos = 8'd91;  7'd51: oneminuscos = 8'd95;
                7'd52: oneminuscos = 8'd98;  7'd53: oneminuscos = 8'd102;
                7'd54: oneminuscos = 8'd105; 7'd55: oneminuscos = 8'd109;
                7'd56: oneminuscos = 8'd112; 7'd57: oneminuscos = 8'd116;
                7'd58: oneminuscos = 8'd120; 7'd59: oneminuscos = 8'd124;
                7'd60: oneminuscos = 8'd127; 7'd61: oneminuscos = 8'd131;
                7'd62: oneminuscos = 8'd135; 7'd63: oneminuscos = 8'd139;
                7'd64: oneminuscos = 8'd143; 7'd65: oneminuscos = 8'd147;
                7'd66: oneminuscos = 8'd151; 7'd67: oneminuscos = 8'd155;
                7'd68: oneminuscos = 8'd159; 7'd69: oneminuscos = 8'd164;
                7'd70: oneminuscos = 8'd168; 7'd71: oneminuscos = 8'd172;
                7'd72: oneminuscos = 8'd176; 7'd73: oneminuscos = 8'd180;
                7'd74: oneminuscos = 8'd185; 7'd75: oneminuscos = 8'd189;
                7'd76: oneminuscos = 8'd193; 7'd77: oneminuscos = 8'd198;
                7'd78: oneminuscos = 8'd202; 7'd79: oneminuscos = 8'd206;
                7'd80: oneminuscos = 8'd211; 7'd81: oneminuscos = 8'd215;
                7'd82: oneminuscos = 8'd220; 7'd83: oneminuscos = 8'd224;
                7'd84: oneminuscos = 8'd228; 7'd85: oneminuscos = 8'd233;
                7'd86: oneminuscos = 8'd237; 7'd87: oneminuscos = 8'd242;
                7'd88: oneminuscos = 8'd246; 7'd89: oneminuscos = 8'd251;
                default: oneminuscos = 8'd251;
            endcase
        end
    endfunction
    // ---------------- 双向能量泵（PUMP_MODE=4，2026-10-23 用户定案）----------------
    // 用户原话："向左向右反复晃荡到合适的时机注入能量……还是会旋转，可以好好
    // 考虑刹车吗。" 所以这里不是"到点停推"，而是**总能量伺服 + 反推泄能**：
    //   E = ½α̇² + (g/l)(1+cosα)，(g/l)=58.7（实测周期 822ms ⇒ ω0=7.64）
    //   E < 目标 − 迟滞 → 按泵能方向推（加能）
    //   E > 目标 + 迟滞 → **反推**（同一相位律反向 = 做负功，把多余能量刹掉）
    //   带内 → 滑行，等捕获窗
    // 只用 |α̇| 判停的教训（实测）：泵在半高处收手后摆杆继续把势能变动能，
    // 底部冲到 1652°/s（过顶只需 877°/s）⇒ 直接转圈。必须用总能量判。
    // 单位 0.01(rad/s)²。目标：过顶要 2·(g/l)=117.4，取 120 ⇒ 过 15° 窗
    // |α̇|≈150°/s，正落在捕获动量带 100~400 中间。
    reg        bdir;     // 1 = +duty
    reg [9:0]  bdwell;   // 方向驻留计时
    wire       b_cos_pos = (alpha_abs16b < FLING_ALPHA_END);   // cosα>0 ⇔ |α|<90°
    wire       b_near90  = (alpha_abs16b > 16'sd8000) && (alpha_abs16b < 16'sd10000);
    wire       b_ok_dir  = (adot_abs16 > BPUMP_ADOT_MIN) && !b_near90;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)               begin bdir <= 1'b1; bdwell <= 10'd0; end
        else if (!swing_active)   begin bdir <= 1'b1; bdwell <= 10'd0; end
        else if (tick) begin
            if (bdwell != 10'd1023) bdwell <= bdwell + 10'd1;
            if (b_ok_dir && (bdwell >= BPUMP_DWELL_MS)) begin
                bdir   <= ((alpha_dot > 16'sd0) != b_cos_pos);
                bdwell <= 10'd0;
            end
        end
    end
    // ---- 能量估计（0.01(rad/s)²）：6 级流水（tick 拍，总延迟 6ms 无感）----
    // 组合链是 α→×41→ROM→×23 与 α̇→平方→×1277，串起来 Fmax 掉到 32.5MHz，
    // 所以每一级乘法/查表都单独打拍。能量比相位慢得多，6ms 滞后不影响决策。
    reg [15:0] e_absa1, e_va1;
    reg        e_up1;
    reg [16:0] e_xa2;
    reg [31:0] e_v22;
    reg [21:0] e_idxp3;
    reg [31:0] e_kp3;
    reg [7:0]  e_rom4;
    reg [9:0]  e_onepc4;
    reg [31:0] e_k015;
    reg [31:0] e_u015;
    reg [31:0] e_e01_q;
    reg        e_low_q, e_high_q;
    wire [7:0] e_rom_r = oneminuscos(e_rom4[6:0]);
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            e_absa1<=0; e_va1<=0; e_up1<=0; e_xa2<=0; e_v22<=0; e_idxp3<=0;
            e_kp3<=0; e_rom4<=0; e_onepc4<=0; e_k015<=0; e_u015<=0;
            e_e01_q<=0; e_low_q<=0; e_high_q<=0;
        end else if (tick) begin
            // 级1：输入
            e_absa1 <= alpha_abs16b;
            e_va1   <= (adot_abs16 > 16'sd16000) ? 16'sd16000 : adot_abs16;
            e_up1   <= (alpha_abs16b >= 16'sd9000);
            // 级2：索引前值 / 速度平方
            e_xa2   <= e_up1 ? (17'sd18000 - {1'b0, e_absa1}) : {1'b0, e_absa1};
            e_v22   <= e_va1 * e_va1;
            // 级3：≈÷100 与 K 定标乘法
            // ★ 必须先把 v² 右移 8 位再乘：v²(31b)×1277(11b)=42b，直接乘会溢出
            //   32 位寄存器 ⇒ 能量恒偏小 ⇒ 伺服永远加能、摆杆飞到 2263°/s。
            e_idxp3 <= e_xa2[15:0] * 13'd41;
            e_kp3   <= (e_v22 >> 8) * 32'd1277;
            // 级4：ROM 查表（索引寄存后一拍）+ 1+cos 映射
            e_rom4  <= (e_idxp3[20:12] > 13'd89) ? 8'd89 : e_idxp3[19:12];
            e_k015  <= e_kp3 >> 15;
            // 级5：U 定标
            e_onepc4 <= e_up1 ? {2'b00, e_rom_r} : (10'd510 - {2'b00, e_rom_r});
            e_u015   <= {22'd0, e_onepc4} * 10'd23;
            // 级6：总能量与判决
            e_e01_q  <= e_u015 + e_k015;
            e_low_q  <= ((e_u015 + e_k015) <  BPUMP_E_TARGET);
            e_high_q <= ((e_u015 + e_k015) > (BPUMP_E_TARGET + BPUMP_E_HYST));
        end
    end
    // ---- 滑行相微剂量能量伺服（2026-10-24；判据全部寄存，见 rethrow_phase 教训）----
    reg        dose_run;
    reg [1:0]  dose_cnt;
    reg [9:0]  dose_ms, dose_gap;
    reg        dose_dir;      // 1 = +duty
    // 给药带 [15°,35°]（2026-10-24 MATLAB 泄漏模型定标：泄漏≈4067±963 单位且按
    // 路径均摊 45/°；带中角越小剩余泄漏越小、cos 耦合越强——96° 处出手 96→17°
    // 途中被吃掉，25° 处出手只剩 ~1100 且 cos=0.91 是 96° 处的 9 倍效率）。
    wire dose_band_ok = (alpha_abs16b > 16'sd1500) && (alpha_abs16b < 16'sd3500);
    wire dose_inward  = (alpha[15] != alpha_dot[15]);   // sign(α)≠sign(α̇)=朝顶
    // ===== 触发器 v3（2026-10-24 17:50 逐拍破案）：泄漏感知阈值 =====
    // v1（总能量）与 v2（静态动能余量 kin×2<def×M）同病：都"以为能量够"——
    // 实测 500°/s 进给药带（静态账充足）仍在最后 32° 漏 3379 单位（≈113/°，
    // 远高于全程均值 45/°）停在 18°。v3 判据：当前 E < 11730 + 尾程泄漏，
    // 尾程泄漏 = |α|×M/8（M=coast_margin id15，语义改为泄漏系数，默认 9 ⇒
    // 1.125×|α|，按实测 1.12 拟合）。剂量停止同阈值（自限幅防过热）。
    // 阈值寄存一拍（Fmax 教训：乘法进判据链 = 三连败）。
    reg [31:0] dose_thr;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)    dose_thr <= 32'd11730;
        else if (tick) dose_thr <= 32'd11730 +
                             (({32'd0, alpha_abs16b} * $unsigned(coast_margin)) >>> 3);
    end
    wire dose_need_e = (e_e01_q < dose_thr);
    wire dose_start_c = COAST_DOSE_EN && (coast_amp != 16'sd0) &&
                        (fph == F_HOLD) && !dose_run &&
                        (dose_cnt < COAST_MAX) && (dose_gap >= COAST_GAP_MS) &&
                        dose_band_ok && dose_inward && dose_need_e;
    reg  dose_start;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)     dose_start <= 1'b0;
        else if (tick)  dose_start <= dose_start_c;
    end
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dose_run <= 1'b0; dose_cnt <= 2'd0; dose_ms <= 10'd0; dose_gap <= 10'd0;
            dose_dir <= 1'b0;
        end else if (!swing_active) begin
            dose_run <= 1'b0; dose_cnt <= 2'd0; dose_ms <= 10'd0; dose_gap <= 10'd0;
        end else if (tick) begin
            if (fph == F_ACCEL || fph == F_BRAKE) begin
                // 新一杆（start/rethrow/hyb 都先经过这两相）：清本杆剂量计数
                dose_run <= 1'b0; dose_ms <= 10'd0; dose_cnt <= 2'd0;
                if (dose_gap != 10'd1023) dose_gap <= dose_gap + 10'd1;
            end else if (dose_start) begin
                dose_run <= 1'b1; dose_ms <= 10'd0; dose_gap <= 10'd0;
                // 正功方向 = duty 与 α̇·cosα 异号；cos>0 ⇔ |α|<90°
                dose_dir <= ((alpha_dot > 16'sd0) != (alpha_abs16b < 16'sd9000));
            end else if (dose_run) begin
                if ((dose_ms >= COAST_DOSE_MS) || !dose_need_e ||
                    !dose_band_ok || !dose_inward) begin
                    dose_run <= 1'b0; dose_cnt <= dose_cnt + 2'd1; dose_gap <= 10'd0;
                end else
                    dose_ms <= dose_ms + 10'd1;
            end else begin
                if (dose_gap != 10'd1023) dose_gap <= dose_gap + 10'd1;
            end
        end
    end
    wire signed [15:0] coast_amp_s = coast_amp;
    wire signed [15:0] dose_out = dose_run ? (dose_dir ? coast_amp_s : -coast_amp_s)
                                           : 16'sd0;
    wire b_low  = e_low_q;
    wire b_high = e_high_q;
    // 起摆第一推：悬垂静止时没有相位可用，给一记固定正冲把它晃起来
    wire b_kick = (alpha_abs16b > 16'sd15000) && (adot_abs16 <= BPUMP_ADOT_MIN) &&
                  (tdot_abs_f < 16'sd800);
    wire signed [15:0] bpump_amp = BPUMP_AMP;
    // 当前相位要求的方向 vs 已锁存方向：不一致就不出力（律恒成立 + 防抖）
    wire b_dir_now = ((alpha_dot > 16'sd0) != b_cos_pos);
    wire signed [15:0] duty_bpump = b_kick ? bpump_amp :
                                    (!b_ok_dir) ? 16'sd0 :
                                    (b_dir_now != bdir) ? 16'sd0 :
                                    b_low  ? (bdir ? bpump_amp : -bpump_amp) :
                                    b_high ? (bdir ? -bpump_amp : bpump_amp) :
                                             16'sd0;

    // 级 0 输入寄存：tick 拍采样（泵是 1ms 拍算法）
    reg signed [15:0] p0_alpha, p0_adot, p0_tdot;
    reg        p0_tick, p0_valid;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            p0_alpha <= 16'sd0; p0_adot <= 16'sd0; p0_tdot <= 16'sd0; p0_tick <= 1'b0; p0_valid <= 1'b0;
        end else begin
            p0_tick <= tick;
            if (tick) begin
                p0_alpha <= alpha; p0_adot <= alpha_dot; p0_tdot <= theta_dot;
                p0_valid <= alpha_ok;
            end
        end
    end
    // 级 1：索引/查表/动能乘法
    wire signed [16:0] alpha_abs17 = p0_alpha[15] ? -$signed(p0_alpha) : $signed(p0_alpha);
    wire [21:0] idx_full = alpha_abs17[15:0] * 13'd41;   // ≤32767×41=1.34M
    wire [8:0]  idx_pre  = idx_full[20:12];              // ÷4096, ≤328
    wire [6:0]  rom_idx  = (idx_pre > 9'd89) ? 7'd89 : idx_pre[6:0];
    wire [7:0]  pot_rom  = oneminuscos(rom_idx);
    wire [15:0] av_abs = p0_adot[15] ? -p0_adot : p0_adot;       // ≤32768
    wire [31:0] kin_sq = av_abs * av_abs;                        // ≤2^30
    reg  [7:0]  pot_q1;  reg [31:0] kin_sq_q1;  reg signed [15:0] adot_q1;
    reg signed [15:0] tdot_q1;  reg co_pos_q1;  reg signed [15:0] alpha_q1;
    always @(posedge clk) begin
        pot_q1 <= pot_rom; kin_sq_q1 <= kin_sq; adot_q1 <= p0_adot;
        tdot_q1 <= p0_tdot; co_pos_q1 <= (alpha_abs17 < 17'sd9000); // cosα>0 ⇔ |α|<90°
        alpha_q1 <= p0_alpha;
    end
    // 级 1 组合输出：静止判据与泵方向（短逻辑，随级 1 寄存进入流水）。
    // signpump 照抄 pump_trace（av·cosα 异号→−1，同号→+1，av=0→+1）：
    //   下半程 co>0：av>0→+1 / av<0→−1；上半程 co<0：av>0→−1 / av<0→+1。
    // |α̇|<5°/s(500 c01) 视作静止恒 +1（实机 α̇ 噪声 ±3°/s=300 c01 会把方向
    // 判据高频打穿 → 净输出 0 → 垂挂永不起振；静止恒向一记推力=起摆第一推）。
    wire av_pos = (adot_q1 > 0);
    wire arm_slow = (tdot_q1 > -RETHROW_TD_MAX) && (tdot_q1 < RETHROW_TD_MAX);
    wire av_zero = (~adot_q1[15] && adot_q1 < 16'sd500) ||
                   ( adot_q1[15] && -adot_q1 < 16'sd500);
    wire signed [15:0] pump_sign =
        av_zero ? 16'sd1 :
        ((av_pos && !co_pos_q1) || (!av_pos && co_pos_q1)) ? -16'sd1 : 16'sd1;
    // 级 2（纯乘法寄存级）：kin×1636 与 pot×10400 两条并行小乘法单独一拍，
    // 切断"乘法串乘法"关键路径（pot_q1→e_pot→pump_raw→饱和→×sign 五连在
    // 26MHz 才收敛，时序报告最差路径 pot_q1/DO→pump_cmd_q2/D）。
    reg [45:0] kin_prod_q;  reg [15:0] epot_q;
    always @(posedge clk) begin
        kin_prod_q <= kin_sq_q1 * 16'd1636;          // ≤2^42
        epot_q     <= pot_q1 * 32'd10400 >>> 8;      // 0..10400（乘数须 32 位：
                                                     // 8×16 乘积被 Verilog 截 16 位）
    end
    // 级 3：E 合成（切片+加法，短）→ 泵乘法 → 寄存
    wire signed [31:0] e_kin  = {1'b0, kin_prod_q[45:30]};       // 0..~17600
    wire signed [31:0] e_now  = e_kin + $signed({1'b0, epot_q});
    wire signed [47:0] pump_raw = $signed(e_now - PUMP_E_TARGET) * $signed(PUMP_KE);
    // raw = duty×256（KE 为 Q8）→ duty = raw>>>8。32 位饱和后截 16 位。
    // 饱和比较须同宽：{1'b0,amplitude} 无符号拼接须包 $signed，否则三目上下文
    // 被污染（arm_vel blend_ext / traj_gen snap_target 同类教训）。
    reg signed [47:0] pump_raw_q;
    always @(posedge clk) pump_raw_q <= pump_raw;
    // 级 4 寄存：饱和值/方向/门/臂慢/回拉 全部同拍打齐（1ms 拍算法多一拍无感）
    wire signed [31:0] pump_q0  = pump_raw_q[39:8];
    wire signed [15:0] amp_pos  = $signed({1'b0, amplitude});
    wire signed [15:0] pump_mag =
        (pump_q0 >  $signed({1'b0, amplitude})) ? amp_pos :
        (pump_q0 < -$signed({1'b0, amplitude})) ? -amp_pos : pump_q0[15:0];
    reg signed [15:0] pump_mag_q;  reg signed [15:0] sign_q;
    reg arm_slow_q2;  reg fall_phase_q, start_kick_q;  reg signed [47:0] home_pull_q3;
    always @(posedge clk) begin
        pump_mag_q <= pump_mag;
        sign_q <= pump_sign;
        arm_slow_q2 <= arm_slow;
        fall_phase_q <= ((alpha_q1 >  16'sd10000) && (alpha_q1 <  16'sd17000) && (adot_q1 > 0)) ||
                        ((alpha_q1 < -16'sd10000) && (alpha_q1 > -16'sd17000) && (adot_q1 < 0));
        start_kick_q <= (((alpha_q1 > 16'sd17000) || (alpha_q1 < -16'sd17000)) && av_zero);
        home_pull_q3 <= -$signed(theta) * 32'sd14;   // >>>8 在级 5 组合
    end
    // 级 5：方向× → 寄存。sign_q ∈ {−1,+1}，乘法退化为条件取反
    //（16×16 全宽乘法综合出 LUT 级联 49.4MHz 差 0.35ns；取反 mux <2ns）。
    wire signed [15:0] pump_cmd = sign_q[15] ? -pump_mag_q : pump_mag_q;
    reg signed [15:0] pump_cmd_q;
    reg over_e_q;  reg signed [15:0] adot_q2;
    always @(posedge clk) begin
        pump_cmd_q <= pump_cmd;
        over_e_q <= (pump_q0 > 32'sd0);   // E 超标（阻尼模式判据，早一拍同源）
        adot_q2  <= adot_q1;              // 级 5 方向选择同拍用
    end
    // 级 6 组合：起步一击 | 下落段节奏泵 | 全域整流阻尼 | 滑行。
    // 阻尼方向律（2026-10-12 定案）：E 超标时恒逆 α̇（纯逆速整流）——
    // pump_trace 律（av·cosα 异号 −1）在均匀旋转态平均功率恒 0（cosα 正负
    // 半周相消），实测 25s 泄不掉"贴势垒爬圈"（60-75°/s 零捕获）；
    // −sign(ω) 的功率 −|a·cosα·ω| ≤ 0 恒负，任意相域都泄能。
    // damp 基于幅值 mag（pump_mag_q 与 adot_q2 同拍）而非含方向的 pump_cmd_q
    //（后者已乘过 sign，再取负会把相域符号乘回来）。
    wire signed [15:0] damp_cmd = adot_q2[15] ? pump_mag_q : -pump_mag_q;
    wire signed [15:0] pump_gated = start_kick_q ? amp_pos :
                                    over_e_q     ? damp_cmd :
                                    fall_phase_q ? pump_cmd_q : 16'sd0;
    wire signed [31:0] pump_sum = $signed({{16{pump_gated[15]}}, pump_gated}) +
                                  home_pull_q3[47:8];
    wire signed [15:0] pump_lim =
        (pump_sum > $signed({1'b0, amplitude})) ? $signed({1'b0, amplitude}) :
        (pump_sum < -$signed({1'b0, amplitude})) ? -$signed({1'b0, amplitude}) :
         pump_sum[15:0];
    wire signed [15:0] duty_pump = (alpha_ok && arm_slow_q2) ? pump_lim : 16'sd0;

    // ---------------- 共振激振模式（PUMP_MODE=2，2026-10-12 用户手动数据定案）----------------
    // 真实手动起摆实测（162806）：臂持续 ±2.5°/2-3Hz 小幅共振抖动，摆响应 4s
    // 达 269°/s。sign(α̇) 跟随式（v1-v6）停在稳态；扫频 bang-bang 版（v7）达成
    // 全自动链路（捕获 31°/s→稳摆 12.8s）但目标阶跃+高增益伺服=33.4Hz 抖振
    // 咔咔声（机械危害）且能量耗于抖振。v8=**连续三角波目标**：osc_ref 斜坡
    // 积分器（slope 8/7/6 c01/拍三段扫频），伺服小误差静音跟踪，折返即扫频；
    //   幅 A=±3000 c01（±2.5°，与手一致），臂速闸防飞转。
    reg signed [31:0] osc_ref;   // 相对中心的偏移（−A..+A 三角）
    reg        osc_dir;          // 三角方向
    reg [1:0]  osc_seg;          // 扫频段
    reg [14:0] seg_ms;           // 段计时（8s 换段）
    reg signed [31:0] theta_c;   // 段起始中心（θ 锁存）
    wire signed [31:0] osc_slope = (osc_seg == 2'd0) ? 32'sd12 :
                                   (osc_seg == 2'd1) ? 32'sd15 : 32'sd18;  // c01/拍
                                                                           // =12/15/18°/s
                                                                           // ↔ 2.0/2.5/3.0Hz
                                                                           // （手动数据：
                                                                           //  ±2.5°@2-3Hz）
    wire signed [31:0] osc_amp   = 32'sd3000;                               // ±2.5°
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            osc_ref <= 32'sd0; osc_dir <= 1'b0; osc_seg <= 2'd0; seg_ms <= 15'd0;
            theta_c <= 32'sd0;
        end else if (tick) begin
            if (tick && start) begin
                osc_ref <= 32'sd0; osc_dir <= 1'b0; osc_seg <= 2'd0; seg_ms <= 15'd0;
                theta_c <= theta;
            end else begin
                if (seg_ms >= 15'd8000) begin
                    seg_ms <= 15'd0;
                    osc_seg <= (osc_seg == 2'd2) ? 2'd0 : osc_seg + 2'd1;
                end else
                    seg_ms <= seg_ms + 15'd1;
                // 三角积分：到 ±A 折返；回程端点重锁中心（臂绕当前位置摆，防漂）
                if (osc_dir) begin
                    if (osc_ref >= osc_amp) begin
                        osc_dir <= 1'b0;
                        theta_c <= theta;
                    end else
                        osc_ref <= osc_ref + osc_slope;
                end else begin
                    if (osc_ref <= -osc_amp) begin
                        osc_dir <= 1'b1;
                        theta_c <= theta;
                    end else
                        osc_ref <= osc_ref - osc_slope;
                end
            end
        end
    end
    wire signed [31:0] osc_target = theta_c + osc_ref;
    wire arm_fast_res = (tdot_q1 > 16'sd1200) || (tdot_q1 < -16'sd1200); // |θ̇|>120°/s
    // 包络滞回（2026-10-12 手动"三波"节奏定案）：|α̇| 峰值保持 >250°/s（过顶
    // 转圈=能量够）→ 撒手等摩擦衰减；衰减到 <60°/s（接近捕获带）→ 恢复推。
    // v8 无撒手锁的教训：转圈衰减期推力持续干扰，40s 穿窗全错位零捕获。
    reg signed [15:0] env_peak;
    reg        res_hold;
    reg [8:0]  env_dly;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            env_peak <= 16'sd0; res_hold <= 1'b0; env_dly <= 9'd0;
        end else if (tick) begin
            if (tick && start) begin
                env_peak <= 16'sd0; res_hold <= 1'b0;
            end else begin
                if (p0_valid && (p0_adot[15] ? -p0_adot : p0_adot) > env_peak)
                    env_peak <= p0_adot[15] ? -p0_adot : p0_adot;
                else if (env_dly == 9'd511) begin
                    env_dly <= 9'd0;
                    env_peak <= env_peak - (env_peak >>> 4);   // 512ms ×15/16
                end else
                    env_dly <= env_dly + 9'd1;
                // 滞回：>250°/s（转圈=能量够）撒手；|α̇| 即时 <80°/s 恢复——
                // 恢复判据用即时速度而非峰值保持：撒手段摆已衰减到垂挂姿态
                // （v9 实测 |α|<15° 仅 2 样本），env 峰值 12.8s 才降回阈下=
                // 循环太慢；快循环（8-10s/轮）多轮尝试，衰减尾总会落在倒立
                // 姿态过渡带被捕获门接走（v7 实证 31°/s 完美 pass）。
                if (p0_valid) begin
                    if (env_peak > 16'sd25000)     res_hold <= 1'b1;
                    else if (p0_adot[15] ? -p0_adot < 16'sd8000
                                         :  p0_adot < 16'sd8000) res_hold <= 1'b0;
                end
            end
        end
    end
    // 误差低通（8 拍 EMA）：伺服软化，折返铃振 30Hz→大幅衰减（咔咔声根治）
    reg signed [31:0] res_err_f;
    wire signed [31:0] res_err_raw = osc_target - theta;
    generate if (RES_DRIVE_SHAPING_EN) begin : g_res_reset
        always @(posedge clk or negedge rst_n) begin
            if (!rst_n) res_err_f <= 32'sd0;
            else if (!swing_active || start) res_err_f <= 32'sd0;
            else if (tick) res_err_f <= res_err_f + ((res_err_raw - res_err_f) >>> 3);
        end
    end else begin : g_res_legacy
        always @(posedge clk) begin
            if (tick) res_err_f <= res_err_f + ((res_err_raw - res_err_f) >>> 3);
        end
    end endgenerate
    wire signed [31:0] res_err = res_err_f;
    wire signed [35:0] res_cmd = res_err * KP;
    wire signed [15:0] res_kick =
        (arm_fast_res || res_hold) ? 16'sd0 :
        ((res_cmd > $signed({{20{amplitude[15]}},amplitude})) ? amplitude :
         ((res_cmd < -$signed({{20{amplitude[15]}},amplitude})) ? -amplitude : res_cmd[15:0]));
    wire signed [15:0] res_home = -$signed(theta) * 32'sd14 >>> 8;   // 防漂
    reg signed [15:0] res_kick_q;  reg signed [47:0] res_home_q;  reg res_tick_q;
    always @(posedge clk) begin
        res_kick_q <= res_kick;  res_home_q <= res_home;  res_tick_q <= tick;
    end
    wire signed [31:0] res_sum = $signed({{16{res_kick_q[15]}}, res_kick_q}) +
                                 res_home_q[47:8];
    wire signed [15:0] res_lim =
        (res_sum > $signed({1'b0, amplitude})) ? $signed({1'b0, amplitude}) :
        (res_sum < -$signed({1'b0, amplitude})) ? -$signed({1'b0, amplitude}) :
         res_sum[15:0];
    wire signed [15:0] duty_res = res_lim;

    // 120°/s 门切断整个指令：回中残值不能经死区补偿变成反向点动。
    wire signed [15:0] res_drive_target = (arm_fast_res || res_hold) ? 16'sd0 : duty_res;
    reg signed [15:0] res_drive_q;
    reg res_ready;
    wire signed [16:0] res_drive_delta = $signed({res_drive_target[15],res_drive_target}) -
                                        $signed({res_drive_q[15],res_drive_q});
    wire res_reversing = (res_drive_q != 0) && (res_drive_target != 0) &&
                        (res_drive_q[15] != res_drive_target[15]);
    wire res_growing = res_drive_target[15] ? (res_drive_target < res_drive_q) :
                                             (res_drive_target > res_drive_q);
    wire signed [16:0] res_rise_limited = (res_drive_delta > RES_RISE_STEP) ?
                                         $signed({res_drive_q[15],res_drive_q}) + RES_RISE_STEP :
                                        (res_drive_delta < -RES_RISE_STEP) ?
                                         $signed({res_drive_q[15],res_drive_q}) - RES_RISE_STEP :
                                         $signed({res_drive_target[15],res_drive_target});
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin res_drive_q <= 0; res_ready <= 0; end
        else if (!swing_active) begin res_drive_q <= 0; res_ready <= 0; end
        else if (tick && start) begin res_drive_q <= 0; res_ready <= 1; end
        else if (res_tick_q && res_ready) begin
            // 换向先过零一拍，再缓升；撤力和同向减力直接跟随原指令。
            if (res_drive_target == 0 || res_reversing) res_drive_q <= 0;
            else if (res_growing) res_drive_q <= res_rise_limited[15:0];
            else res_drive_q <= res_drive_target;
        end
    end

    wire signed [31:0] err = target - theta;
    wire signed [35:0] cmd = err * KP;
    // 甩摆到达撤力（2026-10-22）。原实现是纯比例位置伺服（KP=9 duty/cdeg）：
    // 118° 的目标差 ×9 = 10620 duty 被 amplitude 限到 8800，臂被打满舵甩出去；
    // 冲过 target 后 err 变号、duty 立刻反打满舵 —— 形成 ±8800 的 0.5s 自激
    // 振荡。实测一轮 24.5s 里 21 次满舵甩击、平均间隔 0.5s，而补甩节奏是 3s，
    // 多出来的全是伺服过冲振荡。能量耗在抖振上，摆杆 |α| 只到 21~30°，
    // 进不了 15° 捕获窗。
    // 修法：臂进入 SWING_DEAD 半窗后彻底撤力，让它滑行（机械摩擦自然停），
    // 不再反向追。SWING_DEAD=0 退化为原行为（逐位一致，tb_control_path 依赖）。
    wire signed [31:0] err_abs = (err < 0) ? -err : err;
    wire fling_gate = (SWING_DEAD == 32'sd0) || (err_abs > SWING_DEAD);
    wire signed [15:0] duty_fling =
        !fling_gate ? 16'sd0 :
        (cmd > $signed({{20{amplitude[15]}},amplitude})) ? amplitude :
        ((cmd < -$signed({{20{amplitude[15]}},amplitude})) ? -amplitude : cmd[15:0]);

    // fling 路径保持旧行为：每 clk 跟随（组合响应语义，tb_control_path 依赖）；
    // 泵/共振 = 流水算法，tick 延迟后写 duty（写拍时全链已稳定）。
    reg tick_d, tick_d2, tick_d3, tick_d4, tick_d5, tick_d6;
    always @(posedge clk) begin
        tick_d <= tick; tick_d2 <= tick_d; tick_d3 <= tick_d2;
        tick_d4 <= tick_d3; tick_d5 <= tick_d4; tick_d6 <= tick_d5;
    end
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)      duty <= 16'sd0;
        else if (PUMP_MODE == 1) begin
            if (tick_d6) duty <= duty_pump;
        end else if (PUMP_MODE == 2) begin
            if (RES_DRIVE_SHAPING_EN)
                duty <= (!swing_active || !res_ready || start) ? 16'sd0 : res_drive_q;
            else if (res_tick_q) duty <= duty_res;
        end else if (PUMP_MODE == 3) begin
            if (tick) duty <= duty_ratchet;
        end else if (PUMP_MODE == 4) begin
            // 混合：泵主导；交棒后这一杆走甩摆相位机
            if (tick) duty <= hyb_fling ? duty_phase_s : duty_bpump;
        end else if (FLING_BRAKE_EN) duty <= duty_phase_s;
        else             duty <= duty_fling;
    end
endmodule
