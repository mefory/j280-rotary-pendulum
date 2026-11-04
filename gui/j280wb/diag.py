"""θ 传感链诊断（单源 MT6701）——纯逻辑、无 Qt，可无头自测。

背景（2026-10-22 定案）：原霍尔正交编码器**已物理拆除**，θ 只剩电机侧
MT6701 一条链（RTL：`TH_SRC_MT_ONLY=1`，arm_vel/qenc4x 整条链不实例化）。
于是"双源对比/判源"这类判据全部作废，本模块改成**单源健康 + 驱动响应一致性**：

  · θ 口径：`theta` = MT6701 角度事件累加（0.01°，上电位置=0，多圈连续）；
    `encoder` 字段在拆除霍尔后改投**MT6701 原始 14 位角**（0..16383），
    正好用来和累加器做交叉核对——这是拆掉霍尔后仅存的冗余。
  · 一致性：1 原始 LSB = 65536/7042 = 9.3064 c01（K_Q16=7042）。逐样本
    `Δθ/9.3064` 与 `Δ原角` 应一致（含 ±8192 回绕）；偏大=累加器丢步/回绕 bug，
    系统性符号=方向反（TH_MT_DIR_FLIP 该置 1）。
  · **掉线检测（本页的核心）**：MT6701 的径向磁铁叠在电机霍尔磁铁旁，电机大
    电流时磁场互相干扰 → 驱动报失追踪/场异常 → 驱动 hold-last-good → θ 冻结、
    θ̇ 塌到 0（2026-10-22 实测：整场 467 次 θ 停滞里 279 次落在满舵段，密度是
    低占空段的 20 倍）。单源下没有第二把尺，但**驱动-响应**能抓：
    `|duty| ≥ 2500`（死区 640 的 ~4 倍，确属真驱动）而 `|Δθ| < 3 c01` 持续
    ≥ 60ms ⇒ 记一次掉线事件。这条判据就是当天从数据里反推出来的现象，现在
    能在线看见。
  · 静止段噪声：θ̇/α̇ σ（θ̇ 静止噪声底 ~2.8°/s）。
"""
from collections import deque

# 口径常数（与 j280_defs.vh / theta_mt_path.v 保持一致，勿单独改）
GEAR_NUM = 409                 # 减速比 20.45 = 409/20（电机转:臂转）
GEAR_DEN = 20
K_Q16 = 7042                   # theta_mt_path 的 c01 定标（round(144000*20/409)）
RAW_FULL = 16384               # MT6701 原始 14 位一圈
RAW_TO_C01 = 65536.0 / K_Q16   # 1 原始 LSB = 9.3064 c01
# 若将来恢复霍尔编码器：θ/计数按官方口径 36000/1040 = 34.615 c01（放大 2.25%），
# 与 MT6701 真尺度 36000/(52×20.45)=33.860 c01/计数 的差就是当年判源的依据。
HALL_C01_PER_CNT = 36000.0 / 1040.0
TRUE_C01_PER_CNT = 36000.0 / (52 * 20.45)

DRIVE_MIN = 2500        # 驱动门限（duty）：死区补偿 640 的 ~4 倍
STUCK_C01 = 3           # 2ms 内 |Δθ| 低于此值视为"没动"（< 1 个原始 LSB=9.3）
STUCK_MS = 60           # 连续这么多毫秒才算一次掉线（磁干扰掉线实测 10~30ms）
RAW_ERR_OK = 3.0        # 原角一致性允许误差（原始 LSB）
QUIET_DRIVE = 800       # 静止样本要求 |duty| < 此值（否则可能是真实顶住不动）


class ThetaHealthDiag:
    """喂入连续遥测样本，输出单源 θ 健康统计与判定。"""

    def __init__(self):
        self.reset()

    def reset(self):
        self._prev = None
        self._first = None
        self._last = None
        self._samples = 0
        # 运动行程（c01 绝对值累计）与圈数
        self._motion_c01 = 0.0
        # 原角一致性
        self._raw_err_sum = 0.0
        self._raw_err_n = 0
        self._raw_err_max = 0.0
        self._raw_err_prev = 0.0
        self._raw_err_recent = deque(maxlen=50)
        # 驱动-响应（掉线检测）
        self._drive_ms = 0.0
        self._stuck_streak = 0.0
        self._stuck_reported = False
        self._dropout_events = 0
        self._dropout_ms = 0.0
        self._stuck_max_ms = 0.0
        # 静止段（θ 不动 + duty 小）统计
        self._quiet_ms = 0.0
        self._quiet_theta = deque(maxlen=4000)
        self._quiet_theta_dot = deque(maxlen=4000)
        self._quiet_alpha_dot = deque(maxlen=4000)

    # ---------- 喂数 ----------
    def feed(self, sample):
        if self._first is None:
            self._first = sample
        self._last = sample
        prev = self._prev
        self._prev = sample
        if prev is None:
            return
        d_theta = sample["theta"] - prev["theta"]
        dt_ms = max(1, sample["tick"] - prev["tick"]) or 2
        self._samples += 1
        self._motion_c01 += abs(d_theta)
        # 原角一致性：Δ原角按 ±8192 回绕后应与 Δθ/RAW_TO_C01 相等
        d_raw = sample["encoder"] - prev["encoder"]
        if d_raw > RAW_FULL // 2:
            d_raw -= RAW_FULL
        elif d_raw < -(RAW_FULL // 2):
            d_raw += RAW_FULL
        err = d_raw - d_theta / RAW_TO_C01
        self._raw_err_prev = err
        # 静止/低速时累加器取整误差占比高，只在有实际角度事件时统计
        if d_raw != 0 or abs(d_theta) >= RAW_TO_C01:
            self._raw_err_sum += err
            self._raw_err_n += 1
            self._raw_err_max = max(self._raw_err_max, abs(err))
            self._raw_err_recent.append(err)
        # 驱动-响应：指令在真驱动区而 θ 完全不跟 → 掉线/卡死
        duty = sample["duty"]
        if abs(duty) >= DRIVE_MIN:
            self._drive_ms += dt_ms
            if abs(d_theta) < STUCK_C01:
                self._stuck_streak += dt_ms
                self._stuck_max_ms = max(self._stuck_max_ms, self._stuck_streak)
                if self._stuck_streak >= STUCK_MS and not self._stuck_reported:
                    self._stuck_reported = True
                    self._dropout_events += 1
                if self._stuck_reported:
                    self._dropout_ms += dt_ms
            else:
                self._stuck_streak = 0.0
                self._stuck_reported = False
        else:
            self._stuck_streak = 0.0
            self._stuck_reported = False
        # 静止段：θ 不动且没有真驱动
        if abs(d_theta) < STUCK_C01 and abs(duty) < QUIET_DRIVE:
            self._quiet_ms += dt_ms
            self._quiet_theta.append(sample["theta"])
            self._quiet_theta_dot.append(sample["theta_dot"])
            self._quiet_alpha_dot.append(sample["alpha_dot"])

    # ---------- 读数 ----------
    @property
    def motion_c01(self):
        return self._motion_c01

    @property
    def motion_revs(self):
        return self._motion_c01 / 36000.0

    @property
    def raw_err_mean(self):
        return self._raw_err_sum / self._raw_err_n if self._raw_err_n else None

    @property
    def raw_err_max(self):
        return self._raw_err_max if self._raw_err_n else None

    @property
    def raw_err_recent(self):
        """最近 50 个角度事件的增量一致性均值（原始 LSB）。

        必须用**增量**核对：θ 累加器上电清零，而 MT6701 原始角有任意零点，
        绝对域两者天然差一个常数偏置（等于上电位置的原始角），不是错误。
        """
        if not self._raw_err_recent:
            return None
        return sum(self._raw_err_recent) / len(self._raw_err_recent)

    @property
    def dropout_events(self):
        return self._dropout_events

    @property
    def dropout_ms(self):
        return self._dropout_ms

    @property
    def drive_ms(self):
        return self._drive_ms

    @property
    def stuck_max_ms(self):
        return self._stuck_max_ms

    @property
    def dropout_ratio(self):
        """掉线时长 / 驱动时长（0..1）。"""
        return self._dropout_ms / self._drive_ms if self._drive_ms else 0.0

    def _sigma(self, values):
        data = list(values)
        if len(data) < 2:
            return None
        mean = sum(data) / len(data)
        return (sum((v - mean) ** 2 for v in data) / len(data)) ** 0.5

    def verdict(self):
        """检查单：[(项, 状态True/False/None, 说明)] + 结论文本。"""
        checks = []
        # 1) 驱动-响应一致性（掉线检测）
        if self._drive_ms < 200:
            checks.append(("驱动-响应一致性（掉线检测）", None,
                           f"驱动段仅 {self._drive_ms:.0f} ms（需 ≥200 ms："
                           "点动或起摆让 duty 上到 2500 以上）"))
        else:
            ratio = self.dropout_ratio
            ok = ratio <= 0.02 and self._dropout_events <= 3
            checks.append(("驱动-响应一致性（掉线检测）", ok,
                           f"{self._dropout_events} 次 / 累计 {self._dropout_ms:.0f} ms"
                           f"（驱动段 {self._drive_ms:.0f} ms，占比 {ratio*100:.1f}%；"
                           f"单次最长 {self._stuck_max_ms:.0f} ms）"
                           + ("" if ok else
                              " → 电机大电流期间 MT6701 磁场受扰掉线（径向磁铁贴霍尔），"
                              "θ 会冻结、θ̇ 塌到 0；起摆/定位都是靠这条链，"
                              "占比高时要减小电流冲击或给磁铁加屏蔽")))
        # 2) 原角一致性
        if self.raw_err_max is None:
            checks.append(("θ 累加器 vs 原始角", None, "无角度事件样本"))
        else:
            mean = self.raw_err_mean
            ok = self.raw_err_max <= RAW_ERR_OK
            checks.append(("θ 累加器 vs 原始角", ok,
                           f"偏差 均值 {mean:+.2f} / 最大 {self.raw_err_max:.2f} LSB"
                           f"（{RAW_TO_C01:.2f} c01/LSB，容差 ±{RAW_ERR_OK:.0f}）"
                           + ("" if ok else
                              " → 累加器丢步/回绕 bug；若均值系统性为 ±1 LSB 量级"
                              "且整段同号，先查 TH_MT_DIR_FLIP 是否该置 1")))
        # 3) 静止噪声
        if self._quiet_ms >= 500 and len(self._quiet_theta_dot) >= 50:
            td_sigma = self._sigma(self._quiet_theta_dot)
            ad_sigma = self._sigma(self._quiet_alpha_dot)
            th_pp = max(self._quiet_theta) - min(self._quiet_theta)
            checks.append(("静止噪声 θ̇ σ", td_sigma <= 40,
                           f"{td_sigma:.1f} (0.1°/s)，θ 峰峰 {th_pp} c01"))
            checks.append(("静止噪声 α̇ σ", ad_sigma <= 200,
                           f"{ad_sigma:.1f} (0.1°/s)（MT6835 α 链噪声底）"))
        else:
            checks.append(("静止噪声", None,
                           f"静止段 {self._quiet_ms:.0f} ms（需 ≥500 ms：静止 1~2 s 再判定）"))
        # 结论
        if self._drive_ms < 200:
            concl = ("驱动段不足：待机下用「硬件联调」页 IO_TEST 按 ±3000 以上点动，"
                     "或直接起摆一轮，让 duty 进入真驱动区后再判定。")
        elif self.dropout_ratio > 0.02 or self._dropout_events > 3:
            concl = ("驱动期 MT6701 有明显掉线段：径向磁铁与电机霍尔磁铁叠装，"
                     f"大电流时磁场互相干扰（单次最长 {self._stuck_max_ms:.0f} ms）。"
                     "起摆的 ACCEL/BRAKE 已按时间开环、保持相权限夹 4000、"
                     "SWING 内不让该故障锁安全链——但 BALANCE/POS 仍靠这条链闭环，"
                     "掉线占比高时应优先处理磁路（加隔磁片/拉开间隙/换贴装位置）。")
        else:
            concl = (f"θ 单源链健康：驱动段 {self._drive_ms:.0f} ms 内掉线 "
                     f"{self._dropout_ms:.0f} ms，累加器/原角偏差最大 "
                     f"{self.raw_err_max:.2f} LSB，行程 {self.motion_revs:+.2f} 圈。")
        return checks, concl



class WatchMonitor:
    """常开监视：把模式迁移/故障锁存/标定完成写成一条条可读消息。

    另带**驱动-响应掉线检测**（2026-10-22 霍尔拆除后的核心告警）：MT6701 的径向
    磁铁叠在电机霍尔磁铁旁，大电流时磁场互相干扰 → 驱动 hold-last-good → θ 冻结、
    θ̇ 塌到 0。单源下没有第二把尺，但"指令在真驱动区而 θ 完全不动"是确定签名
    （判据同 ThetaHealthDiag，连续 150 拍 ≈0.3s 才点名，避免正常起停误报）。
    """

    DROPOUT_SAMPLES = 150   # 连续拍数（500Hz 遥测 ≈ 0.3s）

    def __init__(self, modes):
        self._modes = modes
        self._prev = None
        self._stuck_streak = 0
        self._dropout_reported = False

    def reset(self):
        self._prev = None
        self._stuck_streak = 0
        self._dropout_reported = False

    def feed(self, sample):
        prev = self._prev
        self._prev = sample
        if prev is None:
            return []
        out = []
        mode, pmode = sample["mode"], prev["mode"]
        if mode != pmode:
            names = (self._modes.get(pmode, pmode), self._modes.get(mode, mode))
            out.append(f"模式：{names[0]} → {names[1]}")
            if mode == 4 and pmode in (2, 3):
                state = (f"α={sample['alpha']/100:+.2f}° "
                         f"θ̇={sample['theta_dot']/10:+.1f}°/s "
                         f"θ={sample['theta']/100:+.2f}°")
                # 三种成因分开点名（2026-10-19 实测教训：安全链中断被文案误报成
                # "掉摆"，与 fault 锁存同拍、摆杆其实仍直立）：fault 分支是 RTL
                # mode_fsm 的确定路径，比角度推断可靠；fault=0 时再用 |α| 启发。
                fell = abs(sample["alpha"]) > 3000
                if sample["fault"]:
                    out.append(
                        "稳摆被安全链中断（不是掉摆，摆杆"
                        + ("同时也已远离直立）" if fell else "仍直立）")
                        + f"：{state}——同拍故障锁存，链路层瞬态坏帧触发；"
                          "清故障后可重试")
                elif fell:
                    out.append(f"掉摆进入安全停止：{state}（|α|>30°）")
                else:
                    out.append(f"稳摆中止（非掉摆、无故障锁存，应为人工停止/超时）：{state}")
        if sample["fault"] and not prev["fault"]:
            out.append(
                f"⚠ 故障锁存（安全链）：mode={self._modes.get(sample['mode'], sample['mode'])} "
                f"α={sample['alpha']/100:+.2f}° θ={sample['theta']/100:+.2f}° "
                f"θ̇={sample['theta_dot']/10:+.1f}°/s。MT6835/MT6701 任一失联或坏帧"
                "都会锁此链（SWING 期间 MT6701 例外：大电流磁扰不锁）——"
                "查接线/磁场告警（CHECK_MG）后清故障再试。")
        if sample["calib_done"] and not prev["calib_done"]:
            out.append(f"标定完成：下垂零偏 {sample['offset']/100:+.2f}°")
        # 驱动-响应掉线：指令在真驱动区而 θ 纹丝不动（判据同 ThetaHealthDiag）
        d_theta = sample["theta"] - prev["theta"]
        if abs(sample["duty"]) >= DRIVE_MIN and abs(d_theta) < STUCK_C01:
            self._stuck_streak += 1
            if self._stuck_streak >= self.DROPOUT_SAMPLES and not self._dropout_reported:
                self._dropout_reported = True
                out.append(
                    f"⚠ 驱动中 θ 不动（连续 {self._stuck_streak} 拍，duty="
                    f"{sample['duty']:+d}，θ̇={sample['theta_dot']/10:+.1f}°/s）："
                    "电机大电流期间 MT6701 磁场受扰掉线（径向磁铁贴霍尔磁铁），"
                    "或臂被机械卡死。掉线时 θ 冻结、θ̇ 塌到 0，闭环里等于失明——"
                    "起摆的加速/刹车已按时间开环兜住，但稳摆/定位仍靠这条链，"
                    "频繁出现要优先处理磁路。")
        else:
            self._stuck_streak = 0
            self._dropout_reported = False
        return out



# ============================================================================
# MT6701 链路级诊断（STATUS 应答尾追 12 字节，2026-10-19）
#
# 与上面 ThetaHealthDiag 的区别：那个判"数据对不对"（驱动-响应 / 累加器一致性），
# 本类判 "SPI/SSI 链路和磁场健康不健康"——直接读驱动内部统计，是 bring-up 第一现场：
#   · Mg[1:0] 磁场强度码：00=正常，01=场太强，10=场太弱（径向磁铁叠装后最可能
#     是 10：磁铁间隙变大或方向没对准；01 则是磁铁贴太近/太强）
#   · Mg[3] 失跟踪（Rev1.5 语义：转速超限/失锁）
#   · CRC6 错帧率：布线过长、SCK 边沿、地噪声的直接指标
#   · 好帧计数停滞：CSN/SCK/DO 接线问题或器件未上电
# 帧率换算：驱动轮询 ~40kHz（FRAME_GAP_CLKS=1000 @50MHz），STATUS 两次读之间
# 用 tick 差折算。
# ============================================================================
MG_CODE_TEXT = {
    0: "正常",
    1: "场太强（磁铁过近/磁铁太强）",
    2: "场太弱（间隙大/磁铁偏/叠装干扰）",
    3: "场强码 11（保留）",
}

# MT6701 锁存原因快照（驱动 st_fault_cause，flags bit7:6）：
# 区分场强告警串与失跟踪串——22:54 分叉的直接答案。
MT6_CAUSE_TEXT = {0: "无", 1: "CRC", 2: "场强", 3: "失跟踪"}


def fmt_sat16(v):
    """16 位错帧计数饱和标注：到 0xFFFF 后不再增长，直接显示会误读成“错得不多”。"""
    return f"{v}（饱和）" if v == 0xFFFF else str(v)


class Mt6701LinkDiag:
    """吃两次 io_status() 的采样，输出链路健康读数、坏帧率与判定。"""

    def __init__(self):
        self._prev = None
        self._prev_tick = None

    def reset(self):
        self._prev = None
        self._prev_tick = None

    @staticmethod
    def describe(status):
        """把一次 io_status 的 mt6701 字段翻译成 UI 行。旧固件返回 None。"""
        mt = status.get("mt6701")
        if mt is None:
            return None, "旧固件：STATUS 无 MT6701 健康字段（需 mt6701-theta 分支位流）"
        lost_track = (mt["mg_code"] >> 2) & 1
        mg_bits = mt["mg_code"] & 3
        rows = {
            "mg_text": MG_CODE_TEXT.get(mg_bits, "未知"),
            "lost_track": bool(lost_track),
            "fault": mt["fault"],
            "selected": mt["selected"],
            "frames": mt["frames"],
            "good": mt["good"],
            "crc_err": mt["crc_err"],
            "mg_err": mt["mg_err"],
            "angle_raw": mt["angle_raw"],
            "angle_deg": mt["angle_raw"] * 360.0 / 16384.0,
            "cause": mt.get("cause", 0),
            "cause_text": MT6_CAUSE_TEXT.get(mt.get("cause", 0), "?"),
        }
        return rows, ""

    def update(self, status, tick=None):
        """喂一次 io_status；返回 (rows|None, 消息列表)。

        tick 用于算帧率与坏帧率（两次 STATUS 之间的增量）；不给则只报累计值。
        """
        rows, note = self.describe(status)
        msgs = []
        if rows is None:
            return None, ([note] if note else [])
        if self._prev and tick is not None and self._prev_tick is not None:
            dt_ms = tick - self._prev_tick
            if dt_ms > 0:
                df = (rows["frames"] - self._prev["frames"]) & 0xFFFF
                dg = (rows["good"] - self._prev["good"]) & 0xFFFF
                dc = (rows["crc_err"] - self._prev["crc_err"]) & 0xFFFF
                dm = (rows["mg_err"] - self._prev["mg_err"]) & 0xFFFF
                rows["frame_rate"] = df * 1000.0 / dt_ms
                rows["bad_rate"] = (df - dg) * 100.0 / df if df else 0.0
                rows["d_crc"] = dc
                rows["d_mg"] = dm
                if df == 0:
                    msgs.append("⚠ MT6701：两次查询间无新帧——CSN/SCK/DO 接线或器件未上电"
                                "（V5.1 位流下 MT6701 为 θ 源，控制已被安全链锁停）")
                elif rows["bad_rate"] > 10.0:
                    msgs.append(f"⚠ MT6701 坏帧率 {rows['bad_rate']:.1f}%（{dc} CRC 错/"
                                f"{dm} Mg 告警/共 {df} 帧）——查 SSI 走线与磁场")
        if rows["fault"] and not (self._prev or {}).get("fault"):
            msgs.append(f"⚠ MT6701 进入故障：Mg={rows['mg_text']}"
                        f"{'、失跟踪' if rows['lost_track'] else ''}"
                        f"、锁存原因={rows['cause_text']}"
                        f"、CRC 累计 {fmt_sat16(rows['crc_err'])}、"
                        f"Mg 告警累计 {fmt_sat16(rows['mg_err'])}")
        if rows["lost_track"] and not (self._prev or {}).get("lost_track"):
            msgs.append("⚠ MT6701 失跟踪（Mg[3]=1）：转速超限或磁铁失锁，角度不可信")
        self._prev = rows
        self._prev_tick = tick
        return rows, msgs

    def verdict(self, rows):
        """bring-up 检查单：[(项, 通过?, 说明)]。"""
        if rows is None:
            return [("MT6701 链路字段", None, "旧固件无此字段")], \
                   "需烧 mt6701-theta 分支位流才能读取 MT6701 链路健康"
        checks = []
        ok = not rows["lost_track"]
        checks.append(("失跟踪 Mg[3]", ok,
                       "无失跟踪" if ok else "失跟踪置位 → 转速/磁场异常，角度不可信"))
        ok = rows["mg_text"] == "正常"
        checks.append(("磁场强度 Mg[1:0]", ok,
                       f"{rows['mg_text']}"
                       + ("" if ok else " → 径向磁铁叠装常见：调间隙/对中，"
                                        "距离过远会一直报警")))
        bad = rows.get("bad_rate")
        ok = bad is None or bad < 1.0
        checks.append(("坏帧率", ok if bad is not None else None,
                       "需两次查询" if bad is None else
                       f"{bad:.2f}%（CRC 错 {rows.get('d_crc', 0)}、Mg 告警 {rows.get('d_mg', 0)}）"
                       + ("" if ok else " → >1% 查 SSI 走线/共地/磁铁抖动")))
        rate = rows.get("frame_rate")
        ok = rate is None or rate > 10000
        checks.append(("轮询帧率", ok if rate is not None else None,
                       "需两次查询" if rate is None else
                       f"{rate:.0f} Hz（期望 ~40000；明显偏低=驱动卡帧/看门狗复位）"))
        if rows["fault"]:
            concl = ("MT6701 当前报故障：它就是唯一的 θ 源。SWING 期间该故障不锁"
                     "安全链（大电流磁扰预期内），BALANCE/POS 期间会锁 SAFE。"
                     "先按上面磁场/坏帧项排查，修复后清故障（STOP 长按或急停复位）。")
        elif rows["selected"]:
            concl = "MT6701 链路健康且是 θ 源（霍尔已拆除，唯一源）——可做稳摆 A/B。"
        else:
            concl = ("MT6701 链路健康但未被选中：板上是旧 V5 跳线位流（A14 悬空，"
                     "θ 用霍尔源）——霍尔已拆除，必须烧 MT 独占位流。")
        return checks, concl


# ============================================================================
# MT6835（摆侧 α）链路级诊断（STATUS 应答二次尾追 14 字节，2026-10-19）
#
# 与 MT6701 版同构，读驱动内部统计判“SPI 链路与器件状态健康不健康”：
#   · 状态位 bit0 超速 / bit1 磁场过弱 / bit2 欠压（器件自报，非 FPGA 判定）
#   · CRC8 错帧率：布线/共地/SCK 边沿质量的直接指标
#   · 好帧停滞：四线（CSN/SCK/MOSI/MISO）接线或上电问题
#   · 未接线签名：MISO 上拉恒 1 → 原始角=2^21-1、状态=111、CRC 必错
# 帧率换算：驱动轮询 ~50kHz（帧周期 ~20µs），两次 STATUS 间用 tick 差折算。
# MT6835 是 α 唯一来源：它的故障无条件进安全链（otr_fault），板上直接锁 SAFE。
# ============================================================================
MT68_STATUS_BITS = ((0, "超速警告"), (1, "磁场过弱"), (2, "欠压"))


def mt68_status_text(code):
    bits = [text for bit, text in MT68_STATUS_BITS if (code >> bit) & 1]
    return "、".join(bits) if bits else "正常"


class Mt6835LinkDiag:
    """吃两次 io_status() 的采样，输出 MT6835 链路健康读数、坏帧率与判定。"""

    def __init__(self):
        self._prev = None
        self._prev_tick = None

    def reset(self):
        self._prev = None
        self._prev_tick = None

    @staticmethod
    def describe(status):
        """把一次 io_status 的 mt6835 字段翻译成 UI 行。旧固件返回 None。"""
        mt = status.get("mt6835")
        if mt is None:
            return None, "旧固件：STATUS 无 MT6835 健康字段（需今日新位流）"
        rows = {
            "status_text": mt68_status_text(mt["status"]),
            "status": mt["status"],
            "fault": mt["fault"],
            "frames": mt["frames"],
            "good": mt["good"],
            "crc_err": mt["crc_err"],
            "status_err": mt["status_err"],
            "angle_raw": mt["angle_raw"],
            "angle_deg": mt["angle_raw"] * 360.0 / 2097152.0,
            "unwired": (mt["angle_raw"] == 2097151 and mt["crc_err"] not in (0,)),
        }
        return rows, ""

    def update(self, status, tick=None):
        """喂一次 io_status；返回 (rows|None, 消息列表)。tick 给出才算帧率。"""
        rows, note = self.describe(status)
        msgs = []
        if rows is None:
            return None, ([note] if note else [])
        if self._prev and tick is not None and self._prev_tick is not None:
            dt_ms = tick - self._prev_tick
            if dt_ms > 0:
                df = (rows["frames"] - self._prev["frames"]) & 0xFFFF
                dg = (rows["good"] - self._prev["good"]) & 0xFFFF
                dc = (rows["crc_err"] - self._prev["crc_err"]) & 0xFFFF
                ds = (rows["status_err"] - self._prev["status_err"]) & 0xFFFF
                rows["frame_rate"] = df * 1000.0 / dt_ms
                rows["bad_rate"] = (df - dg) * 100.0 / df if df else 0.0
                rows["d_crc"] = dc
                rows["d_status"] = ds
                if df == 0:
                    msgs.append("⚠ MT6835：两次查询间无新帧——四线接线或器件未上电"
                                "（α 失效，安全链已锁 SAFE）")
                elif rows["bad_rate"] > 10.0:
                    msgs.append(f"⚠ MT6835 坏帧率 {rows['bad_rate']:.1f}%（{dc} CRC 错/"
                                f"{ds} 状态告警/共 {df} 帧）——查 SPI 走线/共地/磁铁")
        if rows["unwired"] and not (self._prev or {}).get("unwired"):
            msgs.append("⚠ MT6835 原始角=2^21-1 且 CRC 累计非零：MISO 恒高的断线签名"
                        "（未接线/未上电/CSN 没拉到器件）")
        if rows["fault"] and not (self._prev or {}).get("fault"):
            msgs.append(f"⚠ MT6835 进入故障：状态={rows['status_text']}"
                        f"、CRC 累计 {fmt_sat16(rows['crc_err'])}、"
                        f"状态告警累计 {fmt_sat16(rows['status_err'])}——α 唯一来源，"
                        "板上安全链已锁 SAFE")
        elif (rows["status"] and not (self._prev or {}).get("status")
              and not rows["fault"]):
            msgs.append(f"MT6835 状态位非零（{rows['status_text']}）但未达故障门——"
                        "间歇性告警，建议观察坏帧率")
        self._prev = rows
        self._prev_tick = tick
        return rows, msgs

    def verdict(self, rows):
        """bring-up 检查单：[(项, 通过?, 说明)]。"""
        if rows is None:
            return [("MT6835 链路字段", None, "旧固件无此字段")], \
                   "需烧今日新位流（STATUS 57 字节版）才能读取 MT6835 链路健康"
        checks = []
        ok = rows["status"] == 0
        checks.append(("器件状态位", ok,
                       f"{rows['status_text']}"
                       + ("" if ok else " → 按位排查：磁场过弱查磁铁距离/对中，"
                                        "欠压查 3.3V 供电与线阻，超速为瞬时过速")))
        bad = rows.get("bad_rate")
        ok = bad is None or bad < 1.0
        checks.append(("坏帧率", ok if bad is not None else None,
                       "需两次查询" if bad is None else
                       f"{bad:.2f}%（CRC 错 {rows.get('d_crc', 0)}、"
                       f"状态告警 {rows.get('d_status', 0)}）"
                       + ("" if ok else " → >1% 查 SPI 走线/共地")))
        rate = rows.get("frame_rate")
        ok = rate is None or rate > 30000
        checks.append(("轮询帧率", ok if rate is not None else None,
                       "需两次查询" if rate is None else
                       f"{rate:.0f} Hz（期望 ~50000；明显偏低=驱动卡帧/看门狗复位）"))
        ok = not rows["unwired"]
        checks.append(("断线签名", ok,
                       "未见" if ok else
                       "原始角=2^21-1：MISO 恒高——未接线/未上电的确定签名"))
        if rows["fault"]:
            concl = ("MT6835 当前报故障：α 唯一来源，板上安全链已锁 SAFE、电机不出力。"
                     "按上面状态位/坏帧/断线项排查后清故障（STOP 长按或急停复位）。")
        elif rows["unwired"]:
            concl = "MT6835 呈断线签名——先接好 PMOD2 四线（D14/E15/B14/A15）再验。"
        else:
            concl = "MT6835 链路健康，α 数据可信——可配合「开始采集」记录点动验证。"
        return checks, concl


# ============================================================================
# 诊断数据记录（2026-10-19）
#
# 目的：让"实测效果"可复盘——把每个遥测样本连同派生量逐行写 CSV。
# 与 bridge 的 JSONL 记录的区别：那个是协议级原样日志（含命令/参数/身份），
# 本记录是**面向诊断的扁平表**，多了原角折算/累加器偏差/驱动-响应标记，
# 可直接丢进 Excel/pandas 画图或算统计，不需要再解析协议。
#
# 列说明（单位见表头）：
#   tick_ms    控制拍（ms，设备上电起算）
#   mode       0 待机 / 1 起摆 / 2 稳摆 / 3 定位 / 4 安全停止
#   fault      故障锁存
#   alpha_c01, alpha_dot_dps   摆杆角(0.01°)、角速度(0.1°/s)
#   theta_c01, theta_dot_dps   控制源 θ(0.01°)、θ̇(0.1°/s)
#   raw_angle                  MT6701 原始 14 位角（霍尔拆除后 encoder 字段的来源）
#   raw_theta_c01              原角折算 θ = raw_angle × 9.3064（只到一圈，锯齿）
#   raw_err_lsb                Δ原角 − Δθ/9.3064（原始 LSB；累加器丢步/方向反会偏）
#   stuck                      1=本拍处于"真驱动但 θ 不动"（掉线/卡死签名）
#   duty, adc, reference_c01, target_c01
#   d_raw, d_theta             相邻样本增量（±8192 回绕）
# ============================================================================
CSV_COLUMNS = ("tick_ms", "mode", "fault", "alpha_c01", "alpha_dot_dps",
               "theta_c01", "theta_dot_dps", "raw_angle", "raw_theta_c01",
               "raw_err_lsb", "stuck", "duty", "adc", "reference_c01",
               "target_c01", "d_raw", "d_theta")


class DiagRecorder:
    """把遥测样本写成诊断 CSV；行内容由 make_row 从样本+前一拍派生。"""

    def __init__(self, path):
        self.path = path
        self._fh = None
        self._prev = None
        self.rows = 0

    def open_at(self, path):
        self.path = path
        self._fh = open(self.path, "w", encoding="utf-8", newline="")
        self._fh.write(",".join(CSV_COLUMNS) + "\n")
        self._prev = None
        self.rows = 0
        return self.path

    def close(self):
        if self._fh:
            self._fh.flush()
            self._fh.close()
            self._fh = None
        return self.rows

    @property
    def active(self):
        return self._fh is not None

    def feed(self, sample):
        if self._fh is None:
            return
        prev = self._prev
        self._prev = sample
        raw = sample["encoder"] % RAW_FULL
        d_raw = (raw - prev["encoder"] % RAW_FULL) if prev else 0
        if d_raw > RAW_FULL // 2:
            d_raw -= RAW_FULL
        elif d_raw < -(RAW_FULL // 2):
            d_raw += RAW_FULL
        d_theta = (sample["theta"] - prev["theta"]) if prev else 0
        err = d_raw - d_theta / RAW_TO_C01
        stuck = int(abs(sample["duty"]) >= DRIVE_MIN and abs(d_theta) < STUCK_C01)
        row = (sample["tick"], sample["mode"], int(bool(sample["fault"])),
               sample["alpha"], sample["alpha_dot"],
               sample["theta"], sample["theta_dot"],
               raw, round(raw * RAW_TO_C01, 3), round(err, 4), stuck,
               sample["duty"], sample["adc"],
               sample["reference"], sample["target"], d_raw, d_theta)
        self._fh.write(",".join(str(v) for v in row) + "\n")
        self.rows += 1


def load_csv(path):
    """读回诊断 CSV 为 list[dict]（列名做数值转换），供复盘/绘图/自测。"""
    import csv
    out = []
    with open(path, encoding="utf-8", newline="") as fh:
        for row in csv.DictReader(fh):
            out.append({k: float(v) for k, v in row.items()})
    return out


def summarize_csv(rows):
    """从诊断记录算一份复盘摘要（掉线占比/原角一致性/噪声/时长）。"""
    if not rows:
        return {"rows": 0}
    span = rows[-1]["tick_ms"] - rows[0]["tick_ms"]
    dt = [max(1.0, rows[i]["tick_ms"] - rows[i - 1]["tick_ms"])
          for i in range(1, len(rows))]
    drive_ms = sum(d for r, d in zip(rows[1:], dt) if abs(r["duty"]) >= DRIVE_MIN)
    stuck_ms = sum(d for r, d in zip(rows[1:], dt) if r["stuck"])
    errs = [abs(r["raw_err_lsb"]) for r in rows[1:]
            if r["d_raw"] != 0 or abs(r["d_theta"]) >= RAW_TO_C01]
    signed = [r["raw_err_lsb"] for r in rows[1:]
              if r["d_raw"] != 0 or abs(r["d_theta"]) >= RAW_TO_C01]
    quiet = [r["theta_dot_dps"] for r in rows
             if r["stuck"] == 0 and abs(r["d_theta"]) < STUCK_C01
             and abs(r["duty"]) < QUIET_DRIVE]
    if len(quiet) >= 2:
        mean = sum(quiet) / len(quiet)
        td_sigma = (sum((v - mean) ** 2 for v in quiet) / len(quiet)) ** 0.5
    else:
        td_sigma = None
    motion = sum(abs(r["d_theta"]) for r in rows) / 36000.0
    return {
        "rows": len(rows),
        "span_s": round(span / 1000.0, 2),
        "drive_ms": round(drive_ms, 0),
        "stuck_ms": round(stuck_ms, 0),
        "dropout_ratio": round(stuck_ms / drive_ms, 4) if drive_ms else None,
        "raw_err_mean": round(sum(signed) / len(signed), 3) if signed else None,
        "raw_err_max": round(max(errs), 3) if errs else None,
        "quiet_theta_dot_sigma": round(td_sigma, 2) if td_sigma is not None else None,
        "motion_revs": round(motion, 2),
        "fault_ticks": sum(1 for r in rows if r["fault"]),
        "modes": sorted({int(r["mode"]) for r in rows}),

    }
