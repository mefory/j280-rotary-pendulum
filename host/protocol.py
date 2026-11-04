"""V2字节协议，无GUI依赖。CRC-16/CCITT-FALSE，覆盖VERSION至PAYLOAD。"""
from dataclasses import dataclass
import re
import struct
import time
from pathlib import Path

MAGIC = b"\xa5\x5a"
VERSION = 2
# 载荷上限 80B（2026-10-30 对齐 improve-160416 树 packet_rx.v 的接收上限——
# v5 快照 80B 在板；此前 64 只容 v4 快照，v5 遥测被 Decoder 整帧丢弃）。
MAX_PAYLOAD = 88   # v6 遥测 88B（2026-11-02；原 80 会把 v6 帧当 CRC 错误静默丢弃）
HELLO, READ, BEGIN, SET, COMMIT, ABORT, DEFAULTS = range(1, 8)
STOP, CLEAR, START, MOVE, BALANCE, CALIBRATE, STATUS = range(8, 15)
IO_TEST = 16
# 强制进 POS（2026-11-03 夜）：BAL 态下绕过 pos_ready 门（|α|<5°、|α̇|<30°/s、
# 臂慢速），只保留 |α|<15° 安全界。载荷同 MOVE：delta i32（±6000）。
FORCE_POS = 17
# 速度曲线演示（2026-11-03 夜）：载荷 en u16 + V s16(0.1°/s) + A s16(0.1°/s per 10ms)。
# 固件内生三角波 v_ref（0→V→0），LQR 的 θ̇ 项跟踪 (θ̇−v_ref)，pos_tgt 跟随臂。
VEL_DEMO = 18
# 绝对目标定位（2026-11-04）：载荷 target i32（c01，0-360 域或最近等价角）。
# 固件用【当下 θ】算最短路 delta（wrap ±180°），消除上位机数据陈旧的终点偏差。
# 与 MOVE 的区别：MOVE 的 delta 由上位机算（点击那刻的 θ），本命令由固件算。
MOVE_ABS = 20
# HIL 锁步：载荷 adc u16 + enc i32（见 rtl/debug_link.v）。只有 hil_mode=1 的
# 板级构建接受，即设备回包能力位含 CAP_STEP_LOCK；真实传感模式返回 STATE。
# 与板无关：DE10 由 SW[9] 选择，Tang Primer 20K V1 恒为锁步。
STEP = 15
RESPONSE, TELEMETRY = 0x80, 0x90
OK, BAD_LENGTH, BAD_ID, RANGE, STATE, CONFLICT, NO_TXN, BAD_SCHEMA, EXPIRED, SEQUENCE = range(10)
ERRORS = {0:"成功",1:"长度错误",2:"未知参数",3:"参数越界",4:"当前状态不允许",
          5:"配置版本冲突",6:"没有参数事务",7:"协议/参数表不匹配",8:"事务超时",9:"序号错误"}

# ---- 板标识与能力位：HELLO 回包 (schema, build, capabilities) 的后两项 ----
# 位定义见 rtl/debug_link.v 文件头；取值由 tools/generate_debug_config.py 写入
# rtl/j280_debug_params.vh，本模块在导入时解析该文件，两侧不会各写一份常量。
CAP_PARAMS, CAP_MOTION, CAP_TELEMETRY = 0, 1, 2
CAP_CALIB_DC, CAP_STEP_LOCK, CAP_HW_SENSORS, CAP_MOTOR_OUT, CAP_IO_TEST = 3, 4, 5, 6, 7
CAP_LABELS = {CAP_PARAMS: "参数事务", CAP_MOTION: "动作命令", CAP_TELEMETRY: "周期遥测",
              CAP_CALIB_DC: "下垂标定", CAP_STEP_LOCK: "STEP 锁步",
              CAP_HW_SENSORS: "硬件传感", CAP_MOTOR_OUT: "电机输出",
              CAP_IO_TEST: "I/O 联调"}
BOARD_NAMES = {0x0A: "DE10-Standard", 0x14: "Tang Primer 20K", 0x01: "J280 原板"}

_GENERATED = Path(__file__).resolve().parents[1] / "rtl/j280_debug_params.vh"
_FALLBACK = {"DEBUG_BOARD_ID": 0x0A, "DEBUG_BUILD_STAMP": 0x0A260923,
             "DEBUG_BUILD_EPOCH": 0,          # 源码树副本恒 0（非真实构建）
             "DEBUG_CAPS_REAL": 0xEF, "DEBUG_CAPS_HIL": 0x97}


def _read_generated():
    """解析 rtl/j280_debug_params.vh 的板标识/构建戳/能力位。

    该头文件由 tools/generate_debug_config.py 生成，是协议常量的单一事实来源；
    host 解析它而不是另抄一份，避免将来 RTL 改了而 Python 没跟着改。文件缺失
    （例如只拷贝了 host 目录）时退回内置默认值，不阻断导入。
    """
    values = dict(_FALLBACK)
    try:
        text = _GENERATED.read_text(encoding="utf-8")
    except OSError:
        return values
    for key in values:
        match = re.search(rf"`define\s+{key}\s+\d+'h([0-9a-fA-F]+)", text)
        if match:
            values[key] = int(match.group(1), 16)
    return values


_GEN = _read_generated()
BOARD_ID_DEFAULT = _GEN["DEBUG_BOARD_ID"]
BUILD_STAMP_DEFAULT = _GEN["DEBUG_BUILD_STAMP"]
CAPS_REAL = _GEN["DEBUG_CAPS_REAL"]
CAPS_HIL = _GEN["DEBUG_CAPS_HIL"]
BUILD_EPOCH_DEFAULT = _GEN["DEBUG_BUILD_EPOCH"]


def decode_build(stamp):
    """拆 build 戳 → {board_id, board, date}。板名未登记时用十六进制显示。

    ⚠ date 的日期段来自 generate_debug_config.py 里写死的 BUILD_DATE 常量，
    **不随构建变化**（同一提交永远相同）。它只能用来大致判断"哪一代板级配置"，
    绝不能当"这次烧的是哪天版本"。真正的构建时刻看 decode_epoch()。
    """
    board_id = stamp >> 24 & 0xFF
    return {"board_id": board_id,
            "board": BOARD_NAMES.get(board_id, f"未知板 0x{board_id:02x}"),
            "date": f"20{stamp >> 16 & 0xFF:02x}-{stamp >> 8 & 0xFF:02x}-{stamp & 0xFF:02x}",
            "date_is_static": True}


def decode_epoch(epoch):
    """Unix 构建时间戳 → {build_epoch, built, built_age_s}。

    0 表示"不是一次真实构建"（源码树基线、仿真、或旧固件不带该字段），
    此时 built 为 None —— 不要把它显示成 1970 年。
    """
    import time
    epoch = int(epoch or 0) & 0xFFFFFFFF
    if epoch == 0:
        return {"build_epoch": 0, "built": None, "built_age_s": None}
    return {"build_epoch": epoch,
            "built": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(epoch)),
            "built_age_s": round(time.time() - epoch, 1)}


def has_capability(capabilities, bit):
    return bool(capabilities >> bit & 1)


def capability_names(capabilities):
    return [CAP_LABELS[bit] for bit in sorted(CAP_LABELS) if capabilities >> bit & 1]


def crc16(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ (0x1021 if crc & 0x8000 else 0)) & 0xFFFF
    return crc


@dataclass(frozen=True)
class Frame:
    kind: int
    seq: int
    payload: bytes

    def encode(self):
        if len(self.payload) > MAX_PAYLOAD:
            raise ValueError("payload exceeds 64 bytes")
        body = struct.pack("<BBHH", VERSION, self.kind, self.seq, len(self.payload)) + self.payload
        return MAGIC + body + struct.pack("<H", crc16(body))


class Decoder:
    def __init__(self, timeout=0.1):
        self.buffer = bytearray()
        self.last = None
        self.timeout = timeout
        self.errors = 0

    def feed(self, data, now=None):
        now = time.monotonic() if now is None else now
        if self.buffer and self.last is not None and now-self.last > self.timeout:
            self.buffer.clear()
            self.errors += 1
        if data:
            self.last = now
            self.buffer.extend(data)
        result = []
        while len(self.buffer) >= 2:
            index = self.buffer.find(MAGIC)
            if index < 0:
                self.buffer[:] = self.buffer[-1:] if self.buffer[-1] == 0xA5 else b""
                break
            del self.buffer[:index]
            if len(self.buffer) < 8:
                break
            version, kind, seq, size = struct.unpack_from("<BBHH", self.buffer, 2)
            if version != VERSION or size > MAX_PAYLOAD:
                del self.buffer[0]
                self.errors += 1
                continue
            total = 10+size
            if len(self.buffer) < total:
                break
            if crc16(self.buffer[2:total-2]) != struct.unpack_from("<H", self.buffer, total-2)[0]:
                del self.buffer[0]
                self.errors += 1
                continue
            result.append(Frame(kind, seq, bytes(self.buffer[8:total-2])))
            del self.buffer[:total]
        return result


# tick, revision, mode, fault, alpha, alpha_dot, theta, theta_dot,
# duty, reference, target, encoder, adc, offset, calib_done, run_en
# 注意 encoder 是 i32（qenc 多圈计数会超 16 位）、adc 是 u16。
#
# 记录格式 v3（2026-10-21）追加 14 字节诊断量，总 54 字节：
#   pos_trim     POS 积分器实际输出的 duty（臂靠什么力矩站住）
#   alpha_pre    trim 之前的 α（当轮真实零偏，免去 offset−trim 反推）
#   k3_eff/k4_eff 1024ms 渐变之后的实际增益（分离入场瞬态与渐变）
#   duty_pre_dz  死区补偿前的指令（640/740 到底加了多少）
#   obs_active   观测器是否接管（OBS_USE=1 后必须能确认走哪条链）
#   alpha_trim   当前生效的 trim（记录中途写参数时能对上）
# 末尾 1 字节对齐。旧固件发 40 字节：unpack_snapshot 会补齐缺失字段，
# 因此新上位机读旧板仍可用（只是诊断量是 None）。
# Firmware v4 emits a 64-byte snapshot: the legacy 40-byte state, seven
# diagnostic fields, then five LQR term/position-error fields. Keep the parser
# aligned with rtl/debug_link.v; silently dropping a different length makes a
# healthy serial link look like an empty telemetry stream.
SNAPSHOT = struct.Struct("<IIBBhhihhiiiHhBBhhhhhBhxhhhhh")
SNAPSHOT_V2_SIZE = 40          # 旧固件布局（v3 之前）
FIELDS = ("tick","revision","mode","fault","alpha","alpha_dot","theta","theta_dot",
          "duty","reference","target","encoder","adc","offset","calib_done","run_en",
          "pos_trim","alpha_pre","k3_eff","k4_eff","duty_pre_dz","obs_active","alpha_trim",
          # v4（2026-10-27 修）：struct 里一直有这 5 个字段，但 FIELDS 漏登记，
          # zip 静默丢弃 ⇒ theta_e 与分项在落盘日志里完全缺失（今日 POS 诊断
          # 因此只能用 theta−reference 反推、还要处理 ±180° 回绕）。
          "term_k1","term_k2","term_k3","term_k4","theta_e")
# 旧固件没有这些字段，补齐成 None 让分析脚本能区分"值为 0"和"没这个量"
DIAG_FIELDS = ("pos_trim","alpha_pre","k3_eff","k4_eff","duty_pre_dz",
               "obs_active","alpha_trim",
               # v4 追加字段：旧固件（40B/54B）没有，补齐成 None
               "term_k1","term_k2","term_k3","term_k4","theta_e")


# 54 字节布局（v3，2026-10-21 到 10-27 之间的固件）：40B 主体 + 7 个诊断量
# +1 字节对齐。此前它落在 40B 分支里，诊断量被当成 None 丢弃（10-27 发现）。
SNAPSHOT_V3 = struct.Struct("<IIBBhhihhiiiHhBBhhhhhBhx")
# 80 字节布局（v5，2026-10-30 improve-160416 树，audit_20261030_224637 起在板）：
# v4 的 64B + 16B 原始量测链（观测器 mux 之前，植物辨识用）：
#   theta_raw s32 | alpha_raw s16 | theta_dot_raw s16 | alpha_dot_raw s16
#   | k1_eff s16 | k2_eff s16 | pos_i_trim s16（AT 分量 = pos_trim − pos_i_trim）
# 布局事实来源 = worktrees/improve-160416/rtl/debug_link.v（SNAPSHOT_BYTES=80）。
SNAPSHOT_V5 = struct.Struct("<IIBBhhihhiiiHhBBhhhhhBhxhhhhh" "ihhhhhh")
# 88 字节布局（v6，2026-11-02 restore-211437，audit_20261102_150422 起在板）：
# v5 的 80B + 8B POS 机制内部状态（级联/降额/稳定窗/死区表/丢帧计数）：
#   pos_bias s16 | move_blend u16 | settle_ms u16 | dropped_lo u8<<4|dz_idx u8 对齐
SNAPSHOT_V6 = struct.Struct("<IIBBhhihhiiiHhBBhhhhhBhxhhhhh" "ihhhhhh" "hHHBx")
V6_FIELDS = ("pos_bias", "move_blend", "settle_ms", "dz_idx")  # dz_idx 低 4 位|dropped_lo 高 4 位
# v5 追加字段：旧固件（40/54/64B）没有，补齐成 None
V5_FIELDS = ("theta_raw", "alpha_raw", "theta_dot_raw", "alpha_dot_raw",
             "k1_eff", "k2_eff", "pos_i_trim")
# 遥测帧长度白名单：unpack_snapshot 支持的布局档位。Client.poll 用它
# 决定哪些 TELEMETRY 帧进入数据流——2026-10-30 前它只认 SNAPSHOT.size，
# 旧固件 40B/54B 快照被静默丢弃（握手成功但永远没有遥测）；
# 当晚 22:46 板上换成 v5 80B 固件后又全被丢（decoder.errors 狂涨、零有效帧）。
SNAPSHOT_SIZES = frozenset((SNAPSHOT_V2_SIZE, SNAPSHOT_V3.size, SNAPSHOT.size,
                            SNAPSHOT_V5.size, SNAPSHOT_V6.size))


def unpack_snapshot(data):
    """解遥测快照。四层兼容：
       80B = v5（+原始量测链 theta_raw/alpha_raw/…/pos_i_trim，10-30 起）
       64B = v4（+term_k1..k4 +theta_e）
       54B = v3（+7 个诊断量；此前后落进 40B 分支被丢弃，10-27 修）
       40B = v2（诊断量全为 None）
    """
    if len(data) >= SNAPSHOT_V6.size:
        v6 = SNAPSHOT_V6.unpack_from(data)
        result = dict(zip(FIELDS, v6[:len(FIELDS)]))      # 前 29 项 = v4 布局
        result.update(dict(zip(V5_FIELDS, v6[len(FIELDS):len(FIELDS) + len(V5_FIELDS)])))
        result.update(dict(zip(V6_FIELDS, v6[len(FIELDS) + len(V5_FIELDS):])))
        b = result["dz_idx"]
        result["dz_idx"] = b & 0x0F                        # 低 4 位 = LUT 格号
        result["dropped"] = (b >> 4) & 0x0F                # 高 4 位 = dropped 低 4 位
        return result
    if len(data) >= SNAPSHOT_V5.size:
        v5 = SNAPSHOT_V5.unpack_from(data)
        result = dict(zip(FIELDS, v5[:len(FIELDS)]))      # 前 29 项 = v4 布局
        result.update(dict(zip(V5_FIELDS, v5[len(FIELDS):])))
        return result
    if len(data) >= SNAPSHOT.size:
        return dict(zip(FIELDS, SNAPSHOT.unpack_from(data)))
    if len(data) >= SNAPSHOT_V3.size:
        values = SNAPSHOT_V3.unpack_from(data)
        result = dict(zip(FIELDS[:23], values))
        result.update({k: None for k in ("term_k1","term_k2","term_k3","term_k4","theta_e")})
        return result
    values = struct.Struct("<IIBBhhihhiiiHhBB").unpack_from(data)
    result = dict(zip(FIELDS[:16], values))
    result.update({k: None for k in DIAG_FIELDS})
    return result


IO_STATUS = struct.Struct("<IIiHhB")
# MT6701 链路健康尾追扩展（2026-10-19，STATUS 应答 +12 字节）：
#   frames u16 | good u16 | crc_err u16 | mg_err u16 | mg_code u8 |
#   flags u8{bit0 fault, bit1 selected} | angle_raw u16(14b)
# 旧固件只有前 12 字节，unpack_io_status 优雅降级（mt6701=None）。
IO_STATUS_MT = struct.Struct("<HHHHBBH")
# MT6835（摆侧 α）链路健康二次尾追（2026-10-19，+14 字节）：
#   frames u16 | good u16 | crc_err u16 | status_err u16 | status u8(3b) |
#   flags u8{bit0 fault} | angle_raw u32(21b)
# 同样按长度优雅降级（mt6835=None）。
IO_STATUS_MT68 = struct.Struct("<HHHHBBI")
# MT6701 最长连续无好帧段（2026-10-20 12:40 cause=0 揭示看门狗路径发火后追加，
# 帧数 ×~25µs ≈ 段时长——量出扰动段真实长度，标定看门狗放宽是否足够）。
IO_STATUS_GAP = struct.Struct("<H")


def unpack_io_status(data):
    status = dict(zip(("rx_errors", "dropped", "encoder", "adc", "duty", "enabled"),
                      IO_STATUS.unpack(data[:IO_STATUS.size])))
    status["mt6701"] = None
    if len(data) >= IO_STATUS.size + IO_STATUS_MT.size:
        frames, good, crc_err, mg_err, mg_code, flags, angle_raw = \
            IO_STATUS_MT.unpack(data[IO_STATUS.size:IO_STATUS.size + IO_STATUS_MT.size])
        status["mt6701"] = {
            "frames": frames, "good": good, "crc_err": crc_err, "mg_err": mg_err,
            "mg_code": mg_code, "fault": bool(flags & 1), "selected": bool(flags & 2),
            "angle_raw": angle_raw,
            # bit7:6 = 最近一次锁存原因快照（驱动 st_fault_cause）：
            # 0 无 / 1 CRC / 2 场强 Mg[1:0] / 3 失跟踪 Mg[3]。旧固件恒 0。
            "cause": (flags >> 6) & 3,
        }
    status["mt6835"] = None
    off2 = IO_STATUS.size + IO_STATUS_MT.size
    if len(data) >= off2 + IO_STATUS_MT68.size:
        frames, good, crc_err, status_err, st, flags, angle_raw = \
            IO_STATUS_MT68.unpack(data[off2:off2 + IO_STATUS_MT68.size])
        status["mt6835"] = {
            "frames": frames, "good": good, "crc_err": crc_err,
            "status_err": status_err, "status": st, "fault": bool(flags & 1),
            "angle_raw": angle_raw,
        }
    off3 = off2 + IO_STATUS_MT68.size
    if len(data) >= off3 + IO_STATUS_GAP.size:
        (gap,) = IO_STATUS_GAP.unpack(data[off3:off3 + IO_STATUS_GAP.size])
        if status["mt6701"] is not None:
            status["mt6701"]["max_no_good"] = gap
    return status
