"""共享设备桥：一条连接、一个控制线程，GUI 与 Agent 接口共用。

框架无关（纯线程+回调），因此 Agent 服务可无头运行。命令串行经过控制
线程（协议本身是停止等待、单命令在途），STOP/CLEAR 永远插队；人工(GUI)
命令自动抢占 Agent 的控制租约。JSONL 记录格式与 j280_fpga/host 完全
一致，回放后端可读旧记录。
"""
import json
import queue
import struct
import subprocess
import threading
import time
from collections import deque
from datetime import datetime
from pathlib import Path

from host import protocol as p
from host.client import Client, CommandError
from host.devices import SimDevice, SerialDevice, ReplayDevice
from host.hil import HilDevice
from host.schema import PARAMS, ROOT, SCHEMA_ID, raw_value, validate

MODES = {0: "待机", 1: "自动起摆", 2: "稳摆", 3: "定位", 4: "安全停止"}

# 动作命令回 STATE 时的日志可读名（REQUIRE_CALIB=1 后未标定 START 必被拒，
# 只报"当前状态不允许"没法定位是哪道门）。
_CMD_NAMES = {p.STOP: "停止", p.CLEAR: "清除故障", p.START: "启动起摆",
              p.MOVE: "定位移动", p.BALANCE: "进入稳摆", p.CALIBRATE: "下垂零位标定",
              p.STATUS: "状态查询", p.IO_TEST: "IO 点动"}


def _git(cwd, *args):
    """取 git 信息的容错封装：非仓库/无 git 一律返回 None，绝不影响记录。"""
    try:
        out = subprocess.run(["git", "-C", str(cwd), *args],
                             capture_output=True, text=True, timeout=2)
        return out.stdout.strip() or None if out.returncode == 0 else None
    except Exception:
        return None


def _git_head(root):
    return _git(root, "rev-parse", "--short", "HEAD")


def _git_branch(root):
    return _git(root, "rev-parse", "--abbrev-ref", "HEAD")

# 后端对设备能力的要求：板级差异只通过能力位表达，所以这里"问设备"而不是"问板名"。
# 模拟/回放是上位机本地的，不依赖设备能力，故不出现在此表。
_BACKEND_REQUIREMENTS = {
    "锁步HIL": (p.CAP_STEP_LOCK,
                "该设备未上报 STEP 锁步能力。DE10 请把 SW[9] 拨到 ON；"
                "Tang Primer 20K V1 应恒为锁步，若报此错说明固件不是锁步构建"),
    "实板": (p.CAP_HW_SENSORS,
             "该设备当前是锁步模式（传感器由上位机提供、无电机输出），实板后端拿不到节拍。"
             "请改用「锁步HIL」后端"),
}


def _listeners():
    return deque()


def _disconnected():
    raise ConnectionError("连接已断开，命令未发送")


class Bridge:
    def __init__(self, kind, location=""):
        assert kind in ("模拟", "锁步HIL", "实板", "回放")
        self.kind = kind
        self.location = location
        self.label = ""
        self.identity = {}
        self.connected = False
        self.readonly = kind == "回放"
        self.latest = None
        self.active_params = None
        self.revision = 0
        self.error = None
        self.history = deque(maxlen=6000)      # 遥测环，供 telemetry(n) 与绘图
        self._lock = threading.Lock()
        self._cmds = deque()
        self._cv = threading.Condition()
        self._sample_hooks = _listeners()
        self._event_hooks = _listeners()
        self._lease = None                     # (owner, expiry_monotonic)
        self._lease_lock = threading.Lock()
        self._recorder = None
        self._rec_path = None
        self._rec_csv = None
        self._rec_csv_path = None
        self._rec_stats = None
        self._rec_open_warned = False
        # 记录文件被两个线程写：桥线程（采样/命令）+ GUI 线程（note 备注）。
        # 不加锁时多行 jsonl 会交错，事后无法解析。
        self._rec_lock = threading.Lock()
        self._demo = None
        self._swing_rec = None
        self._bat_probe = None                  # 电量探针状态机（battery_probe）
        self.param_failures = []                # 设备拒绝读取的参数 [(id, key, 消息)]
        self._thread = None
        self._shutdown = False

    # ---------- 生命周期 ----------
    def start(self):
        self._thread = threading.Thread(target=self._run, name="bridge", daemon=True)
        self._thread.start()

    def close(self, timeout=3.0):
        self._shutdown = True
        with self._cv:
            self._cv.notify_all()
        if self._thread:
            self._thread.join(timeout)
        # 断开/退出时收尾记录文件：补上 record_end 摘要，避免留下一个
        # 没有结束标记、也没有采样完整度信息的半截 jsonl。
        self._close_recorder("连接关闭")

    @property
    def alive(self):
        return self._thread is not None and self._thread.is_alive()

    # ---------- 订阅 ----------
    def on_sample(self, callback):
        with self._lock:
            self._sample_hooks.append(callback)
            return lambda: self._sample_hooks.remove(callback)

    def on_event(self, callback):
        with self._lock:
            self._event_hooks.append(callback)
            return lambda: self._event_hooks.remove(callback)

    # ---------- 控制租约 ----------
    def lease(self):
        with self._lease_lock:
            if self._lease and self._lease[1] > time.monotonic():
                owner, expiry = self._lease
                return {"owner": owner, "remaining": round(expiry - time.monotonic(), 1)}
            return None

    def acquire_lease(self, owner, seconds=30):
        with self._lease_lock:
            if self._lease and self._lease[1] > time.monotonic() and self._lease[0] != owner:
                return False
            self._lease = (owner, time.monotonic() + seconds)
            return True

    def renew_lease(self, owner, seconds=30):
        with self._lease_lock:
            if self._lease and self._lease[0] == owner:
                self._lease = (owner, time.monotonic() + seconds)
                return True
            return False

    def release_lease(self, owner):
        with self._lease_lock:
            if self._lease and self._lease[0] == owner:
                self._lease = None
            return True

    def _break_lease(self):
        with self._lease_lock:
            self._lease = None

    # ---------- 命令 ----------
    def submit(self, name, value=None, source="gui", wait=True, timeout=6.0):
        """提交一条命令；返回执行结果或在超时/断连时抛异常。"""
        if self.readonly and name not in ("record",):
            raise PermissionError("回放只读，不能发送动作或参数命令")
        if self._thread is not None and not self._thread.is_alive():
            raise ConnectionError("连接已断开，命令未发送")
        urgent = name in ("stop", "clear") or (
            name == "command" and value and value[0] in (p.STOP, p.CLEAR))
        if not urgent and not self.readonly:
            held = self.lease()
            if source.startswith("agent:") and held and held["owner"] != source:
                raise PermissionError(f"控制权由 {held['owner']} 持有，先 acquire 租约")
            if source == "gui" and held:
                self._break_lease()          # 人工操作永远优先
        done = threading.Event()
        box = {}

        def run(fn):
            try:
                box["result"] = fn()
            except Exception as exc:
                box["error"] = exc
            done.set()

        with self._cv:
            if urgent:
                self._cmds.appendleft(("call", run, name, value, source))
            else:
                self._cmds.append(("call", run, name, value, source))
            self._cv.notify_all()
        if not wait:
            return None
        if not done.wait(timeout):
            raise TimeoutError(f"命令 {name} 超时（设备无响应或正忙）")
        if "error" in box:
            raise box["error"]
        return box.get("result")

    # ---------- 控制线程 ----------
    def _open_device(self):
        if self.kind == "模拟":
            return SimDevice()
        if self.kind == "锁步HIL":
            try:
                return HilDevice(self.location)
            except Exception as exc:
                raise ConnectionError(
                    f"无法打开锁步 HIL 串口 {self.location}：{exc}"
                    "（确认 USB-TTL 已接 DE10 排针 1/2/12、板上 SW[9]=ON、串口未被占用）") from exc
        if self.kind == "回放":
            return ReplayDevice(self.location)
        try:
            return SerialDevice(self.location)
        except Exception as exc:
            raise ConnectionError(
                f"无法打开串口 {self.location}：{exc}（检查串口号是否正确；该串口可能被其他程序占用）") from exc

    def _handshake(self, client):
        """实板/锁步握手自动多试一次：适配器刚打开或板卡刚上电时首个 HELLO 可能丢。"""
        last = None
        for _ in range(2 if self.kind in ("实板", "锁步HIL") else 1):
            try:
                client.connect()
                return
            except Exception as exc:
                last = exc
                time.sleep(0.3)
        raise ConnectionError(
            f"{self.kind} {self.location} 握手失败：{last}"
            "（确认串口与波特率 1M、板卡已上电并运行固件、串口未被其他程序占用；"
            "若刚重建/重烧过固件，先在本目录跑 python sync_kernel.py 同步内核副本）") from last

    def _check_capabilities(self, client):
        """按设备回包的能力位校验所选后端，不匹配就带着可操作提示快速失败。

        在发送任何动作之前做，避免"锁步后端连到真实传感板卡"这种组合先跑一串
        STEP 才在 STATE 上报错——那样既慢又难定位。
        """
        required = _BACKEND_REQUIREMENTS.get(self.kind)
        if required is None:
            return
        bit, hint = required
        capabilities = client.identity.get("capabilities", 0)
        if p.has_capability(capabilities, bit):
            return
        board = client.identity.get("board", "未知板")
        names = "、".join(client.identity.get("cap_names") or []) or "无"
        raise ConnectionError(
            f"{self.kind} 后端与设备能力不匹配：{board} 上报能力 {capabilities:#010x}（{names}），"
            f"缺少「{p.CAP_LABELS[bit]}」。{hint}")

    def _read_params(self, client):
        """回读全部参数，容错降级。

        固件参数表与上位机不一致时（例如固件漏了某个 id），逐个 READ 会被
        设备拒成 BAD_ID——旧写法会让整条回读抛错、连接判死。这里改成：被拒的
        那几项显示"设备拒绝读取"，连接保持，观测与动作照常可用，只挡住提交。
        """
        reader = getattr(client, "read_params_safe", None)
        if reader is None:                      # 旧内核没有容错读参
            values, failures = client.read_params(), []
        else:
            values, failures = reader()
        self.param_failures = failures
        count = getattr(client, "param_count", len(PARAMS))
        if count < len(PARAMS):
            unsupported = "、".join(
                f"id{item['id']}({item['key']})" for item in PARAMS[count:])
            self._event({"type": "message", "text":
                         f"⚠ 旧参数表固件（schema 0x{client.identity.get('schema', 0):08x}，"
                         f"{count}/{len(PARAMS)} 参）：{unsupported}"
                         f" 不可读写、提交时忽略；观测与动作命令不受影响。"})
        if failures:
            detail = "；".join(f"id{pid}({key}) {msg}" for pid, key, msg in failures)
            self._event({"type": "message", "text":
                         f"⚠ {len(failures)}/{len(PARAMS)} 个参数设备拒绝读取——上位机与固件的"
                         f"参数表实现不一致（固件 build 0x{client.identity.get('build', 0):08x}）。"
                         f"连接保持，观测与命令可用；提交配置前请先更新固件。明细：{detail}"})
        return values

    def _run(self):
        device = None
        try:
            device = self._open_device()
            self.label = device.label
            self.readonly = device.readonly
            client = Client(device, self._sample, self._event)
            self._handshake(client)
            self.identity = client.identity          # 先落身份：能力校验失败也能显示板名
            self._check_capabilities(client)
            stepping = hasattr(device, "step_and_integrate")   # 锁步 HIL
            if stepping:
                # STEP 是唯一节拍来源，模式迁移必须夹在 STEP 之间：CLEAR 只清
                # fault_latch（当拍 FSM 仍看到旧 fault 停在 SAFE），离开 SAFE 需要
                # 一次停机类命令，START 又要求已处于 IDLE。顺序见迁移说明 §10。
                def hold(count):
                    device.run(client, count)
                hold(4)
                client.request(p.CLEAR)
                hold(4)
                client.request(p.STOP)
                hold(4)
            if not device.readonly:
                self.active_params = self._read_params(client)
                self.revision = client.revision
            self.connected = True
            self._event({"type": "message", "text": f"已连接：{device.label}"})
            while not self._shutdown:
                started = time.monotonic()
                for _ in range(20):
                    if not self._cmds:
                        break
                    with self._cv:
                        item = self._cmds.popleft()
                    self._dispatch(client, device, item)
                    if self._shutdown:
                        break
                if stepping:
                    device.step_and_integrate(client)   # 一拍：传感器→duty→模型积分
                else:
                    client.poll(10)
                    if self._demo:
                        self._advance_demo(client, device)
                    if self._swing_rec:
                        self._advance_swing(client, device)
                    if self._bat_probe:
                        self._advance_bat(client)
                if self._recorder:
                    self._recorder.flush()
                    if self._rec_csv:
                        self._rec_csv.flush()
                if not stepping:
                    remain = 0.01 - (time.monotonic() - started)
                    if remain > 0:
                        time.sleep(remain)
        except Exception as exc:      # 控制线程异常即断连
            self.error = str(exc)
            self._event({"type": "failed", "text": str(exc)})
        finally:
            self.connected = False
            with self._cv:           # 排队未执行的命令立即失败，不等超时
                pending = list(self._cmds)
                self._cmds.clear()
            for item in pending:
                item[1](_disconnected)
            # 走统一收尾：写 record_end 摘要再关文件（原先直接 close()，
            # 记录里既没有结束标记也没有采样完整度，事后无法判断是否漏帧）
            self._close_recorder("控制线程退出")
            if device:
                device.close()
            self._event({"type": "message", "text": "连接已关闭"})

    def _dispatch(self, client, device, item):
        _, fn, name, value, source = item
        fn(lambda: self._execute(client, device, name, value, source))
        if not self.connected:
            self._shutdown = True

    def _execute(self, client, device, name, value, source):
        if name == "read":
            if self.readonly:
                raise PermissionError("回放只读，没有可读的设备参数")
            self.active_params = self._read_params(client)
            self.revision = client.revision
            return self.active_params
        if name == "apply":
            self.active_params = client.apply(value)
            self.revision = client.revision
            self._event({"type": "message", "text": f"配置 v{self.revision} 已成组生效并回读"})
            return self.active_params
        if name == "defaults":
            self.active_params = client.defaults()
            self.revision = client.revision
            return self.active_params
        if name == "io_status":
            return client.io_status()
        if name == "io_test":
            enable, duty = value
            client.io_test(enable, duty)
            self._event({"type": "message", "text":
                         f"I/O 点动命令已确认：{'使能' if enable else '释放'}，duty={duty}"})
            return None
        if name == "read_one":
            index = value
            return struct.unpack("<i", client.request(p.READ, struct.pack("<H", index)))[0]
        if name == "record":
            if self._recorder:
                self._close_recorder("手动停止记录")
            if value:
                # 混合记录（2026-10-29）：遥测进 CSV（FIELDS 固定 28 列，体积约为
                # JSONL 的 1/4，可直接进 Excel/pandas），指令/参数/备注/结束摘要进
                # 同名 _events.jsonl（嵌套结构一行一条）。tick 是两份文件的共用
                # 对齐键。脚本旧习惯传 .jsonl 结尾的路径也兼容：剥掉当基础名。
                base = Path(value)
                if base.suffix in (".jsonl", ".csv"):
                    base = base.with_suffix("")
                csv_path = base.parent / (base.name + ".csv")
                events_path = base.parent / (base.name + "_events.jsonl")
                base.parent.mkdir(parents=True, exist_ok=True)
                self._rec_csv = csv_path.open("w", encoding="utf-8", newline="")
                self._rec_csv.write(",".join(p.FIELDS) + "\n")
                self._recorder = events_path.open("w", encoding="utf-8")
                self._rec_path = events_path
                self._rec_csv_path = csv_path
                self._rec_stats = {"samples": 0, "first_tick": None, "last_tick": None,
                                   "modes": {}, "events": {}}
                self._event(self._metadata_record())
                return {"csv": str(csv_path), "events": str(events_path)}
            return ""
        if name == "command":
            kind, data = value
            try:
                extra = client.request(kind, data)
            except CommandError as exc:
                # 固件拒绝动作（STATE 门：未标定/姿态窗/故障锁存）是调参常态而
                # 不是静默失败——补一条点名哪道门的日志（响应事件本身已进记录），
                # 异常继续向调用方传播，保持 GUI"失败：…"与 Agent 报错的原语义。
                hint = ""
                latest = getattr(self, "latest", None) or {}
                if latest.get("calib_done") == 0 and kind in (p.START, p.BALANCE):
                    hint = "；REQUIRE_CALIB：摆杆未做下垂零位标定，先在「标定」页执行零位标定"
                elif kind == p.CALIBRATE:
                    hint = "；标定要求设备处于待机/安全态且无配置锁"
                self._event({"type": "message", "text":
                             f"{_CMD_NAMES.get(kind, hex(kind))}被固件拒绝：{exc}{hint}"})
                raise
            if kind == p.STOP:
                self._demo = None
                self._swing_rec = None
                self._bat_probe = None
            return {"kind": kind, "code": 0, "revision": client.revision, "extra": extra.hex()}
        raise ValueError(f"未知命令 {name}")

    # ---------- 回调 ----------
    def _sample(self, sample):
        with self._lock:
            self.history.append(sample)
        self.latest = sample
        self.revision = sample["revision"]
        with self._lock:
            hooks = list(self._sample_hooks)
        for hook in hooks:
            try:
                hook(sample)
            except Exception:
                pass
        if self._recorder:
            failed = False
            with self._rec_lock:
                if not self._recorder:
                    return
                try:
                    self._rec_csv.write(",".join(
                        "" if sample[k] is None else str(sample[k])
                        for k in p.FIELDS) + "\n")
                except Exception:
                    failed = True
                else:
                    stats = self._rec_stats
                    if stats is not None:
                        stats["samples"] += 1
                        if stats["first_tick"] is None:
                            stats["first_tick"] = sample["tick"]
                        stats["last_tick"] = sample["tick"]
                        name = MODES.get(sample["mode"], str(sample["mode"]))
                        stats["modes"][name] = stats["modes"].get(name, 0) + 1
            if failed:
                self._rec_write_failed()

    # ---------- 记录文件：元数据 / 结束摘要 / 人工备注 ----------
    # 排查一次掉摆要同时回答四个问题：跑的是哪个固件、用的是哪套参数、
    # 谁在什么时刻下了什么命令、控制器的中间量是多少。旧记录只答得出
    # 第二、四条的前半段（active_params 为原始整数列表、唯一的固件标识是
    # 一个写死的日期常量），命令有记录但没有任何时间戳，GUI 日志一行都不落盘。
    _RECORDED_EVENTS = ("command", "response", "metadata", "message", "failed",
                        "note", "record_end", "identify")

    def _metadata_record(self):
        """记录开头的一次性快照：能独立回答"这条数据是什么条件下采的"。"""
        latest = self.latest or {}
        params = self.active_params
        detail = []
        if params is not None:
            for item, value in zip(PARAMS, params):
                detail.append({"id": item["id"], "key": item["key"],
                               "label": item.get("label", item["key"]),
                               "value": value, "scale": item.get("scale"),
                               "unit": item.get("unit", "")})
        return {
            "type": "metadata", "version": 3, "created": datetime.now().isoformat(),
            "device": self.label, "schema": SCHEMA_ID, "identity": self.identity,
            "parameters": params,               # 原始列表：旧分析脚本照旧可读
            "parameters_detail": detail,         # 带标签/单位：人可直接核对
            "revision": self.revision,
            # 零点在记录开始时已经是多少——以前只能从第一节 sample 里捞，
            # 而 offset 在 CAL 之后会变，需要有一个明确的"起始值"。
            "calibration": {
                "offset_c01": latest.get("offset"),
                "alpha_trim_c01": (params[13] if params and len(params) > 13 else None),
                "calib_done": latest.get("calib_done"),
                "alpha_deg": (latest.get("alpha") / 100 if latest.get("alpha") is not None else None),
            },
            "firmware": {
                # 注意：build 戳的高 8 位是板 ID，低 24 位是**写死的** BUILD_DATE
                # 常量（tools/generate_debug_config.py），所以这里的 date 不能当
                # 构建日期用——真正的区分靠 host_git 和人工填的 bitstream 字段。
                "build_stamp": (self.identity or {}).get("build"),
                "build_stamp_date": (self.identity or {}).get("date"),
                "build_stamp_note": "date 字段来自固件里的常量，不随构建变化",
                "capabilities": (self.identity or {}).get("capabilities"),
                "host_git": _git_head(ROOT),
                "host_git_branch": _git_branch(ROOT),
            },
            "app": "j280wb",
            # 遥测在 <基础名>.csv、事件在 <基础名>_events.jsonl（host/records.py 读）
            "record_format": "csv+events",
        }

    def _close_recorder(self, reason):
        """停记录：先写结束摘要（采样完整度、模式直方图），再关文件。"""
        if not self._recorder:
            return
        stats = self._rec_stats or {}
        first, last = stats.get("first_tick"), stats.get("last_tick")
        span = (last - first) if (first is not None and last is not None) else 0
        expected = span // 2 + 1 if span > 0 else 0     # 遥测 2ms 一帧
        summary = {
            "type": "record_end", "created": datetime.now().isoformat(),
            "reason": reason, "path": str(self._rec_path or ""),
            "csv": str(self._rec_csv_path or ""),
            "samples": stats.get("samples", 0),
            "first_tick": first, "last_tick": last,
            "span_ms": span,
            "expected_samples": expected,
            "missing_ratio": (round(1 - stats.get("samples", 0) / expected, 6)
                              if expected else None),
            "mode_histogram": stats.get("modes", {}),
        }
        try:
            with self._rec_lock:
                if self._recorder:
                    self._recorder.write(json.dumps(summary, ensure_ascii=False) + "\n")
                    self._recorder.close()
                    self._recorder = None
                    if self._rec_csv:
                        self._rec_csv.close()
                        self._rec_csv = None
                    self._rec_csv_path = None
                    self._rec_stats = None
        except Exception:
            self._recorder = None
            self._rec_csv = None
            self._rec_csv_path = None
            self._rec_stats = None
        self._event({"type": "message", "text":
                     f"记录已停止（{reason}）：{summary['samples']} 帧 / {span/1000:.1f}s 跨度"})

    def note(self, text, level="info"):
        """GUI 主日志里的每一行都进 JSONL，事后能和命令/遥测对时间轴。"""
        self._event({"type": "note", "level": level, "text": str(text)})

    def _rec_write_failed(self):
        """在锁外调用：提示一次并放弃记录，绝不抛出。"""
        if self._rec_open_warned:
            return
        self._rec_open_warned = True
        self._recorder = None
        self._rec_csv = None
        self._rec_csv_path = None
        self._rec_stats = None
        self._event({"type": "failed", "text": "记录写入失败，已停止记录"})

    def _event(self, entry):
        with self._lock:
            hooks = list(self._event_hooks)
        for hook in hooks:
            try:
                hook(entry)
            except Exception:
                pass
        if self._recorder and entry.get("type") in self._RECORDED_EVENTS:
            record = dict(entry)
            record.setdefault("time", datetime.now().isoformat())
            if self.latest:
                record.setdefault("tick", self.latest.get("tick"))
            failed = False
            with self._rec_lock:
                if not self._recorder:
                    return
                try:
                    self._recorder.write(json.dumps(record, ensure_ascii=False) + "\n")
                    stats = self._rec_stats
                    if stats is not None:
                        key = str(record.get("type"))
                        stats["events"][key] = stats["events"].get(key, 0) + 1
                except Exception:
                    failed = True
            if failed:                     # 锁内不重入 _event，出锁后再提示
                self._rec_write_failed()

    # ---------- 自动试验（与 host.app 相同口径） ----------
    def run_trial(self, source, delta_cdeg):
        if self.kind == "锁步HIL":
            raise ValueError("自动试验暂不支持锁步后端，请用 python -m host.acceptance --hil")
        last = self.latest
        if last is None or last["mode"] not in (0, 4):
            raise ValueError("自动试验从待机/安全状态开始")
        self.submit("command", (p.CLEAR, b""), source=source)
        self.submit("command", (p.START, b""), source=source)
        self._demo = {"phase": "settle", "start": self.latest["tick"], "delta": delta_cdeg}
        self._event({"type": "message", "text": "试验开始：起摆→8s稳摆→相对定位→17s观察"})

    def _advance_demo(self, client, device):
        s = client.last_sample
        if s is None:
            return
        job = self._demo
        # move 阶段 mode==2 合法：MOVE 受理后固件排队、执行拍才切 POS
        #（新固件语义，2026-10-24），期间仍处于稳摆态不算掉摆。
        if s["fault"] or (job["phase"] == "move" and s["mode"] not in (2, 3)):
            self._demo = None
            client.request(p.STOP)
            self._event({"type": "failed", "text": "自动试验中断：故障或掉摆，已停止"})
            return
        if job["phase"] == "settle" and s["tick"] - job["start"] >= 8000:
            # 2026-10-24 适配：新固件稳摆 2.5s 后自动进 POS（AUTO_POS_MS），8s 点
            # 设备通常已在 mode 3；α/θ̇ 准入检查删除——MOVE 已改为"立即受理+
            # 固件内排队执行"，不再被状态门拒绝（门只影响执行时机）。
            if s["mode"] not in (2, 3):
                self._demo = None
                client.request(p.STOP)
                self._event({"type": "failed", "text": "8s 未起摆成功（非稳摆/定位态），试验已停止"})
                return
            client.request(p.MOVE, struct.pack("<i", job["delta"]))
            job.update(phase="move", start=s["tick"], peak=0.0, late=0.0)
        elif job["phase"] == "move":
            elapsed = s["tick"] - job["start"]
            job["peak"] = max(job["peak"], abs(s["alpha"]) / 100)
            if elapsed >= 15000:
                job["late"] = max(job["late"], abs(s["theta"] - s["target"]) / 100)
            if elapsed >= 17000:
                self._demo = None
                report = {"type": "trial", "delta": job["delta"], "moving_alpha": job["peak"],
                          "late_error": job["late"], "pass": bool(job["peak"] < 3 and job["late"] < 2)}
                self._event(report)
                client.request(p.STOP)

    # ---------- 竞赛录制分镜控制（2026-11-03，导演式：每功能独立触发）----------
    # 每个按钮=一个"镜头"：起摆（自适应甩距重试）/定位步进/扰动观察。
    # swing_rec 只负责"把摆稳住"这一件事；位置/速度展示由 GUI 直接发 MOVE。
    VIDEO_BASE = [17751, 10000, 600, 5300, 640, 740, 8800, 1500,
                  9000, 3000, 11800, 1200, 12000, 0]

    def demo_swing(self, source):
        """镜头1：起摆。甩幅 8800 起，失败 +250 自适应重试（≤4 次，≤9800）。
        成功后停在稳摆态（BALANCE），供后续镜头/展示任意长时间。
        命令全部异步入队（wait=False）——本函数可能在 GUI 线程被调，
        同步等桥线程会让 GUI 卡 6s；也避免与桥线程自提交互锁。"""
        if self.kind == "锁步HIL":
            raise ValueError("录制分镜不支持锁步后端")
        last = self.latest
        if last is None or last["mode"] not in (0, 4):
            raise ValueError("起摆镜头从待机/安全状态开始")
        self.submit("apply", list(self.VIDEO_BASE), source=source, wait=False)
        self.submit("command", (p.CLEAR, b""), source=source, wait=False)
        self.submit("command", (p.START, b""), source=source, wait=False)
        self._swing_rec = {"attempt": 1, "sw_amp": 8800,
                           "phase_start": self.latest["tick"]}
        self._event({"type": "message", "text": "镜头1 起摆：甩幅 8800（自适应重试≤4 次）"})

    def demo_move(self, source, delta_cdeg):
        """镜头2/4：稳摆/定位态发一步相对 MOVE。由 GUI 直接调用（异步）。"""
        self.submit("command", (p.MOVE, struct.pack("<i", int(delta_cdeg))),
                    source=source, wait=False)

    def demo_force_pos(self, source, delta_cdeg=0):
        """镜头3：BAL 态强制切 POS（绕过 pos_ready；delta=0 就地保持）。
        固件侧只保留 |α|<15° 安全界，入场瞬态由 1.5s 稳定窗吸收。"""
        self.submit("command", (p.FORCE_POS, struct.pack("<i", int(delta_cdeg))),
                    source=source, wait=False)

    def demo_vel(self, source, enabled, v_01dps=300, a=3):
        """镜头3：速度曲线演示开/关。v_01dps=0.1°/s（300=30°/s），
        a=0.1°/s 每 10ms（3=30°/s²）。固件内生三角波 v_ref，LQR 跟踪。"""
        payload = struct.pack("<Hhh", 1 if enabled else 0, int(v_01dps), int(a))
        self.submit("command", (p.VEL_DEMO, payload), source=source, wait=False)

    def stop_video(self, source):
        self._swing_rec = None
        self.submit("command", (p.STOP, b""), source=source, wait=False)
        self._event({"type": "message", "text": "分镜已停止"})

    # ---------- 电量探针 + K4 适配（2026-11-04 夜）----------
    # 依据：IO_TEST +800 duty 0.9s 的臂速 v 与电池电量强相关（满电 ~74°/s、
    # 60% ~44.6°/s，20261102_225822 / 11-3 19:09 实测）；满电 K4 甜点 14000、
    # 60% 甜点 8000（11-4 凌晨扫描）⇒ 线性内插 K4 ≈ 204·v − 1100，截 [6500,13000]。
    # 用户在满电定 11000（离 16000 悬崖留余量），故满电插值 ~14000 会被提示但
    # 应用值再压到 ≤12000。
    def battery_probe(self, source, apply_k4=False):
        """SAFE/IDLE 态一键电量探针：+800 duty 脉冲测臂速 → 电量% → K4 建议。
        apply_k4=True 时把建议值写入 POS K4（id12，成组提交+回读）。"""
        if self.kind == "锁步HIL":
            raise ValueError("电量探针不支持锁步后端")
        last = self.latest
        if last is None or last.get("mode") not in (0, 4):
            raise ValueError("电量探针需待机/安全态（先 STOP）")
        self._bat_probe = {"phase": "arm", "apply": bool(apply_k4),
                           "t0": 0, "td": []}
        self._event({"type": "message", "text": "电量探针：0.9s +800 脉冲（臂会转动，确认前方空间）"})

    def _advance_bat(self, client):
        rec = self._bat_probe
        try:
            if rec["phase"] == "arm":
                client.request(p.STOP)
                time.sleep(0.4)
                client.request(p.CLEAR)
                time.sleep(0.3)
                client.request(p.IO_TEST, struct.pack("<hH", 800, 1))
                rec["t0"] = (client.last_sample or {}).get("tick") or 0
                rec["td"] = []
                rec["phase"] = "pulse"
            elif rec["phase"] == "pulse":
                s = client.last_sample or {}
                if s.get("theta_dot") is not None:
                    rec["td"].append(s["theta_dot"])
                if s.get("tick", 0) - rec["t0"] >= 900:
                    client.request(p.IO_TEST, struct.pack("<hH", 0, 0))
                    rec["phase"] = "calc"
            elif rec["phase"] == "calc":
                td = rec["td"]
                tail = td[int(len(td) * 0.6):]
                v = (sum(tail) / len(tail) / 10.0) if tail else 0.0
                self._bat_probe = None
                if v < 15:
                    self._event({"type": "battery", "v": v, "ok": False,
                                 "text": "电量探针无效（v=%.1f°/s，摆杆未下垂或臂被挡）" % v})
                    return
                pct = min(100, max(30, int((v - 44.6) / (74.0 - 44.6) * 40 + 60)))
                k4 = min(13000, max(6500, int(204 * v - 1100)))
                applied = False
                if rec["apply"]:
                    params = list(self.active_params)
                    if len(params) > 12 and params[12] is not None:
                        params[12] = k4
                        client.apply(params)
                        applied = True
                self._event({"type": "battery", "v": round(v, 1), "pct": pct,
                             "k4": k4, "applied": applied, "ok": True,
                             "text": "电量 ≈%d%%（v=%.1f°/s）→ POS K4 建议 %d%s" %
                                     (pct, v, k4, "，已写入" if applied else "")})
        except Exception as exc:
            self._bat_probe = None
            try:
                client.request(p.IO_TEST, struct.pack("<hH", 0, 0))
            except Exception:
                pass
            self._event({"type": "failed", "text": "电量探针失败：%s" % exc})

    def _advance_swing(self, client, device):
        """桥线程内每 tick 调用：重试逻辑里所有命令【直接 client.request】
        （同线程顺序执行，不经命令队列），并且每个 request 都必须吞异常——
        任何抛出都会沿 _run 的 while 逃逸杀死桥线程（表现为 GUI 断连）。
        此前版本在桥线程内调 submit(wait=True) 自等自家队列 = 必死锁 6s。"""
        s = client.last_sample
        if s is None:
            return
        rec = self._swing_rec
        tick = s["tick"]
        elapsed = tick - rec["phase_start"]

        if s["mode"] == 1:
            rec["phase_start"] = tick          # SWING 中刷新计时
            return
        if s["mode"] == 2:
            self._swing_rec = None
            self._event({"type": "swing_ok", "text":
                         "稳摆成立 ✓（第 %d 次尝试，甩幅 %d）——可进行扰动/定位镜头" %
                         (rec["attempt"], rec["sw_amp"])})
            return
        if s["mode"] == 3:
            self._swing_rec = None
            self._event({"type": "swing_ok", "text": "已进入定位态（自动进 POS 生效）"})
            return
        # 失败（超时/故障/掉摆锁存）
        if elapsed >= 60000 or s["fault"] or s["mode"] in (0, 4):
            if rec["attempt"] >= 4:
                self._swing_rec = None
                try:
                    client.request(p.STOP)
                except Exception:
                    pass
                self._event({"type": "failed", "text":
                             "起摆 4 次失败（甩幅已到 %d）——请检查电池/机械" % rec["sw_amp"]})
                return
            rec["attempt"] += 1
            rec["sw_amp"] = min(9800, rec["sw_amp"] + 250)
            params = list(self.VIDEO_BASE)
            params[6] = rec["sw_amp"]
            try:
                client.request(p.STOP)
                client.apply(params)
                client.request(p.CLEAR)
                client.request(p.START)
                rec["phase_start"] = client.last_sample["tick"]
                self._event({"type": "message", "text":
                             "起摆重试 %d：甩幅→%d（电压自适应）" % (rec["attempt"], rec["sw_amp"])})
            except Exception as exc:
                # 重试链路任一环失败：终止编排（不杀桥线程），交还 GUI
                self._swing_rec = None
                try:
                    client.request(p.STOP)
                except Exception:
                    pass
                self._event({"type": "failed", "text": "自适应重试失败：%s" % exc})

    # ---------- 回调 ----------
    def _sample(self, sample):
        with self._lock:
            self.history.append(sample)
        self.latest = sample
        self.revision = sample["revision"]
        with self._lock:
            hooks = list(self._sample_hooks)
        for hook in hooks:
            try:
                hook(sample)
            except Exception:
                pass
        if self._recorder:
            failed = False
            with self._rec_lock:
                if not self._recorder:
                    return
                try:
                    self._rec_csv.write(",".join(
                        "" if sample[k] is None else str(sample[k])
                        for k in p.FIELDS) + "\n")
                except Exception:
                    failed = True
                else:
                    stats = self._rec_stats
                    if stats is not None:
                        stats["samples"] += 1
                        if stats["first_tick"] is None:
                            stats["first_tick"] = sample["tick"]
                        stats["last_tick"] = sample["tick"]
                        name = MODES.get(sample["mode"], str(sample["mode"]))
                        stats["modes"][name] = stats["modes"].get(name, 0) + 1
            if failed:
                self._rec_write_failed()

    # ---------- 记录文件：元数据 / 结束摘要 / 人工备注 ----------
    # 排查一次掉摆要同时回答四个问题：跑的是哪个固件、用的是哪套参数、
    # 谁在什么时刻下了什么命令、控制器的中间量是多少。旧记录只答得出
    # 第二、四条的前半段（active_params 为原始整数列表、唯一的固件标识是
    # 一个写死的日期常量），命令有记录但没有任何时间戳，GUI 日志一行都不落盘。
    _RECORDED_EVENTS = ("command", "response", "metadata", "message", "failed",
                        "note", "record_end", "identify")

    def _metadata_record(self):
        """记录开头的一次性快照：能独立回答"这条数据是什么条件下采的"。"""
        latest = self.latest or {}
        params = self.active_params
        detail = []
        if params is not None:
            for item, value in zip(PARAMS, params):
                detail.append({"id": item["id"], "key": item["key"],
                               "label": item.get("label", item["key"]),
                               "value": value, "scale": item.get("scale"),
                               "unit": item.get("unit", "")})
        return {
            "type": "metadata", "version": 3, "created": datetime.now().isoformat(),
            "device": self.label, "schema": SCHEMA_ID, "identity": self.identity,
            "parameters": params,               # 原始列表：旧分析脚本照旧可读
            "parameters_detail": detail,         # 带标签/单位：人可直接核对
            "revision": self.revision,
            # 零点在记录开始时已经是多少——以前只能从第一节 sample 里捞，
            # 而 offset 在 CAL 之后会变，需要有一个明确的"起始值"。
            "calibration": {
                "offset_c01": latest.get("offset"),
                "alpha_trim_c01": (params[13] if params and len(params) > 13 else None),
                "calib_done": latest.get("calib_done"),
                "alpha_deg": (latest.get("alpha") / 100 if latest.get("alpha") is not None else None),
            },
            "firmware": {
                # 注意：build 戳的高 8 位是板 ID，低 24 位是**写死的** BUILD_DATE
                # 常量（tools/generate_debug_config.py），所以这里的 date 不能当
                # 构建日期用——真正的区分靠 host_git 和人工填的 bitstream 字段。
                "build_stamp": (self.identity or {}).get("build"),
                "build_stamp_date": (self.identity or {}).get("date"),
                "build_stamp_note": "date 字段来自固件里的常量，不随构建变化",
                "capabilities": (self.identity or {}).get("capabilities"),
                "host_git": _git_head(ROOT),
                "host_git_branch": _git_branch(ROOT),
            },
            "app": "j280wb",
            # 遥测在 <基础名>.csv、事件在 <基础名>_events.jsonl（host/records.py 读）
            "record_format": "csv+events",
        }

    def _close_recorder(self, reason):
        """停记录：先写结束摘要（采样完整度、模式直方图），再关文件。"""
        if not self._recorder:
            return
        stats = self._rec_stats or {}
        first, last = stats.get("first_tick"), stats.get("last_tick")
        span = (last - first) if (first is not None and last is not None) else 0
        expected = span // 2 + 1 if span > 0 else 0     # 遥测 2ms 一帧
        summary = {
            "type": "record_end", "created": datetime.now().isoformat(),
            "reason": reason, "path": str(self._rec_path or ""),
            "csv": str(self._rec_csv_path or ""),
            "samples": stats.get("samples", 0),
            "first_tick": first, "last_tick": last,
            "span_ms": span,
            "expected_samples": expected,
            "missing_ratio": (round(1 - stats.get("samples", 0) / expected, 6)
                              if expected else None),
            "mode_histogram": stats.get("modes", {}),
        }
        try:
            with self._rec_lock:
                if self._recorder:
                    self._recorder.write(json.dumps(summary, ensure_ascii=False) + "\n")
                    self._recorder.close()
                    self._recorder = None
                    if self._rec_csv:
                        self._rec_csv.close()
                        self._rec_csv = None
                    self._rec_csv_path = None
                    self._rec_stats = None
        except Exception:
            self._recorder = None
            self._rec_csv = None
            self._rec_csv_path = None
            self._rec_stats = None
        self._event({"type": "message", "text":
                     f"记录已停止（{reason}）：{summary['samples']} 帧 / {span/1000:.1f}s 跨度"})

    def note(self, text, level="info"):
        """GUI 主日志里的每一行都进 JSONL，事后能和命令/遥测对时间轴。"""
        self._event({"type": "note", "level": level, "text": str(text)})

    def _rec_write_failed(self):
        """在锁外调用：提示一次并放弃记录，绝不抛出。"""
        if self._rec_open_warned:
            return
        self._rec_open_warned = True
        self._recorder = None
        self._rec_csv = None
        self._rec_csv_path = None
        self._rec_stats = None
        self._event({"type": "failed", "text": "记录写入失败，已停止记录"})

    def _event(self, entry):
        with self._lock:
            hooks = list(self._event_hooks)
        for hook in hooks:
            try:
                hook(entry)
            except Exception:
                pass
        if self._recorder and entry.get("type") in self._RECORDED_EVENTS:
            record = dict(entry)
            record.setdefault("time", datetime.now().isoformat())
            if self.latest:
                record.setdefault("tick", self.latest.get("tick"))
            failed = False
            with self._rec_lock:
                if not self._recorder:
                    return
                try:
                    self._recorder.write(json.dumps(record, ensure_ascii=False) + "\n")
                    stats = self._rec_stats
                    if stats is not None:
                        key = str(record.get("type"))
                        stats["events"][key] = stats["events"].get(key, 0) + 1
                except Exception:
                    failed = True
            if failed:                     # 锁内不重入 _event，出锁后再提示
                self._rec_write_failed()

    # ---------- 自动试验（与 host.app 相同口径） ----------
    def run_trial(self, source, delta_cdeg):
        if self.kind == "锁步HIL":
            raise ValueError("自动试验暂不支持锁步后端，请用 python -m host.acceptance --hil")
        last = self.latest
        if last is None or last["mode"] not in (0, 4):
            raise ValueError("自动试验从待机/安全状态开始")
        self.submit("command", (p.CLEAR, b""), source=source)
        self.submit("command", (p.START, b""), source=source)
        self._demo = {"phase": "settle", "start": self.latest["tick"], "delta": delta_cdeg}
        self._event({"type": "message", "text": "试验开始：起摆→8s稳摆→相对定位→17s观察"})

    def _advance_demo(self, client, device):
        s = client.last_sample
        if s is None:
            return
        job = self._demo
        # move 阶段 mode==2 合法：MOVE 受理后固件排队、执行拍才切 POS
        #（新固件语义，2026-10-24），期间仍处于稳摆态不算掉摆。
        if s["fault"] or (job["phase"] == "move" and s["mode"] not in (2, 3)):
            self._demo = None
            client.request(p.STOP)
            self._event({"type": "failed", "text": "自动试验中断：故障或掉摆，已停止"})
            return
        if job["phase"] == "settle" and s["tick"] - job["start"] >= 8000:
            # 2026-10-24 适配：新固件稳摆 2.5s 后自动进 POS（AUTO_POS_MS），8s 点
            # 设备通常已在 mode 3；α/θ̇ 准入检查删除——MOVE 已改为"立即受理+
            # 固件内排队执行"，不再被状态门拒绝（门只影响执行时机）。
            if s["mode"] not in (2, 3):
                self._demo = None
                client.request(p.STOP)
                self._event({"type": "failed", "text": "8s 未起摆成功（非稳摆/定位态），试验已停止"})
                return
            client.request(p.MOVE, struct.pack("<i", job["delta"]))
            job.update(phase="move", start=s["tick"], peak=0.0, late=0.0)
        elif job["phase"] == "move":
            elapsed = s["tick"] - job["start"]
            job["peak"] = max(job["peak"], abs(s["alpha"]) / 100)
            if elapsed >= 15000:
                job["late"] = max(job["late"], abs(s["theta"] - s["target"]) / 100)
            if elapsed >= 17000:
                self._demo = None
                report = {"type": "trial", "delta": job["delta"], "moving_alpha": job["peak"],
                          "late_error": job["late"], "pass": bool(job["peak"] < 3 and job["late"] < 2)}
                self._event(report)
                client.request(p.STOP)

    # ---------- 竞赛视频录制状态机（2026-11-03）----------
    # 借鉴 run_trial 的 tick 驱动 demo 模式与起摆线"自适应甩距"思想：
    # 电压/机械状态漂移 → 起摆失败时甩幅按 5% 步进自适应加大重试（≤3 次），
    # 成功率优先且每轮起始参数回安全基线。
    VIDEO_BASE = [17751, 10000, 600, 5300, 640, 740, 8800, 1500,
                  9000, 3000, 11800, 1200, 12000, 0]
    STEP_C01 = 500            # 每步 5°
    SLOW_GAP_TICKS = 1600     # 慢档步间延时（速度曲线对比档）
    SETTLE_TOL = 250          # |θe| 到位判据

    def run_video(self, source, slow_gap_ms=SLOW_GAP_TICKS):
        if self.kind == "锁步HIL":
            raise ValueError("录制状态机不支持锁步后端")
        last = self.latest
        if last is None or last["mode"] not in (0, 4):
            raise ValueError("录制从待机/安全状态开始")
        self.submit("apply", list(self.VIDEO_BASE), source=source)
        self.submit("command", (p.CLEAR, b""), source=source)
        self.submit("command", (p.START, b""), source=source)
        self._video = {
            "phase": "swing", "attempt": 1, "sw_amp": 8800,
            "phase_start": self.latest["tick"], "step_idx": 0, "dir_sign": +1,
            "steps": [], "slow_gap": slow_gap_ms, "alpha_bal_peak": 0.0,
        }
        self._event({"type": "message", "text": "录制编排开始：起摆(自适应≤3)→稳摆22s→快档+15°→保持8s→慢档-15°→收尾"})

    def stop_video(self, source):
        self._video = None
        self.submit("command", (p.STOP, b""), source=source)
        self._event({"type": "message", "text": "录制编排已停止"})

    def _advance_video(self, client, device):
        s = client.last_sample
        if s is None:
            return
        job = self._video
        tick = s["tick"]
        elapsed = tick - job["phase_start"]
        phase = job["phase"]

        # ---- 起摆/稳摆段：自适应重试 ----
        if phase == "swing":
            if s["mode"] == 1:
                job["phase_start"] = tick          # SWING 中刷新计时
                job["alpha_bal_peak"] = 0.0
                return
            if s["mode"] == 2:
                # 稳摆计时 22s（tick 口径）
                job["alpha_bal_peak"] = max(job["alpha_bal_peak"], abs(s["alpha"]) / 100)
                if elapsed >= 22000:
                    job.update(phase="calm", phase_start=tick)
                    self._event({"type": "message", "text": "稳摆 22s 达成，等臂平静窗口…"})
                return
            # SWING 超时（60s）或掉摆锁存 → 重试，甩幅 +5% 步进（电压自适应）
            if elapsed >= 60000 or s["fault"]:
                if job["attempt"] >= 3:
                    self._video = None
                    client.request(p.STOP)
                    self._event({"type": "failed", "text": "起摆 3 次失败（甩幅已自适应到 %d），编排终止" % job["sw_amp"]})
                    return
                job["attempt"] += 1
                job["sw_amp"] = min(9500, job["sw_amp"] + 250)
                params = list(self.VIDEO_BASE)
                params[6] = job["sw_amp"]
                client.request(p.STOP)
                self.submit("apply", params, source="video")
                self.submit("command", (p.CLEAR, b""), source="video")
                self.submit("command", (p.START, b""), source="video")
                job["phase_start"] = self.latest["tick"] if self.latest else tick
                self._event({"type": "message", "text": "起摆重试 %d：甩幅→%d（电压自适应）" % (job["attempt"], job["sw_amp"])})
            return

        # 任一 POS 段掉摆 → 编排失败终止
        if phase in ("calm", "move_fast", "hold", "move_slow", "final") and s["fault"]:
            self._video = None
            client.request(p.STOP)
            self._event({"type": "failed", "text": "POS 段掉摆，编排终止（可重拍）"})
            return

        if phase == "calm":
            # 等臂平静：|θ̇|<5°/s 且 |α|<3° 连续 0.5s
            quiet = abs(s["theta_dot"]) < 50 and abs(s["alpha"]) < 300
            job["calm_cnt"] = job.get("calm_cnt", 0) + 1 if quiet else 0
            if job["calm_cnt"] >= 250 or elapsed >= 20000:
                client.request(p.MOVE, struct.pack("<i", self.STEP_C01))
                job.update(phase="move_fast", phase_start=tick, step_idx=1)
                self._event({"type": "message", "text": "S2 快档 +15°（3×5°）"})
            return

        if phase == "move_fast":
            settled = abs(s.get("theta_e") or 0) < self.SETTLE_TOL
            if s["mode"] == 3 and settled and elapsed >= 600:
                if job["step_idx"] < 3:
                    client.request(p.MOVE, struct.pack("<i", self.STEP_C01))
                    job["step_idx"] += 1
                    job["phase_start"] = tick
                else:
                    job.update(phase="hold", phase_start=tick)
                    self._event({"type": "message", "text": "S3 保持 8s"})
            elif elapsed >= 15000:
                self._video = None
                client.request(p.STOP)
                self._event({"type": "failed", "text": "快档 15s 未完成，编排终止"})
            return

        if phase == "hold":
            if elapsed >= 8000:
                job.update(phase="move_slow", phase_start=tick, step_idx=0)
                client.request(p.MOVE, struct.pack("<i", -self.STEP_C01))
                self._event({"type": "message", "text": "S4 慢档 -15°（速度曲线对比）"})
            return

        if phase == "move_slow":
            settled = abs(s.get("theta_e") or 0) < self.SETTLE_TOL
            if s["mode"] == 3 and settled and elapsed >= 600 + job["slow_gap"]:
                if job["step_idx"] < 2:
                    client.request(p.MOVE, struct.pack("<i", -self.STEP_C01))
                    job["step_idx"] += 1
                    job["phase_start"] = tick
                else:
                    job.update(phase="final", phase_start=tick)
                    self._event({"type": "message", "text": "S5 收尾保持 4s"})
            elif elapsed >= 20000:
                self._video = None
                client.request(p.STOP)
                self._event({"type": "failed", "text": "慢档 20s 未完成，编排终止"})
            return

        if phase == "final":
            if elapsed >= 4000:
                self._video = None
                client.request(p.STOP)
                self._event({"type": "video_done", "text": "编排完成 ✓"})
            return
