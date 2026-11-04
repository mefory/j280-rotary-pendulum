"""烧录后端：Tang Primer 20K（Gowin programmer_cli）与 DE10（Quartus quartus_pgm）。

框架无关（纯线程+回调），GUI 与脚本共用。四条纪律来自本项目的现场记录：

1. **T20K 先关串口再跑 JTAG**：CH340 的 COM 句柄开着时 programmer_cli 会挂死
   （见 docs/T20K完整接线图.md §7）。调用方必须在烧录前断开同板连接。
2. **SRAM 用 `--run 2`（只烧不校验）**：BL702 桥回读兼容性问题会让带校验的
   操作报 "Verify failed of at 0" 假失败；成功判据改为**读 UserCode 复核**
   （.fs 头里的 `//UserCode` 就是期望值，烧后再读板上码比对）。
3. **DE10 用 25.1std 的 quartus_pgm**：18.1 报 cable not detected；FPGA 是 JTAG
   链上第 2 个器件，故命令为 `-o p;<sof>@2`。
4. **运行时参数不随位流固化**：上电/重烧后参数回到编译默认值（板级覆盖见
   tools/generate_debug_config.py），需要时用参数页重新下发。

工具路径解析：环境变量 `J280_GOWIN_CLI` / `J280_QUARTUS_PGM` 优先，其次已知
安装位（Gowin 取版本号最高的一个）。
"""
import json
import os
import re
import subprocess
import tempfile
import threading
import time
from collections import deque
from pathlib import Path

GOWIN_DEVICE = "GW2A-18"          # Tang Primer 20K 的 GW2A-LV18PG256C8/I7
DE10_DEVICE_INDEX = 2             # DE10-Standard 的 FPGA 在 JTAG 链上是第 2 个器件

# Gowin --run 操作码（programmer_cli --help 的完整表里只用这几个）
GOWIN_RUN_SRAM = 2                # SRAM Program（不校验）
GOWIN_RUN_FLASH = 9               # exFlash Erase,Program,Verify（断电保留）
GOWIN_RUN_FLASH_NOVERIFY = 8      # exFlash Erase,Program（桥回读校验不可靠时的退路）
GOWIN_RUN_FLASH_ID = 51           # exFlash Detect ID（只读探测外部 Flash 器件）
GOWIN_RUN_USERCODE = 73           # Read User Code

# 外部 SPI Flash JEDEC ID → 器件名（高云 GUI 里要手选的"器件"，CLI 自动探测
# 不需要选——2026-11-01 实测本机 T20K 板载颗粒应答 0x0B4017（XTX 8MB 兼容
# 片，非真 Winbond W25Q128），W25Q128 档位照烧无碍；映射表用于把探测结果
# 翻译成人话并提醒容量档位）。
SPI_FLASH_IDS = {
    0xEF4018: "W25Q128（Winbond 16MB）",
    0xEF4017: "W25Q64（Winbond 8MB）",
    0x0B4018: "XT25F128B（XTX 16MB，W25Q128 档兼容）",
    0x0B4017: "XT25F64B（XTX 8MB，W25Q64/128 档兼容）",
}


def parse_flash_id(text):
    """从 programmer_cli 输出取 `Detected Flash id: 0x...`。"""
    match = re.search(r"Detected\s*Flash\s*id:?\s*0x([0-9A-Fa-f]+)", text or "")
    return int(match.group(1), 16) if match else None


def flash_device_name(flash_id):
    """JEDEC ID → 人话；未知 ID 返回带原值的提示（不阻断烧录）。"""
    if flash_id is None:
        return None
    return SPI_FLASH_IDS.get(flash_id, f"未知串行 Flash（ID 0x{flash_id:06X}，兼容档可烧）")

BOARDS = {"t20k": "Tang Primer 20K", "de10": "DE10-Standard"}


# ---------- 工作区与工具探测 ----------
def workspace_root():
    """定位工作区根（含 t20k_build/de10_build 的目录）。"""
    env = os.environ.get("J280_WS", "").strip()
    if env:
        return Path(env)
    here = Path(__file__).resolve()
    for cand in here.parents[2:5]:          # 上位机 → lab → j280
        if (cand / "t20k_build").is_dir() or (cand / "de10_build").is_dir():
            return cand
    return here.parents[3]


def find_gowin_cli():
    """找 programmer_cli.exe；环境变量优先，其次 Gowin 安装位（版本号最高者）。"""
    cands = []
    env = os.environ.get("J280_GOWIN_CLI", "").strip()
    if env:
        cands.append(Path(env))
    for drive in ("E:/", "D:/", "C:/"):
        base = Path(drive + "Gowin")
        if base.is_dir():
            cands += sorted(base.glob("Gowin_V*/Programmer/bin/programmer_cli.exe"), reverse=True)
    for path in cands:
        if path.is_file():
            return path
    return None


def find_quartus_pgm():
    """找 quartus_pgm.exe；25.1std 优先（18.1 在新板上报 cable not detected）。"""
    cands = []
    env = os.environ.get("J280_QUARTUS_PGM", "").strip()
    if env:
        cands.append(Path(env))
    for drive in ("E:/", "D:/", "C:/"):
        after = Path(drive + "altera")
        if after.is_dir():
            cands += sorted(after.glob("*/qprogrammer/bin64/quartus_pgm.exe"), reverse=True)
    cands += [Path("E:/FPGA/quartus/bin64/quartus_pgm.exe"),
              Path("C:/altera_lite/25.1std/nios2eds/bin/quartus_pgm.exe")]
    for path in cands:
        if path.is_file():
            return path
    return None


def find_jtagconfig():
    """DE10 侧读 JTAG 链用；就在 quartus_pgm 同目录。"""
    pgm = find_quartus_pgm()
    if pgm and (pgm.parent / "jtagconfig.exe").is_file():
        return pgm.parent / "jtagconfig.exe"
    return None


def detect_tool(board):
    """返回 {path, ok, note}；note 是给人看的可操作提示。"""
    if board == "t20k":
        path = find_gowin_cli()
        if path:
            return {"path": str(path), "ok": True, "note": "Gowin programmer_cli"}
        return {"path": "", "ok": False, "note":
                "未找到 programmer_cli.exe。安装 Gowin IDE，或用环境变量 "
                "J280_GOWIN_CLI 指向该可执行文件"}
    path = find_quartus_pgm()
    if path:
        return {"path": str(path), "ok": True, "note": "Quartus Prime Programmer"}
    return {"path": "", "ok": False, "note":
            "未找到 quartus_pgm.exe。安装 Quartus（建议 25.1std），或用环境变量 "
            "J280_QUARTUS_PGM 指向该可执行文件"}


# ---------- 比特流扫描 ----------
def parse_fs_usercode(path):
    """读 .fs 头部 `//UserCode: 0x...`；这是烧录后比对板上码的期望值。"""
    try:
        head = Path(path).read_bytes()[:4096].decode("utf-8", "replace")
    except OSError:
        return None
    match = re.search(r"//UserCode:\s*0x([0-9A-Fa-f]+)", head)
    return int(match.group(1), 16) if match else None


def _scan_paths(board, root):
    """glob 构建目录里的比特流，按修改时间新→旧，[(path, stat)]。

    每个文件只 stat 一次（此前排序键+循环体各 stat 一次，480 份翻倍浪费）。
    """
    root = Path(root) if root else workspace_root()
    found = []
    if board == "t20k":
        base = root / "t20k_build"
        if base.is_dir():
            # 三种布局：主线 audit_x/t20k/impl/pnr、诊断 diag_x/t20kxx/impl/pnr、
            # 工作树 <工作树>/audit_x/t20k/impl/pnr（各占一层通配，2026-10-20
            # 前只有前两种，工作树构建从未出现在烧录页列表里）
            for pattern in ("*/impl/pnr/*.fs", "*/*/impl/pnr/*.fs",
                            "*/*/*/impl/pnr/*.fs"):
                found += [p for p in base.glob(pattern) if p.is_file()]
    else:
        base = root / "de10_build"
        if base.is_dir():
            found += [p for p in base.glob("*/de10/*.sof") if p.is_file()]
    seen, statted = set(), []
    for path in found:
        if path in seen:
            continue
        seen.add(path)
        try:
            st = path.stat()
        except OSError:
            continue
        statted.append((st.st_mtime, path, st))
    statted.sort(key=lambda t: t[0], reverse=True)
    return statted, base


def _entry_for(board, path, stat, base):
    """单份比特流 → 列表条目 dict（构建目录归属 + usercode）。"""
    # t20k_build/audit_x/t20k/impl/pnr/x.fs → 构建目录名在 parents[3]；
    # de10_build/audit_x/de10/x.sof → parents[1]。诊断构建同构，索引一致。
    index = 3 if board == "t20k" else 1
    try:
        build = path.parents[index].name
    except IndexError:
        build = path.parent.name
    try:
        source_dir = path.parents[index + 1]
    except IndexError:
        source_dir = None
    source = source_dir.name if (source_dir is not None and source_dir != base) else ""
    return {"path": str(path), "build": build, "source": source,
            "usercode": parse_fs_usercode(path) if path.suffix == ".fs" else None,
            "mtime": stat.st_mtime, "size": stat.st_size}


def list_bitstreams(board, root=None, limit=None):
    """扫描构建目录里的比特流，新→旧。

    默认全量枚举（2026-10-30 前写死 limit=25，旧固件被截在列表外——
    实测构建目录下有 439 份 .fs 只见 25 份）。limit 仍可传整数
    截断；usercode 解析只读文件头 4KB，全量代价可忽略。

    返回 [{path, build, source, usercode, mtime, size}]。source 是比特流出自
    哪个工作树的构建目录（t20k_build/<工作树>/audit_*/…，如 mt6701-theta）；
    直接躺在构建根下的 audit_*（工作树迁移前的主线/历史构建）source=""。
    """
    statted, base = _scan_paths(board, root)
    return [_entry_for(board, path, st, base) for _, path, st in statted[:limit]]


# ---- 扫描缓存（2026-11-01）：全量枚举后冷缓存首扫实测 ~44s（480 份头文件
# 读取散布冷读 + 目录树遍历），且每次新构建都会新增文件。按 (mtime,size) 增量
# 复用磁盘缓存：只有新增/变动的比特流才重新读头解析 usercode；GUI 侧配合后
# 台线程，刷新不再卡界面。缓存放系统临时目录（纯加速件，丢了只是慢一次）。
_SCAN_CACHE_FILE = Path(tempfile.gettempdir()) / "j280_flash_scan_cache.json"
_scan_cache_lock = threading.Lock()
_scan_cache = {}


def list_bitstreams_cached(board, root=None):
    """list_bitstreams 的增量缓存版：GUI 刷新用，语义与全量版一致。"""
    root_path = str(Path(root) if root else workspace_root())
    key = f"{board}@{root_path}"
    with _scan_cache_lock:
        cache = _scan_cache.get(key)
        if cache is None:
            try:
                raw = json.loads(_SCAN_CACHE_FILE.read_text(encoding="utf-8"))
                cache = raw.get(key, {})
            except (OSError, ValueError):
                cache = {}
        statted, base = _scan_paths(board, root_path)
        result, fresh = [], {}
        for mtime, path, st in statted:
            ck = str(path)
            old = cache.get(ck)
            if old is not None and old["mtime"] == mtime and old["size"] == st.st_size:
                entry = old
            else:
                entry = _entry_for(board, path, st, base)
            fresh[ck] = entry
            result.append(entry)
        _scan_cache[key] = fresh
        try:
            _SCAN_CACHE_FILE.write_text(
                json.dumps({**_load_cache_file(), key: fresh}, ensure_ascii=False),
                encoding="utf-8")
        except OSError:
            pass
    return result


def _load_cache_file():
    try:
        raw = json.loads(_SCAN_CACHE_FILE.read_text(encoding="utf-8"))
        return raw if isinstance(raw, dict) else {}
    except (OSError, ValueError):
        return {}


def sha256_of(path, chunk=1 << 20):
    """大文件分块 SHA256；烧录前给用户当完整性对账依据（audit 流程同款）。"""
    import hashlib
    digest = hashlib.sha256()
    with Path(path).open("rb") as fh:
        for block in iter(lambda: fh.read(chunk), b""):
            digest.update(block)
    return digest.hexdigest().upper()


# ---------- 命令构造（纯函数，可单测） ----------
def _gowin_argv(tool, run_op=None, bitstream=None, cable_index=1, cable_location=None,
                device=GOWIN_DEVICE):
    argv = [str(tool), "--device", device]
    if cable_location is not None:
        argv += ["--location", str(cable_location)]
    else:
        argv += ["--cable-index", str(cable_index)]
    if bitstream:
        argv += ["--fsFile", str(bitstream)]
    if run_op is not None:
        argv += ["--run", str(run_op)]
    return argv


def make_job(board, tool, bitstream=None, *, action="sram", cable_index=1,
             cable_location=None, device_index=DE10_DEVICE_INDEX, device=None,
             flash_verify=True):
    """构造烧录作业：[{argv, label, usercode?, timeout?}, ...]，逐步顺序执行。

    T20K 的烧录作业自动追加"读 UserCode 复核"一步——`--run 2` 不校验，
    成功与否只能靠板上 UserCode 是否变成 .fs 头里的值来判断。Flash 固化同理，
    且 `flash_verify=False` 时用 `--run 8` 绕开桥回读校验（现场已知假失败）。
    """
    device = device or GOWIN_DEVICE
    steps = []
    if board == "t20k":
        def gowin(op, **kw):
            return _gowin_argv(tool, op, cable_index=cable_index,
                               cable_location=cable_location, device=device, **kw)
        if action == "scan_cables":
            return [{"argv": [str(tool), "--scan-cables", "F"], "label": "扫描 Gowin JTAG 线缆",
                     "timeout": 60.0}]
        if action == "read_usercode":
            return [{"argv": gowin(GOWIN_RUN_USERCODE), "label": "读取板上 UserCode",
                     "usercode": True, "timeout": 60.0}]
        if action == "detect_flash":
            return [{"argv": gowin(GOWIN_RUN_FLASH_ID),
                     "label": "检测外部 Flash（--run 51，只读）",
                     "flash_id": True, "timeout": 60.0}]
        if action not in ("sram", "flash"):
            raise ValueError(f"未知烧录动作 {action}")
        if not bitstream:
            raise ValueError("未选择比特流文件")
        if action == "sram":
            steps.append({"argv": gowin(GOWIN_RUN_SRAM, bitstream=bitstream),
                          "label": "SRAM 烧录（--run 2，不校验）", "timeout": 300.0})
        else:
            # 前置 Detect ID：把高云 GUI 里"器件选 W25Q128"那步变成自动探测
            # （CLI 无器件选择项，靠 JEDEC ID 自识别；日志里能看到实际颗粒）
            steps.append({"argv": gowin(GOWIN_RUN_FLASH_ID),
                          "label": "检测外部 Flash（--run 51，只读）",
                          "flash_id": True, "timeout": 60.0})
            op = GOWIN_RUN_FLASH if flash_verify else GOWIN_RUN_FLASH_NOVERIFY
            label = ("外部 Flash 固化（--run 9，擦除+编程+校验）" if flash_verify
                     else "外部 Flash 固化（--run 8，擦除+编程）")
            steps.append({"argv": gowin(op, bitstream=bitstream),
                          "label": label, "timeout": 900.0})
        steps.append({"argv": gowin(GOWIN_RUN_USERCODE),
                      "label": "读取板上 UserCode（复核）", "usercode": True, "timeout": 60.0})
        return steps
    if board == "de10":
        # DE10 只有"经 JTAG 烧 FPGA"这一种动作；GUI 统一用 sram/program 都接受，
        # 因为对 DE10 而言 sof 经 JTAG 配置 SRAM 本就是同一件事。
        if action not in ("program", "sram"):
            raise ValueError(f"未知烧录动作 {action}")
        if not bitstream:
            raise ValueError("未选择比特流文件")
        # quartus_pgm 的 -o 参数用正斜杠（现场验证过的写法）
        target = f"p;{Path(bitstream).as_posix()}@{device_index}"
        return [{"argv": [str(tool), "-m", "jtag", "-o", target],
                 "label": f"Quartus 烧录（链上器件 {device_index}）", "timeout": 600.0}]
    raise ValueError(f"未知目标板 {board}")


def parse_usercode(text):
    """从 programmer_cli 输出里取 `User Code is: 0x...`（大小写与空格容错）。"""
    match = re.search(r"User\s*Code\s*is:?\s*0x([0-9A-Fa-f]+)", text or "")
    return int(match.group(1), 16) if match else None


def parse_cables(text):
    """从 `--scan-cables F` 输出里取线缆列表：`Cable found: 名字 (USB location:NNNNN)`。"""
    cables = []
    for line in (text or "").splitlines():
        match = re.search(r"Cable found:\s*(.+?)\s*\(USB location:(\d+)\)", line)
        if match:
            cables.append({"name": match.group(1).strip(), "location": int(match.group(2))})
    return cables


# ---------- 执行 ----------
class Flasher:
    """在独立线程里顺序执行作业步骤，逐行回调输出，可取消。

    与 bridge.Bridge 同构：框架无关、回调订阅、线程内串行。GUI 用 Qt 信号把
    回调转到主线程；脚本可直接轮询 `lines` 与 `result`。
    """

    def __init__(self):
        self.lines = deque(maxlen=4000)
        self.running = False
        self.result = None
        self._steps = []
        self._proc = None
        self._thread = None
        self._lock = threading.Lock()
        self._line_hooks = deque()
        self._done_hooks = deque()
        self._cancel_requested = False

    # ---------- 订阅 ----------
    def on_line(self, callback):
        with self._lock:
            self._line_hooks.append(callback)
            return lambda: self._line_hooks.remove(callback)

    def on_done(self, callback):
        with self._lock:
            self._done_hooks.append(callback)
            return lambda: self._done_hooks.remove(callback)

    def _emit_line(self, text):
        with self._lock:
            self.lines.append(text)
            hooks = list(self._line_hooks)
        for hook in hooks:
            try:
                hook(text)
            except Exception:
                pass

    def _emit_done(self, result):
        with self._lock:
            hooks = list(self._done_hooks)
        for hook in hooks:
            try:
                hook(result)
            except Exception:
                pass

    # ---------- 生命周期 ----------
    def start(self, steps, *, expected_usercode=None, cwd=None, tag=None):
        """启动作业；steps 来自 make_job()。返回 False 表示已有任务在跑。

        tag 原样带进结果字典——GUI 用它在同一个完成回调里区分"烧录"与"扫线缆"。
        """
        if self.running:
            return False
        self.running = True
        self.result = None
        self._cancel_requested = False
        self._tag = tag
        self._steps = list(steps)
        self._thread = threading.Thread(
            target=self._run, args=(expected_usercode, cwd), name="flasher", daemon=True)
        self._thread.start()
        return True

    def cancel(self):
        with self._lock:
            self._cancel_requested = True
        proc = self._proc
        if proc and proc.poll() is None:
            self._emit_line("… 取消：正在终止烧录进程")
            try:
                proc.terminate()
            except Exception:
                pass

    def close(self, timeout=5.0):
        self.cancel()
        if self._thread:
            self._thread.join(timeout)

    # ---------- 执行线程 ----------
    def _run(self, expected, cwd):
        started = time.monotonic()
        ok, error, cancelled = True, None, False
        usercode = None
        flash_id = None
        try:
            for step in self._steps:
                if self._cancel_requested:
                    ok, error, cancelled = False, "已取消", True
                    break
                self._emit_line("$ " + " ".join(step["argv"]))
                self._emit_line("-- " + step["label"])
                rc, was_cancelled, was_timeout, text = self._popen(
                    step["argv"], step.get("timeout", 300.0), cwd)
                if was_cancelled:
                    ok, error, cancelled = False, "已取消", True
                    break
                if was_timeout:
                    ok, error = False, f"超时（>{step.get('timeout', 300.0):.0f}s），已终止"
                    break
                if rc != 0:
                    ok, error = False, f"退出码 {rc}"
                    break
                if step.get("usercode"):
                    usercode = parse_usercode(text)
                if step.get("flash_id"):
                    flash_id = parse_flash_id(text)
                    name = flash_device_name(flash_id)
                    if name:
                        self._emit_line(f"外部 Flash 器件：{name}（ID 0x{flash_id:06X}）")
                    else:
                        self._emit_line(f"未解析到外部 Flash ID（ID 解析失败）")
        except Exception as exc:
            ok, error = False, str(exc)
        match = None
        if usercode is not None and expected is not None:
            match = usercode == expected
        result = {"ok": ok, "error": error, "cancelled": cancelled,
                  "elapsed": round(time.monotonic() - started, 1),
                  "usercode": usercode, "expected_usercode": expected,
                  "usercode_match": match,
                  "flash_id": flash_id, "flash_device": flash_device_name(flash_id),
                  "tag": getattr(self, "_tag", None)}
        self.result = result
        self._proc = None
        self.running = False
        self._emit_done(result)

    def _popen(self, argv, timeout, cwd):
        """执行一步：逐行回调输出；返回 (rc, 是否被取消, 是否超时, 全文)。

        取消靠 `terminate()`（Windows 上等价 TerminateProcess），只要返回码非 0
        且是我们主动取消的，就按"已取消"而不是"失败"上报。
        """
        timed_out = False
        flags = getattr(subprocess, "CREATE_NO_WINDOW", 0)
        proc = subprocess.Popen(argv, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                text=True, encoding="utf-8", errors="replace",
                                bufsize=1, cwd=cwd, creationflags=flags)
        self._proc = proc

        def on_timeout():
            nonlocal timed_out
            if proc.poll() is None:
                timed_out = True
                self._emit_line(f"… 超过 {timeout:.0f}s 未结束，终止进程")
                try:
                    proc.kill()
                except Exception:
                    pass

        timer = threading.Timer(timeout, on_timeout)
        timer.daemon = True
        timer.start()
        chunks = []
        try:
            for line in proc.stdout:
                line = line.rstrip()
                if line:
                    chunks.append(line)
                    self._emit_line(line)
            proc.wait()
        finally:
            timer.cancel()
        cancelled = bool(self._cancel_requested)
        return proc.returncode, cancelled, timed_out, "\n".join(chunks)
