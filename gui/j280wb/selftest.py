"""无头自测：不启动 GUI，验证 桥→Agent接口→CLI路径 的完整任务链。

流程：连接模拟设备 → 清故障 → 起摆 → 等捕获 → MOVE +30° → 核对锚点 →
停止 → 核对 SAFE；期间用真实 TCP 客户端执行 status/params/租约/watch。
产物：logs/selftest.json
"""
import json
import socket
import time
from pathlib import Path

from host import protocol as p
from host.records import load_record
from host.schema import PARAMS

from . import AGENT_DEFAULT_PORT
from .agent import AgentServer
from .bridge import Bridge
from .cli import request

T_START = 0  # 由桥连接后的 tick 基准推算
N_PARAMS = len(PARAMS)   # 参数个数随参数表演进（勿硬编码）


def tcp(port):
    return socket.create_connection(("127.0.0.1", port), timeout=5)


# ---------- 传感诊断单元（j280wb/diag.py，合成序列，无需设备） ----------
def _sample(tick, encoder, theta, mode=0, fault=0, alpha=0, alpha_dot=0, theta_dot=0):
    return {"tick": tick, "revision": 0, "mode": mode, "fault": fault,
            "alpha": alpha, "alpha_dot": alpha_dot, "theta": theta,
            "theta_dot": theta_dot, "duty": 0, "reference": 0, "target": 0,
            "encoder": encoder, "adc": 0, "offset": 0, "calib_done": 0, "run_en": 0}


def diag_unit():
    """单源 θ 健康：驱动-响应掉线 / 原角一致性 / 噪声 / CSV 五类合成序列。

    2026-10-22 霍尔编码器物理拆除后，原"双源判源"判据作废。现在遥测 `encoder`
    字段是 MT6701 原始 14 位角，与 θ 多圈累加器互校（1 LSB=9.3064 c01），
    核心判据换成驱动-响应：指令在真驱动区而 θ 不动 = 掉线/卡死。
    """
    from .diag import (RAW_FULL, RAW_TO_C01, STUCK_MS, ThetaHealthDiag,
                       WatchMonitor)

    def sample(tick, raw, theta, duty=0, mode=0, fault=0, alpha=0,
               alpha_dot=0, theta_dot=0, adc=0, offset=0, calib=0):
        return {"tick": tick, "revision": 0, "mode": mode, "fault": fault,
                "alpha": alpha, "alpha_dot": alpha_dot, "theta": theta,
                "theta_dot": theta_dot, "duty": duty, "reference": 0, "target": 0,
                "encoder": raw, "adc": adc, "offset": offset,
                "calib_done": calib, "run_en": 0}

    def spin(duty_eff, steps=1500, lsb=2.0):
        """合成臂旋转：原角按 lsb/拍 前进，θ 按 RAW_TO_C01 同比例累计。"""
        diag = ThetaHealthDiag()
        raw_f = theta_f = 0.0
        for i in range(steps):
            raw_f += lsb
            theta_f += lsb * RAW_TO_C01
            diag.feed(sample(i * 2, int(round(raw_f)) % RAW_FULL,
                              int(round(theta_f)), duty=duty_eff))
        return diag

    # 1) 正常驱动 + θ 跟得上 → 无掉线、原角一致
    ok = spin(3000)
    checks, concl = ok.verdict()
    dr = next(c for c in checks if "驱动-响应" in c[0])
    assert dr[1] is True, dr[2]
    assert ok._dropout_events == 0, ok._dropout_events
    cons = next(c for c in checks if "累加器 vs 原始角" in c[0])
    assert cons[1] is True, cons[2]
    drive_ok = dr[1] is True

    # 2) 大电流但 θ 纹丝不动 → 记掉线，判据点名（这条就是今天的实测现象）
    stalled = ThetaHealthDiag()
    for i in range(800):
        stalled.feed(sample(i * 2, 1000, 5000, duty=9000))
    checks, concl = stalled.verdict()
    dr = next(c for c in checks if "驱动-响应" in c[0])
    assert dr[1] is False, dr[2]
    assert stalled._dropout_events >= 1, stalled._dropout_events
    assert "掉线" in concl, concl
    dropout_events = stalled._dropout_events
    dropout_hint = concl[:24]

    # 3) 驱动很大但 θ 在动（正常加速）→ 不能误判成掉线
    fast = spin(9000, steps=600, lsb=6.0)
    assert fast._dropout_events == 0, fast._dropout_events

    # 4) 方向反（Δ原角与 Δθ 异号）→ 原角一致性未过，提示 TH_MT_DIR_FLIP
    wrong = ThetaHealthDiag()
    for i in range(400):
        wrong.feed(sample(i * 2, (100 + i * 2) % RAW_FULL, -i * 2 * int(RAW_TO_C01),
                          duty=3000))
    checks, _ = wrong.verdict()
    cons = next(c for c in checks if "累加器 vs 原始角" in c[0])
    assert cons[1] is False, cons[2]
    assert "TH_MT_DIR_FLIP" in cons[2], cons[2]

    # 5) 静止噪声：θ 不动、duty 小 → 噪声项有值
    quiet = ThetaHealthDiag()
    for i in range(600):
        quiet.feed(sample(i * 2, 500, 500, duty=0,
                          theta_dot=(i * 7) % 5 - 2, alpha_dot=(i * 11) % 7 - 3))
    checks, _ = quiet.verdict()
    noise = [c for c in checks if "静止噪声" in c[0]]
    assert noise and all(c[1] is not None for c in noise), "静止段足够时噪声项应有值"

    # ---- 常开监视：模式迁移/故障/掉摆三态 + 驱动中 θ 不动 ----
    watch = WatchMonitor({0: "待机", 1: "起摆", 2: "稳摆", 4: "安全停止"})
    msgs = []
    msgs += watch.feed(sample(0, 0, 0, mode=0))
    msgs += watch.feed(sample(2, 0, 0, mode=2))
    msgs += watch.feed(sample(4, 0, 0, mode=4, fault=1, alpha=-6100, theta_dot=-350))
    assert any("模式" in m for m in msgs), msgs
    assert any("故障锁存" in m for m in msgs), msgs
    assert any("安全链中断" in m for m in msgs), msgs
    # 掉摆三态拆分：fault=0 大角=真掉摆；fault=0 小角=人工/超时
    watch.feed(sample(6, 0, 0, mode=2))
    fall = watch.feed(sample(8, 0, 0, mode=4, fault=0, alpha=-6500, theta_dot=-350))
    assert any("掉摆进入安全停止" in m for m in fall), fall
    watch.feed(sample(10, 0, 0, mode=2))
    manual = watch.feed(sample(12, 0, 0, mode=4, fault=0, alpha=-200))
    assert any("非掉摆" in m for m in manual), manual
    # 驱动中 θ 不动：连续 150 拍才点名，且只报一次
    watch2 = WatchMonitor({0: "待机"})
    run_msgs = []
    for i in range(1, 152):
        run_msgs += watch2.feed(sample(i * 2, 900, 500, duty=9000))
    assert any("驱动中 θ 不动" in m for m in run_msgs), run_msgs
    for i in range(151, 301):
        run_msgs += watch2.feed(sample(i * 2, 900, 500, duty=9000))
    assert sum("驱动中 θ 不动" in m for m in run_msgs) == 1, "应只报一次"
    # 驱动中 θ 在动 → 不报
    watch3 = WatchMonitor({0: "待机"})
    for i in range(1, 301):
        watch3.feed(sample(i * 2, i * 2 % RAW_FULL, int(RAW_TO_C01 * i * 2), duty=9000))
    assert not any("驱动中 θ 不动" in m for m in
                   watch3.feed(sample(602, 602 % RAW_FULL, int(RAW_TO_C01 * 602), duty=9000))), \
        "正常运动不应触发掉线告警"

    watch_msgs = len(msgs)

    # ---- MT6701 链路级诊断（STATUS 尾追 12 字节）----
    from .diag import Mt6701LinkDiag

    def io_status(frames, good, crc=0, mg=0, code=0, flags=0, angle=0):
        return {"rx_errors": 0, "dropped": 0, "encoder": 0, "adc": 0, "duty": 0,
                "enabled": 0,
                "mt6701": {"frames": frames, "good": good, "crc_err": crc,
                           "mg_err": mg, "mg_code": code, "fault": bool(flags & 1),
                           "selected": bool(flags & 2), "angle_raw": angle}}

    link = Mt6701LinkDiag()
    assert link.update({"mt6701": None}, 0)[0] is None, "旧固件应优雅降级"
    rows, msgs = link.update(io_status(1000, 1000, angle=4096), 1000)
    assert rows["mg_text"] == "正常" and not rows["fault"], rows
    assert abs(rows["angle_deg"] - 90.0) < 1e-6, rows["angle_deg"]
    rows, msgs = link.update(io_status(2000, 1990), 1500)   # +1000 帧 / 500ms
    assert 1990 < rows["frame_rate"] < 2010, rows["frame_rate"]
    assert 0.9 < rows["bad_rate"] < 1.1, rows["bad_rate"]
    rows, msgs = link.update(io_status(2100, 2000, crc=60, mg=30, code=2, flags=2), 1550)
    assert rows["mg_text"].startswith("场太弱"), rows["mg_text"]
    assert rows["selected"], "flags bit1 应解析为 selected"
    checks, concl = link.verdict(rows)
    assert next(c for c in checks if "磁场强度" in c[0])[1] is False, "场弱应判未过"
    assert "MT6701" in concl
    rows, _ = link.update(io_status(2200, 2210, code=4, flags=3), 1600)  # Mg[3] 失跟踪
    assert rows["lost_track"], "mg_code bit2 应为失跟踪"
    assert link.verdict(rows)[0][0][1] is False
    stale = Mt6701LinkDiag()
    stale.update(io_status(5000, 5000), 1000)
    _, smsgs = stale.update(io_status(5000, 5000), 2000)
    assert any("无新帧" in m for m in smsgs), smsgs
    # 锁存原因快照（flags bit7:6）协议层打包/解析：cause=2 且 selected/fault 兼容
    raw = (p.IO_STATUS.pack(0, 0, 0, 0, 0, 0)
           + p.IO_STATUS_MT.pack(1, 1, 0, 0, 0, 0b10000011, 0)
           + b"\x00" * p.IO_STATUS_MT68.size
           + p.IO_STATUS_GAP.pack(321))
    stt = p.unpack_io_status(raw)
    assert stt["mt6701"]["cause"] == 2 and stt["mt6701"]["selected"] \
        and stt["mt6701"]["fault"], stt["mt6701"]
    assert stt["mt6835"] is not None, "43 字节载荷应同时解析 MT6835 段"
    assert stt["mt6701"]["max_no_good"] == 321, stt["mt6701"]

    # ---- MT6835 链路级诊断（STATUS 二次尾追 14 字节）----
    from .diag import Mt6835LinkDiag, fmt_sat16

    def io_status68(frames, good, crc=0, serr=0, code=0, flags=0, angle=0):
        return {"rx_errors": 0, "dropped": 0, "encoder": 0, "adc": 0, "duty": 0,
                "enabled": 0, "mt6701": None,
                "mt6835": {"frames": frames, "good": good, "crc_err": crc,
                           "status_err": serr, "status": code, "fault": bool(flags & 1),
                           "angle_raw": angle}}

    link68 = Mt6835LinkDiag()
    assert link68.update({"mt6835": None}, 0)[0] is None, "旧固件应优雅降级"
    rows, _ = link68.update(io_status68(20000, 20000, angle=1048576), 1000)
    assert rows["status_text"] == "正常" and not rows["fault"], rows
    assert abs(rows["angle_deg"] - 180.0) < 1e-6, rows["angle_deg"]
    rows, msgs = link68.update(io_status68(70000, 69930), 2000)   # +50000 帧 / 1s
    assert 49900 < rows["frame_rate"] < 50100, rows["frame_rate"]
    assert 0.1 < rows["bad_rate"] < 0.2, rows["bad_rate"]
    # 弱磁场 + 欠压组合状态位（bit1|bit2 = 6）
    rows, _ = link68.update(io_status68(75000, 74990, code=6), 3000)
    assert rows["status_text"] == "磁场过弱、欠压", rows["status_text"]
    checks, concl = link68.verdict(rows)
    assert next(c for c in checks if "器件状态位" in c[0])[1] is False, "状态位应判未过"
    assert "MT6835" in concl
    # 未接线签名：角=2^21-1、CRC 累计非零
    rows, wmsgs = link68.update(io_status68(80000, 75000, crc=5010, angle=2097151), 4000)
    assert rows["unwired"], rows
    assert any("断线签名" in m for m in wmsgs), wmsgs
    assert link68.verdict(rows)[0][3][1] is False, "断线签名项应判未过"
    stale68 = Mt6835LinkDiag()
    stale68.update(io_status68(90000, 90000, angle=1000), 1000)
    _, smsgs68 = stale68.update(io_status68(90000, 90000, angle=1000), 2000)
    assert any("无新帧" in m for m in smsgs68), smsgs68
    # 饱和标注：65535 显示带（饱和）后缀
    rows, _ = link68.update(io_status68(95000, 95000, crc=0xFFFF, serr=0xFFFF), 5000)
    assert "饱和" in fmt_sat16(rows["crc_err"]), rows["crc_err"]
    assert fmt_sat16(1234) == "1234"

    # ---- 诊断记录 CSV：写入→读回→摘要 ----
    import tempfile, os
    from .diag import DiagRecorder, load_csv, summarize_csv
    csv_path = os.path.join(tempfile.gettempdir(), "j280_diag_selftest.csv")
    rec = DiagRecorder(csv_path)
    rec.open_at(csv_path)
    for i in range(600):
        rec.feed(sample(i * 2, (i * 2) % RAW_FULL, int(RAW_TO_C01 * i * 2), duty=3000))
    written = rec.close()
    assert written == 600, written
    back = load_csv(csv_path)
    assert len(back) == 600 and "raw_err_lsb" in back[0], sorted(back[0])
    summary = summarize_csv(back)
    assert summary["rows"] == 600, summary
    assert summary["drive_ms"] > 0 and summary["dropout_ratio"] == 0, summary
    assert abs(summary["raw_err_max"]) <= 1.0, summary
    assert summary["motion_revs"] > 0, summary

    return {"drive_ok": drive_ok, "dropout_events": dropout_events,
            "dropout_hint": dropout_hint, "watch_msgs": watch_msgs,
            "mt6701_link": "ok", "mt6835_link": "ok", "csv_rows": written,
            "csv_drive_ms": summary["drive_ms"]}


def flasher_unit():
    """烧录后端纯函数：构建目录来源分组 + UserCode/SHA256 + 命令构造。

    来源（source）= 比特流出自哪个工作树的构建目录（t20k_build/<工作树>/audit_*），
    根下散装 audit_* 归为主线/历史（source=""）——GUI 烧录页"来源"下拉的数据基础。
    """
    import tempfile
    from . import flasher as fl
    root = Path(tempfile.mkdtemp(prefix="j280_fl_"))
    p_wt = root / "t20k_build" / "mt6701-theta" / "audit_20261020_132102" / "t20k" / "impl" / "pnr"
    p_main = root / "t20k_build" / "audit_20261001_000000" / "t20k" / "impl" / "pnr"
    p_wt.mkdir(parents=True)
    p_main.mkdir(parents=True)
    (p_wt / "a.fs").write_bytes(b"//UserCode: 0x1234\n" + b"\x00" * 64)
    (p_main / "b.fs").write_bytes(b"//UserCode: 0xABCD\n" + b"\x00" * 64)
    by = {Path(e["path"]).name: e for e in fl.list_bitstreams("t20k", root=root)}
    assert set(by) == {"a.fs", "b.fs"}, by
    assert by["a.fs"]["source"] == "mt6701-theta" and by["a.fs"]["usercode"] == 0x1234, by["a.fs"]
    assert by["b.fs"]["source"] == "" and by["b.fs"]["usercode"] == 0xABCD, by["b.fs"]
    assert fl.sha256_of(p_wt / "a.fs") == fl.sha256_of(p_wt / "a.fs")

    steps = fl.make_job("t20k", "tool.exe", "x.fs", action="sram", cable_location=77)
    assert len(steps) == 2 and steps[1].get("usercode"), steps
    argv = steps[0]["argv"]
    assert argv[argv.index("--location") + 1] == "77", argv
    assert argv[argv.index("--run") + 1] == "2", argv
    argv = fl.make_job("t20k", "tool.exe", "x.fs", action="flash",
                       flash_verify=False)[0]["argv"]
    assert argv[argv.index("--run") + 1] == "8", argv
    argv = fl.make_job("de10", "pgm.exe", "x.sof", action="program",
                       device_index=2)[0]["argv"]
    assert argv[-1] == "p;x.sof@2", argv
    for board, action, bs in (("t20k", "sram", None), ("de10", "flash", "x.sof"),
                              ("t20k", "polish", "x.fs")):
        try:
            fl.make_job(board, "t.exe", bs, action=action)
            raise AssertionError(f"应拒绝非法组合 {board}/{action}/{bs}")
        except ValueError:
            pass
    return {"source_group": "ok", "usercode": "ok", "argv": "ok"}


def record_unit(bridge, out_dir):
    """验证混合记录（遥测 CSV + 事件 JSONL）的可分析性。

    判据都是"分析时真的会用到"的东西：
      - events 里 metadata 带 parameters_detail + calibration + firmware
      - CSV 表头 = FIELDS，行数与 record_end.samples 对得上账
      - command/note 带 tick（能否和遥测对齐）
      - record_end 带 samples/missing_ratio（能否确认没被截断）
      - 采样缺失率 ≈ 0（2ms 一帧，抖动容 5%）
    """
    base = Path(out_dir) / "selftest_record"
    csv_path = base.with_suffix(".csv")
    events_path = base.with_name(base.name + "_events.jsonl")
    for stale in (csv_path, events_path):
        if stale.exists():
            stale.unlink()
    bridge.submit("record", str(base))
    deadline = time.monotonic() + 5
    while bridge._recorder is None and time.monotonic() < deadline:
        time.sleep(0.02)
    assert bridge._recorder is not None, "记录未能启动"
    time.sleep(0.6)
    bridge.submit("command", (p.STATUS, b""), source="selftest")
    time.sleep(0.4)
    bridge.note("selftest 备注：验证 note 落盘")
    time.sleep(0.3)
    bridge.submit("record", None)          # 停记录 → 触发 record_end
    deadline = time.monotonic() + 5
    while bridge._recorder is not None and time.monotonic() < deadline:
        time.sleep(0.02)
    assert bridge._recorder is None, "记录未能停止"

    assert csv_path.exists() and events_path.exists(), "混合记录没有成对落盘"
    meta, samples, events = load_record(base)
    types = {r.get("type") for r in events}
    # 新格式里 metadata 由 load_record 单独归入 meta，不进 events
    for need in ("command", "note", "record_end"):
        assert need in types, f"事件文件缺 {need}，实际有 {sorted(types)}"
    assert "sample" not in types, "sample 不应再出现在事件文件里（遥测归 CSV）"
    assert samples, "遥测 CSV 没有任何行"
    assert list(samples[0]) == list(p.FIELDS), "CSV 列与 FIELDS 不一致"

    assert meta is not None, "事件文件缺 metadata"
    assert meta.get("version") == 3, f"元数据版本 {meta.get('version')}"
    assert meta.get("record_format") == "csv+events", meta.get("record_format")
    assert len(meta.get("parameters_detail") or []) == N_PARAMS, "参数明细不完整"
    assert (meta.get("calibration") or {}).get("offset_c01") is not None, "缺零点快照"
    assert (meta.get("firmware") or {}).get("build_stamp") is not None, "缺固件标识"

    cmds = [r for r in events if r["type"] == "command"]
    assert cmds, "没有命令记录"
    assert all("tick" in c and "time" in c for c in cmds), "命令记录缺 tick/time，无法对齐遥测"

    notes = [r for r in events if r["type"] == "note"]
    assert any("selftest 备注" in (n.get("text") or "") for n in notes), "note 未落盘"

    end = next(r for r in events if r["type"] == "record_end")
    assert end.get("samples") == len(samples), \
        f"record_end 与 CSV 行数不一致：{end.get('samples')} ≠ {len(samples)}"
    assert end.get("samples", 0) > 100, f"record_end 采样数异常 {end.get('samples')}"
    miss = end.get("missing_ratio")
    assert miss is not None and miss < 0.05, f"采样缺失率 {miss} 过高"
    return {"metadata": "ok", "command_ticks": "ok", "record_end": "ok",
            "samples": end.get("samples"), "missing_ratio": miss}


def wait_mode(bridge, modes, timeout=12.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        last = bridge.latest
        if last and last["mode"] in modes:
            return last
        time.sleep(0.02)
    raise AssertionError(f"等待模式 {modes} 超时，最后状态 mode={bridge.latest and bridge.latest['mode']}")


def wait_move_gate(bridge, timeout=15.0):
    """POS 准入：稳摆且 |α|<5°、|θ̇|<20°/s。"""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        s = bridge.latest
        if s and s["mode"] == 2 and abs(s["alpha"]) < 500 and abs(s["theta_dot"]) < 200:
            return s
        time.sleep(0.02)
    raise AssertionError(f"等待 MOVE 准入超时：mode={s and s['mode']} α={s and s['alpha']}")


def run():
    started = time.monotonic()
    out = Path(__file__).resolve().parents[1] / "logs"
    out.mkdir(exist_ok=True)
    diag_summary = diag_unit()
    flasher_summary = flasher_unit()
    bridge = Bridge("模拟")
    server = AgentServer(bridge, port=0)   # 0=自动选空闲端口
    bridge.start()
    server.start()
    issues = []
    try:
        deadline = time.monotonic() + 10
        while bridge.latest is None and time.monotonic() < deadline:
            time.sleep(0.02)
        assert bridge.latest, "模拟设备未产生遥测"
        assert bridge.active_params and len(bridge.active_params) == N_PARAMS, "初始参数回读失败"
        assert bridge.identity.get("schema"), "握手身份缺失"

        # --- Agent TCP 通道：观测 ---
        with tcp(server.port) as sock:
            hello = request(sock, {"id": 1, "cmd": "hello"})
            assert hello["ok"] and hello["data"]["connected"], hello
            params = request(sock, {"id": 2, "cmd": "params"})["data"]
            assert len(params["values"]) == N_PARAMS and len(params["schema"]) == N_PARAMS
            base_tick = bridge.latest["tick"]

            # --- 租约互斥：B 客户端不能抢 A 的租约 ---
            assert request(sock, {"id": 3, "cmd": "acquire"})["ok"]
            with tcp(server.port) as other:
                denied = request(other, {"id": 1, "cmd": "move", "delta": 100})
                assert not denied["ok"] and "控制权" in denied["error"], denied
                stop_ok = request(other, {"id": 2, "cmd": "stop"})  # STOP 无需租约
                assert stop_ok["ok"], stop_ok

            # --- 动作任务：清故障→起摆→捕获→定位→停止 ---
            assert request(sock, {"id": 4, "cmd": "clear"})["ok"]
            assert request(sock, {"id": 5, "cmd": "start"})["ok"]
            wait_mode(bridge, (2,), timeout=12)
            capture_tick = bridge.latest["tick"] - base_tick
            wait_move_gate(bridge)
            move = request(sock, {"id": 6, "cmd": "move", "delta": 3000})
            assert move["ok"], move
            time.sleep(0.3)
            hist = list(bridge.history)
            last_bal = next(s for s in reversed(hist) if s["mode"] == 2)
            pos = next(s for s in reversed(hist) if s["mode"] == 3)
            anchor = pos["target"] - last_bal["theta"]
            assert abs(anchor - 3000) <= 100, f"POS 锚点 {anchor} ≠ +3000"
            assert request(sock, {"id": 7, "cmd": "stop"})["ok"]
            wait_mode(bridge, (4,), timeout=5)

            # --- watch 推送：起摆 1 秒，数收到的遥测推送帧 ---
            pushes = []
            assert request(sock, {"id": 8, "cmd": "watch", "on": True, "hz": 25},
                           on_push=pushes.append)["ok"]
            assert request(sock, {"id": 9, "cmd": "clear"},
                           on_push=pushes.append)["ok"]
            request(sock, {"id": 10, "cmd": "start"}, on_push=pushes.append)
            time.sleep(1.0)
            request(sock, {"id": 11, "cmd": "stop"}, on_push=pushes.append)
            request(sock, {"id": 12, "cmd": "watch", "on": False}, on_push=pushes.append)
            assert len(pushes) >= 10, f"watch 推送过少：{len(pushes)}"

            replacement = Bridge("模拟")
            replacement.start()
            try:
                deadline = time.monotonic() + 10
                while replacement.latest is None and time.monotonic() < deadline:
                    time.sleep(0.02)
                assert replacement.latest, "重连设备未产生遥测"
                server.set_bridge(replacement)
                rebound = request(sock, {"id": 13, "cmd": "status"})
                assert rebound["ok"] and rebound["data"]["identity"] == replacement.identity
            finally:
                server.set_bridge(bridge)
                replacement.close()

        # --- 记录格式：元数据/命令时间戳/结束摘要 ---
        # 这段守的是"排查时手上有没有可用的记录"。record_end 曾经因为控制
        # 线程 finally 里直接 close() 而永远写不出来（记录看不出是否漏帧），
        # 命令也曾经没有 tick，无法与遥测对齐。
        record_summary = record_unit(bridge, out)

        summary = {
            "status": "PASS",
            "capture_s": round(capture_tick / 1000, 3),
            "anchor_cdeg": anchor,
            "watch_pushes": len(pushes),
            "agent_port": server.port,
            "lease_mutex": True,
            "diag": diag_summary,
            "flasher": flasher_summary,
            "record": record_summary,
            "elapsed_s": round(time.monotonic() - started, 1),
        }
    except Exception as exc:
        summary = {"status": "FAIL", "error": str(exc)}
        raise
    finally:
        (out / "selftest.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False),
                                           encoding="utf-8")
        server.close()
        bridge.close()
    print(json.dumps(summary, ensure_ascii=True))


if __name__ == "__main__":
    run()
