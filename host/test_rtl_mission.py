"""端到端任务：Python 时间线 -> 真实 UART 引脚 -> 实际顶层 + 非线性对象 -> 遥测判据。

紧凑任务：清故障 -> 起摆 -> 捕获稳摆 -> 3.5 s 后相对定位 +30° -> 追踪 2 s
-> 停止。判据：所有命令 OK 确认、CRC 零错误、捕获 <5 s、不掉摆、POS 入口
锚点恰为 +30°、窗口内剩余误差朝目标收敛 ≥2°、移动峰值 |α|<3°、停止后进入
SAFE。θ 对 30° 目标全收敛约需 10 s（见 tb_closedloop 时间线），长时收敛与
末段指标由 tb_closedloop 与 host.acceptance 的 15 s 窗口覆盖；本测试专注
真实 UART 命令链路这一唯一增量风险（iverilog 全设计约 2 万时钟/秒）。
"""
import csv
import json
import struct
import subprocess
import time

from . import protocol as p
from .schema import ROOT, SCHEMA_ID

# 2026-09-28 甩摆 v3 速度剖面：甩程比旧定点伺服长（加速到 ω_target 再刹车），
# 捕获后移，MOVE/STOP 时间线相应后移，给足捕获+静定裕量。
T_HELLO, T_CLEAR, T_START, T_MOVE, T_STOP, T_END = 300, 400, 500, 8000, 12000, 12300
DELTA = 3000
SLACK = 10   # snapshot tick lags the testbench tick counter by a couple of ticks
TREND_CDEG = 200  # arm must close at least 2° of the 30° gap inside the window


def build_timeline():
    token = 0x5A280C19
    seq = 0
    plan = []

    def emit(tick, kind, payload=b"", handshake=False):
        nonlocal seq
        body = payload if handshake else struct.pack("<I", token) + payload
        plan.append((tick, p.Frame(kind, seq, body).encode(), kind, seq, handshake))
        seq = (seq + 1) & 65535

    emit(T_HELLO, p.HELLO, struct.pack("<II", SCHEMA_ID, token), handshake=True)
    emit(T_CLEAR, p.CLEAR)
    emit(T_START, p.START)
    emit(T_MOVE, p.MOVE, struct.pack("<i", DELTA))
    emit(T_STOP, p.STOP)
    lines = []
    for tick, wire, *_ in plan:
        lines.append(f"0 {tick}")
        lines.append(f"{len(wire)} " + " ".join(f"{b:02x}" for b in wire))
    lines.append(f"0 {T_END}")
    return token, plan, "\n".join(lines) + "\n"


def run():
    started = time.monotonic()
    token, plan, stimulus = build_timeline()
    out = ROOT / "sim/out"
    out.mkdir(exist_ok=True)
    (out / "uart_mission_stimulus.txt").write_text(stimulus, encoding="ascii")
    # Board shells live under rtl/boards/<board>/; compile them too so the
    # same regression keeps working regardless of which top is elaborated.
    # .vh included: regenerated headers (schema hash, defaults) must trigger rebuild.
    sources = [ROOT / "tb/tb_uart_mission.v", *sorted((ROOT / "rtl").glob("*.v")),
               *sorted((ROOT / "rtl").glob("*.vh")),
               *sorted((ROOT / "rtl/boards").rglob("*.v"))]
    target = out / "uart_mission.vvp"
    # The vvp build costs ~a minute; skip it when nothing changed.
    stale = not target.exists() or any(s.stat().st_mtime > target.stat().st_mtime
                                       for s in sources)
    if stale:
        subprocess.run(["iverilog", "-g2012", "-I", "rtl", "-s", "tb_uart_mission", "-o",
                        str(target), "tb/tb_uart_mission.v", *map(str, sources[1:])],
                       cwd=ROOT, check=True)
    result = subprocess.run(["vvp", str(target)], cwd=ROOT, check=True,
                            capture_output=True, text=True, timeout=1500)
    (out / "uart_mission.log").write_text(result.stdout, encoding="utf-8")
    wire = bytes(int(line.split()[1], 16) for line in result.stdout.splitlines()
                 if line.startswith("RX "))
    decoder = p.Decoder()
    frames = decoder.feed(wire)
    # Telemetry streams forever, so $finish always cuts the final frame; a
    # short tail buffer is the truncated last frame, not a protocol fault.
    assert decoder.errors == 0 and len(decoder.buffer) < 10 + p.MAX_PAYLOAD, \
        "Invalid/incomplete UART frames"
    snaps = [p.unpack_snapshot(f.payload) for f in frames
             if f.kind == p.TELEMETRY and len(f.payload) in p.SNAPSHOT_SIZES]
    assert snaps, "No telemetry decoded"
    # CSV 先于断言落盘：断言失败时也保留完整证据链
    with (out / "uart_mission.csv").open("w", newline="", encoding="utf-8") as trace:
        writer = csv.writer(trace)
        writer.writerow(["tick", "mode", "alpha_cdeg", "theta_cdeg", "target_cdeg", "duty"])
        for s in snaps:
            writer.writerow([s["tick"], s["mode"], s["alpha"], s["theta"], s["target"], s["duty"]])
    # Every command must be acknowledged exactly once, OK, with our token.
    replies = [f for f in frames if f.kind == p.RESPONSE]
    for (tick, request, kind, seq, handshake) in plan:
        match = [f for f in replies if f.seq == seq]
        assert len(match) == 1, (kind, seq, len(match))
        command, code, revision, stamp, ack_token = struct.unpack_from("<BBIII", match[0].payload)
        assert (command, code, ack_token) == (kind, p.OK, token), (kind, code, ack_token)


    captures = [s for s in snaps if s["mode"] == 2 and s["tick"] >= T_START - SLACK]
    assert captures, "Never entered BALANCE"
    capture_tick = captures[0]["tick"]
    hold = [s for s in snaps if capture_tick < s["tick"] < T_STOP - SLACK]
    dropped = [s["tick"] for s in hold if s["mode"] in (1, 4)]
    assert not dropped, f"Dropped out of BALANCE at ticks {dropped[:5]}"

    pos = [s for s in hold if s["mode"] == 3]
    assert pos, "Never entered POS"
    # mode can latch one snapshot before pos_tgt; the anchor is the first
    # sample where target-theta equals the requested relative displacement.
    anchor_shot = next((s for s in pos
                        if abs(s["target"] - s["theta"] - DELTA) <= 100), None)
    assert anchor_shot, "No POS snapshot anchored at the requested +30°"
    anchor = anchor_shot["target"] - anchor_shot["theta"]
    settled = [s for s in pos if s["tick"] >= anchor_shot["tick"]]
    # Full 30° convergence takes ~10 s (tb_closedloop timeline); inside this
    # compact window only require steady progress toward the target.
    progress = anchor - (settled[-1]["target"] - settled[-1]["theta"])
    assert progress >= TREND_CDEG, f"Arm not closing the gap: {progress} cdeg"
    moving_peak = max(abs(s["alpha"]) for s in settled) / 100
    tail = [s for s in snaps if s["tick"] >= T_STOP + 5 * SLACK]
    assert tail and all(s["mode"] == 4 for s in tail), "STOP did not reach SAFE"
    assert all(s["revision"] == 0 for s in snaps), "Unexpected revision change"

    summary = {
        "capture_ms": (capture_tick - T_START),
        "anchor_cdeg": anchor,
        "progress_cdeg": progress,
        "moving_alpha": round(moving_peak, 3),
        "pos_samples": len(settled),
        "snapshots": len(snaps),
        "responses": len(replies),
        "crc_errors": decoder.errors,
        "status": "PASS",
    }
    assert summary["capture_ms"] < 5000 and moving_peak < 3
    (out / "uart_mission_result.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print("UART mission PASS", json.dumps(summary), f"elapsed {time.monotonic()-started:.0f}s")


if __name__ == "__main__":
    run()
