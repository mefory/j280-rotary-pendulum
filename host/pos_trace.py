"""定位(POS)入口行为对照：RTL 实测 vs Python 模型，同一口径打印误差与输出轨迹。

为什么需要它：`host.test_rtl_mission` 只给一个"2 秒内朝目标收敛 ≥2°"的通过/失败
结论，不告诉你失败在哪里。定位环的问题几乎都出在**入口瞬态**——进入 POS 的头几
百毫秒输出方向/幅度不对，之后才收敛。本模块把模型与 RTL 的入口轨迹并排打出来，
直接看出是"判据过时"还是"RTL 与模型不一致"。

用法（在仓库根目录）：
    python -m host.pos_trace                 # 模型 + 当前 RTL 日志 + 上次提交的 RTL 日志
    python -m host.pos_trace --model         # 只跑模型
    python -m host.pos_trace --rtl           # 只解已有日志，不跑仿真

RTL 轨迹来自 `sim/out/uart_mission.log`（跑 `python -m host.test_rtl_mission` 生成）；
"上次提交的"来自 `git show HEAD:sim/out/uart_mission.log`，用于和本次对照。
"""
import argparse
import struct
import subprocess
from pathlib import Path

from . import protocol as p
from .client import Client
from .devices import SimDevice
from .schema import ROOT

T_STOP, SLACK = 6000, 10          # 与 host.test_rtl_mission 一致
DELTA = 3000                      # +30°，与两处测试一致


def _decode_log(text):
    wire = bytes(int(line.split()[1], 16) for line in text.splitlines()
                 if line.startswith("RX "))
    frames = p.Decoder().feed(wire)
    # 同 client.poll：按白名单收 v2/v3/v4 三代快照，旧固件日志不再丢帧
    return [p.unpack_snapshot(f.payload) for f in frames
            if f.kind == p.TELEMETRY and len(f.payload) in p.SNAPSHOT_SIZES]


def _model_samples():
    device = SimDevice(7)
    samples = []
    client = Client(device, samples.append)
    client.connect()
    client.request(p.CLEAR)
    client.poll(30)               # 与 host.acceptance 一致：先在 IDLE 里预热传感器历史
    client.request(p.START)
    start = device.plant.ticks
    entered = None
    for _ in range(2500):
        client.poll(10)
        if device.plant.ticks - start >= 8000 and entered is None:
            client.request(p.MOVE, struct.pack("<i", DELTA))
            entered = device.plant.ticks
    return samples


def report(samples, label):
    """打印 POS 入口后的误差/输出轨迹；2 秒窗口口径与 test_rtl_mission 相同。"""
    print(f"\n=== {label}：{len(samples)} 帧 ===")
    pos = [s for s in samples if s["mode"] == 3]
    if not pos:
        print("  未进入 POS")
        return
    anchor = next((s for s in pos if abs(s["target"] - s["theta"] - DELTA) <= 100), None)
    if anchor is None:
        print("  未找到锚点（target-theta 应≈3000 cdeg）")
        return
    t0 = anchor["tick"]
    window = [s for s in pos if t0 <= s["tick"] < t0 + 2000]
    if not window:
        print("  2 秒窗口为空")
        return
    errors = [s["target"] - s["theta"] for s in window]
    reverse = max(errors) - errors[0]
    print(f"  锚点 tick={t0}  首值={errors[0]} → 窗口末={errors[-1]}  "
          f"进度={errors[0] - errors[-1]}（判据 ≥200 → "
          f"{'PASS' if errors[0] - errors[-1] >= 200 else 'FAIL'}）")
    print(f"  入口反向幅度={reverse} cdeg；POS 段 |alpha| 峰值="
          f"{max(abs(s['alpha']) for s in pos) / 100:.2f}°")
    print("  每 250ms：err / duty")
    for s in window:
        if (s["tick"] - t0) % 250 < 2:
            print(f"    t={(s['tick'] - t0) / 1000:5.2f}s  err={s['target'] - s['theta']:5d}  "
                  f"duty={s['duty']:7d}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", action="store_true", help="只跑模型")
    parser.add_argument("--rtl", action="store_true", help="只解已有 RTL 日志，不跑仿真")
    args = parser.parse_args()
    if not args.rtl:
        report(_model_samples(), "Python 模型（SimDevice + plant.py 控制器镜像）")
    if not args.model:
        current = ROOT / "sim/out/uart_mission.log"
        if current.is_file():
            report(_decode_log(current.read_text(encoding="utf-8")), "本次 RTL（sim/out/uart_mission.log）")
        else:
            print("\n（没有 sim/out/uart_mission.log；先跑 python -m host.test_rtl_mission）")
        head = subprocess.run(["git", "show", "HEAD:sim/out/uart_mission.log"], cwd=ROOT,
                              capture_output=True, text=True, encoding="utf-8", errors="replace")
        if head.returncode == 0 and head.stdout:
            report(_decode_log(head.stdout), "HEAD 提交里的 RTL 日志（上次通过的对照）")


if __name__ == "__main__":
    main()
