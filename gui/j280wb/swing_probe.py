"""DE10 单次短时起摆探针：记录遥测，超时急停并恢复原参数。"""
import argparse
import json
import socket
import threading
import time
from datetime import datetime
from pathlib import Path

from .cli import request


def command(sock, name, **fields):
    response = request(sock, {"cmd": name, **fields}, timeout=4.0)
    if not response.get("ok"):
        raise RuntimeError(f"{name}: {response.get('error')}")
    return response["data"]


def emergency_stop():
    try:
        with socket.create_connection(("127.0.0.1", 28080), timeout=2) as sock:
            command(sock, "stop")
    except (OSError, RuntimeError):
        pass


def wait_mode(sock, expected, timeout=0.5):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        status = command(sock, "status")
        if status["mode"] == expected:
            return status
        time.sleep(0.02)
    raise RuntimeError(f"等待模式 {expected} 超时")


def ensure_safe(sock, timeout=0.6):
    status = command(sock, "status")
    if status["mode"] not in (0, 4) or status["run_en"]:
        command(sock, "stop")
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        status = command(sock, "status")
        if status["mode"] in (0, 4) and not status["run_en"] and not status["duty"]:
            return status
        time.sleep(0.02)
    raise RuntimeError("停止后未确认输出归零")


def run(amplitude, fling_delta, duration, continuous_rotation_ok=False):
    record_base = Path("logs") / f"de10_swing_{amplitude}_{fling_delta}_{datetime.now():%Y%m%d_%H%M%S}"
    with socket.create_connection(("127.0.0.1", 28080), timeout=3) as sock:
        status = command(sock, "status")
        if (not status["connected"] or status["backend"] != "实板" or
            status["identity"]["board"] != "DE10-Standard" or
            status["mode"] not in (0, 4) or status["fault"] or
            not status["calib_done"] or status["run_en"] or status["duty"] or
            abs(abs(status["alpha"]) - 18000) > 1500 or
            abs(status["alpha_dot"]) > 300 or abs(status["theta_dot"]) > 200):
            raise RuntimeError("预检失败：设备未处于静止、下垂、已标定的安全状态")
        time.sleep(0.25)
        settled = command(sock, "status")
        if (settled["mode"] not in (0, 4) or settled["fault"] or
            settled["run_en"] or settled["duty"] or
            abs(abs(settled["alpha"]) - 18000) > 1500 or
            abs(settled["alpha_dot"]) > 300 or
            abs(settled["theta_dot"]) > 200 or
            abs(settled["theta"] - status["theta"]) > 50):
            raise RuntimeError("预检失败：摆杆或摆臂仍在运动，等待完全静止再测")
        status = settled
        if not continuous_rotation_ok and abs(status["theta"]) + 15000 > 36000:
            raise RuntimeError("累计臂角加本次最大行程可能超过 ±360°；先确认线束或手动回中")
        original = command(sock, "params")["values"]
        if len(original) != 11:
            raise RuntimeError("参数表不完整，拒绝起摆")
        command(sock, "acquire", seconds=20)
        watchdog = threading.Timer(duration + 1.0, emergency_stop)
        watchdog.start()
        stopped = False
        try:
            desired = original.copy()
            desired[6] = amplitude
            desired[10] = fling_delta
            command(sock, "apply", values=desired)
            command(sock, "record", path=str(record_base))
            if status["mode"] == 4:
                command(sock, "stop")
                wait_mode(sock, 0)
            command(sock, "start")
            start = time.monotonic()
            arm_start = status["theta"]
            balance_since = None
            reason = "限时结束"
            while time.monotonic() - start < duration:
                time.sleep(0.04)
                current = command(sock, "status")
                if current["mode"] == 2:
                    if balance_since is None:
                        balance_since = time.monotonic()
                    elif time.monotonic() - balance_since >= 0.4:
                        reason = "进入稳摆并观察 0.4 s"
                        break
                elif current["mode"] == 4:
                    reason = "已回安全态"
                    break
                if abs(current["theta"] - arm_start) > 15000:
                    reason = "摆臂位移超过 150°"
                    break
                if current["fault"]:
                    reason = "硬件故障"
                    break
            final_status = ensure_safe(sock)
            stopped = True
            print(json.dumps({"record_csv": str(record_base) + ".csv",
                              "record_events": str(record_base) + "_events.jsonl",
                              "reason": reason,
                              "elapsed_s": round(time.monotonic()-start, 3),
                              "last": final_status}, ensure_ascii=False))
        finally:
            if not stopped:
                try:
                    ensure_safe(sock)
                    stopped = True
                except (OSError, RuntimeError):
                    pass
            if stopped:
                watchdog.cancel()
            try:
                command(sock, "record", path=None)
                current = command(sock, "status")
                if current["mode"] in (0, 4) and not current["run_en"] and not current["duty"]:
                    command(sock, "apply", values=original)
            finally:
                command(sock, "release")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--amplitude", type=int, required=True, help="甩摆限幅，单位 0.01%%")
    parser.add_argument("--fling-delta", type=int, default=11800, help="甩摆目标，单位 0.01°")
    parser.add_argument("--duration", type=float, default=3.0, help="最长通电时间，单位 s")
    parser.add_argument("--continuous-rotation-ok", action="store_true",
                        help="已确认摆臂线束经滑环，允许累计臂角超过 ±360°")
    args = parser.parse_args()
    if not 1000 <= args.amplitude <= 7500 or not 3000 <= args.fling_delta <= 14000 or not 0.5 <= args.duration <= 4.0:
        parser.error("超出短时探针安全范围")
    run(args.amplitude, args.fling_delta, args.duration, args.continuous_rotation_ok)
