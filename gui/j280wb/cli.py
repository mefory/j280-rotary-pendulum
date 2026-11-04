"""Agent 命令行客户端：脚本/agent 框架驱动上位机的入口。

用法（默认连 127.0.0.1:28080）：
  python -m j280wb.cli status
  python -m j280wb.cli params
  python -m j280wb.cli clear|start|stop|balance|calibrate
  python -m j280wb.cli move 3000
  python -m j280wb.cli trial 3000
  python -m j280wb.cli read 0
  python -m j280wb.cli apply 22893 22375 413 14793 375 375 6000 3000 6000 3000
  python -m j280wb.cli defaults
  python -m j280wb.cli watch 5            # 5Hz 打印遥测
  python -m j280wb.cli record logs/x | record off
                             # 产出 x.csv（遥测）+ x_events.jsonl（事件）；传 .jsonl/.csv 结尾也兼容
  python -m j280wb.cli acquire|renew|release
退出码 0 成功；ok=false 或异常时非 0 并打印错误。机器可读输出始终为单行 JSON。
"""
import argparse
import json
import socket
import sys
import time

from . import AGENT_DEFAULT_PORT


def request(sock, obj, timeout=8.0, on_push=None):
    """发送一条请求并等待其响应；途经的推送行交给 on_push 或跳过。"""
    sock.sendall((json.dumps(obj) + "\n").encode("utf-8"))
    sock.settimeout(timeout)
    buffer = bytearray()
    while True:
        while b"\n" not in buffer:
            data = sock.recv(4096)
            if not data:
                raise ConnectionError("连接被关闭")
            buffer.extend(data)
        line, _, rest = bytes(buffer).partition(b"\n")
        buffer[:] = rest
        obj_in = json.loads(line)
        if "push" in obj_in:
            if on_push:
                on_push(obj_in)
            continue
        return obj_in


def watch(sock, hz, duration):
    pushes = []
    request(sock, {"cmd": "watch", "on": True, "hz": hz}, on_push=pushes.append)
    deadline = time.monotonic() + duration
    sock.settimeout(1.0)
    buffer = bytearray()
    while time.monotonic() < deadline:
        try:
            data = sock.recv(4096)
        except socket.timeout:
            continue
        if not data:
            break
        buffer.extend(data)
        while b"\n" in buffer:
            line, _, rest = bytes(buffer).partition(b"\n")
            buffer[:] = rest
            obj = json.loads(line)
            if obj.get("push") == "sample":
                s = obj["data"]
                print(json.dumps({"tick": s["tick"], "mode": s["mode"], "alpha": s["alpha"],
                                  "theta": s["theta"], "target": s["target"], "duty": s["duty"]}))
                pushes.append(obj)
    request(sock, {"cmd": "watch", "on": False}, on_push=pushes.append)
    return len(pushes)


def main():
    parser = argparse.ArgumentParser(description="J280 上位机 agent 客户端")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=AGENT_DEFAULT_PORT)
    parser.add_argument("command")
    parser.add_argument("args", nargs="*")
    args = parser.parse_args()

    sock = socket.create_connection((args.host, args.port), timeout=5)
    try:
        if args.command == "watch":
            watch(sock, float(args.args[0]) if args.args else 5.0,
                  float(args.args[1]) if len(args.args) > 1 else 10.0)
            return 0
        payload = {"cmd": args.command}
        if args.command == "move" or args.command == "trial":
            payload["delta"] = int(args.args[0])
        elif args.command == "read":
            payload["id"] = int(args.args[0])
        elif args.command == "apply":
            payload["values"] = [int(v) for v in args.args]
        elif args.command == "record":
            payload["path"] = None if (args.args and args.args[0] == "off") else args.args[0]
        elif args.command in ("acquire", "renew") and args.args:
            payload["seconds"] = float(args.args[0])
        elif args.command == "telemetry" and args.args:
            payload["n"] = int(args.args[0])
        result = request(sock, payload)
        print(json.dumps(result, ensure_ascii=True))
        if not result.get("ok"):
            return 2
        return 0
    finally:
        sock.close()


if __name__ == "__main__":
    sys.exit(main())
