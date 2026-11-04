"""Agent JSON 接口：本机 TCP 换行分隔 JSON，多客户端可同时观察，命令经
共享 Bridge 串行执行。观测永远开放；动作命令需要控制租约（STOP/CLEAR
例外，安全优先）。所有响应 {"id":..,"ok":..,"data"|"error":..}，主动推送
{"push":..,"data":..}。

命令一览：
  hello                        身份/后端/协议
  status                       最新状态摘要
  params                       当前生效参数+参数表
  read id                      读单个参数
  apply values                 停机成组提交 10 个定点值
  defaults                     恢复设备默认（停机）
  clear|start|stop|balance|calibrate   动作命令
  move delta                   相对定位（0.01°）
  trial delta                  25s 全流程自动试验
  telemetry n                  最近 n 拍遥测（≤500）
  watch on/off hz              遥测推送开关（≤50Hz，慢客户端丢帧计数）
  acquire|renew|release        控制租约（默认 30s）
  record path|null             开始/停止混合记录（<基础名>.csv 遥测 + <基础名>_events.jsonl 事件；
                               path 传 .jsonl/.csv 结尾也兼容，剥掉当基础名）
"""
import json
import queue
import socket
import threading
import time
import uuid

from host import protocol as p

from . import AGENT_DEFAULT_PORT
from .bridge import Bridge, MODES


def status_payload(bridge):
    latest = bridge.latest
    data = {"connected": bridge.connected, "backend": bridge.kind, "label": bridge.label,
            "readonly": bridge.readonly, "revision": bridge.revision, "lease": bridge.lease(),
            "identity": bridge.identity}
    if latest:
        data.update({k: latest[k] for k in
                     ("tick", "mode", "alpha", "alpha_dot", "theta", "theta_dot",
                      "duty", "reference", "target", "fault", "calib_done", "run_en")})
        data["mode_name"] = MODES.get(latest["mode"], "未知")
    return data


class AgentConnection:
    def __init__(self, server, sock):
        self.server = server
        self.sock = sock
        self.id = uuid.uuid4().hex[:8]
        self.out = queue.Queue(maxsize=2000)
        self.push_samples = False
        self.min_interval = 0.0
        self._last_push = 0.0
        self.dropped = 0

    def send(self, obj):
        try:
            self.out.put_nowait(obj)
        except queue.Full:
            self.dropped += 1


class AgentServer:
    def __init__(self, bridge, port=AGENT_DEFAULT_PORT):
        self.bridge = bridge
        self.port = port
        self.clients = {}
        self._hook = None
        self._stop = threading.Event()
        self._thread = None
        self._listener = None

    def start(self):
        # bind 同步完成，调用方此后读到的 self.port 一定是真实端口
        self._listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._listener.bind(("127.0.0.1", self.port))
        self._listener.listen(4)
        self._listener.settimeout(0.5)
        self.port = self._listener.getsockname()[1]
        self._hook = self.bridge.on_sample(self._on_sample)
        self._thread = threading.Thread(target=self._accept_loop, name="agent", daemon=True)
        self._thread.start()

    def set_bridge(self, bridge):
        if self._hook:
            self._hook()
            self._hook = None
        self.bridge = bridge
        if self._listener:
            self._hook = bridge.on_sample(self._on_sample)

    def close(self):
        self._stop.set()
        if self._listener:
            try:
                self._listener.close()
            except OSError:
                pass
        if self._thread:
            self._thread.join(2)
        for conn in list(self.clients):
            self._drop(conn)
        if self._hook:
            self._hook()

    @property
    def client_count(self):
        return len(self.clients)

    # ---------- 遥测推送 ----------
    def _on_sample(self, sample):
        now = time.monotonic()
        for conn in list(self.clients.values()):
            if not conn.push_samples or now - conn._last_push < conn.min_interval:
                continue
            conn._last_push = now
            conn.send({"push": "sample", "data": sample})

    # ---------- 监听 ----------
    def _accept_loop(self):
        while not self._stop.is_set():
            try:
                sock, _ = self._listener.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            conn = AgentConnection(self, sock)
            self.clients[conn.id] = conn
            threading.Thread(target=self._reader, args=(conn,), daemon=True).start()
            threading.Thread(target=self._writer, args=(conn,), daemon=True).start()

    def _drop(self, conn):
        self.clients.pop(conn.id, None)
        self.bridge.release_lease(f"agent:{conn.id}")
        try:
            conn.sock.close()
        except OSError:
            pass

    def _writer(self, conn):
        try:
            while True:
                obj = conn.out.get()
                if obj is None:
                    break
                conn.sock.sendall((json.dumps(obj, ensure_ascii=False) + "\n").encode("utf-8"))
        except OSError:
            pass
        finally:
            self._drop(conn)

    def _reader(self, conn):
        buffer = bytearray()
        try:
            while not self._stop.is_set():
                data = conn.sock.recv(4096)
                if not data:
                    break
                buffer.extend(data)
                while b"\n" in buffer:
                    line, _, rest = bytes(buffer).partition(b"\n")
                    buffer[:] = rest
                    if line.strip():
                        reply = self._handle(conn, line)
                        conn.send(reply)
        except (OSError, ValueError):
            pass
        finally:
            try:
                conn.out.put_nowait(None)
            except queue.Full:
                pass

    # ---------- 请求处理 ----------
    def _handle(self, conn, line):
        request_id = None
        try:
            req = json.loads(line.decode("utf-8"))
            request_id = req.get("id")
            cmd = req.get("cmd", "")
            source = f"agent:{conn.id}"
            data = self._execute(conn, source, cmd, req)
            return {"id": request_id, "ok": True, "data": data}
        except Exception as exc:
            payload = {"id": request_id, "ok": False, "error": str(exc)}
            code = getattr(exc, "code", None)
            if code is not None:
                payload["code"] = code
            return payload

    def _execute(self, conn, source, cmd, req):
        b = self.bridge
        if cmd in ("hello", "status"):
            data = status_payload(b)
            data["clients"] = self.client_count
            return data
        if cmd == "params":
            from host.schema import PARAMS
            if b.active_params is None:
                values = [] if b.readonly else b.submit("read", source=source)
            else:
                values = b.active_params
            return {"values": values, "revision": b.revision,
                    "schema": [{k: item[k] for k in ("id", "key", "label", "unit", "scale",
                                                     "min", "max", "default", "group")}
                               for item in PARAMS]}
        if cmd == "read":
            return b.submit("read_one", int(req["id"]), source=source)
        if cmd == "apply":
            return {"values": b.submit("apply", [int(v) for v in req["values"]], source=source),
                    "revision": b.revision}
        if cmd == "defaults":
            return {"values": b.submit("defaults", source=source), "revision": b.revision}
        if cmd in ("clear", "start", "stop", "balance", "calibrate"):
            kinds = {"clear": p.CLEAR, "start": p.START, "stop": p.STOP,
                     "balance": p.BALANCE, "calibrate": p.CALIBRATE}
            return b.submit("command", (kinds[cmd], b""), source=source)
        if cmd == "move":
            return b.submit("command", (p.MOVE, __import__("struct").pack("<i", int(req["delta"]))),
                            source=source)
        if cmd == "trial":
            b.run_trial(source, int(req["delta"]))
            return {"started": True, "delta": int(req["delta"])}
        if cmd == "telemetry":
            n = max(1, min(int(req.get("n", 1)), 500))
            with b._lock:
                return list(b.history)[-n:]
        if cmd == "watch":
            on = bool(req.get("on", True))
            conn.push_samples = on
            conn.min_interval = 1.0 / max(1, min(float(req.get("hz", 25)), 50.0))
            return {"watch": on, "hz": round(1.0 / conn.min_interval, 1)}
        if cmd == "acquire":
            if not b.acquire_lease(source, float(req.get("seconds", 30))):
                raise PermissionError(f"控制权由 {b.lease()['owner']} 持有")
            return {"lease": b.lease()}
        if cmd == "renew":
            if not b.renew_lease(source, float(req.get("seconds", 30))):
                raise PermissionError("当前不持有控制权")
            return {"lease": b.lease()}
        if cmd == "release":
            b.release_lease(source)
            return {"released": True}
        if cmd == "record":
            return {"record": b.submit("record", req.get("path"), source=source)}
        raise ValueError(f"未知命令 {cmd}")
