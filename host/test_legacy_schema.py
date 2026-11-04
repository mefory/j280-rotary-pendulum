"""旧参数表固件（LEGACY_SCHEMAS 降级）回归（2026-11-01）。

场景：audit_20261020_200909 一代的 14 参固件（schema 0x099ff7a2）——
上位机现表 16 参（15/16 为 10-24 追加）。此前 client.connect 对 schema
不一致一律拒连；现按白名单降级：只读写设备支持的前 N 项。
"""
import struct
import unittest

from . import protocol as p
from .client import Client
from .schema import PARAMS, SCHEMA_ID, validate

LEGACY_SCHEMA = 0x099ff7a2     # 与 host.schema.LEGACY_SCHEMAS 一致
LEGACY_COUNT = 14


class LegacyDevice:
    """V2 协议服务端最小实现：模拟 14 参固件。

    READ/SET 只接受 id 1..14（其余回 BAD_ID=2）；HELLO 回 12 字节
    （无构建时间戳epoch，古早固件形态）。
    """
    readonly = False
    baudrates = (None,)

    def __init__(self, schema=LEGACY_SCHEMA, count=LEGACY_COUNT):
        self.schema, self.count = schema, count
        self._rx = p.Decoder()
        self._pending = b""
        self.sets = []             # 记录收到的 (id, value)
        self.token = None
        # 参数存储：SET 落库、READ 回读（模拟真固件的 COMMIT 生效语义）
        self.values = {i: 1000 + i for i in range(count)}

    def write(self, data):
        for frame in self._rx.feed(data):
            self._serve(frame)

    def _serve(self, frame):
        if frame.kind == p.HELLO:
            # handshake=True：载荷 = <II>(schema_id, token)，无 token 前缀
            token = struct.unpack_from("<I", frame.payload, 4)[0]
            offered_schema = struct.unpack_from("<I", frame.payload, 0)[0]
            if offered_schema != self.schema:
                # 古早固件实测行为（audit_20261020_200909 在板）：不认识的
                # schema 回 code=7（BAD_SCHEMA）且 token=0
                self._reply(frame.kind, frame.seq,
                            struct.pack("<BBIII", p.HELLO, 7, 0, 0, 0))
                return
            self.token = token
            # 身份 12 字节：schema + build + caps（无 epoch，古早固件形态）
            self._reply(frame.kind, frame.seq,
                        struct.pack("<BBIII", p.HELLO, 0, 1, 0, token)
                        + struct.pack("<III", self.schema, 0x14260923, 0xEF))
            return
        token = struct.unpack_from("<I", frame.payload, 0)[0]
        self.token = token
        frame = p.Frame(frame.kind, frame.seq, frame.payload[4:])
        head = lambda kind: struct.pack("<BBIII", kind, 0, 1, 0, token)
        if frame.kind in (p.BEGIN, p.COMMIT, p.ABORT, p.DEFAULTS):
            self._reply(frame.kind, frame.seq, head(frame.kind))
        elif frame.kind == p.READ:
            (pid,) = struct.unpack_from("<H", frame.payload)
            if pid < self.count:
                self._reply(frame.kind, frame.seq,
                            head(p.READ) + struct.pack("<i", self.values[pid]))
            else:
                self._reply(frame.kind, frame.seq,
                            struct.pack("<BBIII", p.READ, 2, 1, 0, token))
        elif frame.kind == p.SET:
            pid, value = struct.unpack_from("<Hi", frame.payload)
            self.sets.append((pid, value))
            if pid < self.count:
                self.values[pid] = value
                code = 0
            else:
                code = 2
            self._reply(frame.kind, frame.seq, struct.pack("<BBIII", p.SET, code, 1, 0, token))
        else:
            self._reply(frame.kind, frame.seq, head(frame.kind))

    def _ack(self, kind, seq):
        self._reply(kind, seq, struct.pack("<BBIII", kind, 0, 1, 0, self.token or 0))

    def _reply(self, kind, seq, payload):
        # 一切应答（含 HELLO）都是 RESPONSE 帧；kind 参数仅记录用
        self._pending += p.Frame(p.RESPONSE, seq, payload).encode()

    def read(self):
        out, self._pending = self._pending, b""
        return out

    def advance(self, ms):
        pass

    def close(self):
        pass

    def set_baudrate(self, baudrate):
        pass


class LegacySchemaTests(unittest.TestCase):
    def _connect(self, device):
        client = Client(device)
        client.connect()
        return client

    def test_legacy_schema_connects_degraded(self):
        dev = LegacyDevice()
        client = self._connect(dev)
        self.assertTrue(client.connected)
        self.assertEqual(client.param_count, LEGACY_COUNT)
        self.assertFalse(client.identity["schema_matched"])
        self.assertEqual(client.identity["param_count"], LEGACY_COUNT)

    def test_unknown_schema_still_rejected(self):
        dev = LegacyDevice(schema=0x0BADF00D)
        with self.assertRaises(ValueError):
            self._connect(dev)

    def test_apply_sends_only_first_14_sets(self):
        dev = LegacyDevice()
        client = self._connect(dev)
        values = [p["default"] for p in PARAMS]     # GUI 传全表 16 项
        result = client.apply(values)
        self.assertEqual([pid for pid, _ in dev.sets], list(range(0, 14)))
        self.assertEqual(len(dev.sets), 14)
        # 回读=写入值；返回恒 16 长，设备表外补 None
        self.assertEqual(result[:14], values[:14])
        self.assertEqual(result[14:], [None, None])

    def test_apply_out_of_range_still_rejected(self):
        dev = LegacyDevice()
        client = self._connect(dev)
        values = [p["default"] for p in PARAMS]
        values[0] = 99999                            # 越界
        with self.assertRaises(ValueError):
            client.apply(values)
        self.assertEqual(dev.sets, [])               # 事务未发任何 SET

    def test_read_params_safe_pads_to_full_table(self):
        dev = LegacyDevice()
        client = self._connect(dev)
        values, failures = client.read_params_safe()
        self.assertEqual(len(values), len(PARAMS))
        self.assertEqual(values[:14], [1000 + i for i in range(0, 14)])
        self.assertEqual(values[14:], [None, None])
        self.assertEqual(failures, [])               # 未尝试的 id 不算失败

    def test_validate_count_parameter(self):
        values14 = [p["default"] for p in PARAMS[:14]]
        validate(values14, count=14)                 # 不抛
        with self.assertRaises(ValueError):
            validate(values14)                       # 全表口径下 14 项算错


if __name__ == "__main__":
    unittest.main()
