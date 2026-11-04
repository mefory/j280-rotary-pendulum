"""遥测快照新旧固件兼容回归（2026-10-30）。

根因：Client.poll 只分发 len(payload)==SNAPSHOT.size 的 TELEMETRY 帧，
旧固件 40B/54B 快照被静默丢弃 ⇒ "握手成功但永远没有数据"。
unpack_snapshot 本身有三层兼容，过滤器与解包器自相矛盾。
"""
import struct
import unittest

from . import protocol as p
from .client import Client


class _FakeDevice:
    """最小串口替身：write 丢弃，read 吐预置帧字节。"""
    readonly = False

    def __init__(self, frames):
        self._buf = b"".join(frames)
        self.baudrates = (None,)

    def write(self, data):
        pass

    def read(self):
        out, self._buf = self._buf, b""
        return out

    def advance(self, ms):
        pass

    def close(self):
        pass


def telemetry_frame(payload):
    return p.Frame(p.TELEMETRY, 0, payload).encode()


class SnapshotCompatTests(unittest.TestCase):
    def _client_receives(self, payload):
        samples = []
        client = Client(_FakeDevice([telemetry_frame(payload)]),
                        sample_callback=samples.append)
        client.poll(1)
        return samples

    def test_v4_64byte_snapshot_still_flows(self):
        payload = bytes(SNAPSHOT_SIZE := p.SNAPSHOT.size)
        samples = self._client_receives(payload)
        self.assertEqual(len(samples), 1)
        self.assertEqual(samples[0]["mode"], 0)  # 补零字段可解

    def test_v3_54byte_snapshot_not_dropped(self):
        payload = bytes(p.SNAPSHOT_V3.size)
        samples = self._client_receives(payload)
        self.assertEqual(len(samples), 1)
        self.assertIsNone(samples[0]["theta_e"])   # v4 字段补 None
        self.assertIsNotNone(samples[0]["pos_trim"])

    def test_v2_40byte_snapshot_not_dropped(self):
        payload = bytes(40)
        samples = self._client_receives(payload)
        self.assertEqual(len(samples), 1)
        self.assertIsNone(samples[0]["pos_trim"])  # 诊断量补 None

    def test_v5_80byte_snapshot_decodes_raw_chain(self):
        # v5（improve-160416 树，audit_20261030_224637 在板）：64B v4 + 16B 量测链。
        # 前 64B 照 v4 布局解，后 16B 是 7 个新字段。
        payload = bytearray(p.SNAPSHOT_V5.size)
        payload[8] = 2                       # mode=2（BALANCE）：tick 4B + revision 4B → mode 在 byte8
        # 尾部 16B：theta_raw s32=0x000186a0(100000)、其余按序填 1..6
        tail = struct.pack("<ihhhhhh", 100000, 1, 2, 3, 4, 5, 6)
        payload[64:] = tail
        samples = self._client_receives(bytes(payload))
        self.assertEqual(len(samples), 1)
        self.assertEqual(samples[0]["mode"], 2)          # v4 头部布局未变
        self.assertEqual(samples[0]["theta_raw"], 100000)
        self.assertEqual(samples[0]["pos_i_trim"], 6)

    def test_v5_fits_max_payload(self):
        # 协议 MAX_PAYLOAD=64 的上限断言只管 Frame.encode（命令侧），
        # 遥测帧由 RTL 发 80B——这里钉死白名单与 struct 的一致性。
        self.assertEqual(p.SNAPSHOT_V5.size, 80)
        self.assertIn(80, p.SNAPSHOT_SIZES)

    def test_garbage_length_still_dropped(self):
        # 既不是 40/54/64 的噪声帧不能放进数据流（例如半帧/坏长帧）。
        # >64B 的帧在 encode 层就被 MAX_PAYLOAD 拒绝，不可能到达 poll。
        for size in (0, 1, 39, 41, 53, 55, 63):
            self.assertEqual(self._client_receives(bytes(size)), [],
                             f"payload={size} 不应被当作有效快照")

    def test_unpack_snapshot_matches_client_filter(self):
        # 解包器声称支持的每种长度，过滤器都必须放行
        for size in (40, p.SNAPSHOT_V3.size, p.SNAPSHOT.size):
            payload = bytes(size)
            self.assertEqual(self._client_receives(payload).__len__(), 1,
                             f"unpack_snapshot 支持的 {size}B 被过滤器丢弃")


if __name__ == "__main__":
    unittest.main()
