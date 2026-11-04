"""烧录页比特流枚举回归（2026-10-30；2026-11-01 增补增量缓存用例）。

用户问题：可烧录固件只有 25 个——list_bitstreams 写死 limit=25 按修改
时间截断，不是"不能枚举全部"。
2026-11-01 追加：全量枚举后冷缓存首扫 ~44s 且新构建即触发全量重读——
list_bitstreams_cached 按 (mtime,size) 增量复用，只解析新增/变动文件。
"""
import tempfile
import time
import unittest
from pathlib import Path
from unittest import mock

from . import flasher


class ListBitstreamsCachedTests(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.cache_file = Path(self._tmp.name) / "cache.json"
        patcher = mock.patch.object(flasher, "_SCAN_CACHE_FILE", self.cache_file)
        patcher.start()
        self.addCleanup(patcher.stop)
        flasher._scan_cache.clear()
        self.parses = []
        self._real = flasher.parse_fs_usercode

        def counting(path):
            self.parses.append(Path(path).name)
            return self._real(path)

        p2 = mock.patch.object(flasher, "parse_fs_usercode", counting)
        p2.start()
        self.addCleanup(p2.stop)

    def _root_with(self, n):
        root = Path(self._tmp.name) / f"tree{n}"
        base = root / "t20k_build"
        import os
        for i in range(n):
            d = base / f"audit_{i}" / "t20k" / "impl" / "pnr"
            d.mkdir(parents=True)
            f = d / f"fw{i}.fs"
            f.write_bytes(b"\x00" * 16)
            # 显式旧时间戳：同秒内创建的文件 mtime 并列会让"新→旧"排序不稳定
            old = time.time() - 1000
            os.utime(f, (old, old))
        return root

    def test_second_scan_reuses_all(self):
        root = self._root_with(3)
        first = flasher.list_bitstreams_cached("t20k", root=root)
        self.assertEqual(len(first), 3)
        self.assertEqual(len(self.parses), 3)
        self.parses.clear()
        second = flasher.list_bitstreams_cached("t20k", root=root)
        self.assertEqual(len(self.parses), 0)        # 未变文件零重读
        self.assertEqual([e["path"] for e in second],
                         [e["path"] for e in first])

    def test_new_build_parses_only_new(self):
        root = self._root_with(3)
        flasher.list_bitstreams_cached("t20k", root=root)
        self.parses.clear()
        d = root / "t20k_build" / "audit_new" / "t20k" / "impl" / "pnr"
        d.mkdir(parents=True)
        (d / "fw_new.fs").write_bytes(b"\x00" * 16)
        result = flasher.list_bitstreams_cached("t20k", root=root)
        self.assertEqual(len(result), 4)
        self.assertEqual(self.parses, ["fw_new.fs"])  # 只有新文件被解析
        self.assertEqual(result[0]["build"], "audit_new")

    def test_cache_persists_across_cache_dict_loss(self):
        # 进程重启（_scan_cache 空）后从磁盘缓存恢复，依旧零重读
        root = self._root_with(2)
        flasher.list_bitstreams_cached("t20k", root=root)
        self.parses.clear()
        flasher._scan_cache.clear()
        flasher.list_bitstreams_cached("t20k", root=root)
        self.assertEqual(len(self.parses), 0)


class ListBitstreamsTests(unittest.TestCase):
    def _make_fs(self, base, name, age_s=0):
        d = base / "audit_x" / "t20k" / "impl" / "pnr"
        d.mkdir(parents=True, exist_ok=True)
        f = d / f"{name}.fs"
        f.write_bytes(b"\x00" * 16)
        stamp = time.time() - age_s
        import os
        os.utime(f, (stamp, stamp))
        return f

    def test_default_enumerates_all(self):
        # 2026-10-30 起：默认全量枚举（此前写死 25 截断，439 份只显示 25 份）
        with tempfile.TemporaryDirectory() as tmp:
            base = Path(tmp) / "t20k_build"
            for i in range(30):
                d = base / f"audit_{i}" / "t20k" / "impl" / "pnr"
                d.mkdir(parents=True)
                f = d / f"fw{i}.fs"
                f.write_bytes(b"\x00" * 16)
            result = flasher.list_bitstreams("t20k", root=Path(tmp))
            self.assertEqual(len(result), 30)

    def test_explicit_limit_still_truncates(self):
        # limit 保留整数截断能力（调用方想省 stat 时可用）
        with tempfile.TemporaryDirectory() as tmp:
            base = Path(tmp) / "t20k_build"
            for i in range(30):
                d = base / f"audit_{i}" / "t20k" / "impl" / "pnr"
                d.mkdir(parents=True)
                f = d / f"fw{i}.fs"
                f.write_bytes(b"\x00" * 16)
            result = flasher.list_bitstreams("t20k", root=Path(tmp), limit=5)
            self.assertEqual(len(result), 5)

    def test_can_enumerate_all(self):
        # 期望：limit=None 枚举全部，不再藏住旧固件
        with tempfile.TemporaryDirectory() as tmp:
            base = Path(tmp) / "t20k_build"
            for i in range(30):
                d = base / f"audit_{i}" / "t20k" / "impl" / "pnr"
                d.mkdir(parents=True)
                f = d / f"fw{i}.fs"
                f.write_bytes(b"\x00" * 16)
            result = flasher.list_bitstreams("t20k", root=Path(tmp), limit=None)
            self.assertEqual(len(result), 30)

    def test_sorted_newest_first(self):
        with tempfile.TemporaryDirectory() as tmp:
            base = Path(tmp) / "t20k_build"
            self._make_fs(base, "old", age_s=1000)
            self._make_fs(base, "new", age_s=0)
            result = flasher.list_bitstreams("t20k", root=Path(tmp), limit=None)
            self.assertEqual([r["build"] for r in result], ["audit_x", "audit_x"])


if __name__ == "__main__":
    unittest.main()
