"""烧录作业构造与外部 Flash 探测回归（2026-11-01）。

背景：用户手动流程 = 高云 Programmer GUI → External Flash Mode → 器件选
W25Q128 → Program。CLI（programmer_cli）没有器件选择项，但 Flash 操作靠
JEDEC ID 自动探测（实测本机板载颗粒 0x0B4017 = XTX 8MB 兼容片，W25Q128
档可烧）。此组用例钉住：固化作业前置只读 Detect ID、detect_flash 动作、
ID 解析与器件名映射。
"""
import unittest

from . import flasher as fl

SAMPLE_DETECT_OUTPUT = """\
 Target Cable: Gowin USB Cable(FT2CH)/0/None/null@2.5MHz
 Target Device: GW2A-18(0x0000081B)
 Operation "exFlash Detect ID" for device#1...
 Detected Flash id: 0x0B4017!
 Finished.
 Cost 0.48 second(s)
"""


class MakeJobTests(unittest.TestCase):
    TOOL = "C:/fake/programmer_cli.exe"

    def test_flash_job_prepends_detect_id(self):
        steps = fl.make_job("t20k", self.TOOL, "x.fs", action="flash",
                            flash_verify=True)
        self.assertEqual(len(steps), 3)
        self.assertIn("--run", steps[0]["argv"])
        self.assertEqual(steps[0]["argv"][steps[0]["argv"].index("--run") + 1],
                         str(fl.GOWIN_RUN_FLASH_ID))
        self.assertTrue(steps[0]["flash_id"])
        self.assertEqual(steps[1]["argv"][steps[1]["argv"].index("--run") + 1],
                         str(fl.GOWIN_RUN_FLASH))
        self.assertTrue(steps[2].get("usercode"))

    def test_flash_job_noverify_uses_run8(self):
        steps = fl.make_job("t20k", self.TOOL, "x.fs", action="flash",
                            flash_verify=False)
        self.assertEqual(steps[1]["argv"][steps[1]["argv"].index("--run") + 1],
                         str(fl.GOWIN_RUN_FLASH_NOVERIFY))

    def test_detect_flash_action_single_readonly_step(self):
        steps = fl.make_job("t20k", self.TOOL, None, action="detect_flash")
        self.assertEqual(len(steps), 1)
        self.assertEqual(steps[0]["argv"][steps[0]["argv"].index("--run") + 1],
                         str(fl.GOWIN_RUN_FLASH_ID))
        self.assertTrue(steps[0]["flash_id"])

    def test_sram_job_unchanged(self):
        steps = fl.make_job("t20k", self.TOOL, "x.fs", action="sram")
        self.assertEqual([s["argv"][s["argv"].index("--run") + 1] for s in steps],
                         [str(fl.GOWIN_RUN_SRAM), str(fl.GOWIN_RUN_USERCODE)])


class FlashIdTests(unittest.TestCase):
    def test_parse_flash_id_from_real_output(self):
        self.assertEqual(fl.parse_flash_id(SAMPLE_DETECT_OUTPUT), 0x0B4017)
        self.assertIsNone(fl.parse_flash_id("no id here"))
        self.assertIsNone(fl.parse_flash_id(None))

    def test_device_name_mapping(self):
        self.assertIn("W25Q128", fl.flash_device_name(0xEF4018))
        self.assertIn("XT25F64B", fl.flash_device_name(0x0B4017))
        self.assertIn("0x123456", fl.flash_device_name(0x123456))
        self.assertIsNone(fl.flash_device_name(None))


if __name__ == "__main__":
    unittest.main()
