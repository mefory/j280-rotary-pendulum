import json
import math
import struct
import unittest

from . import protocol as p
from .schema import ROOT,PARAMS,SCHEMA_ID,raw_value
from .devices import SimDevice,ReplayDevice
from .client import Client,CommandError
from .plant import Plant
from .acceptance import _verdict


class ProtocolTests(unittest.TestCase):
    def test_acceptance_reports_no_capture_without_move(self):
        result,_=_verdict(3000,7,[{"tick":0,"mode":1}],0,None)
        self.assertIsNone(result["capture_s"])
        self.assertFalse(result["hold"])
        self.assertFalse(result["pass"])

    def test_identified_plant_profile_is_opt_in(self):
        baseline=Plant()
        identified=Plant(profile=ROOT/"config/plant_de10_identified.json")
        self.assertEqual(baseline.offset,8344)
        self.assertEqual(identified.offset,8751)
        self.assertEqual(baseline.p["motor_gain"],1.0)
        self.assertLess(identified.torque_for(5000,1),baseline.torque_for(5000,1))
        self.assertNotEqual(identified.sample_sensors()[0],baseline.sample_sensors()[0])

    def test_crc_and_fragmentation(self):
        self.assertEqual(p.crc16(b"123456789"),0x29B1)
        frame=p.Frame(p.SET,65535,b"\xa5\x5a\x00\xff")
        decoder=p.Decoder();frames=[]
        for b in b"noise"+frame.encode()*2:
            frames+=decoder.feed(bytes([b]))
        self.assertEqual(frames,[frame,frame]);self.assertEqual(decoder.errors,0)

    def test_bad_crc_length_and_timeout(self):
        good=p.Frame(p.READ,5,b"").encode();bad=bytearray(good);bad[-1]^=4
        decoder=p.Decoder()
        self.assertEqual(decoder.feed(bytes(bad)+good),[p.Frame(p.READ,5,b"")])
        self.assertGreater(decoder.errors,0)
        decoder.feed(good[:5],now=0)
        self.assertEqual(decoder.feed(good,now=1),[p.Frame(p.READ,5,b"")])
        corrupt=b"\xa5\x5a\x02\x01\x00\x00\xff\xff"
        self.assertEqual(decoder.feed(corrupt+good,now=2),[p.Frame(p.READ,5,b"")])

    def test_physical_units(self):
        self.assertEqual(raw_value(PARAMS[9],"-30"),-3000)
        self.assertEqual(raw_value(PARAMS[4],"3.75"),375)
        with self.assertRaises(ValueError):raw_value(PARAMS[4],"-1")


class DeviceTests(unittest.TestCase):
    def setUp(self):
        self.device=SimDevice();self.client=Client(self.device);self.client.connect()

    def test_transaction_atomic_revision_and_defaults(self):
        initial=self.client.read_params();updated=initial.copy();updated[0]=22900
        self.client.request(p.BEGIN,struct.pack("<I",0))
        self.client.request(p.SET,struct.pack("<Hi",0,22900))
        self.assertEqual(self.client.read_params(),initial)
        self.client.request(p.COMMIT,struct.pack("<I",0))
        self.assertEqual(self.client.read_params(),updated)
        self.assertEqual(self.client.revision,1)
        self.assertEqual(self.client.defaults(),initial)
        self.assertEqual(self.client.revision,2)

    def test_invalid_id_range_and_busy(self):
        with self.assertRaises(CommandError) as ctx:self.client.request(p.READ,struct.pack("<H",16))
        self.assertEqual(ctx.exception.code,p.BAD_ID)
        self.client.request(p.CLEAR);self.client.request(p.START)
        with self.assertRaises(CommandError) as ctx:self.client.apply([x["default"] for x in PARAMS])
        self.assertEqual(ctx.exception.code,p.STATE)
        self.client.request(p.STOP);self.assertEqual(self.device.plant.mode,4)

    def test_expiry_and_duplicate(self):
        self.client.request(p.BEGIN,struct.pack("<I",0));self.device.advance(5001)
        with self.assertRaises(CommandError) as ctx:self.client.request(p.COMMIT,struct.pack("<I",0))
        self.assertEqual(ctx.exception.code,p.EXPIRED)
        self.client.apply([x["default"] for x in PARAMS])
        cached=self.device.last
        revision=self.device.revision
        self.device.write(cached.encode())
        self.assertEqual(self.device.revision,revision)
        self.device.plant.mode=2;self.device.plant.fault=0
        self.device.plant.alpha=0;self.device.plant.theta_dot=0
        f=p.Frame(p.MOVE,(cached.seq+1)&65535,struct.pack("<Ii",self.device.session,3000))
        self.device.write(f.encode());target=self.device.plant.target
        self.device.write(f.encode())
        self.assertEqual(self.device.plant.target,target)

    def test_manual_balance_requires_upright_and_still(self):
        """待机手扶稳摆只在摆杆直立且两轴静止时接管，并以当前位置为零点。"""
        self.client.request(p.CLEAR)
        for alpha,alpha_dot,theta_dot in ((600,0,0),(0,300,0),(0,0,200)):
            self.device.plant.alpha=alpha
            self.device.plant.alpha_dot=alpha_dot
            self.device.plant.theta_dot=theta_dot
            with self.assertRaises(CommandError) as ctx:
                self.client.request(p.BALANCE)
            self.assertEqual(ctx.exception.code,p.STATE)
            self.assertEqual(self.device.plant.mode,0)
        self.device.plant.alpha=499
        self.device.plant.alpha_dot=-299
        self.device.plant.theta_dot=199
        self.device.plant.theta=1234
        self.device.plant.x=(0.,0.,0.,0.)
        self.device.plant.sensor=self.device.plant.sample_sensors()
        self.device.plant.ah=[0]*4
        self.client.request(p.BALANCE)
        self.assertEqual(self.device.plant.mode,2)
        self.assertEqual(self.device.plant.reference,1234)
        self.assertEqual(self.device.plant.target,1234)

    def test_position_can_still_return_to_balance(self):
        self.device.plant.mode=3
        self.device.plant.fault=0
        self.device.plant.x=(0.,0.,0.,0.)
        self.device.plant.sensor=self.device.plant.sample_sensors()
        self.device.plant.ah=[0]*4
        self.device.plant.warm=2
        self.device.plant.alpha=0
        self.device.plant.theta=2468
        self.client.request(p.BALANCE)
        self.assertEqual(self.device.plant.mode,2)
        self.assertEqual(self.device.plant.reference,2468)

    def test_drop_out_stops_after_single_swing_attempt(self):
        self.device.plant.mode=2
        self.device.plant.fault=0
        self.device.plant.warm=2
        self.device.plant.x=(0.,math.radians(100.),0.,0.)
        self.device.plant.sensor=self.device.plant.sample_sensors()
        self.device.plant.ah=[0]*4
        self.device.plant.step(self.device.active)
        self.assertEqual(self.device.plant.mode,4)
        self.device.plant.x=(0.,0.,0.,0.)
        self.device.plant.sensor=self.device.plant.sample_sensors()
        self.device.plant.step(self.device.active)
        self.assertEqual(self.device.plant.mode,4)

    def test_swing_timeout_stops_without_capture(self):
        plant=self.device.plant
        plant.mode=1;plant.fault=0;plant.warm=2;plant.swing_ticks=24999
        plant.sensor=plant.sample_sensors()
        plant.step(self.device.active)
        self.assertEqual(plant.mode,4)
        plant.step(self.device.active)
        self.assertEqual(plant.mode,4)

    def test_matlab_derivative_vectors(self):
        data=json.loads((ROOT/"config/plant.json").read_text(encoding="utf-8"))
        for x,tau,expected in zip(data["states"],data["torques"],data["derivatives"]):
            actual=self.device.plant.rhs(x,tau)
            for a,b in zip(actual,expected):self.assertAlmostEqual(a,b,places=10)


class IdentityTests(unittest.TestCase):
    """板标识与能力位：HELLO 第二三项契约，也是"上位机不认板名"的基础。"""

    def test_capability_bits_decode(self):
        self.assertIn("STEP 锁步",p.capability_names(p.CAPS_HIL))
        self.assertNotIn("STEP 锁步",p.capability_names(p.CAPS_REAL))
        self.assertIn("硬件传感",p.capability_names(p.CAPS_REAL))
        self.assertTrue(p.has_capability(p.CAPS_REAL,p.CAP_MOTOR_OUT))
        self.assertFalse(p.has_capability(p.CAPS_HIL,p.CAP_HW_SENSORS))
        self.assertFalse(p.has_capability(p.CAPS_HIL,p.CAP_CALIB_DC))

    def test_build_stamp_round_trip(self):
        info=p.decode_build(p.BUILD_STAMP_DEFAULT)
        self.assertEqual(info["board_id"],p.BOARD_ID_DEFAULT)
        self.assertRegex(info["date"],r"^20\d\d-\d\d-\d\d$")
        self.assertTrue(info["board"])
        # date 是写死的常量，必须自带标记，免得再被当成构建日期用
        self.assertTrue(info["date_is_static"])

    def test_build_epoch_is_the_real_timestamp(self):
        """构建时刻走独立字段：0=非真实构建，非 0 才给出可读时间。"""
        baseline=p.decode_epoch(0)
        self.assertEqual(baseline["build_epoch"],0)
        self.assertIsNone(baseline["built"])          # 不能显示成 1970 年
        self.assertIsNone(baseline["built_age_s"])
        # 源码树副本永远是 0（保证 --check 在同提交上可复现）
        self.assertEqual(p.BUILD_EPOCH_DEFAULT,0,
                         "源码树 j280_debug_params.vh 应是不带时间戳的基线")
        known=p.decode_epoch(1760000000)
        self.assertEqual(known["build_epoch"],1760000000)
        self.assertRegex(known["built"],r"^20\d\d-\d\d-\d\d \d\d:\d\d:\d\d$")

    def test_generator_embeds_now(self):
        """--now 必须真的把当前时间写进头文件，而不是留常量。"""
        import importlib.util
        spec=importlib.util.spec_from_file_location(
            "gen_dbg",ROOT/"tools/generate_debug_config.py")
        gen=importlib.util.module_from_spec(spec);spec.loader.exec_module(gen)
        stamp=1760000000
        text=gen.render("t20k",epoch=stamp)
        self.assertIn(f"`define DEBUG_BUILD_EPOCH 32'h{stamp:08x}",text)
        self.assertIn("DEBUG_BUILD_STAMP 32'h14260923",text)   # build 戳不变
        self.assertIn("DEBUG_BUILD_EPOCH 32'h00000000",gen.render("t20k",epoch=0))

    def test_rtl_header_is_single_source_of_truth(self):
        """host 解析的常量必须与 RTL 生成头文件逐字一致（防两侧各改一份）。"""
        text=(ROOT/"rtl/j280_debug_params.vh").read_text(encoding="utf-8")
        self.assertIn(f"`define DEBUG_BOARD_ID 8'h{p.BOARD_ID_DEFAULT:02x}",text)
        self.assertIn(f"`define DEBUG_BUILD_STAMP 32'h{p.BUILD_STAMP_DEFAULT:08x}",text)
        self.assertIn(f"`define DEBUG_BUILD_EPOCH 32'h{p.BUILD_EPOCH_DEFAULT:08x}",text)
        self.assertIn(f"`define DEBUG_CAPS_REAL 32'h{p.CAPS_REAL:08x}",text)
        self.assertIn(f"`define DEBUG_CAPS_HIL 32'h{p.CAPS_HIL:08x}",text)

    def test_sim_reports_identity_over_hello(self):
        """模拟器可以假扮别的板：界面与迁移流程无硬件也能先跑通。"""
        for board_id,caps in ((p.BOARD_ID_DEFAULT,p.CAPS_REAL),(0x14,p.CAPS_HIL)):
            client=Client(SimDevice(board_id=board_id,capabilities=caps));client.connect()
            self.assertEqual(client.identity["board_id"],board_id)
            self.assertEqual(client.identity["capabilities"],caps)
        self.assertEqual(p.decode_build(0x14<<24)["board"],"Tang Primer 20K")


if __name__=="__main__":
    unittest.main()
