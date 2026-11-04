"""锁步 HIL 设备：FPGA 跑真实控制 RTL，PC 跑被控对象模型。

与 SerialDevice 的区别在于时钟归属。真实机械本体上 FPGA 自行产生 1ms 节拍，
上位机只是旁观；没有机械本体或传感前端的板卡（DE10 验证、T20K V1）传感器
无从产生，因此由本模块接管节拍：每步用 STEP 命令把模型算出的传感器字送给
FPGA，FPGA 据此推进恰好一个仿真毫秒并回遥测，本模块再用遥测里的 duty 积分
模型走下一步。

连接哪块板由 HELLO 回包的 build 戳与能力位标识（见 protocol.decode_build），
本模块只认 CAP_STEP_LOCK，不认板名——因此同一份代码在 DE10 与 T20K 上通用。

FPGA 内仍跑完整、未经改动的控制链路（ADC 标定 → α 回绕 → 差分角速度 →
M/T 测速 → 模式机 → 起摆/LQR/位置环 → 死区补偿 → 限幅），上位机只提供
"物理世界"。节拍严格乒乓同步、每拍一一对应，故轨迹与 1kHz 闭环仿真一致；
代价是墙钟时间（受 UART 往返限制，约 1 kHz 步频）。

死区补偿的归属要注意：控制器算出的 duty 已是"指令占空比"，电机死区扣减
（±350）属于执行器物理，因此由本模块在积分时扣除，FPGA 侧不重复扣。
协议见 rtl/debug_link.v 的 STEP 分支；Plant 侧见 plant.py 的 sample_sensors
与 torque_for。
"""

import struct
import time

from . import protocol as p
from .plant import Plant

# 一拍的墙钟预算：STEP(20B)+ACK(14B)+遥测(40B) ≈ 74B @1Mbaud ≈ 0.74ms，
# 留出充裕余量应对 USB 桥与驱动调度抖动。
DEFAULT_STEP_TIMEOUT = 5.0


class HilDevice:
    """锁步 HIL：STEP 推进 FPGA，本地模型积分物理。"""

    readonly = False

    def __init__(self, port, seed=7, step_timeout=DEFAULT_STEP_TIMEOUT, plant_profile=None):
        import serial
        self.label = f"锁步 HIL {port}"
        self.serial = serial.Serial(port, 1000000, timeout=0, write_timeout=.3)
        self.baudrate = 1000000
        self.baudrates = (1000000, 2000000)
        self.serial.reset_input_buffer()
        self.plant = Plant(seed, profile=plant_profile)
        self.decoder = p.Decoder(timeout=.05)
        self.output = bytearray()
        self.step_timeout = step_timeout
        self.steps = 0
        self.samples = []

    def set_baudrate(self, baudrate):
        if self.baudrate == baudrate:
            return
        self.serial.reset_input_buffer()
        self.serial.baudrate = baudrate
        self.baudrate = baudrate
        self.serial.reset_input_buffer()

    def close(self):
        self.serial.close()

    def write(self, data):
        self.serial.write(data)

    def read(self):
        return self.serial.read(self.serial.in_waiting)

    def advance(self, milliseconds):
        # 锁步推进由 step_once() 显式驱动；此处的等待只用于 Client.request
        # 内部的轮询节奏，避免无板时打成忙等。
        time.sleep(min(milliseconds, 50) / 1000.0)

    # ---- 锁步驱动 ----

    def step_once(self, client, adc, enc):
        """送一拍传感器并等待该拍遥测；返回含 duty 的遥测字典。"""
        client.request(p.STEP, struct.pack("<Hi", adc & 0xFFFF, enc))
        expected = self.steps + 1
        deadline = time.monotonic() + self.step_timeout
        while time.monotonic() < deadline:
            sample = client.last_sample
            if sample is not None and sample["tick"] >= expected:
                self.steps = expected
                self.plant.ticks = expected
                self.samples.append(sample)
                return sample
            client.poll(1)
        raise TimeoutError(
            f"HIL 步进超时（第 {expected} 拍）：FPGA 未在 {self.step_timeout}s 内回遥测")

    def step_and_integrate(self, client):
        """完整一拍：模型出传感器 → FPGA 出 duty → 模型积分。

        顺序与 tb_closedloop 一致：传感器反映积分前的状态，控制器用当拍状态
        算 duty，该 duty 驱动本拍积分。
        """
        adc, enc = self.plant.sample_sensors()
        sample = self.step_once(client, adc, enc)
        self.plant.physics_step(sample["duty"], sample["mode"], sample["fault"])
        return sample

    def run(self, client, ticks):
        """连续跑 ticks 拍；返回采样列表。"""
        for _ in range(int(ticks)):
            self.step_and_integrate(client)
        return self.samples
