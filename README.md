# 旋转倒立摆实时姿态控制系统

基于高云（Gowin）GW2A FPGA 的**旋转倒立摆**全栈开源实现：1 kHz 全定点硬件闭环、
自动起摆、LQR 稳摆、0~360° 绝对角度定位、500 Hz 遥测与在线参数整定。

> 硬件平台：Sipeed Tang Primer 20K（GW2A-LV18PG256C8/I7，27 MHz）
> 控制核与板级外壳解耦，同一套 RTL 亦可运行于 DE10-Standard / J280 平台。

---

## 一、它做了什么

| 功能 | 实现方式 | 实测结果 |
|---|---|---|
| **自动起摆** | 三相位甩摆 + 捕获窗（±30°，刹车相 440~470 ms） | 单杆捕获 1.0~1.3 s；历史最优 4/4 连成，连续稳摆 100.6 s |
| **稳摆自平衡** | LQR 全状态反馈 `u = K1·α + K2·ωα + K3·θe + K4·ωθ` | 摆角偏差 RMS **1.9~3.2°**（30~120 s 多轮统计） |
| **位置定位** | MOVE_ABS 绝对目标 + 20°/s 目标斜坡 + ≤55° 分步 + 末段突发微步 | 满电 \|θe\| 均值 **8.9~12.3°**、峰峰 42.5~60.9°（K4 六档扫描）；执行精度 **Δ=0.0°** |
| **静摩擦锁保持** | 安静且到位 → duty 强制 0，磁编码器核验真静止 | duty 恒零最长连续 **2.81 s**，占比 3.7% |
| **在线整定** | 500 Hz 遥测 + 14 参数事务式读写（开始—逐项写—提交/回滚） | 无需重编译，逐帧可归因 |
| **安全** | 看门狗 / 传感器坏帧锁存 / 上电静默 / 故障 coast | 故障时电机不抱死 |

**构建指标**（本次发布构建）：Fmax **27.219 MHz**（27 MHz 约束，0 时序违例）；
逻辑 13 585/20 736（66%）、FF 8 009（50%）、DSP 16/20（80%）、BSRAM 2/29（5%）。

---

## 二、架构

```
                    ┌──────────────────────────────────────────┐
  MT6835 (SPI)  ──► │  angle_proc  →  α (0.01°)                 │
                    │                                    ┌─────┤
                    │  mt6701_ssi_if → theta_mt_path → θ (0.01°) │
                    │                                    │     │
                    │  loop_timer (1 kHz tick) ──────────┤     │
                    │                                    ▼     │
                    │  mode_fsm: IDLE→SWING→BALANCE⇄POS→SAFE   │
                    │       │            │       │             │
                    │       ▼            ▼       ▼             │
                    │  swingup_core   lqr_core  pos/creep       │
                    │       └────────────┴───────┘             │
                    │                     ▼                     │
                    │  out_stage: 软死区 → 速度自适应 → 16 格   │
                    │             位置摩擦补偿 → 换向守卫        │
                    │                     ▼                     │
                    │  motor_pwm (21.6 kHz) ──► TB6612 ──► 电机 │
                    │                                           │
                    │  debug_link (CRC16 帧, 500 Hz 遥测)        │
                    └──────────────────────────────────────────┘
```

**核心设计取舍**：

1. **板级无关控制核**：`rtl/ctrl_core.v` 不含任何引脚/器件假设，换板只写外壳
   （`rtl/boards/{t20k,de10,j280}/`），这是本项目最重要的架构决定。
2. **全定点、单时钟域**：角度 0.01°、角速度 0.1°/s、占空比 0.01%，增益 Q3.13；
   27 MHz 晶振直驱无 PLL，控制环相位可精确建模。
3. **模型驱动**：MATLAB 脚本推导 LQR/观测器 → 生成 Verilog 头文件 → RTL 引用，
   模型与实现共享同一套状态定义（`matlab/lqr_to_rtl.m`、`obs_to_rtl.m`）。
4. **四段增益调度**：稳摆 / 捕获 / 定位 / 移动使用不同增益，切换走 1 024 ms
   凸组合渐变，避免硬切换冲击。
5. **调试链路即基础设施**：CRC16 二进制帧协议 + 参数事务 + 500 Hz 完整状态遥测，
   是本项目能在真实硬件上快速收敛的根本原因。

---

## 三、目录结构

```
rtl/                 25 个可综合 Verilog 模块 + 4 个生成的头文件
  boards/t20k/       Tang Primer 20K 板级外壳（引脚与传感/驱动接线）
  boards/de10/       DE10-Standard 外壳（HIL 锁步验证用）
  boards/j280/       J280 外壳
tb/                  30 个 testbench（单模块 + 闭环 + 起摆 + HIL 锁步）
cons/                引脚约束与时序约束
matlab/              模型辨识、LQR/观测器推导、定点化生成、数据分析
host/                Python 协议库（帧解析、客户端、设备抽象、参数表）
gui/                 上位机（PySide6 实时波形/参数面板/遥测记录/烧录页）
build/                Gowin 构建脚本（T20K）
docs/                架构、协议、控制设计、调参指南
```

---

## 四、快速开始

### 4.1 仿真（无需硬件）

```bash
# 需要 iverilog
iverilog -o /tmp/tb_lqr tb/tb_lqr_core.v rtl/lqr_core.v rtl/j280_defs.vh
vvp /tmp/tb_lqr

# 闭环仿真（含植物模型）
iverilog -o /tmp/tb_cl tb/tb_closedloop.v rtl/*.v tb/j280_plant_params.vh
vvp /tmp/tb_cl
```

### 4.2 构建位流（Gowin EDA）

```powershell
# Windows PowerShell（需先安装 Gowin EDA 1.9.x）
.\build\run_t20k.ps1 -Mode full
# 产物在 <仓库>/build_output/audit_<时间戳>/t20k/impl/pnr/t20k.fs
```

### 4.3 上位机

```bash
cd gui
pip install -r requirements.txt      # PySide6 建议钉 6.9.3
python main.py
```

连接串口（1 Mbaud）后：握手 → 下垂标定 → START 起摆 → 稳摆 → 定位。
详细操作见 [docs/tuning-guide.md](docs/tuning-guide.md)。

### 4.4 硬件接线

见 [docs/hardware.md](docs/hardware.md)（含 Tang Primer 20K 引脚表、编码器接线、
6 线滑环走线要点）。

---

## 五、文档

| 文档 | 内容 |
|---|---|
| [docs/architecture.md](docs/architecture.md) | 系统组成、数据流、模块划分、定点数值约定 |
| [docs/protocol.md](docs/protocol.md) | 串口协议 V2：帧格式、命令集、参数事务、遥测字段 |
| [docs/control-design.md](docs/control-design.md) | 旋转倒立摆建模、LQR 推导、观测器、定点化与增益调度 |
| [docs/tuning-guide.md](docs/tuning-guide.md) | 标定流程、trim 归零、K4 电量适配、故障排查 |
| [docs/hardware.md](docs/hardware.md) | 引脚表、编码器方案、驱动电路、机械特性 |

---

## 六、已知限制与工程真相

这个项目在真实硬件上调试了数月，以下是我们踩过的坑，也是本仓库最有价值的部分：

- **静摩擦是位置精度的物理下限**：齿轮箱静摩擦带约 640~740 duty（0.01% 占空比），
  K3 太软时 5° 误差只产生 73 duty，顶不开死区。位置误差的直流分量主要来自这里，
  靠 `trim` 参数（α 零位补偿）与 16 格位置相关摩擦补偿缓解，无法根除。
- **`trim` 是标定相关的**：每次重新做下垂标定，α 零位都会移动，最优 trim 随之变化
  （实测两次标定最优值相差 140 cdeg ≈ 20° 臂误差）。**重标定后必须重扫 trim**。
- **K4（臂速阻尼）是甜点型参数且随电池电压移动**：满电甜点约 9 500~14 000，
  60% 电量约 8 000；越过悬崖（满电 >16 000）会直接泵摆掉杆。GUI 提供电量探针辅助。
- **速度信号质量受机械粘滑限制**：编码器量化只占噪声 1/6，其余是真实机械粘滑与电磁干扰，
  换更高分辨率编码器收益有限。
- **0.25 Hz 慢摆是自持极限环**：Hilbert 包络分析显示阻尼 ζ≈0，能量由死区继电器与
  粘滑持续注入。单纯抬 K3/K4 阻尼无法消除（两者都已在稳定边界）。

完整调试史料与失败路径记录见各文档的「实测记录」章节。

---

## 七、许可

MIT License，见 [LICENSE](LICENSE)。

本仓库不包含厂商数据手册、厂商位流与 EDA 工具二进制；使用 Gowin 工具链、
MT6835/MT6701 器件与各板卡请遵守对应厂商许可。

## 八、致谢

- 高云半导体 GW2A 系列与 Gowin EDA
- 江苏多维科技 MT6835 / MT6701 磁编码器
- Åström & Furuta 的起摆能量控制方法
