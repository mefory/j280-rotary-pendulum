# 串口协议 V2

上位机与 FPGA 通过 1 Mbaud UART 通信，二进制帧协议，CRC-16/CCITT-FALSE 校验。

## 1. 帧格式

```
+--------+--------+------+------+--------+-----------+--------+
| MAGIC  | VERSION| kind | seq  |  size  |  payload  |  crc16 |
| 2 B    |  1 B   | 1 B  | 2 B  |  2 B   |  size B   |  2 B   |
| 0xA5 0x5A       |              |                  |
+--------+--------+------+------+--------+-----------+--------+
                  └──────── CRC 覆盖范围 ─────────────┘
```

- `MAGIC = 0xA5 0x5A`
- `VERSION = 2`
- `seq`：序号，上位机自增，设备在应答中原样回传（用于配对）
- `size`：payload 长度，上限 **88 字节**（v6 遥测帧）
- `crc16`：覆盖 VERSION 到 payload 末尾，多项式 0x1021（CCITT-FALSE）

## 2. 命令集

| code | 名称 | payload | 说明 |
|---|---|---|---|
| 1 | `HELLO` | `<II` schema_id, token | 握手；设备回 `<III` schema, build, capabilities（+4B epoch 可选） |
| 2 | `READ` | `<H` param_id | 读单个参数，回 `<i` 值 |
| 3 | `BEGIN` | `<I` revision | 参数事务开始（仅 IDLE/SAFE 可写） |
| 4 | `SET` | `<Hi` param_id, value | 事务内写一项（写 shadow） |
| 5 | `COMMIT` | `<I` revision | 提交事务，下一控制拍生效 |
| 6 | `ABORT` | — | 回滚事务 |
| 7 | `DEFAULTS` | — | 恢复 ROM 默认值（事务内） |
| 8 | `STOP` | — | 停机 → SAFE |
| 9 | `CLEAR` | — | 清除故障锁存 |
| 10 | `START` | — | 启动起摆（IDLE → SWING） |
| 11 | `MOVE` | `<i` delta_c01 | 相对位移（±60° 受理限） |
| 12 | `BALANCE` | — | 强制进入稳摆 |
| 13 | `CALIBRATE` | — | 下垂零点标定（摆杆须静止下垂） |
| 14 | `STATUS` | — | 读原始编码器/ADC/链路健康 |
| 15 | `STEP` | `<I` 拍数 | HIL 锁步推进（需能力位） |
| 16 | `IO_TEST` | `<hH` duty, enable | 安全态电机点动（限 ±2000） |
| 17 | `FORCE_POS` | `<i` delta | 强制进 POS（绕过 pos_ready，仅 \|α\|<15°） |
| 18 | `VEL_DEMO` | `<Hhh` en, v, a | 速度曲线演示 |
| 20 | `MOVE_ABS` | `<i` 绝对目标 c01 | 绝对目标定位（最短路门禁 ±330°） |

### 应答格式

所有命令应答统一为：

```
+------+------+----------+---------+-------+
| kind | code | revision | tick    | token |
| 1 B  | 1 B  |   4 B    |  4 B    |  4 B  |
+------+------+----------+---------+-------+
```

- `code = 0` 成功；非 0 为错误码（`BAD_SCHEMA=7`、`STATE`、`RANGE`、`BAD_ID`、`BAD_LENGTH` 等）
- `tick`：命令被执行的拍号
- `token`：HELLO 时协商的会话令牌，非 HELLO 命令必须匹配

## 3. 参数事务

参数写入是**两阶段提交**，防止半生效：

```
上位机                        设备
  BEGIN(rev) ──────────────►  锁定 shadow 区
  SET(id, value) × N ───────► 逐项写 shadow
  COMMIT(rev) ──────────────► 下一控制拍整体切到 active
       ◄──────────── ACK ──── 
  READ(id) × N ─────────────► 回读校验
```

- 事务只能在 **IDLE / SAFE** 状态提交（`STATE` 错误码拒绝运行期写）
- `revision` 是参数版本号，设备在每次 COMMIT 后自增；上位机须带当前版本，
  版本不匹配说明有别的客户端改过参数
- 提交后设备有 1 拍延迟才生效，立即回读可能读到旧值

## 4. 参数表（14 项）

| id | key | 默认值 | 单位 | 说明 |
|---|---|---|---|---|
| 0 | `k_alpha` | 17751 | duty/cdeg | LQR K1（摆角） |
| 1 | `k_alphadot` | 10000 | duty/(0.1°/s) | LQR K2（摆角速度） |
| 2 | `k_theta_bal` | 600 | duty/cdeg | 稳摆臂角 K3 |
| 3 | `k_thetadot_bal` | 5300 | duty/(0.1°/s) | 稳摆臂速 K4 |
| 4 | `dz_pos` | 640 | duty | 死区补偿正向 |
| 5 | `dz_neg` | 740 | duty | 死区补偿反向 |
| 6 | `swing_amp` | 7000~8800 | duty | 甩摆幅值 |
| 7 | `catch_window` | 1500 | cdeg | 捕获窗（±15°） |
| 8 | `catch_alpha_max` | 9000 | cdeg | 捕获最大摆角 |
| 9 | `catch_alpha_dot` | 3000 | 0.1°/s | 捕获最大摆角速度 |
| 10 | `fling_delta` | 11800 | cdeg | 甩摆角偏移 δ=118° |
| 11 | `k_theta_pos` | 1200 | duty/cdeg | POS K3 |
| 12 | `k_thetadot_pos` | 9500 | duty/(0.1°/s) | POS K4 |
| 13 | `alpha_trim` | 245 | cdeg | α 零点补偿（标定相关） |

**参数表指纹**：`schema_id = sha256(parameters.json)[:8]`，写在
`rtl/j280_debug_params.vh` 的 `DEBUG_SCHEMA` 宏里。固件与上位机 schema
不一致时 HELLO 回 `BAD_SCHEMA(7)`，强制双方同版。

## 5. 遥测帧（500 Hz）

每 2 ms 一帧，**88 字节**（v6 布局），字段顺序：

| 字段 | 类型 | 单位 | 说明 |
|---|---|---|---|
| `tick` | u32 | 1 ms | 控制拍号 |
| `revision` | u32 | — | 参数版本 |
| `mode` | u8 | — | 0=IDLE 1=SWING 2=BALANCE 3=POS 4=SAFE |
| `fault` | u8 | — | 故障位图 |
| `alpha` | i16 | 0.01° | 摆角 |
| `alpha_dot` | i16 | 0.1°/s | 摆角速度 |
| `theta` | i32 | 0.01° | 臂角（多圈连续） |
| `theta_dot` | i16 | 0.1°/s | 臂角速度 |
| `duty` | i16 | 0.01% | 最终输出 |
| `reference` | i32 | 0.01° | 参考轨迹 |
| `target` | i32 | 0.01° | 位置目标 |
| `encoder` | i32 | counts | 电机编码器原始计数 |
| `adc` | u16 | — | ADC 原始值 |
| `offset` | i32 | 0.01° | 标定零点 |
| `calib_done` | u8 | — | 标定完成标志 |
| `run_en` | u8 | — | 运行使能 |
| `pos_trim` | i16 | duty | 位置积分器输出 |
| `alpha_pre` | i16 | 0.01° | 原始 α（未减 trim） |
| `k3_eff` | i16 | duty/cdeg | 当前生效 K3 |
| `k4_eff` | i16 | duty/(0.1°/s) | 当前生效 K4 |
| `duty_pre_dz` | i16 | 0.01% | 死区补偿前 duty |
| `obs_active` | u8 | — | 观测器启用 |
| `alpha_trim` | i16 | cdeg | 当前 trim |
| `term_k1`~`term_k4` | i16 ×4 | — | 诊断探针（用途随版本变） |
| `theta_e` | i16 | 0.01° | 位置误差（钳位 ±45°） |

**探针字段用途（当前固件）**：
- `term_k1`：空闲档回传 `{sb_on, sb_zone, burst_on, creep_zone}` 状态位；
  VEL_DEMO 激活时回传速度参考
- `term_k2`：最近一次 MOVE/MOVE_ABS 的载荷
- `term_k3`：`pos_tgt_cmd − θ`（真实"离目标方位还差多少"，上位机用它做到位判据）
- `term_k4`：`{move_pending, move_pending_d}` 握手位

**兼容性**：帧长度 40/54/88 三种历史布局，上位机按长度分支解析，
缺失字段补 `None`（区分"值为 0"与"无此字段"）。

## 6. HELLO 与能力发现

HELLO 应答 payload：`<III`（schema, build, capabilities），新固件追加 `<I` epoch。

**能力位**（上位机据此启用功能，而非按板名写死）：

| bit | 名称 | 含义 |
|---|---|---|
| 0 | `CAP_PARAM_TXN` | 参数事务 |
| 1 | `CAP_ACTIONS` | 动作命令 |
| 2 | `CAP_TELEMETRY` | 周期遥测 |
| 3 | `CAP_CALIB_DC` | 下垂标定 |
| 4 | `CAP_STEP_LOCK` | STEP 锁步（HIL） |
| 5 | `CAP_HW_SENSORS` | 硬件传感 |
| 6 | `CAP_MOTOR_OUT` | 电机输出 |
| 7 | `CAP_IO_TEST` | I/O 联调 |

`schema_matched = false` 时上位机进入"只读观测"降级模式（仅已知历史 schema 白名单）。
