"""HIL 锁步 RTL 回归：验证 STEP 命令的契约，经真实 UART 引脚驱动 de10_top。

为什么这样设计：DE10 上不存在真实倒立摆机构，传感器必须由上位机供给，
因此"每拍推进多少"完全取决于 STEP 的实现是否正确。若 STEP 偶发多推一拍、
漏推一拍，或回传的 duty 不对应本拍，闭环轨迹就会系统性偏离——这是 DE10
验证路径最根本的失效模式，故单独在 RTL 层验穿。

本用例把传感器固定为**倒立点零速**（adc=50009 使 angle_proc 输出 α≈0；
编码器恒 0 使 θ̇ 判停为 0），于是断言：
  1) 每条 STEP 的遥测 tick 相对上一条**严格 +1**（既不多推也不漏推）
  2) STEP 应答同时带回 duty/mode/fault，且 mode 与模式机的实际进展相符
  3) 注入值确实进入控制链：IDLE→SWING→(捕获)→BALANCE 能自动完成
  4) 闭环 duty 在此工况下收敛到小量（倒立点零速不需要大力矩）

激励写入 sim/out/hil_stimulus.txt，由 tb_hil_lockstep.v 回放；TX 以
"RX <hex>" 打印，此处解码判定。
"""

import struct
import subprocess
import sys
from pathlib import Path

from . import protocol as p
from .schema import ROOT, SCHEMA_ID

TB_NAME = "tb_hil_lockstep"
STIMULUS = ROOT / "sim/out/hil_stimulus.txt"

# 倒立点传感器字：adc 使 angle_proc 的 raw ≈ 26345，
# alpha = wrap(26345 - 默认offset 8344 + 18000) = wrap(36001) = 1 (0.01°)
INVERTED_ADC = 50009
STEP_COUNT = 60

M_BALANCE = 2
M_SWING = 1


def build_stimulus():
    """生成帧字节序列。

    【远程启动序列】HIL 下 STEP 是唯一节拍来源（自由运行的 1ms 计数器被旁路），
    而 mode_fsm / safety 都在 tick 上推进。实测正确的远程顺序是：

        CLEAR → (几拍) → STOP → (几拍) → START → (多拍)

    原因是两条既有 RTL 语义（非 HIL 引入，实板按键因按住数十毫秒而天然跨过）：
      1) CLEAR 只清 fault_latch；而 mode_fsm 在同一沿看到的仍是旧 fault=1，
         故当拍 SAFE 不迁移。remote_clear 是单拍脉冲，下一拍已撤，于是
         "一次 CLEAR" 停在 SAFE。
      2) 离开 SAFE 需要 key_start/key_stop 且 !fault，且 START 命令自身要求
         当前已是 IDLE（否则返回 STATE）。因此第二次"停机类"命令由 STOP 提供，
         它使 SAFE→IDLE，随后 START 才能 IDLE→SWING。

    这段顺序对 host/hil.py 同样适用，已写入迁移文档。
    """
    token = 0x5a5a1234
    frames = []
    sequence = [0]      # 每帧递增；闭包里必须用可变对象，否则会重复用同一序号
                        # （重复序号会被 RTL 判为"同一命令重传"而重放旧应答）

    frames.append(p.Frame(p.HELLO, 0, struct.pack("<II", SCHEMA_ID, token)).encode())

    def auth(kind, payload=b""):
        sequence[0] += 1
        return p.Frame(kind, sequence[0], struct.pack("<I", token) + payload).encode()

    def hold(count):
        """插入 count 个 STEP，传感器取倒立点零速。"""
        return [auth(p.STEP, struct.pack("<Hi", INVERTED_ADC, 0))
                for _ in range(count)]

    frames.extend(hold(6))        # 复位后先推进几拍
    frames.append(auth(p.CLEAR))  # 清上电故障（mode 仍停在 SAFE）
    frames.extend(hold(4))
    frames.append(auth(p.STOP))   # SAFE → IDLE
    frames.extend(hold(4))
    frames.append(auth(p.START))  # IDLE → SWING
    frames.extend(hold(STEP_COUNT))
    return frames


def compile_and_run():
    sources = [ROOT / f"tb/{TB_NAME}.v",
               *sorted((ROOT / "rtl").glob("*.v")),
               *sorted((ROOT / "rtl").glob("*.vh")),
               *sorted((ROOT / "rtl/boards").rglob("*.v"))]
    binary = ROOT / f"sim/out/{TB_NAME}.vvp"
    if not binary.exists() or any(s.stat().st_mtime > binary.stat().st_mtime for s in sources):
        result = subprocess.run(
            ["iverilog", "-g2012", "-I", "rtl", "-s", TB_NAME, "-o", str(binary),
             *[str(s) for s in sources]],
            cwd=ROOT, capture_output=True, text=True,
            encoding="utf-8", errors="replace")
        if result.returncode:
            print(result.stdout, result.stderr)
            raise RuntimeError(f"{TB_NAME} compile failed")
    run = subprocess.run(["vvp", str(binary)], cwd=ROOT, capture_output=True,
                         text=True, encoding="utf-8", errors="replace", timeout=900)
    (ROOT / f"sim/out/{TB_NAME}.log").write_text(run.stdout + run.stderr, encoding="utf-8")
    return run.stdout


def run():
    frames = build_stimulus()
    STIMULUS.write_text(
        "".join(f"{len(frame)} " + " ".join(f"{b:02x}" for b in frame) + "\n"
                for frame in frames),
        encoding="utf-8")

    stdout = compile_and_run()
    if "tb_hil_lockstep PASS" not in stdout and "PASS" not in stdout:
        # 测试台自身的 PASS 只是"激励回放完毕"，真判定在下方
        pass

    wire = bytes(int(line.split()[1], 16)
                 for line in stdout.splitlines() if line.startswith("RX "))
    decoder = p.Decoder(timeout=10.0)
    replies = [f for f in decoder.feed(wire, now=0.0) if f.kind == p.RESPONSE]
    if not replies:
        raise AssertionError(f"未收到任何应答；TX 字节 {len(wire)} 个\n{stdout[-2000:]}")

    errors = []
    step_replies = []
    for frame in replies:
        if len(frame.payload) < 14:
            continue
        command, code, revision, stamp, token = struct.unpack_from("<BBIII", frame.payload)
        if command == p.STEP:
            if code != p.OK:
                errors.append(f"STEP seq={frame.seq} 返回错误码 {code}({p.ERRORS.get(code)})")
                continue
            extra = frame.payload[14:]
            if len(extra) != 8:
                errors.append(f"STEP 应答载荷应为 8 字节，实际 {len(extra)}")
                continue
            tick, duty, mode, fault = struct.unpack("<IhBB", extra)
            step_replies.append(dict(seq=frame.seq, tick=tick, duty=duty,
                                     mode=mode, fault=fault))

    if not step_replies:
        raise AssertionError("没有任何成功的 STEP 应答——锁步机制未生效")

    # ---- 断言 1：每条 STEP 恰好推进一个控制毫秒 ----
    for previous, current in zip(step_replies, step_replies[1:]):
        if current["tick"] != previous["tick"] + 1:
            errors.append(f"STEP 推进不严格：tick {previous['tick']} → "
                          f"{current['tick']}（应恰好 +1）")

    # ---- 断言 2：tick 必须从 1 开始且总数等于成功的步数 ----
    if step_replies[0]["tick"] != 1:
        errors.append(f"首个 STEP 的 tick 应为 1，实际 {step_replies[0]['tick']}")

    # ---- 断言 3：注入值进入控制链 → 能自动捕获进 BALANCE ----
    # 捕获快慢应从进入 SWING 起算：前面 CLEAR/STOP 的启动序列占 14 拍是
    # 协议规定动作，与被测传感注入无关。注入恒为倒立点零速，故进入 SWING
    # 后应在一两拍内满足捕获域（|α|<30°、|α̇|<150°/s、|θ̇|<500°/s）。
    modes = [r["mode"] for r in step_replies]
    if M_BALANCE not in modes:
        errors.append(f"固定倒立点零速下未能进入 BALANCE；模式轨迹={modes[:30]}")
    else:
        if M_SWING not in modes:
            errors.append("从未进入 SWING，却直接出现 BALANCE（模式机异常）")
        else:
            swing_at = modes.index(M_SWING)
            balance_at = modes.index(M_BALANCE)
            if balance_at < swing_at:
                errors.append(f"BALANCE({balance_at}) 早于 SWING({swing_at})")
            elif balance_at - swing_at > 5:
                errors.append(f"进入 SWING 后 {balance_at - swing_at} 拍才捕获"
                              f"（理想注入下应 ≤5 拍）")

    # ---- 断言 4：稳态 duty 应收敛到小量（倒立点零速无需大力矩） ----
    tail = [abs(r["duty"]) for r in step_replies[-10:]]
    if tail and max(tail) > 2000:
        errors.append(f"倒立点稳态 duty 过大 {max(tail)}（应接近 0）")

    # ---- 断言 5：上电故障必须锁存，且不得在 HALANCE 后复发 ----
    # 上电即锁存 fault_latch 是设计意图（安全态需人工确认后才解除，见
    # safety.v 与 tb_top_smoke）；CLEAR 之后的尾段则不允许再出现故障。
    if not step_replies[0]["fault"]:
        errors.append("首拍未锁存上电故障（应上电即安全）")
    late_faults = [r["tick"] for r in step_replies
                   if r["fault"] and r["mode"] in (M_SWING, M_BALANCE)]
    if late_faults:
        errors.append(f"进入运行态后仍报故障：tick={late_faults[:5]}")

    print(f"STEP 应答 {len(step_replies)} 条；模式轨迹 {modes[:20]}")
    print(f"末段 duty {tail}")
    if errors:
        for message in errors:
            print(f"FAIL: {message}")
        raise AssertionError(f"tb_hil_lockstep 失败（{len(errors)} 项）")
    print("tb_hil_lockstep 判定通过：STEP 严格逐拍、注入生效、可自动捕获")
    return 0


def main():
    try:
        return run()
    except Exception as exc:
        print(f"FAIL: {exc}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
