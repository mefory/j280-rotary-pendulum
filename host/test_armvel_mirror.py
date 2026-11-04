"""host/test_armvel_mirror.py — 摆臂测速 Python 镜像 vs RTL(tb_arm_vel) 对拍回归。

用与 tb/tb_arm_vel.v 完全相同的边沿序列驱动 host.plant.Plant._arm_speed()，
对照 iverilog 实测值。镜像与 RTL 任何一处不同口径（窗口、除数、判停、EMA、饱和）
都会在这里掉出来——这是"模型镜像忠实于 RTL"的守门测试。
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from host.plant import Plant  # noqa: E402

# tb/tb_arm_vel.v 的实测值（iverilog，发布参数 USE_T=1/STALL_TICKS=300/4 拍窗）
CASES = [
    # (说明, period_ticks, delta, 运行拍数, 期望, 容差)
    ("T 1/250tick +13",              250,   1,  2 * 250 + 20,   13,   2),
    ("T 1/115tick +30",              115,   1,  2 * 115 + 20,   30,   3),
    ("T 1/60tick  +57",               60,   1,  2 * 60 + 20,    57,   2),
    ("T 1/12tick +288",               12,   1,  2 * 12 + 20,   288,  25),
    ("T/M cross 1/4tick +865",         4,   1,  2 * 4 + 20,    864,  20),
    ("M cross 1/2tick +1731",          2,   1,  2 * 2 + 20,   1729,  30),
    ("M fast 1/1tick +3462",           1,   1,  2 * 1 + 20,   3454,  30),
    ("decay after stop（无陈旧值）",     0,   0,  80,             45,  80),
    ("T 1/115tick -30（反向）",        115,  -1,  2 * 115 + 20,  -30,   3),
    ("stall -> 0（判停+EMA 尾巴）",      0,   0,  440,             0,   1),
    ("re-acquire +30",                115,   1,  2 * 115 + 20,   30,   3),
]


def drive(plant, period, delta, ticks):
    """与 tb 同口径：每 period 拍把 pos 加 delta（period=0 表示不动），返回末拍 θ̇。"""
    cnt = plant.ch[0]
    i = 0
    last = 0
    for _ in range(ticks):
        plant.ticks += 1
        if period and i >= period:
            i = 0
            cnt += delta
        i += 1
        last = plant._arm_speed(cnt)
    return last


def run():
    errors = 0
    plant = Plant()
    print("case                              expect  got   状态")
    for name, period, delta, ticks, expect, tol in CASES:
        got = drive(plant, period, delta, ticks)
        ok = abs(got - expect) <= tol
        errors += 0 if ok else 1
        print(f"{name:<32} {expect:7d} {got:5d}   {'OK' if ok else 'FAIL'}")
    print("test_armvel_mirror " + ("PASS" if errors == 0 else f"FAIL errors={errors}"))
    return errors


if __name__ == "__main__":
    sys.exit(1 if run() else 0)
