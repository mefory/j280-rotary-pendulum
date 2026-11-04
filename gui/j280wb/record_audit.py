"""分析一条记录：它能回答哪些问题、哪些问题答不了。

用法（新旧格式通吃：混合记录传 .csv / *_events.jsonl / 无扩展名基础名均可，
旧单文件 .jsonl 照旧）：
    python -m j280wb.record_audit logs/xxxx.csv
    python -m j280wb.record_audit logs/xxxx_events.jsonl
    python -m j280wb.record_audit logs/*.jsonl --brief

设计意图是"先体检再分析"：过去几轮排查都在猜"日志里到底有没有这个量"，
结果反复出现"以为记录了其实没记"（例如 duty 的死区补偿量、POS 积分器的
输出、观测器的接管状态都不在遥测里）。这个工具把"记录里有什么"变成可
机读的事实，缺什么就明确说缺，而不是让分析脚本静默算出一堆 nan。
"""
import argparse
import math
import sys
from collections import Counter

from host.records import load_record

MODES = {0: "待机", 1: "自动起摆", 2: "稳摆", 3: "定位", 4: "安全停止"}
MODES_INV = {v: k for k, v in MODES.items()}

# 协议命令号 → 名称（audit 文本报告与曲线页复盘标记共用）
COMMAND_NAMES = {1: "HELLO", 2: "READ", 3: "BEGIN", 4: "SET", 5: "COMMIT", 6: "ABORT",
                 7: "DEFAULTS", 8: "STOP", 9: "CLEAR", 10: "START", 11: "MOVE",
                 12: "BALANCE", 13: "CALIBRATE", 14: "STATUS", 15: "STEP", 16: "IO_TEST"}

# 遥测快照里有哪些量（固件 debug_link.v take_snapshot 的顺序）
TELEMETRY = ["tick", "revision", "mode", "fault", "alpha", "alpha_dot", "theta",
             "theta_dot", "duty", "reference", "target", "encoder", "adc",
             "offset", "calib_done", "run_en"]

# 分析时真正想用、但当前遥测**没有**的量。每条都标注它挡住了什么问题。
MISSING = [
    ("pos_trim (积分器输出)", "无法判断 POS 里臂靠什么力矩站住、积分器是否饱和"),
    ("pos_i (积分器状态)", "无法分辨'预热值正确'与'积分器又跑飞了'"),
    ("k3_eff / k4_eff (渐变后实际增益)", "无法把 POS 入场瞬态和增益渐变分离"),
    ("obs_active (观测器是否接管)", "OBS_USE=1 后无法从数据确认走的是哪条链路"),
    ("alpha_raw (trim 前)", "无法反推当轮真实零偏，只能靠 offset+trim 猜"),
    ("duty_pre_dz (死区补偿前)", "死区 640/740 只能靠反算，无法直接核对"),
    ("mode_prev / 模式迁移计数", "0.17-0.4s 的短驻留完全靠拍数推断"),
]


def load(path):
    """读一条记录 → (meta, samples, events)；新旧格式自动识别（host/records.py）。"""
    return load_record(path)


def runs(samples, key="mode"):
    """把连续相同 mode 的采样压成段。"""
    out = []
    prev = None
    start = 0
    for i, s in enumerate(samples):
        if s[key] != prev:
            if prev is not None:
                out.append((prev, start, i - 1))
            prev = s[key]
            start = i
    if samples:
        out.append((prev, start, len(samples) - 1))
    return out


def rms(xs):
    xs = [x for x in xs if x is not None]
    return math.sqrt(sum(x * x for x in xs) / len(xs)) if xs else float("nan")


def mean(xs):
    xs = [x for x in xs if x is not None]
    return sum(xs) / len(xs) if xs else float("nan")


def audit(path, brief=False):
    meta, samples, events = load(path)
    print("=" * 78)
    print(f"记录：{path}")
    if not samples:
        print("  ⚠ 没有任何 sample，无法分析")
        return 1

    # ---------- 1. 元数据：这是哪套固件/参数 ----------
    print("\n[1] 元数据")
    if meta is None:
        print("  ✗ 没有 metadata 记录（老格式），固件与参数完全不可考")
    else:
        ver = meta.get("version")
        fmt = meta.get("record_format")
        print(f"  格式版本 {ver}{f'（{fmt}）' if fmt else ''}  "
              f"采集于 {meta.get('created', '?')}  设备 {meta.get('device', '?')}")
        ident = meta.get("identity") or {}
        print(f"  板 {ident.get('board', '?')}  build {ident.get('build')}  "
              f"capabilities {ident.get('capabilities')}")
        if ident.get("date"):
            print(f"  ⚠ 固件自报日期 {ident['date']} 是 generate_debug_config.py 里的"
                  f"常量 BUILD_DATE，不随构建变化——不能用来区分比特流版本")
        fw = meta.get("firmware") or {}
        if fw.get("host_git"):
            print(f"  上位机 git {fw.get('host_git')} ({fw.get('host_git_branch')})")
        params = meta.get("parameters")
        if params is None:
            print("  ✗ 没有参数快照：无法知道这轮用的是哪组增益")
        else:
            detail = meta.get("parameters_detail") or []
            if detail:
                for item in detail:
                    print(f"    id{item['id']:>2} {item['label']:<12} = {item['value']:>7}"
                          f"  ({item['key']}, scale {item.get('scale')})")
            else:
                print(f"  参数（原始列表，无标签）：{params}")
        cal = meta.get("calibration") or {}
        if cal:
            print(f"  记录开始时零点：offset={cal.get('offset_c01')} "
                  f"trim={cal.get('alpha_trim_c01')} calib_done={cal.get('calib_done')} "
                  f"α={cal.get('alpha_deg')}")

    # ---------- 2. 采样完整度 ----------
    print("\n[2] 采样完整度")
    ticks = [s["tick"] for s in samples]
    gaps = [b - a for a, b in zip(ticks, ticks[1:])]
    span = ticks[-1] - ticks[0]
    expected = span // 2 + 1 if span > 0 else 0
    print(f"  样本 {len(samples)}  跨度 {span} tick = {span/1000:.2f}s")
    print(f"  按 2ms/帧应有 {expected} 帧 → 缺失率 "
          f"{(1 - len(samples)/expected)*100 if expected else 0:.3f}%")
    if gaps:
        big = [g for g in gaps if g > 2]
        print(f"  最大间隔 {max(gaps)} tick，>2 tick 的断点 {len(big)} 处")
        if big:
            print(f"  ⚠ 有丢帧，时间轴分析（频率/相位）需要先插值")

    # ---------- 3. 命令与事件 ----------
    print("\n[3] 命令 / 日志事件")
    kinds = Counter(e.get("kind") for e in events if e.get("type") == "command")
    KM = COMMAND_NAMES
    if kinds:
        print("  命令：" + "、".join(f"{KM.get(k, k)}×{v}" for k, v in sorted(kinds.items())))
    else:
        print("  命令：无")
    typed = Counter(e.get("type") for e in events)
    print("  记录类型：" + "、".join(f"{k}×{v}" for k, v in typed.items()))
    cmds = [e for e in events if e.get("type") == "command"]
    if cmds and "tick" not in cmds[0]:
        print("  ⚠ 命令记录没有 tick/时间戳，无法与遥测对齐（旧格式）")
    if typed.get("record_end"):
        end = next(e for e in events if e.get("type") == "record_end")
        print(f"  结束摘要：{end.get('samples')} 帧 / {end.get('span_ms')}ms，"
              f"missing_ratio={end.get('missing_ratio')}")
        print("  模式直方图：" + "、".join(f"{k}×{v}" for k, v in
                                     (end.get("mode_histogram") or {}).items()))
    else:
        print("  ⚠ 没有 record_end：无法确认记录是否被中途截断")

    # ---------- 4. 各模式的可用量 ----------
    print("\n[4] 分段与关键指标")
    for mode, a, b in runs(samples):
        seg = samples[a:b + 1]
        n = len(seg)
        if n < 20:
            continue
        al = [s["alpha"] / 100 for s in seg]
        du = [s["duty"] for s in seg]
        line = (f"  {MODES.get(mode, mode):<6} {n:>6}帧 {n*2/1000:>7.2f}s  "
                f"α均值 {mean(al):>+7.2f}°  αRMS {rms(al):>6.2f}°  "
                f"|α|max {max(abs(x) for x in al):>6.2f}°  "
                f"duty 均值 {mean(du):>+7.0f} std {rms([x - mean(du) for x in du]):>6.0f}")
        print(line)
        if mode == 3 and n > 100:
            te = [s["theta"] - s["reference"] for s in seg]
            print(f"         θe 均值 {mean(te)/100:>+7.2f}°  θ 行程 "
                  f"{(seg[-1]['theta']-seg[0]['theta'])/100:>+7.1f}°")
            print(f"         遥测缺 pos_trim → 无法判断臂是靠积分器还是 K3 站住的")

    # ---------- 5. 关键参数的可推断性 ----------
    print("\n[5] 当前遥测能算 / 不能算")
    have = [k for k in TELEMETRY if any(k in s for s in samples)]
    print(f"  遥测字段（{len(have)}）：" + "、".join(have))
    const = [k for k in have if len({s.get(k) for s in samples}) == 1]
    if const:
        print(f"  全程常量（信息量为零）：" + "、".join(const))
    print("  缺失但分析需要的量：")
    for name, why in MISSING:
        print(f"    ✗ {name} — {why}")

    if brief:
        return 0

    # ---------- 6. 由记录反推的关键量 ----------
    print("\n[6] 反算（用于核对）")
    meta_params = (meta or {}).get("parameters")
    if meta_params and len(meta_params) >= 14:
        k1, k2, k3, k4 = meta_params[0], meta_params[1], meta_params[2], meta_params[3]
        dzp, dzn = meta_params[4], meta_params[5]
        trim = meta_params[13]
        offs = {s.get("offset") for s in samples}
        print(f"  死区补偿实际生效值 = 参数值本身：+{dzp} / −{dzn}"
              f"（debug_link 里 <<<1、out_stage 里 >>>1 相互抵消）")
        # 单位纪律：α/θ/θe 是 cdeg(0.01°)，α̇/θ̇ 是 0.1°/s；增益 scale 8192。
        # 写成"duty/1°α"必须先 ×100 换成 cdeg，否则差两个数量级。
        k1_per_deg = k1 / 8192 * 100
        print(f"  K1 = {k1_per_deg:.1f} duty/1° α（= {k1/8192:.4f} duty/cdeg）")
        print(f"  ⇒ α 偏置越过死区 {dzp} 只需 {dzp/k1_per_deg:.2f}°，"
              f"越过 {dzn} 需 {dzn/k1_per_deg:.2f}°")
        for k, v, unit in (("K2", k2, "duty/(0.1°/s)"), ("K3", k3, "duty/cdeg"),
                           ("K4", k4, "duty/(0.1°/s)")):
            print(f"  {k} = {v} (scale 8192) → {v/8192:.4f} {unit}")
        print(f"  trim = {trim} cdeg；采样里 offset 取值 "
              f"{sorted(offs) if len(offs) < 6 else f'{len(offs)} 种'}")
        print(f"  ⇒ 当轮 α 零位 = offset − trim = "
              f"{'?' if len(offs) != 1 else sorted(offs)[0] - trim} c01"
              f"（angle_proc: α = raw − offset − trim）")
    else:
        print("  ✗ 无参数快照，无法反算任何增益/死区关系")
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description="分析记录（CSV+事件成对或旧单 JSONL）的可分析性")
    ap.add_argument("paths", nargs="+")
    ap.add_argument("--brief", action="store_true", help="只看元数据与完整度")
    args = ap.parse_args(argv)
    rc = 0
    for path in args.paths:
        rc |= audit(path, args.brief)
    return rc


if __name__ == "__main__":
    sys.exit(main())
