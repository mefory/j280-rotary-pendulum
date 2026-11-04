"""用实测 duty 回放起摆初段，比较对象模型误差（不控制硬件）。"""
import argparse
import json
import math
from pathlib import Path

from .plant import Plant, wrap
from .records import load_record
from .schema import ROOT


def load_swing(path):
    _, samples, _ = load_record(path)   # 新旧记录格式通吃
    swing = [sample for sample in samples if sample["mode"] == 1]
    if not swing:
        raise ValueError(f"{path}: 未记录到起摆")
    return swing


def compare(swing, profile=None, arm_horizon=650, rod_horizon=480, motor_gain=None):
    plant = Plant(profile=profile)
    if motor_gain is not None:
        plant.p["motor_gain"] = motor_gain
    initial_tick = swing[0]["tick"]
    initial_theta = swing[0]["theta"]
    previous = swing[0]
    arm_errors = []
    rod_errors = []
    for sample in swing:
        elapsed = sample["tick"] - initial_tick
        if elapsed > max(arm_horizon, rod_horizon):
            break
        for _ in range(sample["tick"] - previous["tick"]):
            plant.integrate(plant.torque_for(previous["duty"], 1))
        adc, encoder = plant.sample_sensors()
        predicted_theta = encoder * 35446 // 1024
        predicted_alpha = wrap(adc * 34534 // 65536 - plant.offset + 18000)
        if elapsed <= arm_horizon:
            arm_errors.append((predicted_theta - (sample["theta"] - initial_theta)) / 100)
        if elapsed <= rod_horizon and 1000 < sample["adc"] < 60000:
            rod_errors.append(wrap(predicted_alpha - sample["alpha"]) / 100)
        previous = sample
    rms = lambda errors: round(math.sqrt(sum(error * error for error in errors) / len(errors)), 2) if errors else None
    return {"arm_rms_deg": rms(arm_errors), "rod_rms_deg": rms(rod_errors),
            "arm_samples": len(arm_errors), "rod_samples": len(rod_errors)}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("logs", nargs="+", type=Path)
    parser.add_argument("--profile", type=Path,
                        default=ROOT / "config/plant_de10_identified.json")
    parser.add_argument("--fit-motor-gain", action="store_true",
                        help="逐条扫描电机等效增益，诊断运行间差异；不修改参数文件")
    args = parser.parse_args()
    for path in args.logs:
        swing = load_swing(path)
        result = {"log": str(path), "default_early": compare(swing),
                  "candidate_early": compare(swing, args.profile),
                  "candidate_850ms": compare(swing, args.profile, 850, 850)}
        if args.fit_motor_gain:
            _, gain = min((compare(swing, args.profile, motor_gain=index / 100)["arm_rms_deg"],
                           index / 100) for index in range(50, 81))
            result["motor_gain_fit"] = {"gain": gain,
                                        "early": compare(swing, args.profile, motor_gain=gain),
                                        "at_850ms": compare(swing, args.profile, 850, 850, gain)}
        print(json.dumps(result, ensure_ascii=False))
