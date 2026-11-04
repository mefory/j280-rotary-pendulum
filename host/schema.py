import hashlib
import json
from decimal import Decimal, ROUND_HALF_UP
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCHEMA = json.loads((ROOT/"config/parameters.json").read_text(encoding="utf-8"))
PARAMS = SCHEMA["parameters"]
SCHEMA_ID = int.from_bytes(hashlib.sha256(json.dumps(
    SCHEMA, sort_keys=True, separators=(",", ":"), ensure_ascii=False).encode()).digest()[:4], "little")

# 已知历史参数表（schema_id → 参数个数），供连接古早固件时降级放行。
# schema id 是生成器对整张 parameters.json 的 SHA256 前 32 位，同一哈希
# ⇒ 参数表逐字节相同 ⇒ id→语义映射可信。0x099ff7a2（14 参，~2026-09-29
# 到 10-23 的主线，含稳摆冠军 audit_20261020_200909）已逐项比对该构建
# 快照的 j280_debug_params.vh（2026-11-01）：id1-14 的 key/min/max 与现表
# 完全一致，仅默认值随整定代际不同（默认值不参与读写兼容）。末尾追加型
# 差异（15/16 coast_amp/coast_margin）按"超出设备表"降级：不读不写。
# 不在名单里的未知 schema 仍拒连——参数语义无法核验，写错增益是安全问题。
LEGACY_SCHEMAS = {0x099ff7a2: 14}


def raw_value(param, physical):
    value = int((Decimal(str(physical))*param["scale"]).to_integral_value(rounding=ROUND_HALF_UP))
    if not param["min"] <= value <= param["max"]:
        raise ValueError(f"{param['label']} 超出范围")
    return value


def validate(values, count=None):
    """校验定点参数值。count=None 校验全表（16 项）；连接旧参数表固件时
    传 client.param_count 只校验设备实际支持的前 N 项（跨字段约束
    values[8]>values[7] 在 14 参时代就存在，两种长度都覆盖）。"""
    params = PARAMS if count is None else PARAMS[:count]
    if len(values) != len(params):
        raise ValueError("参数数量错误")
    for p, v in zip(params, values):
        if type(v) is not int or not p["min"] <= v <= p["max"]:
            raise ValueError(f"{p['label']} 定点值越界")
    if len(values) > 8 and values[8] <= values[7]:
        raise ValueError("掉摆角度必须大于捕获角度")
