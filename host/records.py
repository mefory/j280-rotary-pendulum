"""记录文件读取：混合格式（遥测 CSV + 事件 JSONL 成对）与旧单文件 JSONL 通吃。

2026-10-29 起记录拆成两个文件（bridge.record 写出）：
  <基础名>.csv             遥测样本，表头 = protocol.FIELDS（固定 28 列，旧固件
                           没有的量写空），体积约为 JSONL 的 1/4
  <基础名>_events.jsonl    元数据/命令/备注/结束摘要（嵌套结构，一行一条）

两份文件用 tick 列对齐时间轴。旧的单文件 <名>.jsonl（sample 与事件混排）
继续可读，回放与审计都自动识别，传任意一个成员路径或无扩展名基础名均可。
"""
import csv
import json
from pathlib import Path

EVENTS_SUFFIX = "_events.jsonl"


def resolve_pair(path):
    """任意成员/基础路径 → (csv_path, events_path)；旧格式返回 (None, jsonl_path)。

    识别规则：.csv → 成对；*_events.jsonl → 成对；无扩展名且同名 .csv 存在
    → 成对基础名；其余（含一切 .jsonl）→ 旧单文件。
    """
    p = Path(path)
    name = p.name
    if p.suffix == ".csv":
        return p, p.with_name(p.stem + EVENTS_SUFFIX)
    if name.endswith(EVENTS_SUFFIX):
        return p.with_name(name[:-len(EVENTS_SUFFIX)] + ".csv"), p
    if not p.suffix and p.with_name(name + ".csv").exists():
        return p.with_name(name + ".csv"), p.with_name(name + EVENTS_SUFFIX)
    return None, p


def _jsonl_records(path):
    """逐行产出解析后的记录；坏行产出 {"type": "_parse_error"} 而不是中断。"""
    with Path(path).open(encoding="utf-8") as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            try:
                yield json.loads(line)
            except json.JSONDecodeError:
                yield {"type": "_parse_error"}


def read_metadata(path):
    """记录开头的 metadata 快照；没有则 None（旧格式文件可能没有）。"""
    csv_path, events_path = resolve_pair(path)
    if csv_path is None:
        # 旧格式：metadata 按约定是首行，但老文件可能没有——扫到为止
        for rec in _jsonl_records(events_path):
            if rec.get("type") == "metadata":
                return rec
            if rec.get("type") == "sample":     # 碰到样本即确认没有 metadata
                return None
        return None
    if not events_path.exists():
        return None
    for rec in _jsonl_records(events_path):
        if rec.get("type") == "metadata":
            return rec
    return None


def open_sample_stream(path):
    """打开遥测流，返回 (文件句柄, 迭代器)；用完必须 close 句柄。

    迭代器逐条产出 sample dict：CSV 空 cell → None（区分"值为 0"和
    "旧固件没这个量"）。回放/审计共用。
    """
    csv_path, events_path = resolve_pair(path)
    if csv_path is None:
        fh = events_path.open(encoding="utf-8")

        def it():
            for line in fh:
                line = line.strip()
                if not line:
                    continue
                try:
                    rec = json.loads(line)
                except json.JSONDecodeError:
                    continue
                if rec.get("type") == "sample":
                    yield rec["sample"]
        return fh, it()
    fh = csv_path.open(encoding="utf-8", newline="")
    rows = csv.reader(fh)
    header = next(rows, None) or []
    if not header:
        return fh, iter(())

    def it():
        for row in rows:
            if not row:
                continue
            yield {k: (int(v) if v != "" else None) for k, v in zip(header, row)}
    return fh, it()


def load_record(path):
    """整条记录读进内存 → (meta, samples, events)。审计/分析用；回放走流式。"""
    csv_path, events_path = resolve_pair(path)
    meta, samples, events = None, [], []
    if csv_path is None:
        # 旧格式单文件：sample 与事件混排，单遍分流
        for rec in _jsonl_records(events_path):
            kind = rec.get("type")
            if kind == "sample":
                samples.append(rec["sample"])
            elif kind == "metadata":
                meta = rec
            else:
                events.append(rec)
        return meta, samples, events
    # 新格式：events 文件里没有 sample（遥测全在 CSV）
    if events_path.exists():
        for rec in _jsonl_records(events_path):
            if rec.get("type") == "metadata":
                meta = rec
            else:
                events.append(rec)
    fh, it = open_sample_stream(path)
    try:
        samples = list(it)
    finally:
        fh.close()
    return meta, samples, events
