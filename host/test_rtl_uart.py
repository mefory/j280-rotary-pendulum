"""端到端测试：Python编码 -> UART引脚 -> 实际顶层 -> UART引脚 -> Python解码。"""
import json
from pathlib import Path
import struct
import subprocess

from . import protocol as p
from .schema import PARAMS, ROOT, SCHEMA_ID

# 参数默认值以 parameters.json 为唯一事实来源，不在此硬编码（增益回退时免改）。
K1_DEFAULT = next(x["default"] for x in PARAMS if x["id"] == 0)


def run():
    token = 0x78563412
    seq = 0
    requests, checks = [], []

    def add(kind, payload=b"", status=0, extra=None):
        nonlocal seq
        body = struct.pack("<I", token)+payload
        frame = p.Frame(kind, seq, body)
        requests.append(frame.encode())
        checks.append((seq, kind, status, extra))
        seq = (seq+1) & 65535
        return frame

    hello = p.Frame(p.HELLO, 0, struct.pack("<II", SCHEMA_ID, token))
    requests.append(hello.encode());checks.append((0,p.HELLO,0,None));seq=1
    add(p.READ,struct.pack("<H",0),extra=K1_DEFAULT)
    add(p.BEGIN,struct.pack("<I",0))
    add(p.SET,struct.pack("<Hi",0,17800))
    add(p.READ,struct.pack("<H",0),extra=K1_DEFAULT)  # shadow cannot leak
    commit = add(p.COMMIT,struct.pack("<I",0))
    requests.append(commit.encode());checks.append((commit.seq,p.COMMIT,0,None))
    add(p.READ,struct.pack("<H",0),extra=17800)
    add(p.READ,struct.pack("<H",0x10),p.BAD_ID)  # no low-nibble alias
    add(p.BEGIN,struct.pack("<I",0),p.CONFLICT)
    add(p.BEGIN,struct.pack("<I",1))
    add(p.SET,struct.pack("<Hi",4,-1),p.RANGE)
    add(p.SET,struct.pack("<Hi",0,65536),p.RANGE)
    add(p.ABORT)
    add(p.COMMIT,struct.pack("<I",1),p.NO_TXN)
    bad = bytearray(p.Frame(p.START,seq,struct.pack("<I",token)).encode());bad[-1]^=1
    requests.append(bytes(bad))  # no ACK, no sequence consumption, no action
    # Partial frame, then wait beyond FPGA parser timeout.
    requests.append(p.Frame(p.READ,seq,struct.pack("<IH",token,0)).encode()[:5])
    # Wait 2.5ms per row is below timeout5ms: add another empty wait.
    requests.append(b"");requests.append(b"")
    add(p.STATUS)
    add(p.CLEAR)
    # 手扶直立且静止时，IDLE 直接进入 BALANCE，不经过 SWING。
    add(p.BALANCE)
    add(p.BEGIN,struct.pack("<I",1),p.STATE)
    move=add(p.MOVE,struct.pack("<i",3000))
    requests.append(move.encode());checks.append((move.seq,p.MOVE,0,None))
    add(p.BALANCE)
    add(p.STOP)
    add(p.BEGIN,struct.pack("<I",1))
    add(p.DEFAULTS)
    add(p.COMMIT,struct.pack("<I",1))
    add(p.READ,struct.pack("<H",0),extra=K1_DEFAULT)
    out=ROOT/"sim/out";out.mkdir(exist_ok=True)
    (out/"debug_uart_stimulus.txt").write_text(
        "\n".join(f"{len(frame)} "+" ".join(f"{b:02x}" for b in frame) for frame in requests),
        encoding="ascii")
    # Board shells live under rtl/boards/<board>/; compile them too so the
    # same regression keeps working regardless of which top is elaborated.
    # .vh included: regenerated headers (schema hash, defaults) must trigger rebuild.
    sources=[ROOT/"tb/tb_debug_uart.v",*sorted((ROOT/"rtl").glob("*.v")),
             *sorted((ROOT/"rtl").glob("*.vh")),
             *sorted((ROOT/"rtl/boards").rglob("*.v"))]
    target=out/"debug_uart.vvp"
    # The vvp build costs tens of seconds; skip it when nothing changed.
    if not target.exists() or any(s.stat().st_mtime>target.stat().st_mtime for s in sources):
        subprocess.run(["iverilog","-g2012","-I","rtl","-s","tb_debug_uart","-o",
                        str(target),"tb/tb_debug_uart.v",*map(str,sources[1:])],cwd=ROOT,check=True)
    result=subprocess.run(["vvp",str(target)],cwd=ROOT,check=True,
                          capture_output=True,text=True,timeout=180)
    (out/"debug_uart.log").write_text(result.stdout,encoding="utf-8")
    wire=bytes(int(line.split()[1],16) for line in result.stdout.splitlines() if line.startswith("RX "))
    decoder=p.Decoder();frames=decoder.feed(wire)
    assert decoder.errors==0 and not decoder.buffer, "Invalid/incomplete UART frames"
    replies=[f for f in frames if f.kind==p.RESPONSE]
    assert len(replies)==len(checks),(len(replies),len(checks),[(r.seq,r.payload[:2].hex()) for r in replies])
    for f,(s,k,status,extra) in zip(replies,checks):
        assert (f.seq,f.payload[0],f.payload[1])==(s,k,status),(f,s,k,status)
        assert struct.unpack_from("<I",f.payload,10)[0]==token
        if extra is not None:
            assert struct.unpack_from("<i",f.payload,14)[0]==extra
    duplicates=[f for f in replies if f.seq==commit.seq]
    assert duplicates[0]==duplicates[1]
    assert struct.unpack_from("<I",duplicates[0].payload,2)[0]==1,"Commit replay changed revision"
    # STATUS 扩展字段端到端：应答前缀 14 字节 + 载荷。新版固件（e629499 起）
    # 载荷 29 字节（17 基础 + 12 MT6701 链路健康）；今日版再尾追 14 字节
    # MT6835 链路健康，共 43 字节。旧版 17。
    # 这里断言位流确实发出了扩展尾巴——否则上位机会误报"旧固件"。
    status_replies=[f for f in replies if f.payload[0]==p.STATUS]
    assert status_replies, "No STATUS reply captured"
    status_payload=status_replies[-1].payload[14:]
    assert len(status_payload)>=p.IO_STATUS.size+p.IO_STATUS_MT.size+p.IO_STATUS_MT68.size, \
        f"STATUS payload {len(status_payload)} bytes < {p.IO_STATUS.size+p.IO_STATUS_MT.size+p.IO_STATUS_MT68.size}" \
        "（固件未发 MT6701/MT6835 链路健康扩展）"
    mt=status_payload[p.IO_STATUS.size:p.IO_STATUS.size+p.IO_STATUS_MT.size]
    frames_mt,good_mt,crc_mt,mg_mt,code_mt,flags_mt,angle_mt=p.IO_STATUS_MT.unpack(mt)
    off68=p.IO_STATUS.size+p.IO_STATUS_MT.size
    mt68=status_payload[off68:off68+p.IO_STATUS_MT68.size]
    frames68,good68,crc68,serr68,code68,flags68,angle68=p.IO_STATUS_MT68.unpack(mt68)
    # 本测试跑 j280_top（无 MT6701/MT6835，健康字恒 0）——只验长度与可解析，
    # 真实驱动的非零计数由 tb_mt6701_ssi_if / tb_mt6835_spi_if 与 t20k 冒烟覆盖。
    print(f"MT6701 link fields: frames={frames_mt} good={good_mt} crc={crc_mt} "
          f"mg={mg_mt} code={code_mt} flags={flags_mt} angle={angle_mt}")
    print(f"MT6835 link fields: frames={frames68} good={good68} crc={crc68} "
          f"serr={serr68} code={code68} flags={flags68} angle={angle68}")
    snapshots=[p.unpack_snapshot(f.payload) for f in frames if f.kind==p.TELEMETRY]
    assert snapshots and all(s["theta"]< -70000 for s in snapshots),"Truncated/incorrect signed theta"
    # HELLO 契约（2026-10-21）：14 字节公共前缀 + 4 字段 = 30 字节
    # （schema/build/caps/build_epoch）。此前只校验 code，字段漏发也照样 PASS，
    # 上位机便只能拿到 12 字节、显示不出构建时刻。
    hello=[f for f in frames if f.kind==p.RESPONSE and f.seq==0][0]
    assert len(hello.payload)==30,f"HELLO 应为 30 字节(14+16)，实际 {len(hello.payload)}"
    h_schema,h_build,h_caps,h_epoch=struct.unpack_from("<IIII",hello.payload,14)
    assert h_schema==SCHEMA_ID,f"HELLO schema 失配：{h_schema:#x}"
    assert h_caps==p.CAPS_REAL,f"HELLO 能力位应为真实模式 {p.CAPS_REAL:#x}，实际 {h_caps:#x}"
    # 仿真跑的是源码树头文件（基线副本），epoch 必须为 0 —— 只有构建脚本
    # 带 --now 生成的构建目录副本才非 0。这条同时守住"源码树不被污染"。
    assert h_epoch==0,f"源码树仿真的构建戳应为 0，实际 {h_epoch:#x}"
    assert any(s["mode"]==2 for s in snapshots),"Manual BALANCE did not enter mode 2"
    pos=[s for s in snapshots if s["mode"]==3]
    assert pos and all(s["target"]-s["theta"]==3000 for s in pos),"Relative move repeated or wrong anchor"
    assert any(s["revision"]==2 for s in snapshots)
    summary={"responses":len(replies),"snapshots":len(snapshots),"crc_errors":decoder.errors,
             "position_target":pos[0]["target"],"status":"PASS"}
    (out/"debug_uart_result.json").write_text(json.dumps(summary,indent=2),encoding="utf-8")
    print("UART V2 top-level PASS",summary)


if __name__=="__main__":
    run()
