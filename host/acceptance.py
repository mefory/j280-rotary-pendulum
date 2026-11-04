"""Repeatable no-board missions using the real protocol client."""
import argparse
import json
import math
import statistics
import struct
import time
from . import protocol as p
from .devices import SimDevice
from .client import Client
from .schema import ROOT


def run_case(delta=3000,seed=7,plant_profile=None):
    device=SimDevice(seed,plant_profile=plant_profile);samples=[]
    client=Client(device,samples.append);client.connect()
    client.request(p.CLEAR)
    # Warm sensor histories in IDLE like the physical FPGA before START.
    client.poll(30);client.request(p.START)
    start=device.plant.ticks
    entered=None
    for _ in range(2500):
        client.poll(10)
        elapsed=device.plant.ticks-start
        if elapsed>=8000 and entered is None and device.plant.mode in (2,3):
            client.request(p.MOVE,struct.pack("<i",delta))
            entered=device.plant.ticks
    return _verdict(delta,seed,samples,start,entered)


def run_hil(port,delta=3000,seed=7,ticks=25000,plant_profile=None):
    """锁步 HIL 任务：真实 FPGA 跑控制链，本机跑被控对象模型。

    与板无关——只要设备回包能力位含 CAP_STEP_LOCK（DE10 `SW[9]=ON`，或
    Tang Primer 20K V1 的常开锁步）即可运行。

    节拍由上位机经 STEP 逐拍驱动（每步恰好一个仿真毫秒），因此可用与
    run_case 相同的判据衡量"真实 RTL 在真实模型下的定位表现"。墙钟耗时
    受 UART 往返限制：ticks 拍约为 ticks/1000 秒量级（1kHz 步频）。
    """
    from .hil import HilDevice
    device=HilDevice(port,seed,plant_profile=plant_profile);samples=[]
    client=Client(device,samples.append)
    client.connect()
    # 启动序列必须夹在 STEP 之间，且必须显式补一个 STOP：HIL 下 STEP 是唯一
    # 节拍来源（自由运行的 1ms 计数器被旁路），而 mode_fsm 靠 tick 推进；
    # CLEAR 只清 fault_latch、当拍 FSM 仍看到旧 fault 故停在 SAFE，而离开 SAFE
    # 需要一次"停机类"命令，START 又要求当前已是 IDLE。故顺序为
    # CLEAR → (几拍) → STOP → (几拍) → START。已由 host.test_hil_lockstep 逐拍验证。
    def hold(count):
        device.run(client,count)
    hold(4)
    client.request(p.CLEAR);hold(4)
    client.request(p.STOP);hold(4)
    client.request(p.START);hold(4)
    start=device.plant.ticks
    entered=None
    while device.plant.ticks-start<ticks:
        sample=device.step_and_integrate(client)
        elapsed=device.plant.ticks-start
        if elapsed>=8000 and entered is None and sample["mode"] in (2,3):
            client.request(p.MOVE,struct.pack("<i",delta))
            entered=device.plant.ticks
    return _verdict(delta,seed,samples,start,entered)


def _verdict(delta,seed,samples,start,entered):
    """按统一判据评估一次定位任务；仿真与 HIL 共用，保证结论可比。"""
    captured=next((s["tick"] for s in samples if s["mode"]==2 and s["tick"]>=start),None)
    moving=[s for s in samples if entered and s["tick"]>=entered]
    late=moving[-1000:]
    sigma=statistics.stdev(s["alpha"]/100 for s in late) if len(late)>1 else None
    maxerr=max((abs((s["theta"]-s["target"])/100) for s in late),default=None)
    peak=max((abs(s["alpha"]/100) for s in moving),default=None)
    error15=next((abs((s["theta"]-s["target"])/100) for s in moving
                  if s["tick"]>=entered+15000),None) if entered else None
    result={"seed":seed,"delta":delta,"capture_s":None if captured is None else (captured-start)/1000,
            "error15":error15,"late_error":maxerr,"moving_alpha":peak,"sigma":sigma,
            "hold":bool(captured and all(s["mode"] in (2,3) for s in samples if s["tick"]>=captured))}
    result["pass"]=bool(captured and result["capture_s"]<5 and result["hold"] and
                        error15 is not None and maxerr is not None and peak is not None and
                        sigma is not None and error15<2 and maxerr<2 and peak<3 and sigma<.5)
    return result,samples


if __name__=="__main__":
    parser=argparse.ArgumentParser()
    parser.add_argument("--all",action="store_true")
    parser.add_argument("--hil",metavar="PORT",
                        help="对真实板卡跑锁步 HIL 任务（例：--hil COM5，DE10 或 Tang Primer 20K）")
    parser.add_argument("--ticks",type=int,default=25000,
                        help="HIL 任务长度（仿真毫秒）；默认 25s 仿真量")
    parser.add_argument("--plant-profile",type=str,
                        help="可选对象参数文件；辨识候选仅供探索，不作验收结论")
    args=parser.parse_args();results=[];started=time.monotonic()
    for delta in ([-6000,-3000,3000,6000] if args.all else [3000]):
        if args.hil:
            result,samples=run_hil(args.hil,delta,ticks=args.ticks,plant_profile=args.plant_profile)
        else:
            result,samples=run_case(delta,plant_profile=args.plant_profile)
        results.append(result);print(json.dumps(result),flush=True)
    filename=("hil" if args.hil else "host")+("_profile_explore" if args.plant_profile else "_acceptance")+".json"
    destination=ROOT/"sim/out"/filename
    destination.write_text(json.dumps(results,indent=2),encoding="utf-8")
    print(f"Elapsed {time.monotonic()-started:.1f}s")
    assert args.plant_profile or all(r["pass"] for r in results), (
        "HIL mission failed on hardware" if args.hil else
        "No-board mission failed; do not label simulator validated")
