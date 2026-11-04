"""Transport reads/writes V2 bytes; only simulation has an explicit advance clock."""
from collections import deque
import struct
from pathlib import Path
from . import protocol as p
from . import records
from .schema import PARAMS,SCHEMA_ID,validate
from .plant import Plant


class SimDevice:
    label="模拟设备（非硬件）"
    readonly=False

    def __init__(self, seed=7, board_id=None, capabilities=None, plant_profile=None):
        self.plant=Plant(seed, profile=plant_profile)
        self.active=[x["default"] for x in PARAMS];self.shadow=None
        self.revision=0;self.session=0;self.last=None;self.last_reply=None
        self.txn_age=0;self.expired=False;self.seq=0
        self.decoder=p.Decoder(timeout=.005);self.output=bytearray()
        self.pending=None
        self.io_test_enable=0;self.io_test_duty=0
        # HELLO 上报的板身份：默认"DE10-Standard 真实传感模式"。传 board_id/
        # capabilities 可以让模拟器假扮别的组合（例如 T20K + 锁步），供界面与
        # 迁移流程在无硬件时先跑通。实板以设备真实回包为准。
        self.board_id=p.BOARD_ID_DEFAULT if board_id is None else board_id
        self.capabilities=p.CAPS_REAL if capabilities is None else capabilities
        self.build=(self.board_id<<24)|(p.BUILD_STAMP_DEFAULT&0xFFFFFF)

    def close(self):
        pass

    def write(self, data):
        for frame in self.decoder.feed(data,now=self.plant.ticks/1000):
            self._execute(frame)

    def read(self):
        result=bytes(self.output);self.output.clear();return result

    def _reply(self, frame, code, extra=b"", cache=True):
        answer=p.Frame(p.RESPONSE,frame.seq,struct.pack(
            "<BBIII",frame.kind,code,self.revision,self.plant.ticks,self.session)+extra)
        self.output.extend(answer.encode())
        if cache:self.last=frame;self.last_reply=answer

    def _execute(self, f):
        if f==self.last:
            self.output.extend(self.last_reply.encode());return
        if f.kind==p.HELLO and f.seq==0:
            if len(f.payload)!=8:self._reply(f,p.BAD_LENGTH);return
            schema,token=struct.unpack("<II",f.payload)
            if schema!=SCHEMA_ID or token==0:self._reply(f,p.BAD_SCHEMA);return
            self.session=token;self.shadow=None;self.expired=False
            # 16 字节：schema+build+caps+构建时间戳（与固件 HELLO 布局一致）。
            # 模拟器不是真实构建，时间戳填当前的"仿真启动时刻"便于观察链路。
            import time as _t
            self._reply(f,0,struct.pack("<IIII",SCHEMA_ID,self.build,
                                        self.capabilities,int(_t.time())));return
        if (not self.session or len(f.payload)<4 or
            struct.unpack_from("<I",f.payload)[0]!=self.session or
            self.last is None or f.seq!=(self.last.seq+1)&65535):
            self._reply(f,p.SEQUENCE,cache=False);return
        data=f.payload[4:];safe=self.plant.mode in (0,4);code=0;extra=b""
        sizes={p.READ:2,p.BEGIN:4,p.SET:6,p.COMMIT:4,p.ABORT:0,p.DEFAULTS:0,
               p.STOP:0,p.CLEAR:0,p.START:0,p.MOVE:4,p.BALANCE:0,p.CALIBRATE:0,p.STATUS:0,
               p.STEP:6,p.IO_TEST:4}
        if f.kind not in sizes:code=p.BAD_ID
        elif len(data)!=sizes[f.kind]:code=p.BAD_LENGTH
        elif f.kind==p.READ:
            index=struct.unpack("<H",data)[0]
            if index>=len(self.active):code=p.BAD_ID
            else:extra=struct.pack("<i",self.active[index])
        elif f.kind==p.BEGIN:
            if not safe or self.plant.calibrating:code=p.STATE
            elif struct.unpack("<I",data)[0]!=self.revision:code=p.CONFLICT
            else:self.shadow=self.active.copy();self.txn_age=0;self.expired=False
        elif f.kind==p.SET:
            index,value=struct.unpack("<Hi",data)
            if not safe:code=p.STATE
            elif self.shadow is None:code=p.EXPIRED if self.expired else p.NO_TXN
            elif index>=len(self.active):code=p.BAD_ID
            elif not PARAMS[index]["min"]<=value<=PARAMS[index]["max"]:code=p.RANGE
            else:self.shadow[index]=value;self.txn_age=0
        elif f.kind==p.COMMIT:
            if not safe or self.plant.calibrating:code=p.STATE
            elif self.shadow is None:code=p.EXPIRED if self.expired else p.NO_TXN
            elif struct.unpack("<I",data)[0]!=self.revision:code=p.CONFLICT
            else:
                try:validate(self.shadow)
                except ValueError:code=p.RANGE
                else:self.pending=f;return
        elif f.kind==p.ABORT:self.shadow=None;self.expired=False
        elif f.kind==p.DEFAULTS:
            if not safe:code=p.STATE
            elif self.shadow is None:code=p.NO_TXN
            else:self.shadow=[x["default"] for x in PARAMS]
        elif f.kind==p.STOP:
            self.plant.mode=4;self.plant.duty=0;self.shadow=None
            self.io_test_enable=0;self.io_test_duty=0
        elif f.kind==p.CLEAR:
            if not safe:code=p.STATE
            else:self.plant.fault=0;self.plant.mode=0
        elif f.kind==p.START:
            if self.plant.mode!=0 or self.plant.fault or self.shadow is not None or self.plant.calibrating:code=p.STATE
            else:self.plant.mode=1
        elif f.kind==p.MOVE:
            delta=struct.unpack("<i",data)[0]
            if (self.plant.mode!=2 or self.plant.fault or abs(self.plant.alpha)>=500 or
                abs(self.plant.theta_dot)>=200):code=p.STATE
            elif abs(delta)>6000 or not -2**31<=self.plant.theta+delta<2**31:code=p.RANGE
            else:
                self.plant.reference=self.plant.theta;self.plant.target=self.plant.theta+delta
                self.plant.iq=0;self.plant.mode=3
        elif f.kind==p.BALANCE:
            manual_ready=(self.plant.mode==0 and abs(self.plant.alpha)<500 and
                          abs(self.plant.alpha_dot)<300 and abs(self.plant.theta_dot)<200)
            if self.plant.fault or self.shadow is not None or self.plant.calibrating or \
                    (self.plant.mode!=3 and not manual_ready):
                code=p.STATE
            else:
                self.plant.mode=2;self.plant.reference=self.plant.theta
                if manual_ready:
                    self.plant.target=self.plant.theta;self.plant.iq=0
        elif f.kind==p.CALIBRATE:
            if not safe or self.shadow is not None:code=p.STATE
            else:self.plant.calib_samples=[];self.plant.calibrating=True;self.plant.calib_done=0
        elif f.kind==p.STATUS:
            extra=p.IO_STATUS.pack(self.decoder.errors,0,self.plant.encoder,self.plant.adc,
                                   self.io_test_duty,self.io_test_enable)
        elif f.kind==p.IO_TEST:
            duty,enable=struct.unpack("<hH",data)
            if self.plant.mode not in (0,4):code=p.STATE
            elif enable not in (0,1) or not -2000<=duty<=2000:code=p.RANGE
            else:self.io_test_duty=duty;self.io_test_enable=enable
        elif f.kind==p.STEP:code=p.STATE  # 需要 CAP_STEP_LOCK；模拟器默认是真实传感模式
        self._reply(f,code,extra)

    def advance(self, milliseconds):
        for _ in range(milliseconds):
            self.plant.step(self.active)
            if self.pending is not None:
                request=self.pending;self.pending=None
                if self.plant.mode not in (0,4):self._reply(request,p.STATE)
                else:
                    self.active=self.shadow;self.shadow=None;self.revision+=1
                    self._reply(request,0)
            if self.shadow is not None:
                self.txn_age+=1
                if self.txn_age>=5000:self.shadow=None;self.expired=True
            if self.plant.ticks%2==0:
                values=self.plant.snapshot(self.revision)
                self.output.extend(p.Frame(p.TELEMETRY,self.seq,p.SNAPSHOT.pack(
                    *(values[k] for k in p.FIELDS))).encode())
                self.seq=(self.seq+1)&65535


class SerialDevice:
    readonly=False

    def __init__(self, port):
        import serial
        self.label=f"实板 {port}"
        self.serial=serial.Serial(port,1000000,timeout=0,write_timeout=.3)
        self.baudrate=1000000
        self.baudrates=(1000000,2000000)
        self.serial.reset_input_buffer()

    def set_baudrate(self, baudrate):
        if self.baudrate==baudrate:return
        self.serial.reset_input_buffer()
        self.serial.baudrate=baudrate
        self.baudrate=baudrate
        self.serial.reset_input_buffer()

    def write(self,data):
        self.serial.write(data)

    def read(self):
        return self.serial.read(self.serial.in_waiting)

    def advance(self,milliseconds):
        pass

    def close(self):
        self.serial.close()


class ReplayDevice:
    readonly=True

    def __init__(self, path):
        self.label="只读回放"
        # 新旧记录格式都走 host/records.py：成对（.csv + _events.jsonl）或旧单文件
        self.metadata=records.read_metadata(path)
        self._fh,self._samples=records.open_sample_stream(path)
        self.elapsed=0;self.origin=None;self.next_sample=None;self.seq=0
        self.output=bytearray();self._next()

    def _next(self):
        self.next_sample=None
        for sample in self._samples:
            self.next_sample=sample
            if self.origin is None:self.origin=sample["tick"]
            break

    def write(self,data):
        raise PermissionError("回放是只读的，不能发送动作或参数")

    def read(self):
        data=bytes(self.output);self.output.clear();return data

    def advance(self,milliseconds):
        self.elapsed+=milliseconds
        while self.next_sample and self.next_sample["tick"]-self.origin<=self.elapsed:
            # `or 0`：CSV 空 cell / 旧记录显式 null 的诊断量打包成 0（与"字段
            # 缺失补 0"同一语义），否则 struct.pack 收到 None 直接炸。
            self.output.extend(p.Frame(p.TELEMETRY,self.seq,p.SNAPSHOT.pack(
                *(self.next_sample.get(k) or 0 for k in p.FIELDS))).encode())
            self.seq=(self.seq+1)&65535;self._next()

    def close(self):
        self._fh.close()
