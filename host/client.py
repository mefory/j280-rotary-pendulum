"""Single-outstanding stop-and-wait client, used by CLI and GUI worker."""
import json
import secrets
import struct
import time
from . import protocol as p
from .schema import PARAMS,SCHEMA_ID,validate,LEGACY_SCHEMAS


class CommandError(RuntimeError):
    def __init__(self, code):
        self.code=code
        super().__init__(p.ERRORS.get(code,f"设备错误 {code}"))


class Client:
    def __init__(self, device, sample_callback=None, event_callback=None):
        self.device=device;self.decoder=p.Decoder();self.sequence=0;self.token=0
        self.revision=0;self.connected=False;self.last_sample=None
        # 设备实际支持的参数个数（旧参数表固件 < len(PARAMS)，见 LEGACY_SCHEMAS）
        self.param_count=len(PARAMS)
        self.sample_callback=sample_callback or (lambda sample:None)
        self.event_callback=event_callback or (lambda event:None)
        self.replies={};self.identity={}

    def poll(self, milliseconds=10):
        self.device.advance(milliseconds)
        for frame in self.decoder.feed(self.device.read()):
            # 长度白名单覆盖 v2/v3/v4 三代固件快照（40/54/64B）；只认单档
            # 会让旧固件的"健康串口"看起来像空遥测流。
            if frame.kind==p.TELEMETRY and len(frame.payload) in p.SNAPSHOT_SIZES:
                sample=p.unpack_snapshot(frame.payload)
                self.last_sample=sample;self.sample_callback(sample)
            elif frame.kind==p.RESPONSE:
                self.replies[frame.seq]=frame

    def _hello_once(self, schema_id):
        """发一次 HELLO 并等应答。返回 (status, payload)：
        ("ok", identity载荷) / ("reject", None)——固件回 BAD_SCHEMA(7)，换候选
        schema / ("timeout", None)——无应答，上层换波特率。
        与 request() 分开实现：这代固件对不认识的 schema 回 code=7 且 token=0，
        request() 的 token 校验会把它当噪声丢弃 → 外层只见超时，古早固件永远
        连不上（2026-11-01 实测 audit_20261020_200909 即此表现）。"""
        frame=p.Frame(p.HELLO,self.sequence,struct.pack("<II",schema_id,self.token))
        self.event_callback({"type":"command","kind":p.HELLO,"seq":self.sequence,
                             "payload":frame.payload.hex()})
        self.device.write(frame.encode())
        deadline=time.monotonic()+.3
        while time.monotonic()<deadline:
            self.poll(1)
            response=self.replies.pop(frame.seq,None)
            if response is None or len(response.payload)<14:
                continue
            command,code,revision,stamp,token=struct.unpack_from("<BBIII",response.payload)
            if command!=p.HELLO:
                continue
            if code==p.BAD_SCHEMA:
                return "reject",None
            if code:
                raise CommandError(code)
            if token!=self.token:
                continue
            self.sequence=(self.sequence+1)&65535;self.revision=revision
            self.event_callback({"type":"response","kind":p.HELLO,"code":0,
                                 "revision":revision,"applied_tick":stamp,"seq":frame.seq})
            return "ok",response.payload[14:]
        return "timeout",None

    def connect(self):
        if self.device.readonly:
            self.connected=True;self.identity={"replay":True};return
        baudrates=getattr(self.device,"baudrates",(None,))
        last_error=None
        # HELLO 候选 schema：现表优先，被 code=7 拒绝则依次试已知旧表。
        candidates=[SCHEMA_ID]+[s for s in LEGACY_SCHEMAS if s!=SCHEMA_ID]
        for baudrate in baudrates:
            if baudrate is not None and hasattr(self.device,"set_baudrate"):
                self.device.set_baudrate(baudrate)
            self.sequence=0;self.token=secrets.randbits(32) or 1
            self.replies.clear();self.decoder=p.Decoder()
            try:
                # 每个候选 3 次机会：适配器刚打开/板卡刚上电时首个 HELLO 可能丢
                extra=None;rejected_all=True
                for sid in candidates:
                    for _ in range(3):
                        status,payload=self._hello_once(sid)
                        if status=="ok":
                            extra=payload;rejected_all=False;break
                        if status=="reject":
                            break            # 该 schema 明确被拒，换下一个
                    if extra is not None:
                        break
                if extra is None:
                    if rejected_all:
                        raise ValueError("设备参数表与上位机不匹配"
                                         "（固件拒绝了全部已知参数表 schema）")
                    raise TimeoutError("HELLO 无应答（已重发同序号）")
                # 16 字节 = 新固件（含构建时间戳），12 字节 = 旧固件
                schema,build,capabilities=struct.unpack_from("<III",extra)
                epoch=struct.unpack_from("<I",extra,12)[0] if len(extra)>=16 else 0
                # 参数表兼容：同 schema 全量；已知历史表（LEGACY_SCHEMAS，纯尾
                # 部追加差异）降级到设备参数个数；未知 schema 仍拒连——参数
                # 语义无法核验，写错增益是安全问题。
                if schema==SCHEMA_ID:
                    self.param_count=len(PARAMS)
                else:
                    legacy_count=LEGACY_SCHEMAS.get(schema)
                    if legacy_count is None:
                        raise ValueError("设备参数表与上位机不匹配")
                    self.param_count=legacy_count
                # board/date 来自 build 戳的板标识；cap_names 供界面显示"这台设备会什么"，
                # 后端列表与按钮应按它启用，而不是按板名写死分支。
                # decode_epoch 给出真实构建时刻（旧固件/源码树基线 => built=None）。
                self.identity={"schema":schema,"build":build,"capabilities":capabilities,"session":self.token,
                               **p.decode_build(build),**p.decode_epoch(epoch),
                               "param_count":self.param_count,
                               "schema_matched":schema==SCHEMA_ID,
                               "cap_names":p.capability_names(capabilities)}
                self.connected=True
                return
            except TimeoutError as exc:
                last_error=exc
        if last_error is not None:
            raise last_error

    def request(self, kind, payload=b"", handshake=False):
        if self.device.readonly:raise PermissionError("只读回放不能发送命令")
        if not handshake and not self.connected:raise ConnectionError("需要先连接并完成版本握手")
        body=payload if handshake else struct.pack("<I",self.token)+payload
        frame=p.Frame(kind,self.sequence,body)
        self.event_callback({"type":"command","kind":kind,"seq":self.sequence,"payload":payload.hex()})
        for attempt in range(3):
            self.device.write(frame.encode())
            deadline=time.monotonic()+.3
            while time.monotonic()<deadline:
                self.poll(1)
                if frame.seq in self.replies:
                    response=self.replies.pop(frame.seq)
                    if len(response.payload)<14:continue
                    command,code,revision,stamp,token=struct.unpack_from("<BBIII",response.payload)
                    if command!=kind or token!=self.token:continue
                    self.sequence=(self.sequence+1)&65535;self.revision=revision
                    self.event_callback({"type":"response","kind":kind,"code":code,
                                         "revision":revision,"applied_tick":stamp,"seq":frame.seq})
                    if code:raise CommandError(code)
                    return response.payload[14:]
                if not hasattr(self.device,"plant"):time.sleep(.001)
        self.connected=False
        raise TimeoutError("命令未确认，已重发相同序号；请重新连接并回读状态，勿假定命令未执行")

    def read_params(self):
        result=[]
        for item in PARAMS[:self.param_count]:
            result.append(struct.unpack("<i",self.request(p.READ,struct.pack("<H",item["id"])))[0])
        return result

    def read_params_safe(self):
        """容错读参：单个 id 被设备拒绝（BAD_ID 等）不中断连接，
        返回 (values, failures)——values 中失败项为 None，failures 为
        [(id, key, 错误消息)]。用于固件参数表与上位机不一致时的观测降级。
        返回值恒为 len(PARAMS) 长：设备表之外的 id 补 None（不计失败），
        GUI 逐行 zip 时不会留下陈旧文本。"""
        values, failures = [], []
        for item in PARAMS[:self.param_count]:
            try:
                raw = self.request(p.READ, struct.pack("<H", item["id"]))
                values.append(struct.unpack("<i", raw)[0])
            except CommandError as exc:
                values.append(None)
                failures.append((item["id"], item["key"], str(exc)))
        values += [None] * (len(PARAMS) - len(values))
        return values, failures

    def apply(self, values):
        # 旧参数表固件：GUI/agent 仍传全表值，超出设备表的尾部项不发送
        n = self.param_count
        validate(values[:n], count=n)
        revision=self.revision
        self.request(p.BEGIN,struct.pack("<I",revision))
        try:
            for item,value in zip(PARAMS[:n],values[:n]):
                self.request(p.SET,struct.pack("<Hi",item["id"],value))
            self.request(p.COMMIT,struct.pack("<I",revision))
        except Exception:
            if self.connected:
                try:self.request(p.ABORT)
                except Exception:pass
            raise
        actual=self.read_params()
        if actual!=values[:n]:
            # COMMIT 在下一控制拍生效，立即回读可能撞上旧值，宽限一拍再验
            time.sleep(.005)
            actual=self.read_params()
        if actual!=values[:n]:raise RuntimeError("提交后回读不一致")
        # 返回恒 len(PARAMS) 长：设备表之外补 None，GUI 逐行刷新不落空
        return actual + [None]*(len(PARAMS)-n)

    def defaults(self):
        revision=self.revision
        self.request(p.BEGIN,struct.pack("<I",revision))
        self.request(p.DEFAULTS)
        self.request(p.COMMIT,struct.pack("<I",revision))
        return self.read_params() + [None]*(len(PARAMS)-self.param_count)

    def io_test(self, enable, duty):
        """安全态 I/O 点动：duty 为 -2000..2000，enable 为 0/1。"""
        return self.request(p.IO_TEST, struct.pack("<hH", int(duty), 1 if enable else 0))

    def io_status(self):
        """读取原始编码器/ADC以及当前 I/O 点动状态。

        整段透传给 unpack_io_status：新版固件在 17 字节基础字段后尾追 MT6701
        链路健康（+12），此处切片会把扩展扔掉、上位机误报"旧固件"。
        """
        data=self.request(p.STATUS)
        if len(data)<p.IO_STATUS.size:
            raise RuntimeError("设备 STATUS 未返回 I/O 联调字段，请更新 FPGA 固件")
        return p.unpack_io_status(data)
