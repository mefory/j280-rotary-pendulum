"""Windows调试工作台。python -m host.app [--smoke screenshot.png]"""
import argparse
from collections import deque
from datetime import datetime
import hashlib
import json
import queue
import struct
import sys
import time
from pathlib import Path

from PySide6 import QtCore,QtWidgets
import pyqtgraph as pg
import serial.tools.list_ports

from . import protocol as p
from .client import Client
from .devices import SimDevice,SerialDevice,ReplayDevice
from .hil import HilDevice
from .schema import ROOT,PARAMS,SCHEMA_ID,raw_value,validate

MODES={0:"待机",1:"自动起摆",2:"稳摆",3:"定位",4:"安全停止"}


class DeviceWorker(QtCore.QThread):
    samples=QtCore.Signal(list)
    parameters=QtCore.Signal(list,int)
    message=QtCore.Signal(str)
    identity=QtCore.Signal(dict)
    io_status=QtCore.Signal(dict)
    failed=QtCore.Signal(str)

    def __init__(self, kind, location="", parent=None):
        super().__init__(parent);self.kind=kind;self.location=location
        self.commands=queue.Queue();self.batch=[];self.logfile=None;self.client=None
        self.demo=None;self.active_values=None

    def sample(self, sample):
        self.batch.append(sample)
        self.record({"type":"sample","sample":sample})

    def record(self, entry):
        if self.logfile:
            self.logfile.write(json.dumps(entry,ensure_ascii=False)+"\n")

    def dispatch(self, name, value):
        c=self.client
        if name=="read":
            values,failures=c.read_params_safe()
            self.active_values=values;self.parameters.emit(values,c.revision)
            if failures:
                detail="；".join(f"id{id}({key}) {msg}" for id,key,msg in failures)
                build=c.identity.get("build")
                self.message.emit(f"⚠ {len(failures)}/{len(PARAMS)} 个参数设备拒绝读取——上位机与固件的参数表实现不一致"
                                  f"（固件 build 0x{build:08x}）。连接保持，观测与命令可用；提交配置前请先更新固件。明细：{detail}")
            else:
                self.message.emit(f"参数回读完成（{len(PARAMS)}/{len(PARAMS)}）")
        elif name=="apply":
            self.active_values=c.apply(value);self.parameters.emit(self.active_values,c.revision)
            self.message.emit(f"配置 v{c.revision} 已成组生效并回读确认")
        elif name=="defaults":
            self.active_values=c.defaults();self.parameters.emit(self.active_values,c.revision)
            self.message.emit("设备默认参数已恢复并回读；未启动电机")
        elif name=="record":
            if self.logfile:self.logfile.close();self.logfile=None
            if value:
                self.logfile=Path(value).open("w",encoding="utf-8")
                self.record({"type":"metadata","version":2,"created":datetime.now().isoformat(),
                    "device":c.device.label,"schema":SCHEMA_ID,"identity":c.identity,
                    "parameters":self.active_values,"seed":getattr(getattr(c.device,"plant",None),"seed",None),
                    "model_sha256":hashlib.sha256((ROOT/"config/plant.json").read_bytes()).hexdigest()})
                self.message.emit(f"开始记录：{value}")
            else:self.message.emit("记录已保存")
        elif name=="demo":
            if c.last_sample is None or c.last_sample["mode"] not in (0,4):
                raise ValueError("自动试验从待机/安全状态开始")
            c.request(p.CLEAR);c.request(p.START)
            self.demo={"phase":"settle","start":c.last_sample["tick"],"delta":value}
            self.message.emit("试验开始：起摆 → 8秒稳摆 → 相对定位 → 17秒观察")
        elif name=="command":
            kind,data=value
            if kind==p.STOP:self.demo=None
            c.request(kind,data);self.message.emit(f"命令已确认：{kind}")
        elif name=="io_status":
            self.io_status.emit(c.io_status())
        elif name=="io_test":
            enable,duty=value
            c.io_test(enable,duty)
            self.message.emit(f"I/O 点动命令已确认：{'使能' if enable else '释放'}，duty={duty}")

    def advance_demo(self):
        if not self.demo or not self.client.last_sample:return
        s=self.client.last_sample;job=self.demo
        if s["fault"] or (job["phase"]=="move" and s["mode"]!=3):
            self.demo=None
            self.client.request(p.STOP)
            self.failed.emit("自动试验中断：故障或掉摆，已发送停止")
            return
        if job["phase"]=="settle" and s["tick"]-job["start"]>=8000:
            if s["mode"]!=2 or abs(s["alpha"])>=500 or abs(s["theta_dot"])>=200:
                self.demo=None;self.client.request(p.STOP)
                self.failed.emit("8秒时未满足定位准入条件，试验已停止");return
            self.client.request(p.MOVE,struct.pack("<i",job["delta"]))
            job.update(phase="move",start=s["tick"],peak=0.,late_error=0.)
        elif job["phase"]=="move":
            job["peak"]=max(job["peak"],abs(s["alpha"])/100)
            elapsed=s["tick"]-job["start"]
            if elapsed>=15000:job["late_error"]=max(job["late_error"],abs(s["theta"]-s["target"])/100)
            if elapsed>=17000:
                self.demo=None
                report={"type":"trial","delta":job["delta"],"moving_alpha":job["peak"],
                        "late_error":job["late_error"],"pass":job["peak"]<3 and job["late_error"]<2}
                self.record(report);self.message.emit("试验结果："+json.dumps(report,ensure_ascii=False))
                self.client.request(p.STOP)
                self.message.emit("试验结束，已发送停止")

    def run(self):
        device=None
        try:
            device=(SimDevice() if self.kind=="模拟" else
                    ReplayDevice(self.location) if self.kind=="回放" else
                    HilDevice(self.location) if self.kind=="锁步HIL" else
                    SerialDevice(self.location))
            self.client=Client(device,self.sample,self.record)
            self.client.connect();self.identity.emit(self.client.identity)
            if not device.readonly:self.dispatch("read",None)
            self.message.emit(f"已连接：{device.label}")
            while not self.isInterruptionRequested():
                started=time.monotonic()
                try:
                    while not self.commands.empty():
                        name,value=self.commands.get_nowait()
                        try:self.dispatch(name,value)
                        except Exception as exc:
                            self.failed.emit(str(exc))
                            if not self.client.connected:raise
                    self.client.poll(10)
                    self.advance_demo()
                except Exception as exc:
                    self.failed.emit(str(exc));break
                if self.batch:
                    self.samples.emit(self.batch);self.batch=[]
                if self.logfile:self.logfile.flush()
                remaining=.01-(time.monotonic()-started)
                if remaining>0:self.msleep(max(1,int(remaining*1000)))
        except Exception as exc:self.failed.emit(str(exc))
        finally:
            if self.logfile:self.logfile.close();self.logfile=None
            if device:device.close()
            self.message.emit("连接已关闭")


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("J280 · 现场调试工作台")
        self.resize(1260,840);self.worker=None;self.readonly=False;self.active=None;self.revision=0
        self.history=deque(maxlen=15000);self.last=None;self.recording=False;self.last_rx_time=None
        self.ready=False;self.editors=[];self.active_items=[];self.pending_items=[];self.action_widgets=[]
        self.io_test_widgets=[];self.io_test_available=False;self.io_status_data=None;self.io_encoder_zero=None
        self.encoder_mark=None
        root=QtWidgets.QWidget();self.setCentralWidget(root);layout=QtWidgets.QVBoxLayout(root)
        header=QtWidgets.QHBoxLayout()
        title=QtWidgets.QLabel("J280  /  调试工作台");title.setStyleSheet("font-size:22px;font-weight:600;")
        header.addWidget(title);header.addStretch()
        self.backend=QtWidgets.QComboBox();self.backend.addItems(["模拟","实板","锁步HIL","回放"])
        self.port=QtWidgets.QComboBox();self.port.setEditable(True)
        self.refresh_ports=QtWidgets.QPushButton("⟳")
        self.refresh_ports.setFixedWidth(34)
        self.refresh_ports.clicked.connect(self.fill_ports)
        self.fill_ports()
        self.connect_button=QtWidgets.QPushButton("连接");self.connect_button.clicked.connect(self.toggle_connection)
        header.addWidget(self.backend);header.addWidget(self.port);header.addWidget(self.refresh_ports);header.addWidget(self.connect_button)
        layout.addLayout(header)
        self.banner=QtWidgets.QLabel("未连接 · 默认使用无板模拟，不会访问硬件")
        self.banner.setStyleSheet("background:#16334a;color:#edf6fc;padding:10px;border-radius:5px;")
        layout.addWidget(self.banner)
        controls=QtWidgets.QHBoxLayout()
        for label,cmd in [("清除启动故障",p.CLEAR),("启动起摆",p.START),("返回稳摆",p.BALANCE)]:
            b=self.action(label,lambda checked=False,c=cmd:self.command(c));controls.addWidget(b)
        self.stop_button=self.action("停止",lambda:self.command(p.STOP))
        self.stop_button.setStyleSheet("background:#b42333;color:white;font-weight:bold;padding:8px;")
        controls.addWidget(self.stop_button);controls.addStretch()
        self.record_button=QtWidgets.QPushButton("开始记录");self.record_button.clicked.connect(self.toggle_record)
        controls.addWidget(self.record_button);layout.addLayout(controls)
        self.tabs=QtWidgets.QTabWidget();layout.addWidget(self.tabs)
        self.build_overview();self.build_params();self.build_plots();self.build_calibration();self.build_hardware();self.build_trials();self.build_replay()
        self.events=QtWidgets.QPlainTextEdit();self.events.setReadOnly(True);self.events.setMaximumHeight(125)
        self.events.document().setMaximumBlockCount(500);layout.addWidget(self.events)
        self.timer=QtCore.QTimer(self);self.timer.timeout.connect(self.refresh);self.timer.start(40)
        self.setStyleSheet("QWidget{font-family:'Microsoft YaHei';font-size:13px;} QPushButton{padding:6px 12px;} QTabWidget::pane{border:1px solid #cbd5e1;} QHeaderView::section{padding:7px;background:#e8eef5;}")
        self.refresh()

    def action(self,text,callback):
        b=QtWidgets.QPushButton(text);b.clicked.connect(callback);self.action_widgets.append(b);return b

    def page(self,title):
        w=QtWidgets.QWidget();v=QtWidgets.QVBoxLayout(w);self.tabs.addTab(w,title);return v

    def fill_ports(self):
        """枚举本机串口，USB 串口桥（CH340/FTDI 等）排前并带描述；COM 号存 itemData。"""
        self.port.clear()
        ports=list(serial.tools.list_ports.comports())
        ports.sort(key=lambda p:(not ("CH340" in p.description or "FTDI" in p.description
                                       or "USB" in p.description.upper()), p.device))
        for p_ in ports:
            self.port.addItem(f"{p_.device}  {p_.description}", p_.device)
        if self.port.count()==0:self.port.addItem("未检测到串口","")

    def selected_port(self):
        data=self.port.currentData()
        return data or self.port.currentText().strip()

    def build_overview(self):
        v=self.page("总览");self.metrics=QtWidgets.QLabel("连接模拟设备即可开始")
        self.metrics.setStyleSheet("font-size:20px;padding:24px;");v.addWidget(self.metrics)
        self.freshness=QtWidgets.QLabel("遥测：无数据")
        self.freshness.setStyleSheet("padding:6px;border-radius:4px;background:#1d3a52;")
        v.addWidget(self.freshness)
        self.fresh_timer=QtCore.QTimer(self);self.fresh_timer.setInterval(250)
        self.fresh_timer.timeout.connect(self.update_freshness);self.fresh_timer.start()
        text=QtWidgets.QLabel("首次使用：连接 → 清除启动故障 → 启动起摆 → 稳定后相对定位。\n\n"
            "控制参数仅在停机时成组提交；连接时回读设备，不自动覆盖。\n"
            "界面显示角度/速度/%；板端使用定点整数，编辑值与生效值分开显示。\n\n"
            "模拟 / 实板 / 回放共享V2协议。模拟结果用于工具验证，实机仍需标定。")
        text.setWordWrap(True);v.addWidget(text);v.addStretch()

    def update_freshness(self):
        if self.last_rx_time is None or self.last is None:
            self.freshness.setText("遥测：无数据");self.freshness.setStyleSheet("padding:6px;border-radius:4px;background:#1d3a52;")
            return
        age_ms=int((time.monotonic()-self.last_rx_time)*1000)
        if age_ms>500:
            self.freshness.setText(f"⚠ 遥测断流 {age_ms} ms（500Hz 正常应 <10ms）——检查串口链路，"
                "或确认设备在自由节拍模式（HIL 模式无 STEP 时控制链冻结属正常）")
            self.freshness.setStyleSheet("padding:6px;border-radius:4px;background:#7a1f2b;color:#fff;")
        else:
            self.freshness.setText(f"遥测正常 · {age_ms} ms 前 · {len(self.history)} 帧在缓冲")
            self.freshness.setStyleSheet("padding:6px;border-radius:4px;background:#17402a;")

    def build_params(self):
        v=self.page("参数")
        v.addWidget(QtWidgets.QLabel("编辑 → 停止 → 成组提交 → 设备确认 → 回读核对。数值合法不代表闭环稳定。"))
        self.table=QtWidgets.QTableWidget(len(PARAMS),6)
        self.table.setHorizontalHeaderLabels(["分组 / 参数","单位","允许范围","编辑值","待提交定点值","设备生效值"])
        self.table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.ResizeMode.Stretch)
        for row,param in enumerate(PARAMS):
            for col,text in enumerate([param["group"]+" / "+param["label"],param["unit"],
                                      f'{param["min"]/param["scale"]:g} ～ {param["max"]/param["scale"]:g}']):
                item=QtWidgets.QTableWidgetItem(text);item.setFlags(item.flags()&~QtCore.Qt.ItemFlag.ItemIsEditable)
                self.table.setItem(row,col,item)
            editor=QtWidgets.QDoubleSpinBox();editor.setDecimals(6);editor.setRange(param["min"]/param["scale"],param["max"]/param["scale"])
            editor.setValue(param["default"]/param["scale"]);editor.setSingleStep(1/param["scale"])
            self.table.setCellWidget(row,3,editor);self.editors.append(editor)
            pending=QtWidgets.QTableWidgetItem(str(param["default"]));active=QtWidgets.QTableWidgetItem("未回读")
            for item in (pending,active):item.setFlags(item.flags()&~QtCore.Qt.ItemFlag.ItemIsEditable)
            self.table.setItem(row,4,pending);self.table.setItem(row,5,active)
            self.pending_items.append(pending);self.active_items.append(active)
            editor.valueChanged.connect(lambda value,r=row:self.pending_items[r].setText(str(raw_value(PARAMS[r],value))))
        v.addWidget(self.table)
        row=QtWidgets.QHBoxLayout()
        for label,callback in [("回读设备",lambda:self.enqueue("read")),
                               ("停机成组提交",lambda:self.enqueue("apply",self.edited_values())),
                               ("恢复设备默认",lambda:self.enqueue("defaults"))]:
            row.addWidget(self.action(label,callback))
        for label,callback in [("保存编辑配置",self.save_config),("载入到编辑区",self.load_config)]:
            b=QtWidgets.QPushButton(label);b.clicked.connect(callback);row.addWidget(b)
        v.addLayout(row)

    def build_plots(self):
        v=self.page("实时曲线");pg.setConfigOptions(antialias=False)
        graph=pg.GraphicsLayoutWidget();graph.setBackground("#0f1d2d");v.addWidget(graph)
        plots=[]
        for row,label in enumerate(["摆杆 α / °","摆臂 θ / °","最终输出 / %"]):
            plot=graph.addPlot(row=row,col=0);plot.setLabel("left",label);plot.showGrid(x=True,y=True,alpha=.2);plots.append(plot)
        plots[1].setXLink(plots[0]);plots[2].setXLink(plots[0]);plots[2].setLabel("bottom","设备时间 / s")
        plots[1].addLegend()
        self.curves=[plots[0].plot(pen="#6edce8"),plots[1].plot(pen="#6edce8",name="连续臂角"),
                     plots[1].plot(pen="#ffbd69",name="参考"),plots[1].plot(pen="#e789da",name="目标"),
                     plots[2].plot(pen="#91d89b")]

    def build_calibration(self):
        v=self.page("标定")
        label=QtWidgets.QLabel("首版支持：静止下垂零点采集、原始传感器观察、编码器一圈计数辅助。\n"
             "板端标定要求SAFE/IDLE，采集2秒并等待估计器稳定。\n"
             "CPR/极性自动下发、开环试探和电机死区扫描暂不开放，避免未验证动作。")
        label.setWordWrap(True);v.addWidget(label)
        v.addWidget(self.action("静止下垂采集零点（2秒）",lambda:self.command(p.CALIBRATE)))
        self.sensor_label=QtWidgets.QLabel("ADC / 编码器 / 零偏：等待数据");v.addWidget(self.sensor_label)
        b=QtWidgets.QPushButton("记录当前编码器值");b.clicked.connect(self.mark_encoder);v.addWidget(b)
        self.encoder_label=QtWidgets.QLabel("手动转一整圈后观察计数差；只读辅助，不改控制参数");v.addWidget(self.encoder_label)
        v.addStretch()

    def build_hardware(self):
        v=self.page("硬件联调")
        warning=QtWidgets.QLabel(
            "仅用于编码器和 TB6612 单独验收。测试前断开电机负载，确认 TB6612 共地；"
            "点动范围已限制在 ±20%，任何异常立即按停止。控制闭环仍保持 HIL，不会自动接管。"
        )
        warning.setWordWrap(True);warning.setStyleSheet("color:#9b1c1c;padding:8px;");v.addWidget(warning)
        self.io_capability=QtWidgets.QLabel("等待设备能力信息");v.addWidget(self.io_capability)
        form=QtWidgets.QFormLayout()
        self.io_encoder_label=QtWidgets.QLabel("未查询")
        self.io_adc_label=QtWidgets.QLabel("未查询")
        self.io_output_label=QtWidgets.QLabel("未查询")
        form.addRow("原始编码器计数",self.io_encoder_label)
        form.addRow("原始 ADC 码",self.io_adc_label)
        form.addRow("TB6612 输出状态",self.io_output_label)
        v.addLayout(form)
        row=QtWidgets.QHBoxLayout()
        self.io_refresh_button=QtWidgets.QPushButton("刷新硬件状态")
        self.io_refresh_button.clicked.connect(lambda:self.enqueue("io_status"))
        self.io_zero_button=QtWidgets.QPushButton("记录编码器零点")
        self.io_zero_button.clicked.connect(self.mark_io_encoder)
        row.addWidget(self.io_refresh_button);row.addWidget(self.io_zero_button);row.addStretch()
        v.addLayout(row)
        motor=QtWidgets.QGroupBox("TB6612 低速点动")
        motor_row=QtWidgets.QHBoxLayout(motor)
        self.io_duty=QtWidgets.QSpinBox();self.io_duty.setRange(-2000,2000);self.io_duty.setSingleStep(100);self.io_duty.setSuffix(" / 10000")
        self.io_enable_button=QtWidgets.QPushButton("使能点动")
        self.io_enable_button.clicked.connect(lambda:self.enqueue("io_test",(1,self.io_duty.value())))
        self.io_release_button=QtWidgets.QPushButton("释放输出")
        self.io_release_button.clicked.connect(lambda:self.enqueue("io_test",(0,0)))
        motor_row.addWidget(QtWidgets.QLabel("duty"));motor_row.addWidget(self.io_duty)
        motor_row.addWidget(self.io_enable_button);motor_row.addWidget(self.io_release_button);motor_row.addStretch()
        v.addWidget(motor)
        self.io_test_widgets=[self.io_refresh_button,self.io_zero_button,self.io_enable_button,self.io_release_button,self.io_duty]
        v.addStretch()

    def mark_io_encoder(self):
        if self.io_status_data:
            self.io_encoder_zero=self.io_status_data["encoder"]
            self.io_encoder_label.setText(f"{self.io_status_data['encoder']}（零点已记录）")

    def io_status_received(self,status):
        self.io_status_data=status
        encoder=status["encoder"]
        suffix=""
        if self.io_encoder_zero is not None: suffix=f"，相对零点 {encoder-self.io_encoder_zero:+d}"
        self.io_encoder_label.setText(f"{encoder}{suffix}")
        self.io_adc_label.setText(str(status["adc"]))
        self.io_output_label.setText(f"{'使能' if status['enabled'] else '释放'}，duty={status['duty']}")

    def build_trials(self):
        v=self.page("自动试验")
        row=QtWidgets.QHBoxLayout();self.delta=QtWidgets.QDoubleSpinBox();self.delta.setRange(-60,60);self.delta.setValue(30);self.delta.setSuffix(" °")
        row.addWidget(QtWidgets.QLabel("相对位移"));row.addWidget(self.delta)
        row.addWidget(self.action("稳摆时执行定位",lambda:self.command(p.MOVE,struct.pack("<i",round(self.delta.value()*100)))))
        v.addLayout(row)
        v.addWidget(self.action("启动25秒全流程试验",lambda:self.enqueue("demo",round(self.delta.value()*100))))
        v.addWidget(QtWidgets.QLabel("从待机/安全状态开始：自动起摆 → 等待8秒 → 相对定位 → 观察17秒 → 停止。\n"
                                    "移动峰值摆角与末段误差写入日志。按停止可随时中断。\n"
                                    "完整四方向自动验收另可运行：python -m host.acceptance --all"))
        v.addStretch()

    def build_replay(self):
        v=self.page("记录回放")
        v.addWidget(QtWidgets.QLabel("开始记录后保存：设备身份、参数表版本、初始配置、完整快照、命令确认和试验结果。\n"
                                    "格式为JSONL；回放读取同一数据，所有动作与写参数按钮禁用。"))
        b=QtWidgets.QPushButton("选择记录并进入只读回放");b.clicked.connect(self.open_replay);v.addWidget(b)
        v.addStretch()

    def log(self,text):
        self.events.appendPlainText(datetime.now().strftime("%H:%M:%S")+"  "+text)

    def enqueue(self,name,value=None):
        if not self.worker or not self.worker.isRunning():self.log("尚未连接");return
        if self.readonly and name not in ("record",):self.log("回放只读");return
        self.worker.commands.put((name,value))

    def command(self,kind,data=b""):
        self.enqueue("command",(kind,data))

    def edited_values(self):
        return [raw_value(p,e.value()) for p,e in zip(PARAMS,self.editors)]

    def params_received(self,values,revision):
        first=self.active is None;self.active=values;self.revision=revision;self.ready=True
        for row,(param,value) in enumerate(zip(PARAMS,values)):
            if value is None:
                self.active_items[row].setText("设备拒绝读取")
                continue
            self.active_items[row].setText(f'{value/param["scale"]:.6g}  [{value}]')
            if first:self.editors[row].setValue(value/param["scale"])

    def samples_received(self, samples):
        self.history.extend(samples)
        if samples:
            self.last=samples[-1];self.last_rx_time=time.monotonic()

    def mark_encoder(self):
        if self.last:self.encoder_mark=self.last["encoder"]

    def toggle_connection(self):
        if self.worker and self.worker.isRunning():self.disconnect();return
        kind=self.backend.currentText();location=self.selected_port()
        if kind in ("实板","锁步HIL") and not location:
            self.log("未选择串口：点 ⟳ 刷新，或确认 USB-TTL 已插好")
            return
        if kind=="回放":
            location,_=QtWidgets.QFileDialog.getOpenFileName(self,"选择记录",str(ROOT/"logs"),"JSONL (*.jsonl)")
            if not location:return
        self.start_connection(kind,location)

    def start_connection(self,kind,location=""):
        self.disconnect();self.history.clear();self.last=None;self.last_rx_time=None;self.active=None;self.ready=False
        self.readonly=kind=="回放"
        self.worker=DeviceWorker(kind,location,self)
        self.worker.samples.connect(self.samples_received);self.worker.parameters.connect(self.params_received)
        self.worker.message.connect(self.log);self.worker.failed.connect(lambda text:self.log("失败："+text))
        self.worker.identity.connect(self.identity_received)
        self.worker.io_status.connect(self.io_status_received)
        self.worker.start();self.connect_button.setText("断开")
        self.banner.setText(f"{kind} · {'只读，所有动作禁用' if self.readonly else '正在握手与回读'}")

    def identity_received(self,identity):
        if identity.get("replay"):
            return
        caps=identity.get("capabilities",0)
        self.io_test_available=p.has_capability(caps,p.CAP_IO_TEST)
        self.io_capability.setText("设备支持 I/O 联调" if self.io_test_available else "设备未声明 I/O 联调能力，请先更新 FPGA 固件")
        self.io_capability.setStyleSheet("color:#176b3a;" if self.io_test_available else "color:#9b1c1c;")
        detail="、".join(identity.get("cap_names") or []) or "无"
        self.log(f"设备身份：{identity.get('board','未知板')} · 构建 {identity.get('date','?')} · "
                 f"能力 {caps:#010x}（{detail}）")
        if not self.readonly:
            self.banner.setText(self.banner.text()
                                +f" · {identity.get('board','未知板')} 构建 {identity.get('date','?')}")

    def disconnect(self):
        if self.worker:
            self.worker.requestInterruption()
            if not self.worker.wait(2500):
                self.log("后台仍在结束串口操作，请稍候");return
            self.worker=None
        self.recording=False;self.record_button.setText("开始记录");self.connect_button.setText("连接")
        self.ready=False;self.io_test_available=False;self.io_status_data=None

    def toggle_record(self):
        if not self.worker:return
        if self.recording:self.enqueue("record",None);self.recording=False
        else:
            logs=ROOT/"logs";logs.mkdir(exist_ok=True)
            path=logs/(datetime.now().strftime("%Y%m%d_%H%M%S")+".jsonl")
            self.enqueue("record",str(path));self.recording=True
        self.record_button.setText("停止记录" if self.recording else "开始记录")

    def save_config(self):
        path,_=QtWidgets.QFileDialog.getSaveFileName(self,"保存配置",str(ROOT/"config/profile.json"),"JSON (*.json)")
        if path:
            Path(path).write_text(json.dumps({"schema":SCHEMA_ID,"values":self.edited_values()},
                                            ensure_ascii=False,indent=2),encoding="utf-8")
            self.log("编辑配置已保存")

    def load_config(self):
        path,_=QtWidgets.QFileDialog.getOpenFileName(self,"加载配置",str(ROOT/"config"),"JSON (*.json)")
        if not path:return
        try:
            profile=json.loads(Path(path).read_text(encoding="utf-8"))
            if profile["schema"]!=SCHEMA_ID:raise ValueError("配置schema不匹配")
            validate(profile["values"])
            for param,editor,value in zip(PARAMS,self.editors,profile["values"]):editor.setValue(value/param["scale"])
            self.log("配置仅载入编辑区，尚未下发")
        except Exception as exc:self.log("配置加载失败："+str(exc))

    def open_replay(self):
        path,_=QtWidgets.QFileDialog.getOpenFileName(self,"选择记录",str(ROOT/"logs"),"JSONL (*.jsonl)")
        if path:self.backend.setCurrentText("回放");self.start_connection("回放",path)

    def refresh(self):
        running=bool(self.worker and self.worker.isRunning())
        for b in self.action_widgets:b.setEnabled(running and self.ready and not self.readonly)
        for b in self.io_test_widgets:b.setEnabled(running and self.ready and not self.readonly and self.io_test_available)
        self.record_button.setEnabled(running and not self.readonly)
        if not running:self.connect_button.setText("连接")
        if self.last:
            s=self.last
            self.banner.setText(f"{self.worker.kind if running else '已断开，数据保留'} · {MODES.get(s['mode'],'未知')} · 配置 v{s['revision']} · {'故障/待确认' if s['fault'] else '无锁存故障'} · 设备时间 {s['tick']/1000:.3f}s")
            self.metrics.setText(f"摆角  {s['alpha']/100:+.2f}°       摆臂  {s['theta']/100:+.2f}°\n\n"
                                 f"臂速  {s['theta_dot']/10:+.1f}°/s       输出  {s['duty']/100:+.2f}%\n\n"
                                 f"参考  {s['reference']/100:.2f}°       目标  {s['target']/100:.2f}°")
            self.sensor_label.setText(f"ADC={s['adc']}  编码器={s['encoder']}  下垂零偏={s['offset']/100:.2f}°  标定完成={bool(s['calib_done'])}")
            if self.encoder_mark is not None:self.encoder_label.setText(f"从标记点计数差：{s['encoder']-self.encoder_mark}；当前默认1040计数/圈")
        if self.history and self.tabs.currentIndex()==2:
            data=list(self.history);times=[s["tick"]/1000 for s in data]
            for curve,key,scale in zip(self.curves,["alpha","theta","reference","target","duty"],[100]*5):
                curve.setData(times,[s[key]/scale for s in data])

    def closeEvent(self,event):
        self.disconnect()
        if self.worker:event.ignore()
        else:event.accept()


def main():
    parser=argparse.ArgumentParser();parser.add_argument("--smoke",type=Path)
    args=parser.parse_args()
    app=QtWidgets.QApplication(sys.argv[:1]);window=MainWindow();window.show()
    if args.smoke:
        window.start_connection("模拟")
        def exercise():
            window.command(p.CLEAR);window.command(p.START);window.tabs.setCurrentIndex(2)
        QtCore.QTimer.singleShot(1000,exercise)
        def finish():
            args.smoke.parent.mkdir(parents=True,exist_ok=True)
            window.grab().save(str(args.smoke))
            assert window.last and window.last["mode"] in (2,3),window.last
            window.disconnect();window.close();app.quit()
        QtCore.QTimer.singleShot(6000,finish)
    sys.exit(app.exec())


if __name__=="__main__":
    main()
