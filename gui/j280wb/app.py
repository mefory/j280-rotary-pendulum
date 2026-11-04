"""J280 上位机主窗口：总览 / 参数 / 模型 / 曲线，内置 Agent 接口开关。"""
from datetime import datetime
import collections
import html
import json
import math
import re
import struct
import threading
import time
from pathlib import Path

from PySide6 import QtCore, QtGui, QtWidgets
import pyqtgraph as pg

from host import protocol as p
from host.records import load_record
from host.schema import PARAMS, ROOT, SCHEMA_ID, raw_value, validate

from . import AGENT_DEFAULT_PORT
from .agent import AgentServer
from .bridge import MODES, Bridge
from . import flasher as fl
from .diag import (DiagRecorder, Mt6701LinkDiag, Mt6835LinkDiag,
                   ThetaHealthDiag, WatchMonitor, fmt_sat16, load_csv,
                   summarize_csv, RAW_TO_C01, RAW_FULL)
from .model3d import Model3DView, clamp_delta_cdeg, gl_supported
from .modelview import ModelView
from .record_audit import COMMAND_NAMES


CH340_RE = re.compile(r"ch34[0-9x]", re.IGNORECASE)   # CH340/CH341/CH34x 及克隆

# 浅色界面用的模式深色版（与 model3d.MODE_COLORS 同色相、压暗到白字可读）
MODE_CHIP_BG = {0: "#3d5a75", 1: "#b06a20", 2: "#2f8a54", 3: "#2280a8", 4: "#b04550"}

# 微调框上下大箭头：icons/ 下 12×9 三角（normal/hover 两色）。QSS 的
# border 三角技巧在 ::up-arrow 上不渲染（实测画出实心方块），必须走 image url。
SPIN_ARROW_QSS = (
    "QSpinBox::up-arrow,QDoubleSpinBox::up-arrow{width:12px;height:9px;"
    "image:url(%s);}"
    "QSpinBox::up-arrow:hover,QDoubleSpinBox::up-arrow:hover,"
    "QSpinBox::up-arrow:pressed,QDoubleSpinBox::up-arrow:pressed{image:url(%s);}"
    "QSpinBox::down-arrow,QDoubleSpinBox::down-arrow{width:12px;height:9px;"
    "image:url(%s);}"
    "QSpinBox::down-arrow:hover,QDoubleSpinBox::down-arrow:hover,"
    "QSpinBox::down-arrow:pressed,QDoubleSpinBox::down-arrow:pressed{image:url(%s);}"
) % tuple((Path(__file__).resolve().parent / "icons" / name).as_posix()
          for name in ("spin_up.png", "spin_up_hover.png",
                       "spin_down.png", "spin_down_hover.png"))


class DutyBar(QtWidgets.QWidget):
    """中心为零的双向占空比条（−10000..10000 → −100%..+100%），总览页用。

    与模型页 HUD 底部的 duty 条同语义：中线=零输出，|duty|≥90% 转红。"""

    def __init__(self):
        super().__init__()
        self.setFixedHeight(12)
        self.value = 0

    def set_value(self, value):
        self.value = max(-10000, min(10000, int(value)))
        self.update()

    def paintEvent(self, _event):
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        rect = self.rect().adjusted(0, 1, 0, -1)
        painter.setPen(QtGui.QPen(QtGui.QColor("#d9e2ec"), 1))
        painter.setBrush(QtGui.QColor("#eef3f8"))
        painter.drawRoundedRect(rect, 4, 4)
        mid = rect.center().x()
        painter.setPen(QtGui.QPen(QtGui.QColor("#c3d0dc"), 1))
        for frac in (-0.5, 0.5):        # ±50% 刻度
            tx = rect.left() + rect.width() * (0.5 + frac)
            painter.drawLine(int(tx), rect.top() + 1, int(tx), rect.bottom() - 1)
        width = (rect.width() / 2 - 1) * self.value / 10000
        painter.setPen(QtCore.Qt.PenStyle.NoPen)
        painter.setBrush(QtGui.QColor("#b42333" if abs(self.value) >= 9000
                                      else "#2f9e63"))
        painter.drawRect(QtCore.QRectF(min(mid, mid + width), rect.top() + 1,
                                       abs(width), rect.height() - 2))
        painter.setPen(QtGui.QColor("#8a9bb0"))
        painter.drawLine(mid, rect.top() + 1, mid, rect.bottom() - 1)


def port_sort_key(info):
    """串口排序键：CH340（T20K 板载串口）置顶，其余按设备名。

    本机常插多路串口（CH340、USB-TTL、板卡调试口混在一起），每次都要人工
    认 COM 号；T20K 实板后端只认 CH340 那一路，直接排最前并标 ★。
    """
    desc = (getattr(info, "description", "") or "")
    return (0 if CH340_RE.search(desc) else 1, getattr(info, "device", ""))


def list_serial_ports():
    """枚举本机串口，返回 (设备名, 显示标签) 列表；pyserial 不可用时为空。

    CH340 排最前并加 ★ 前缀（T20K 串口），其余按设备名。
    """
    try:
        from serial.tools import list_ports
    except Exception:
        return []
    result = []
    for info in sorted(list_ports.comports(), key=port_sort_key):
        desc = re.sub(r"\s*\((COM\d+|LPT\d+)\)$", "", (info.description or "").strip(),
                      flags=re.IGNORECASE)
        is_ch340 = bool(CH340_RE.search(info.description or ""))
        if desc.lower() in ("", "n/a", "none"):
            label = info.device
        else:
            label = f"{info.device} · {desc}"
        if is_ch340:
            label = "★ " + label
        result.append((info.device, label))
    return result


def ch340_device():
    """第一个 CH340 串口的设备名（T20K 板载串口）；没有则 None。"""
    try:
        from serial.tools import list_ports
    except Exception:
        return None
    for info in list_ports.comports():
        if CH340_RE.search(info.description or ""):
            return info.device
    return None


class BridgeSignals(QtCore.QObject):
    sample = QtCore.Signal(dict)
    event = QtCore.Signal(dict)


class FlasherSignals(QtCore.QObject):
    """烧录线程 → GUI 的跨线程信号（回调在烧录线程里触发）。"""
    line = QtCore.Signal(str)
    done = QtCore.Signal(dict)


class FlashScanSignals(QtCore.QObject):
    """比特流扫描线程 → GUI：gen 是发起代数（连点刷新/切板时丢弃过期结果）。"""
    done = QtCore.Signal(int, list)


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("J280 上位机 · 姿态控制调试台")
        self.resize(1280, 860)
        self.bridge = None
        self.signals = BridgeSignals()
        self.agent = None
        self.recording = False
        self.editors = []
        self.active_items = []
        self.pending_items = []
        self.action_widgets = []      # 动作类按钮：按 CAP_MOTION 启用
        self.param_widgets = []       # 参数类按钮：按 CAP_PARAMS 启用
        self.calib_widgets = []       # 标定类按钮：按 CAP_CALIB_DC 启用
        self.io_test_widgets = []     # 编码器/TB6612 联调控件：按 CAP_IO_TEST 启用
        self.tab_widgets = {}         # 页名 → 页容器，用于按当前页做惰性刷新
        self.capabilities = 0         # 设备回包的能力位；未连接/回放时为 0
        self.encoder_mark = None      # 标定页"一圈转角"辅助的比较基准
        self.io_status_data = None
        self.io_encoder_zero = None
        self.readonly = False
        self._hinted = False
        # 传感诊断（experiment/mt6701-theta 双源 θ）：常开喂入 + 页面可见时刷新
        self.diag = ThetaHealthDiag()
        self.watch = WatchMonitor(MODES)
        # 故障沿自动抓链路计数器（2026-10-19 三连锁存排障）：<2ms 的链路中断在
        # 2ms 遥测上不可见，计数器是唯一证据；手动点按钮总错过时机，改自动。
        self._fault_edge_seen = False
        self._link_prev = None      # (mt6701, mt6835) 上次计数快照，算增量
        self._az_offset = None      # α 零位上次值（自校准提交/人工标定 沿检测）
        # (θ, 零位) 全段 α 标定数据点：遥测 offset 每次变更记一条（2026-10-20
        # 竖直零位自校准上线后，BALANCE 退出提交和人工 CAL 都会走这里）。
        self.cal_points = []
        self._cal_prev_mode = None  # 变更前一样本的 mode，用于推断变更来源
        self.mt6 = Mt6701LinkDiag()
        self.mt68 = Mt6835LinkDiag()
        self._diag_tick = -1
        self.recorder = DiagRecorder("")
        self._diag_plot_t = collections.deque(maxlen=1200)
        self._diag_plot_theta = collections.deque(maxlen=1200)
        self._diag_plot_hall = collections.deque(maxlen=1200)
        self._diag_plot_delta = collections.deque(maxlen=1200)
        self.diag_capturing = False
        self._diag_start_tick = 0
        self.flasher = fl.Flasher()   # 烧录任务（后台线程，与设备桥互相独立）
        self.flash_signals = FlasherSignals()
        self.flash_signals.line.connect(self.on_flash_line)
        self.flash_signals.done.connect(self.on_flash_done)
        self.flasher.on_line(self.flash_signals.line.emit)
        self.flasher.on_done(self.flash_signals.done.emit)
        self.flash_scan_signals = FlashScanSignals()
        self.flash_scan_signals.done.connect(self._on_bitstreams_scanned)
        self._flash_scan_gen = 0      # 扫描代数：只应用最新一次的结果
        self._flash_pending_sel = (None, None)
        self.flash_cables = []        # 最近一次扫线缆结果 [{name, location}]
        self.flash_bitstreams = []    # 当前板卡的比特流候选
        self._flash_disconnected = False   # 本次烧录是否替用户断开了串口（烧完提示重连）
        self.settings = QtCore.QSettings("J280", "host-gui")

        root = QtWidgets.QWidget()
        self.setCentralWidget(root)
        layout = QtWidgets.QVBoxLayout(root)

        header = QtWidgets.QHBoxLayout()
        title = QtWidgets.QLabel("J280 上位机")
        title.setStyleSheet("font-size:20px;font-weight:600;")
        header.addWidget(title)
        header.addStretch()
        header.addWidget(QtWidgets.QLabel("后端"))
        self.backend = QtWidgets.QComboBox()
        self.backend.addItems(["模拟", "锁步HIL", "实板", "回放"])
        self.backend.setItemData(1, "DE10 锁步：FPGA 跑真实控制链，PC 路被控对象模型（SW[9]=ON）",
                                 QtCore.Qt.ItemDataRole.ToolTipRole)
        self.backend.activated.connect(self.on_backend_changed)
        self.port = QtWidgets.QComboBox()
        self.port.setEditable(True)
        self.port.setMinimumWidth(240)
        self.port.lineEdit().setPlaceholderText("选择串口，或输入回放记录路径")
        self.refresh_ports_button = QtWidgets.QPushButton("刷新")
        self.refresh_ports_button.clicked.connect(self.refresh_ports)
        self.connect_button = QtWidgets.QPushButton("连接")
        self.connect_button.clicked.connect(self.toggle_connection)
        header.addWidget(self.backend)
        header.addWidget(self.port)
        header.addWidget(self.refresh_ports_button)
        header.addWidget(self.connect_button)
        layout.addLayout(header)

        self.banner = QtWidgets.QLabel("未连接 · 默认模拟后端，不访问硬件")
        self.banner.setStyleSheet("background:#16334a;color:#edf6fc;padding:9px;border-radius:5px;")
        layout.addWidget(self.banner)

        controls = QtWidgets.QHBoxLayout()
        for label, cmd in [("清除故障", p.CLEAR), ("启动起摆", p.START)]:
            controls.addWidget(self.action(label, lambda _=False, c=cmd: self.command(c)))
        self.balance_button = self.action("手扶稳摆 / 返回稳摆", self.request_balance)
        self.balance_button.setToolTip(
            "待机时：手扶至 |α|<5°、|α̇|<30°/s、|θ̇|<20°/s 后直接进入稳摆；"
            "定位时：返回稳摆")
        controls.addWidget(self.balance_button)
        self.stop_button = self.action("停止 (F9)", lambda: self.command(p.STOP))
        self.stop_button.setStyleSheet(
            "QPushButton{background:#b42333;color:white;font-weight:bold;"
            "padding:7px 16px;border:1px solid #9a1c2b;border-radius:5px;}"
            "QPushButton:hover{background:#c62c3e;}"
            "QPushButton:pressed{background:#9a1c2b;}"
            "QPushButton:disabled{background:#d9a3ab;color:#f5e4e7;"
            "border-color:#d9a3ab;}")
        stop_key = QtGui.QShortcut(QtGui.QKeySequence("F9"), self)
        stop_key.setContext(QtCore.Qt.ShortcutContext.ApplicationShortcut)
        stop_key.activated.connect(lambda: self.command(p.STOP))
        controls.addWidget(self.stop_button)
        controls.addStretch()
        self.record_button = QtWidgets.QPushButton("开始记录")
        self.record_button.clicked.connect(self.toggle_record)
        controls.addWidget(self.record_button)
        self.agent_button = QtWidgets.QPushButton(f"启动Agent接口(:{AGENT_DEFAULT_PORT})")
        self.agent_button.clicked.connect(self.toggle_agent)
        controls.addWidget(self.agent_button)
        layout.addLayout(controls)

        self.tabs = QtWidgets.QTabWidget()
        layout.addWidget(self.tabs, stretch=1)
        self.build_overview()
        self.build_params()
        self.build_model()
        self.build_plots()
        self.build_calibration()
        self.build_hardware()
        self.build_diag()
        self.build_trial()
        self.build_record()
        self.build_flash()

        # 信号只连一次：重复连接会让同一事件触发多次（日志/模型重复刷新）
        self.signals.sample.connect(self.model.set_sample)
        self.signals.event.connect(self.on_event)

        self.events = QtWidgets.QPlainTextEdit()
        self.events.setReadOnly(True)
        self.events.setMaximumHeight(110)
        self.events.document().setMaximumBlockCount(400)
        self.events.setStyleSheet(
            "QPlainTextEdit{background:#ffffff;border:1px solid #d3dde6;border-radius:5px;"
            "color:#33455a;font-family:'Consolas','Microsoft YaHei',monospace;"
            "font-size:12px;}")
        layout.addWidget(self.events)

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(40)
        self.ports_timer = QtCore.QTimer(self)
        self.ports_timer.timeout.connect(self.auto_refresh_ports)
        self.ports_timer.start(2000)
        self.refresh_ports()
        saved_backend = self.settings.value("connection/backend", "", str)
        if saved_backend in ("模拟", "锁步HIL", "实板", "回放"):
            self.backend.setCurrentText(saved_backend)
        saved_port = self.settings.value("connection/port", "", str)
        if saved_port:
            self.port.setEditText(saved_port)
        else:
            dev = ch340_device()
            if dev:
                self.port.setEditText(dev)   # 没有历史选择时默认 CH340（T20K 串口）
        # 统一视觉基调：浅灰铬底 + 青蓝强调色；模型页/曲线页保持深色画布，
        # 形成「浅铬深画布」的专业工具观感。只覆盖视觉，不改任何交互逻辑。
        self.setStyleSheet(
            "QWidget{font-family:'Microsoft YaHei';font-size:13px;color:#1f2d3d;}"
            "QPushButton{padding:6px 14px;border:1px solid #c7d2dd;border-radius:5px;"
            "background:qlineargradient(x1:0,y1:0,x2:0,y2:1,stop:0 #ffffff,stop:1 #f2f6fa);"
            "color:#24354a;}"
            "QPushButton:hover{border-color:#3d97b8;color:#0e7ea6;background:#eef7fb;}"
            "QPushButton:pressed{background:#e0eef5;}"
            "QPushButton:disabled{color:#a4b1bf;border-color:#dde4eb;background:#f4f6f8;}"
            "QTabWidget::pane{border:1px solid #d3dde6;border-radius:4px;top:-1px;}"
            "QTabBar::tab{padding:7px 16px;margin-right:2px;color:#5a6b7e;"
            "border:1px solid transparent;border-bottom:none;}"
            "QTabBar::tab:hover{color:#0e7ea6;}"
            "QTabBar::tab:selected{color:#0e7ea6;font-weight:600;background:#ffffff;"
            "border:1px solid #d3dde6;border-bottom:2px solid #0e7ea6;"
            "border-radius:4px 4px 0 0;}"
            "QHeaderView::section{padding:6px 8px;background:#eef2f7;color:#33455a;"
            "border:none;border-bottom:1px solid #d3dde6;font-weight:600;}"
            "QTableWidget{background:#ffffff;alternate-background-color:#f6f9fc;"
            "selection-background-color:#d6ecf5;selection-color:#12324a;"
            "gridline-color:#e4ebf2;}"
            "QComboBox,QSpinBox,QDoubleSpinBox,QLineEdit{padding:4px 8px;"
            "border:1px solid #c7d2dd;border-radius:4px;background:white;"
            "selection-background-color:#bcdff0;}"
            "QComboBox:hover,QSpinBox:hover,QDoubleSpinBox:hover,QLineEdit:hover"
            "{border-color:#8fb8cc;}"
            "QComboBox:focus,QSpinBox:focus,QDoubleSpinBox:focus,QLineEdit:focus"
            "{border-color:#3d97b8;}"
            # 微调框的上下按钮：默认箭头只有几像素看不清；箭头用 icons/ 下的
            # 三角 PNG（SPIN_ARROW_QSS），按钮加宽到 22px、悬停/按下转强调色
            "QSpinBox::up-button,QDoubleSpinBox::up-button{subcontrol-origin:border;"
            "subcontrol-position:top right;width:22px;border:none;"
            "border-left:1px solid #d3dde6;border-bottom:1px solid #d3dde6;"
            "border-top-right-radius:4px;background:#f2f6fa;}"
            "QSpinBox::down-button,QDoubleSpinBox::down-button{subcontrol-origin:border;"
            "subcontrol-position:bottom right;width:22px;border:none;"
            "border-left:1px solid #d3dde6;border-bottom-right-radius:4px;"
            "background:#f2f6fa;}"
            "QSpinBox::up-button:hover,QDoubleSpinBox::up-button:hover,"
            "QSpinBox::down-button:hover,QDoubleSpinBox::down-button:hover"
            "{background:#d9ecf5;}"
            "QSpinBox::up-button:pressed,QDoubleSpinBox::up-button:pressed,"
            "QSpinBox::down-button:pressed,QDoubleSpinBox::down-button:pressed"
            "{background:#c6e2ef;}"
            "QGroupBox{font-weight:600;border:1px solid #d3dde6;border-radius:6px;"
            "margin-top:12px;padding:8px 4px 4px 4px;}"
            "QGroupBox::title{subcontrol-origin:margin;left:10px;padding:0 4px;"
            "color:#33455a;}"
            "QScrollBar:vertical{background:transparent;width:10px;margin:2px;}"
            "QScrollBar::handle:vertical{background:#c3d0dc;border-radius:4px;min-height:36px;}"
            "QScrollBar::handle:vertical:hover{background:#9fb4c6;}"
            "QScrollBar::add-line:vertical,QScrollBar::sub-line:vertical{height:0;}"
            "QScrollBar:horizontal{background:transparent;height:10px;margin:2px;}"
            "QScrollBar::handle:horizontal{background:#c3d0dc;border-radius:4px;min-width:36px;}"
            "QScrollBar::handle:horizontal:hover{background:#9fb4c6;}"
            "QScrollBar::add-line:horizontal,QScrollBar::sub-line:horizontal{width:0;}"
            "QToolTip{background:#22354a;color:#edf6fc;border:none;"
            "padding:5px 7px;border-radius:4px;}"
            + SPIN_ARROW_QSS)

    # ---------- 通用 ----------
    def action(self, text, callback):
        button = QtWidgets.QPushButton(text)
        button.clicked.connect(callback)
        self.action_widgets.append(button)
        return button

    def param_action(self, text, callback):
        """参数类按钮：与动作类分开登记，按能力位分别启用。"""
        button = self.action(text, callback)
        self.param_widgets.append(button)
        return button

    def calib_action(self, text, callback):
        """标定类按钮：按 CAP_CALIB_DC 启用。"""
        button = self.action(text, callback)
        self.calib_widgets.append(button)
        return button

    def page(self, title, scroll=False):
        """建一个标签页。内容超高时 QTabWidget 会硬压缩布局，QFormLayout 行距
        被压成负值出现文字叠行（传感诊断页实测）——内容多的页传 scroll=True
        包一层 QScrollArea，让内容按自然高度排版、窗口出滚动条。"""
        outer = QtWidgets.QWidget()
        self.tabs.addTab(outer, title)
        self.tab_widgets[title] = outer
        if scroll:
            area = QtWidgets.QScrollArea()
            area.setWidgetResizable(True)
            area.setFrameShape(QtWidgets.QFrame.Shape.NoFrame)
            inner = QtWidgets.QWidget()
            area.setWidget(inner)
            vbox = QtWidgets.QVBoxLayout(outer)
            vbox.setContentsMargins(0, 0, 0, 0)
            vbox.addWidget(area)
            return QtWidgets.QVBoxLayout(inner)
        return QtWidgets.QVBoxLayout(outer)

    def on_page(self, title):
        """当前是否停在某一页；用于只在该页可见时做刷新（绘图/传感器读数）。"""
        return self.tabs.currentWidget() is self.tab_widgets.get(title)

    def log(self, text):
        self.events.appendPlainText(datetime.now().strftime("%H:%M:%S") + "  " + str(text))
        # 同时进 JSONL：界面上看到的每一行（定位下发、参数提交、标定、
        # 掉摆提示…）都能和遥测/命令对同一条时间轴，事后不用回忆。
        if self.bridge:
            try:
                self.bridge.note(text)
            except Exception:
                pass

    # ---------- 总览 ----------
    def build_overview(self):
        """卡片式实时仪表盘：模式/摆杆 α/摆臂 θ/目标方位/输出 duty 五张语义卡，
        一屏读懂当前姿态；数值颜色有语义（α 绿=近竖直、灰=下垂、橙=摆动中，
        duty 绿=正常、红=接近满舵）。"""
        layout = self.page("总览")
        self._value_colors = {}       # value label id → 上次颜色（避免每帧重设样式）

        def card(caption, value_label, sub_widget):
            frame = QtWidgets.QFrame()
            frame.setObjectName("card")
            frame.setStyleSheet(
                "QFrame#card{background:white;border:1px solid #d9e2ec;"
                "border-radius:8px;}")
            frame.setMinimumWidth(150)
            box = QtWidgets.QVBoxLayout(frame)
            box.setContentsMargins(14, 10, 14, 10)
            box.setSpacing(3)
            cap = QtWidgets.QLabel(caption)
            cap.setStyleSheet("color:#7a8ba0;font-size:12px;border:none;")
            value_label.setStyleSheet("font-size:24px;font-weight:700;color:#b7c4d2;")
            sub_widget.setStyleSheet(
                "color:#8a9bb0;font-size:12px;border:none;"
                if isinstance(sub_widget, QtWidgets.QLabel) else "border:none;")
            box.addWidget(cap)
            box.addWidget(value_label)
            box.addWidget(sub_widget)
            return frame

        self.mode_card_val = QtWidgets.QLabel("未连接")
        self.mode_card_sub = QtWidgets.QLabel("连接设备后显示实时状态")
        self.alpha_card_val = QtWidgets.QLabel("—")
        self.alpha_card_sub = QtWidgets.QLabel("α̇ —")
        self.theta_card_val = QtWidgets.QLabel("—")
        self.theta_card_sub = QtWidgets.QLabel("θ̇ —")
        self.target_card_val = QtWidgets.QLabel("—")
        self.target_card_sub = QtWidgets.QLabel("POS 定位目标方位")
        self.duty_card_val = QtWidgets.QLabel("—")
        self.duty_bar = DutyBar()
        strip = QtWidgets.QHBoxLayout()
        strip.setSpacing(10)
        for caption, val, sub in (
                ("模式", self.mode_card_val, self.mode_card_sub),
                ("摆杆 α（竖直偏差）", self.alpha_card_val, self.alpha_card_sub),
                ("摆臂 θ（0-360°）", self.theta_card_val, self.theta_card_sub),
                ("目标方位", self.target_card_val, self.target_card_sub),
                ("输出 duty", self.duty_card_val, self.duty_bar)):
            strip.addWidget(card(caption, val, sub), stretch=1)
        layout.addLayout(strip)
        hint = QtWidgets.QLabel(
            "自动流程：连接 → 清除故障 → 启动起摆 → 稳定后定位。\n"
            "手扶调参：待机 → 扶到直立并静止 →「手扶稳摆 / 返回稳摆」，直接进入 LQR。\n"
            "参数在停机时成组提交；连接时回读设备值，不自动下发界面旧值。\n"
            "「Agent接口」开启后，脚本可用 python -m j280wb.cli 驱动同一连接；\n"
            "人工操作自动抢占 Agent 控制租约，停止/清故障永远优先。")
        hint.setWordWrap(True)
        hint.setStyleSheet("color:#6b7f94;font-size:12px;")
        layout.addWidget(hint)
        layout.addStretch()

    def _set_card_value(self, label, text, color="#1f2d3d"):
        """数值着色只在颜色变化时重设样式：refresh 以 25Hz 运行，
        每帧 setStyleSheet 会强制整页重排版，缓存掉后只剩 setText 的开销。"""
        if self._value_colors.get(id(label)) != color:
            self._value_colors[id(label)] = color
            label.setStyleSheet(f"font-size:24px;font-weight:700;color:{color};")
        label.setText(text)

    def _update_overview(self, latest):
        mode = latest["mode"]
        fault = bool(latest["fault"])
        chip = "#b42333" if fault else MODE_CHIP_BG.get(mode, "#3d5a75")
        self.mode_card_val.setText(("故障 · " if fault else "") + MODES.get(mode, "?"))
        mode_style = (f"font-size:15px;font-weight:700;color:white;background:{chip};"
                      "border:none;border-radius:5px;padding:5px 12px;")
        if self._value_colors.get(id(self.mode_card_val)) != mode_style:
            self._value_colors[id(self.mode_card_val)] = mode_style
            self.mode_card_val.setStyleSheet(mode_style)
        self.mode_card_sub.setText(
            f"配置 v{latest['revision']} · t={latest['tick']/1000:.2f}s"
            + (" · 需先清除故障" if fault else ""))
        a = latest["alpha"] / 100
        color = "#2f9e63" if abs(a) <= 5 else ("#8a9bb0" if abs(a) >= 175 else "#c26a1f")
        self._set_card_value(self.alpha_card_val, f"{a:+.2f}°", color)
        self.alpha_card_sub.setText(f"α̇ {latest['alpha_dot']/10:+.1f}°/s")
        th_m = latest['theta'] / 100.0
        self._set_card_value(self.theta_card_val, f"{th_m % 360.0:6.2f}°")
        self.theta_card_sub.setText(
            f"θ̇ {latest['theta_dot']/10:+.1f}°/s · 多圈 {th_m:+.1f}°")
        tg_m = latest['target'] / 100.0
        self._set_card_value(self.target_card_val, f"{tg_m % 360.0:6.2f}°", "#c26a1f")
        self.target_card_sub.setText(f"多圈 {tg_m:+.1f}° · POS 定位目标")
        d = latest["duty"]
        self._set_card_value(self.duty_card_val, f"{d/100:+.2f}%",
                             "#b42333" if abs(d) >= 9000 else "#2f9e63")
        self.duty_bar.set_value(d)

    # ---------- 参数 ----------
    def build_params(self):
        layout = self.page("参数")
        layout.addWidget(QtWidgets.QLabel(
            "编辑 → 停机 → 成组提交 → 设备确认 → 回读核对；数值合法不代表闭环稳定。"))
        table = QtWidgets.QTableWidget(len(PARAMS), 6)
        table.setHorizontalHeaderLabels(["分组 / 参数", "单位", "允许范围", "编辑值", "待提交定点值", "设备生效值"])
        table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.ResizeMode.Stretch)
        table.setAlternatingRowColors(True)
        table.verticalHeader().setVisible(False)
        for row, param in enumerate(PARAMS):
            for col, text in enumerate([
                    param["group"] + " / " + param["label"], param["unit"],
                    f'{param["min"]/param["scale"]:g} ～ {param["max"]/param["scale"]:g}']):
                item = QtWidgets.QTableWidgetItem(text)
                item.setFlags(item.flags() & ~QtCore.Qt.ItemFlag.ItemIsEditable)
                table.setItem(row, col, item)
            editor = QtWidgets.QDoubleSpinBox()
            editor.setDecimals(6)
            editor.setRange(param["min"] / param["scale"], param["max"] / param["scale"])
            editor.setValue(param["default"] / param["scale"])
            editor.setSingleStep(1 / param["scale"])
            table.setCellWidget(row, 3, editor)
            self.editors.append(editor)
            pending = QtWidgets.QTableWidgetItem(str(param["default"]))
            active = QtWidgets.QTableWidgetItem("未回读")
            for item in (pending, active):
                item.setFlags(item.flags() & ~QtCore.Qt.ItemFlag.ItemIsEditable)
            table.setItem(row, 4, pending)
            table.setItem(row, 5, active)
            self.pending_items.append(pending)
            self.active_items.append(active)
            editor.valueChanged.connect(
                lambda value, r=row: self.pending_items[r].setText(str(raw_value(PARAMS[r], value))))
        layout.addWidget(table)
        row = QtWidgets.QHBoxLayout()
        row.addWidget(self.param_action("回读设备", lambda: self.async_submit("read")))
        row.addWidget(self.param_action("停机成组提交", lambda: self.async_submit("apply", self.edited_values())))
        row.addWidget(self.param_action("恢复设备默认", lambda: self.async_submit("defaults")))
        row.addWidget(self.param_action("载入推荐参数", self.load_recommended))
        row.addWidget(self.param_action("电量探针·适配K4", self.battery_k4))
        row.addStretch()
        layout.addLayout(row)

    # 实测甜点组合（2026-11-04 晨 R6 固件定案）：稳摆 K1/K2=17751/10000、
    # 臂 600/5300；POS 对 1200/9500（真满电实测稳妥值——满电捕获悬崖：捕获
    # 增强目标=pos_k4，11000 时 BAL 在增强窗结束即死，9500 稳；电量下降后
    # 可用「电量探针·适配K4」或手动回调）；trim=245 是**标定相关**值
    # （11-4 晨标定链实测，每次重标定后需重扫）；甩幅 7000（满电下 7800
    # 偏猛）；死区 640/740。尾部 14/15 仅新固件有。
    RECOMMENDED = [17751, 10000, 600, 5300, 640, 740, 7000, 1500, 9000, 3000,
                   11800, 1200, 9500, 245, 1200, 9]

    def load_recommended(self):
        """一键把推荐值填进编辑框（不直接下发——仍需停机后点提交）。"""
        for param, editor, raw in zip(PARAMS, self.editors, self.RECOMMENDED):
            editor.setValue(raw / param["scale"])
        self.log("已载入推荐参数（K1/K2=17751/10000，臂 600/5300，POS 1200/9500，"
                 "trim=245，甩幅 7000）——停机后点「停机成组提交」，再做下垂标定+trim 重扫")

    def battery_k4(self):
        """电量探针：+800 duty 脉冲测臂速 → 电量% → 按甜点-电压关系写 POS K4。
        全流程在桥线程状态机里跑；需 SAFE/IDLE 态且摆杆已下垂。"""
        bridge = self.bridge
        if not bridge:
            self.log("电量探针：未连接设备")
            return
        try:
            bridge.battery_probe("gui", apply_k4=True)
            self.log("电量探针已启动（0.9s 脉冲，臂会转动——确认前方空间）")
        except Exception as exc:
            self.log("电量探针失败：" + str(exc))

    def edited_values(self):
        return [raw_value(param, editor.value()) for param, editor in zip(PARAMS, self.editors)]

    # ---------- 模型 ----------
    def build_model(self):
        layout = self.page("模型")
        if gl_supported():
            self.model = Model3DView()
            self.model.request_move.connect(self.request_move)
            self.model_3d = True
        else:      # GL 3.3 不可用（远程桌面/旧驱动）时回退 2D 伪 3D
            self.model = ModelView()
            self.model_3d = False
        if self.model_3d:
            bar = QtWidgets.QHBoxLayout()
            drag_send = QtWidgets.QCheckBox("拖动定位下发")
            drag_send.setChecked(
                self.settings.value("model/drag_send", True, type=bool))
            drag_send.toggled.connect(self.on_drag_send_toggled)
            self.drag_check = drag_send
            reset_btn = QtWidgets.QPushButton("视角复位")
            reset_btn.setToolTip("把相机恢复到默认机位（等同 3D 视图内双击）")
            reset_btn.clicked.connect(self.model.reset_camera)
            hint = QtWidgets.QLabel(
                "拖摆臂端/杆端松手即下发定位 · 空白处拖动=转视角 · 滚轮=缩放 · 双击=复位")
            hint.setStyleSheet("color:#7a8ba0;font-size:12px;")
            bar.addWidget(drag_send)
            bar.addWidget(reset_btn)
            for label, (yaw, pitch, dist) in (
                    ("正视", (0.0, 16.0, 3.6)),
                    ("侧视", (90.0, 16.0, 3.6)),
                    ("俯视", (35.0, 74.0, 4.8))):
                preset = QtWidgets.QPushButton(label)
                preset.setToolTip(f"切换到{label}机位（锁定臂端平面更直观）")
                preset.clicked.connect(
                    lambda _=False, y=yaw, pt=pitch, dd=dist:
                    self.model.set_camera(y, pt, dd))
                bar.addWidget(preset)
            bar.addWidget(hint)
            bar.addStretch()
            layout.addLayout(bar)
            self.model.set_drag_enabled(drag_send.isChecked())
        else:
            self.drag_check = None
        layout.addWidget(self.model, stretch=1)

    def on_drag_send_toggled(self, on):
        """关闭后 3D 视图仅观察，拖动不再下发定位（实机联调安全档）。"""
        self.model.set_drag_enabled(bool(on))
        self.settings.setValue("model/drag_send", bool(on))
        self.log("拖动定位下发：" + ("开启" if on else "关闭（仅预览）"))

    def request_move(self, delta_rad):
        """3D 视图拖动松手 → 下发相对位移。稳摆/定位模式下都可直接用。

        model3d 现在发的是**相对位移**（拖动预览角 − 按下基准角），不是绝对
        目标角：设备 θ 是多圈累积值，屏幕只能给出 atan2 方位角，两者不同域，
        直接当目标会用错绕圈数。相对位移则与手在屏幕上的动作严格对应。
        POS 模式下固件会直接改写 pos_tgt（不重启轨迹），因此可连续拖动观察动态响应。
        """
        bridge = self.bridge
        if not bridge or not bridge.connected:
            self.log("定位失败：未连接设备")
            return
        last = bridge.latest
        if not last:
            self.log("定位失败：暂无遥测")
            return
        if last["fault"]:
            self.log("定位失败：设备存在故障，请先清除")
            return
        mode = last["mode"]
        if mode not in (2, 3):
            self.log(f"定位失败：当前为{MODES.get(mode, mode)}，仅稳摆/定位时可拖动定位")
            return
        delta = clamp_delta_cdeg(math.degrees(delta_rad))
        if delta == 0:
            self.log("定位：拖动幅度过小，未下发")
            return
        try:
            bridge.submit("command", (p.MOVE, struct.pack("<i", delta)), source="gui")
        except Exception as exc:
            self.log("定位失败：" + str(exc))
            return
        final = (last["theta"] + delta) / 100
        self.log(f"定位：Δ{delta/100:+.2f}°，目标臂角 {final:+.2f}°")
        if self.model_3d:
            self.model.set_note(f"定位指令已下发：Δ{delta/100:+.1f}° → {final:+.1f}°")

    # ---------- 曲线 ----------
    def build_plots(self):
        layout = self.page("曲线")
        pg.setConfigOptions(antialias=False)
        bar = QtWidgets.QHBoxLayout()
        hint = QtWidgets.QLabel(
            "实时：最近 12 秒遥测（500 Hz × 6000 帧环形缓冲，x 轴=设备开机秒数）；"
            "「载入记录」整条复盘：全程曲线 + 模式/故障/命令标记 + 复盘摘要")
        hint.setStyleSheet("color:#7f92a8;")
        bar.addWidget(hint)
        bar.addStretch(1)
        self.load_record_button = QtWidgets.QPushButton("载入记录")
        self.load_record_button.clicked.connect(self.load_record_dialog)
        bar.addWidget(self.load_record_button)
        self.view_toggle_button = QtWidgets.QPushButton("返回实时")
        self.view_toggle_button.clicked.connect(self._toggle_curve_view)
        self.view_toggle_button.hide()          # 载入记录后才出现，实时/复盘二态切换
        bar.addWidget(self.view_toggle_button)
        self.auto_follow = QtWidgets.QCheckBox("自动跟随")
        self.auto_follow.setChecked(True)
        self.auto_follow.setToolTip(
            "勾选：视窗自动锁定最新 12 秒并自适应纵轴；拖动/缩放任一图会自动取消勾选")
        bar.addWidget(self.auto_follow)
        reset = QtWidgets.QPushButton("复位缩放")
        reset.clicked.connect(lambda: self.auto_follow.setChecked(True))
        bar.addWidget(reset)
        layout.addLayout(bar)
        graph = pg.GraphicsLayoutWidget()
        graph.setBackground("#0f1d2d")
        self._live_graph = graph
        self._rec_mode = False                  # 复盘视图激活中（抑制实时逐拍重绘）
        self._record = None
        self._rec_overlays = []                 # (plot, item)：载入/切换时统一清理
        plots = []
        for row, label in enumerate(["摆杆 α / °", "摆臂 θ / °", "最终输出 / %"]):
            plot = graph.addPlot(row=row, col=0)
            plot.setLabel("left", label)
            plot.showGrid(x=True, y=True, alpha=0.2)
            plots.append(plot)
        plots[1].setXLink(plots[0])
        plots[2].setXLink(plots[0])
        plots[2].setLabel("bottom", "设备时间 / s")
        plots[1].addLegend()
        self._plots = plots
        for pl in plots:      # 用户手动拖动/缩放 → 自动退出跟随，不打扰观察
            pl.vb.sigRangeChangedManually.connect(self._on_manual_zoom)
        self.curves = [plots[0].plot(pen="#6edce8"),
                       plots[1].plot(pen="#6edce8", name="臂角 0~360°"),
                       plots[1].plot(pen="#ffbd69", name="参考"),
                       plots[1].plot(pen="#e789da", name="目标"),
                       plots[2].plot(pen="#91d89b")]
        self._build_record_view()
        self.curve_stack = QtWidgets.QStackedWidget()
        self.curve_stack.addWidget(graph)
        self.curve_stack.addWidget(self.rec_container)
        layout.addWidget(self.curve_stack, 1)

    def _on_manual_zoom(self, *_):
        if getattr(self, "auto_follow", None) and self.auto_follow.isChecked():
            self.auto_follow.setChecked(False)

    @staticmethod
    def _break_wrap(ts, ys, thresh=180.0):
        """α 在 ±180 回绕处断线（插 NaN）：静息抖动跨绕时不画出贯穿整图的竖线。"""
        if len(ys) < 2:
            return ts, ys
        out_t, out_y = [ts[0]], [ys[0]]
        prev = ys[0]
        for t, y in zip(ts[1:], ys[1:]):
            if abs(y - prev) > thresh:
                out_t.append(t)
                out_y.append(float("nan"))
            out_t.append(t)
            out_y.append(y)
            prev = y
        return out_t, out_y

    @staticmethod
    def _wrap360(ys):
        """臂角 θ/参考/目标解回绕到 [0,360)：定位展示按 0-360 绝对方位读
        （θ 遥测是多圈连续量，Python 负数取模恒为正，跨多圈也正确）。"""
        return [y % 360.0 for y in ys]

    @classmethod
    def _wrap_break(cls, ts, ys, thresh=180.0):
        """先解回绕 0-360、再在 0/360 接缝断线（NaN）——臂角曲线的展示口径。"""
        return cls._break_wrap(ts, cls._wrap360(ys), thresh)

    @staticmethod
    def _fit_y(plot, ys, min_span):
        """纵轴自适应；数据近恒定时保底 min_span，避免静息噪声铺满整图。"""
        lo, hi = min(ys), max(ys)
        if hi - lo < min_span:
            mid = (lo + hi) / 2
            lo, hi = mid - min_span / 2, mid + min_span / 2
        else:
            pad = (hi - lo) * 0.06
            lo -= pad
            hi += pad
        plot.setYRange(lo, hi, padding=0)

    def update_curves(self):
        """曲线页取数：从桥的遥测环绘制。只在该页可见时调用，避免无谓重绘。

        2026-09-23 修复：此前 self.curves 建好后从未 setData，曲线页一直是三块空图。
        2026-09-24 修复：自动跟随（x 锁最新 12 秒 + 纵轴自适应带保底量程）——
        摆杆静息下垂时 ADC 噪声 ±0.02° 会被 pyqtgraph 自适应放大成"实心墙"，
        甩摆几千度的纵轴残留也会把静息段挤成看不懂的水平线。
        """
        if self._rec_mode:
            return          # 复盘视图是静态整条数据，不需要逐拍重绘
        bridge = self.bridge
        if not bridge or not bridge.history:
            return
        data = list(bridge.history)
        stride = max(1, len(data) // 2000)          # 最多画约 2000 点，500 Hz 下仍跟手
        data = data[::stride]
        times = [s["tick"] / 1000 for s in data]
        vals = {k: [s[k] / 100 for s in data]
                for k in ("alpha", "theta", "reference", "target", "duty")}
        at, ay = self._break_wrap(times, vals["alpha"])
        self.curves[0].setData(at, ay)
        # 摆臂 θ/参考/目标：0-360 绝对方位（解回绕），0/360 接缝断线
        # （2026-11-04 定为录屏展示口径：多圈连续量读不出方位，0-360 一眼对应目标）
        for curve, key in zip(self.curves[1:], ("theta", "reference", "target")):
            wt, wy = self._wrap_break(times, vals[key])
            curve.setData(wt, wy)
        self.curves[4].setData(times, vals["duty"])
        if not self.auto_follow.isChecked():
            return
        p0, p1, p2 = self._plots
        p0.setXRange(times[0], times[-1] + 0.6, padding=0)
        self._fit_y(p0, vals["alpha"], 0.5)
        # 纵轴按回绕后的绝对角自适应（用不带 NaN 的回绕值，min/max 不被毒化）
        self._fit_y(p1, self._wrap360(vals["theta"]) + self._wrap360(vals["reference"])
                    + self._wrap360(vals["target"]), 30.0)
        self._fit_y(p2, vals["duty"], 10.0)

    # ---------- 曲线：记录复盘（整条载入，新旧格式通吃） ----------
    # 背景着色语义：模式段用 MODE_CHIP_BG（与总览页模式章同色相），故障段叠红，
    # 命令竖线（动作类带名、配置类细灰线），人工备注虚线。全部按拍注入三行子图。
    REC_MODE_ALPHA = "#40"        # 模式段背景透明度（#AARRGGBB 前缀）
    REC_CMD_LIMIT = 400           # 竖线超过此数只画动作类，防刷屏
    REC_SEG_LIMIT = 400           # 模式段上限（异常抖动记录不该画爆内存）

    def _build_record_view(self):
        container = QtWidgets.QWidget()
        box = QtWidgets.QHBoxLayout(container)
        box.setContentsMargins(0, 0, 0, 0)
        left = QtWidgets.QVBoxLayout()
        left.setContentsMargins(0, 0, 0, 0)
        self.rec_info = QtWidgets.QLabel("未载入记录")
        self.rec_info.setStyleSheet("color:#7f92a8;")
        left.addWidget(self.rec_info)
        graph = pg.GraphicsLayoutWidget()
        graph.setBackground("#0f1d2d")
        left.addWidget(graph, 1)
        box.addLayout(left, 1)
        self.rec_summary = QtWidgets.QTextEdit()
        self.rec_summary.setReadOnly(True)
        self.rec_summary.setMinimumWidth(360)
        self.rec_summary.setMaximumWidth(430)
        self.rec_summary.setStyleSheet(
            "QTextEdit{background:#0f1d2d;color:#c9d6e4;border:1px solid #1d3448;"
            "font-size:12px;}")
        box.addWidget(self.rec_summary)
        self.rec_container = container
        plots = []
        for row, label in enumerate(["摆杆 α / °", "摆臂 θ / °", "最终输出 / %"]):
            plot = graph.addPlot(row=row, col=0)
            plot.setLabel("left", label)
            plot.showGrid(x=True, y=True, alpha=0.2)
            plots.append(plot)
        plots[1].setXLink(plots[0])
        plots[2].setXLink(plots[0])
        plots[2].setLabel("bottom", "设备时间 / s")
        plots[1].addLegend()
        self._rec_plots = plots
        self.rec_curves = [plots[0].plot(pen="#6edce8"),
                           plots[1].plot(pen="#6edce8"),
                           plots[1].plot(pen="#ffbd69"),
                           plots[1].plot(pen="#e789da"),
                           plots[2].plot(pen="#91d89b")]
        for curve in self.rec_curves:
            # 全程数据量大：视窗外裁剪 + 按视窗峰值降采样，放大时细节自动回来
            curve.setDownsampling(auto=True, method="peak")
            curve.setClipToView(True)

    def load_record_dialog(self):
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "载入记录", str(ROOT / "logs"), "记录 (*.csv *.jsonl)")
        if path:
            self.show_record(path)

    def show_record(self, path):
        """读一条记录进复盘视图：全程曲线 + 事件标记 + 摘要侧栏。"""
        try:
            meta, samples, events = load_record(path)
        except Exception as exc:
            self.log(f"载入记录失败：{exc}")
            return
        if not samples:
            self.log(f"载入记录：{Path(path).name} 里没有遥测样本")
            return
        ticks = [s["tick"] / 1000 for s in samples]

        def series(key):
            return [(s[key] / 100 if s.get(key) is not None else float("nan"))
                    for s in samples]

        at, ay = self._break_wrap(ticks, series("alpha"))
        self.rec_curves[0].setData(at, ay)
        # 复盘与实时同口径：臂角/参考/目标按 0-360 绝对方位、接缝断线
        for curve, key in zip(self.rec_curves[1:],
                              ("theta", "reference", "target")):
            wt, wy = self._wrap_break(ticks, series(key))
            curve.setData(wt, wy)
        self.rec_curves[4].setData(ticks, series("duty"))
        self._draw_record_overlays(samples, events)
        self.rec_info.setText(
            f"{Path(path).name} · {meta.get('record_format') or '旧版单文件 JSONL'}"
            f" · {len(samples)} 帧 / {ticks[-1] - ticks[0]:.1f}s")
        self.rec_summary.setHtml(
            self._record_summary_html(path, meta, samples, events))
        self._record = str(path)
        if not self._rec_mode:
            self._toggle_curve_view()
        self.log(f"已载入记录：{Path(path).name}（{len(samples)} 帧，"
                 f"模式段/命令/备注见背景标记）")

    def _toggle_curve_view(self):
        self._rec_mode = not self._rec_mode
        if self._rec_mode:
            self.curve_stack.setCurrentWidget(self.rec_container)
            self.view_toggle_button.setText("返回实时")
            self.view_toggle_button.setVisible(True)
        else:
            self.curve_stack.setCurrentWidget(self._live_graph)
            self.view_toggle_button.setText("查看记录")

    def _add_overlay(self, factory):
        """每行子图各建一份叠加项（QGraphicsItem 不能共享父级）。"""
        for plot in self._rec_plots:
            item = factory()
            plot.addItem(item)
            self._rec_overlays.append((plot, item))

    def _draw_record_overlays(self, samples, events):
        self._clear_record_overlays()
        # 模式分段：连续同 mode 的拍合成一段背景
        segs = []
        prev, start = None, 0
        for i, s in enumerate(samples):
            m = s.get("mode")
            if m != prev:
                if prev is not None:
                    segs.append((prev, samples[start]["tick"] / 1000,
                                 samples[i - 1]["tick"] / 1000 + 0.002))
                prev, start = m, i
        if samples:
            segs.append((prev, samples[start]["tick"] / 1000,
                         samples[-1]["tick"] / 1000 + 0.002))
        if len(segs) > self.REC_SEG_LIMIT:
            segs = []
        for mode, t0, t1 in segs:
            color = MODE_CHIP_BG.get(mode, "#333f4d")
            def factory(color=color, t0=t0, t1=t1):
                item = pg.LinearRegionItem(
                    values=(t0, t1), orientation="vertical", movable=False,
                    brush=pg.mkBrush(self.REC_MODE_ALPHA + color[1:]),
                    pen=pg.mkPen(color, width=1))
                item.setZValue(-10)
                return item
            self._add_overlay(factory)
        # 故障段：fault==1 的连续拍，红色叠在模式背景上
        spans, f0, prev_f = [], None, False
        for s in samples:
            f = bool(s.get("fault"))
            if f and not prev_f:
                f0 = s["tick"] / 1000
            if not f and prev_f:
                spans.append((f0, s["tick"] / 1000))
            prev_f = f
        if prev_f:
            spans.append((f0, samples[-1]["tick"] / 1000 + 0.002))
        if len(spans) <= self.REC_SEG_LIMIT:
            for t0, t1 in spans:
                def factory(t0=t0, t1=t1):
                    item = pg.LinearRegionItem(
                        values=(t0, t1), orientation="vertical", movable=False,
                        brush=pg.mkBrush(200, 40, 40, 110), pen=pg.mkPen(None))
                    item.setZValue(-5)
                    return item
                self._add_overlay(factory)
        # 命令/备注竖线：动作类命令带名、配置类细灰线；备注虚线带文本
        cmds = [e for e in events
                if e.get("type") == "command" and e.get("tick") is not None]
        action_kinds = {1, 8, 9, 10, 11, 12, 13, 15, 16}
        marked = cmds if len(cmds) <= self.REC_CMD_LIMIT else \
            [e for e in cmds if e.get("kind") in action_kinds]
        for e in marked:
            t = e["tick"] / 1000
            kind = e.get("kind")
            if kind in action_kinds:
                color, label = "#ffbd69", COMMAND_NAMES.get(kind, hex(kind))
            else:
                color, label = "#45586c", None
            def factory(color=color, label=label, t=t):
                line = pg.InfiniteLine(
                    pos=t, angle=90, pen=pg.mkPen(color, width=1.5),
                    label=label, labelOpts={"position": 0.04, "color": color}
                    if label else None)
                line.setZValue(5)
                return line
            self._add_overlay(factory)
        for e in events:
            if e.get("type") != "note" or e.get("tick") is None:
                continue
            text = (e.get("text") or "")[:16]
            def factory(text=text, t=e["tick"] / 1000):
                line = pg.InfiniteLine(
                    pos=t, angle=90, pen=pg.mkPen("#7fd4e8", width=1,
                                                  style=QtCore.Qt.PenStyle.DashLine),
                    label=text, labelOpts={"position": 0.06, "color": "#7fd4e8"})
                line.setZValue(5)
                return line
            self._add_overlay(factory)

    def _clear_record_overlays(self):
        for plot, item in self._rec_overlays:
            plot.removeItem(item)
        self._rec_overlays.clear()

    def _record_summary_html(self, path, meta, samples, events):
        """复盘侧栏：record_audit 的关键事实浓缩成 HTML（可滚动）。"""
        esc = html.escape
        ticks = [s["tick"] for s in samples]
        span = ticks[-1] - ticks[0]
        expected = span // 2 + 1 if span > 0 else 0
        missing = (1 - len(samples) / expected) * 100 if expected else 0.0
        gaps = [b - a for a, b in zip(ticks, ticks[1:])]
        big = sum(1 for g in gaps if g > 2)
        ident = meta.get("identity") or {}
        fw = meta.get("firmware") or {}
        rows = []

        def section(title, kv):
            rows.append(f"<tr><td colspan=2 style='color:#6edce8;"
                        f"padding-top:6px;font-weight:600'>{esc(title)}</td></tr>")
            for k, v in kv:
                rows.append(f"<tr><td style='color:#7f92a8;padding-right:10px'>"
                            f"{esc(k)}</td><td>{esc(str(v))}</td></tr>")

        section("记录", [
            ("文件", Path(path).name),
            ("格式", meta.get("record_format") or "旧版单文件 JSONL"),
            ("采集于", meta.get("created", "?")),
            ("设备", meta.get("device", "?")),
            ("上位机", (fw.get("host_git") or "?")
             + (f" ({fw.get('host_git_branch')})" if fw.get("host_git_branch") else "")),
        ])
        section("完整度", [
            ("样本", f"{len(samples)} 帧 / {span / 1000:.2f}s"),
            ("缺失率", f"{missing:.3f}%（按 2ms/帧应有 {expected}）"),
            ("最大间隔", f"{max(gaps) if gaps else 0} tick，>2 tick 断点 {big} 处"),
        ])
        end = next((e for e in events if e.get("type") == "record_end"), None)
        if end:
            hist = "、".join(f"{k}×{v}" for k, v in
                             (end.get("mode_histogram") or {}).items())
            section("结束摘要", [
                ("原因", end.get("reason", "?")),
                ("对账", f"{end.get('samples')} 帧 / {end.get('span_ms')}ms，"
                 f"missing_ratio={end.get('missing_ratio')}"),
                ("模式直方图", hist or "无"),
            ])
        kinds = collections.Counter(e.get("kind") for e in events
                                    if e.get("type") == "command")
        if kinds:
            section("命令", [("命令", "、".join(
                f"{COMMAND_NAMES.get(k, k)}×{v}" for k, v in sorted(kinds.items())))])
        typed = collections.Counter(e.get("type") for e in events)
        others = "、".join(f"{k}×{v}" for k, v in sorted(typed.items()))
        section("事件", [("事件类型", others or "无")])
        detail = meta.get("parameters_detail") or []
        if detail:
            section("参数快照", [
                (f"{item.get('label', item.get('key'))}", item.get("value"))
                for item in detail])
        else:
            section("参数快照", [("参数", "（此记录没有参数快照）")])
        return ("<table cellspacing=0>" + "".join(rows) + "</table>")

    # ---------- 标定 ----------
    def build_calibration(self):
        """零点标定 + 竖直零位自校准观测（2026-10-20 audit_20261020_132102 起）。

        两条零位来源并存：① 静止下垂 CAL（2 秒平均，摆杆垂下时用）；② BALANCE
        慢 EMA 自校准（稳摆退出沿提交，限幅 ±5°，≥15s 有效样本、|α|<15° 门——
        RTL angle_proc.v AZ_*）。offset 只在 RAM，断电回到编译默认 8344。
        遥测 offset 每次变更记一条 (θ, 零位) 数据点——绝对值编码器世界角不丢圈，
        这些点就是未来 POS 全段 α 标定表的原始素材。
        """
        layout = self.page("标定", scroll=True)
        hint = QtWidgets.QLabel(
            "① 静止下垂采集：设备须处于待机/安全状态，采集 2 秒并等估计器稳定"
            "（摆杆自由下垂时用，线缆拖拽会污染结果——10-20 假平衡点事故即源于此）；\n"
            "② 竖直零位自校准：正常起摆稳摆 ≥15s 后任意方式退出（STOP/掉摆均可），"
            "板内自动把长时 α 均值（=真竖直零偏，限幅 ±5°）提交进 offset，会话内"
            "控制行为零变化；\n"
            "③ offset 只存 RAM，断电回默认 8344；每次上电后的第一次稳摆就是标定轮。")
        hint.setWordWrap(True)
        layout.addWidget(hint)
        row = QtWidgets.QHBoxLayout()
        row.addWidget(self.calib_action(
            "静止下垂采集零点（2 秒）",
            lambda: self.async_submit("command", (p.CALIBRATE, b""))))
        row.addStretch()
        layout.addLayout(row)

        zero = QtWidgets.QGroupBox("α 零位（offset）· 自校准与全段标定数据点")
        zform = QtWidgets.QFormLayout(zero)
        self.calib_offset_label = QtWidgets.QLabel("连接后显示")
        self.calib_offset_label.setStyleSheet("font-size:15px;font-weight:600;")
        zform.addRow("当前 α 零位", self.calib_offset_label)
        layout.addWidget(zero)

        pts_row = QtWidgets.QHBoxLayout()
        self.cal_points_label = QtWidgets.QLabel("尚无零位变更记录")
        self.cal_export_button = QtWidgets.QPushButton("导出标定点 CSV")
        self.cal_export_button.clicked.connect(self.export_cal_points)
        self.cal_clear_button = QtWidgets.QPushButton("清空记录")
        self.cal_clear_button.clicked.connect(self.clear_cal_points)
        pts_row.addWidget(self.cal_points_label)
        pts_row.addStretch()
        pts_row.addWidget(self.cal_export_button)
        pts_row.addWidget(self.cal_clear_button)
        layout.addLayout(pts_row)
        self.cal_points_table = QtWidgets.QTableWidget(0, 7)
        self.cal_points_table.setHorizontalHeaderLabels(
            ["时刻", "tick", "θ / °", "旧零位 / °", "新零位 / °", "Δ / °", "推断来源"])
        self.cal_points_table.horizontalHeader().setSectionResizeMode(
            QtWidgets.QHeaderView.ResizeMode.Stretch)
        self.cal_points_table.setAlternatingRowColors(True)
        self.cal_points_table.verticalHeader().setVisible(False)
        self.cal_points_table.setEditTriggers(
            QtWidgets.QAbstractItemView.EditTrigger.NoEditTriggers)
        self.cal_points_table.setMaximumHeight(220)
        self.cal_points_table.setToolTip(
            "绝对值编码器双向世界角不丢圈：每行 (θ, 零位) 就是一个全段 α 标定数据点；"
            "积累多行后可用 POS 模式走位拟合零位-位置表，供 RTL 分段查表")
        layout.addWidget(self.cal_points_table)

        self.sensor_label = QtWidgets.QLabel("ADC / θ / 下垂零偏：连接并收到遥测后显示")
        layout.addWidget(self.sensor_label)
        mark = QtWidgets.QPushButton("记录当前 θ")
        mark.clicked.connect(self.mark_encoder)
        layout.addWidget(mark)
        self.encoder_label = QtWidgets.QLabel(
            "手动转一整圈后看 θ 转角差（霍尔编码器已拆除，θ 为 MT6701 多圈值）；只读辅助")
        layout.addWidget(self.encoder_label)
        layout.addStretch()

    def record_cal_point(self, sample, old_off, new_off):
        """遥测 offset 变更沿：记一条 (θ, 零位) 数据点并刷新表格。

        来源推断看变更前样本的 mode：稳摆(2)中/退出沿=板内自校准提交，
        待机/安全(0/4)=人工下垂 CAL（CAL 在待机做）或手输。
        """
        prev_mode = self._cal_prev_mode
        if prev_mode == 2:
            origin = "稳摆自校准提交"
        elif prev_mode in (0, 4):
            origin = "人工标定（待机/安全）"
        elif prev_mode is None:
            origin = "（上电首样本）"
        else:
            origin = f"模式{prev_mode}中变更"
        entry = {"time": datetime.now().strftime("%H:%M:%S"),
                 "tick": sample["tick"], "theta": sample["theta"],
                 "old": old_off, "new": new_off, "origin": origin}
        self.cal_points.append(entry)
        if len(self.cal_points) > 500:      # 只留最近 500 条，导出前不丢即可
            del self.cal_points[:-500]
        table = self.cal_points_table
        table.setRowCount(len(self.cal_points))
        cells = [entry["time"], str(entry["tick"]),
                 f"{entry['theta']/100:+.2f}", f"{entry['old']/100:+.2f}",
                 f"{entry['new']/100:+.2f}", f"{(new_off-old_off)/100:+.2f}",
                 entry["origin"]]
        for col, text in enumerate(cells):
            table.setItem(len(self.cal_points) - 1, col,
                          QtWidgets.QTableWidgetItem(text))
        table.scrollToBottom()
        self.cal_points_label.setText(
            f"已积累 {len(self.cal_points)} 个 (θ, 零位) 数据点——POS 全段 α 标定素材")

    def export_cal_points(self):
        if not self.cal_points:
            self.log("标定：暂无数据点可导出")
            return
        out = ROOT / "logs"
        out.mkdir(exist_ok=True)
        path = out / (datetime.now().strftime("cal_points_%Y%m%d_%H%M%S") + ".csv")
        lines = ["time,tick,theta_c01,old_offset_c01,new_offset_c01,delta_c01,origin"]
        for e in self.cal_points:
            lines.append(f"{e['time']},{e['tick']},{e['theta']},{e['old']},"
                         f"{e['new']},{e['new'] - e['old']},{e['origin']}")
        path.write_text("\n".join(lines) + "\n", encoding="utf-8")
        self.log(f"标定：已导出 {len(self.cal_points)} 个数据点 → {path}")

    def clear_cal_points(self):
        self.cal_points.clear()
        self.cal_points_table.setRowCount(0)
        self.cal_points_label.setText("尚无零位变更记录")
        self.log("标定：已清空 (θ, 零位) 数据点记录")

    def mark_encoder(self):
        last = self.bridge.latest if self.bridge else None
        if not last:
            self.log("标定：暂无遥测，无法记录 θ 基准")
            return
        self.encoder_mark = last["theta"]
        self.log(f"标定：θ 基准记为 {self.encoder_mark/100:+.2f}°（{self.encoder_mark} c01）")

    # ---------- 硬件联调 ----------
    def build_hardware(self):
        """独立读取原始编码器/ADC，并以安全低速点动验收 TB6612。"""
        layout = self.page("硬件联调")
        warning = QtWidgets.QLabel(
            "仅用于编码器和 TB6612 单独验收。测试前确认共地、逻辑电平不超过 3.3 V；"
            "点动范围限制在 ±20%，异常立即释放输出。HIL 闭环不会自动接管外接电机。")
        warning.setWordWrap(True)
        warning.setStyleSheet("color:#9b1c1c;padding:8px;")
        layout.addWidget(warning)
        self.io_capability = QtWidgets.QLabel("等待设备能力信息")
        layout.addWidget(self.io_capability)
        form = QtWidgets.QFormLayout()
        self.io_encoder_label = QtWidgets.QLabel("未查询")
        self.io_adc_label = QtWidgets.QLabel("未查询")
        self.io_output_label = QtWidgets.QLabel("未查询")
        form.addRow("MT6701 原始角（14 位）", self.io_encoder_label)
        form.addRow("原始 ADC 码", self.io_adc_label)
        form.addRow("TB6612 输出状态", self.io_output_label)
        layout.addLayout(form)

        row = QtWidgets.QHBoxLayout()
        self.io_refresh_button = QtWidgets.QPushButton("刷新硬件状态")
        self.io_refresh_button.clicked.connect(lambda: self.async_submit("io_status"))
        self.io_zero_button = QtWidgets.QPushButton("记录原角零点")
        self.io_zero_button.clicked.connect(self.mark_io_encoder)
        self.io_auto = QtWidgets.QCheckBox("自动刷新（1s）")
        self.io_auto.setToolTip(
            "本页可见且已连接时每秒读一次原始角/ADC/输出状态；"
            "轮询走旁路来源，不打断 Agent 的控制租约")
        self.io_auto.toggled.connect(self.on_io_auto_toggled)
        row.addWidget(self.io_refresh_button)
        row.addWidget(self.io_zero_button)
        row.addWidget(self.io_auto)
        row.addStretch()
        layout.addLayout(row)

        motor = QtWidgets.QGroupBox("TB6612 低速点动")
        motor_row = QtWidgets.QHBoxLayout(motor)
        self.io_duty = QtWidgets.QSpinBox()
        self.io_duty.setRange(-2000, 2000)
        self.io_duty.setSingleStep(100)
        self.io_duty.setSuffix(" / 10000")
        self.io_jog_ms = QtWidgets.QSpinBox()
        self.io_jog_ms.setRange(100, 10000)
        self.io_jog_ms.setSingleStep(100)
        self.io_jog_ms.setValue(2000)
        self.io_jog_ms.setSuffix(" ms")
        self.io_jog_ms.setToolTip(
            "定时点动时长：判源流程需要「正转约一圈臂」，按当前 duty 的实际转速估；"
            "到时自动释放输出，不怕忘点释放")
        self.io_enable_button = QtWidgets.QPushButton("使能点动")
        self.io_enable_button.clicked.connect(
            lambda: self.async_submit("io_test", (1, self.io_duty.value())))
        self.io_timed_button = QtWidgets.QPushButton("定时点动")
        self.io_timed_button.clicked.connect(self.io_jog_timed)
        self.io_release_button = QtWidgets.QPushButton("释放输出")
        self.io_release_button.clicked.connect(
            lambda: self.async_submit("io_test", (0, 0)))
        motor_row.addWidget(QtWidgets.QLabel("duty"))
        motor_row.addWidget(self.io_duty)
        motor_row.addWidget(QtWidgets.QLabel("时长"))
        motor_row.addWidget(self.io_jog_ms)
        motor_row.addWidget(self.io_enable_button)
        motor_row.addWidget(self.io_timed_button)
        motor_row.addWidget(self.io_release_button)
        motor_row.addStretch()
        layout.addWidget(motor)
        self.io_test_widgets = [self.io_refresh_button, self.io_zero_button,
                                self.io_enable_button, self.io_timed_button,
                                self.io_release_button, self.io_duty,
                                self.io_jog_ms]
        self.io_jog_timer = QtCore.QTimer(self)
        self.io_jog_timer.setSingleShot(True)
        self.io_jog_timer.timeout.connect(self.io_jog_release)
        self.io_poll_timer = QtCore.QTimer(self)
        self.io_poll_timer.setInterval(1000)
        self.io_poll_timer.timeout.connect(self.io_auto_poll)
        layout.addStretch()

    def io_jog_timed(self):
        """使能点动后在 GUI 侧定时自动释放：判源转一圈流程免盯守。"""
        duty = self.io_duty.value()
        ms = self.io_jog_ms.value()
        if not self.bridge or not self.bridge.connected:
            self.log("硬件联调：未连接")
            return
        self.async_submit("io_test", (1, duty))
        self.io_jog_timer.start(ms)
        self.log(f"硬件联调：定时点动 duty={duty}，{ms} ms 后自动释放"
                 f"（期间可随时手动释放）")

    def io_jog_release(self):
        if self.bridge and self.bridge.connected:
            self.async_submit("io_test", (0, 0))
            self.log("硬件联调：定时点动时间到，已释放输出")

    def on_io_auto_toggled(self, on):
        if on:
            self.io_poll_timer.start()
        else:
            self.io_poll_timer.stop()

    def io_auto_poll(self):
        if not (self.on_page("硬件联调") and self.bridge and self.bridge.connected):
            return
        # 旁路来源读状态：不破 Agent 租约（gui 才抢占）
        self.async_submit("io_status", source="状态轮询")

    def mark_io_encoder(self):
        if not self.io_status_data:
            self.log("硬件联调：暂无原始编码器读数")
            return
        self.io_encoder_zero = self.io_status_data["encoder"]
        self.log(f"硬件联调：编码器零点记为 {self.io_encoder_zero}")

    def io_status_received(self, status):
        self.io_status_data = status
        # 霍尔拆除后 `encoder` = MT6701 原始 14 位角（0..16383 ↔ 电机轴一圈，
        # 摆臂折算 ÷20.45）。不再有"每圈 1040 计数"的口径。
        encoder = status["encoder"]
        suffix = ""
        if self.io_encoder_zero is not None:
            suffix = f"，相对零点 {encoder - self.io_encoder_zero:+d} LSB"
        self.io_encoder_label.setText(
            f"{encoder}（摆臂 {encoder * 360.0 / 16384 / 20.45:6.2f}°）{suffix}")
        self.io_adc_label.setText(str(status["adc"]))
        state = "使能" if status["enabled"] else "释放"
        self.io_output_label.setText(
            f"{state}，duty={status['duty']}，RX错误={status['rx_errors']}，丢帧={status['dropped']}")

    # ---------- 传感诊断（单源 θ：MT6701，霍尔已拆除） ----------
    def build_diag(self):
        """θ 单源健康 + 驱动响应一致性（掉线检测）——纯上位机侧计算，零协议改动。

        背景（2026-10-22）：原霍尔正交编码器已物理拆除，θ 只剩电机侧 MT6701
        一条链（RTL `TH_SRC_MT_ONLY=1`）。遥测 `theta` 是该链的多圈累加值，
        `encoder` 字段改投 **MT6701 原始 14 位角**——两者可互校（1 LSB=9.31 c01），
        这是拆掉霍尔后仅存的冗余。
        核心判据从"双源判源"换成**驱动-响应**：MT6701 的径向磁铁叠在电机霍尔
        磁铁旁，大电流时磁场互相干扰，驱动会 hold-last-good ⇒ θ 冻结、θ̇ 塌到 0。
        单源下没有第二把尺，但"指令在真驱动区而 θ 完全不动"是确定签名。
        """
        layout = self.page("传感诊断", scroll=True)
        # 两列分组布局：左列=实时读数，右列=采集/判定操作；中部左右并排两路
        # 链路健康；再往下左右并排双曲线。把原先一整条竖列压缩到约两屏高度。
        intro = QtWidgets.QLabel(
            "用法（霍尔编码器已拆除：θ 只有 MT6701 一条链）：\n"
            "① 待机/安全态，在「硬件联调」页用 IO_TEST 点动（|duty|≥2500 才算真驱动）"
            "或直接起摆一轮 → 回本页「结束并判定」。核心检查单是**驱动-响应一致性**："
            "指令够大而 θ 纹丝不动 = MT6701 掉线或臂卡死；\n"
            "② 静止 1~2 秒后判定看噪声项。另有一项「θ 累加器 vs 原始角」交叉核对，"
            "能抓累加器丢步/方向反（TH_MT_DIR_FLIP）。模式迁移、故障锁存、掉摆、"
            "驱动中 θ 不动都会常开写进下方日志。中部两个区块读 MT6701/MT6835 "
            "的 SPI 帧层统计（掉线是磁场问题，帧层统计是第一现场）。")
        intro.setWordWrap(True)
        intro.setStyleSheet(
            "color:#334e68;padding:8px;background:#f4f8fb;"
            "border:1px solid #dce6ef;border-radius:6px;")
        layout.addWidget(intro)

        top = QtWidgets.QHBoxLayout()
        live = QtWidgets.QGroupBox("实时读数（当前遥测）")
        form = QtWidgets.QFormLayout(live)
        self.diag_source_label = QtWidgets.QLabel("未连接")
        self.diag_theta_label = QtWidgets.QLabel("—")
        self.diag_hall_label = QtWidgets.QLabel("—")
        self.diag_delta_label = QtWidgets.QLabel("—")
        self.diag_rate_label = QtWidgets.QLabel("—")
        form.addRow("驱动-响应 / 掉线统计", self.diag_source_label)
        form.addRow("θ（多圈，上电零点） / 原角折算（单圈，原角零点）", self.diag_theta_label)
        form.addRow("增量一致性偏差", self.diag_delta_label)
        form.addRow("原角 / 原始字 / θ̇ / α̇", self.diag_rate_label)
        top.addWidget(live, stretch=5)

        ctrl = QtWidgets.QGroupBox("采集与判定")
        cgrid = QtWidgets.QGridLayout(ctrl)
        self.diag_start_button = QtWidgets.QPushButton("开始采集")
        self.diag_start_button.clicked.connect(self.diag_start)
        self.diag_stop_button = QtWidgets.QPushButton("结束并判定")
        self.diag_stop_button.clicked.connect(self.diag_finish)
        self.diag_rec_button = QtWidgets.QPushButton("开始记录 CSV")
        self.diag_rec_button.clicked.connect(self.diag_record_toggle)
        self.diag_load_button = QtWidgets.QPushButton("载入记录复盘")
        self.diag_load_button.clicked.connect(self.diag_load_record)
        self.diag_auto = QtWidgets.QCheckBox("自动读取链路健康（1s）")
        self.diag_auto.setToolTip(
            "本页可见且已连接时每秒读一次 STATUS，左右两个链路健康区块自动刷新；"
            "帧率/坏帧率由连续读数自动算出，不必再手动连点两次。"
            "轮询走旁路来源，不打断 Agent 控制租约")
        self.diag_auto.toggled.connect(self.on_diag_auto_toggled)
        self.diag_poll_timer = QtCore.QTimer(self)
        self.diag_poll_timer.setInterval(1000)
        self.diag_poll_timer.timeout.connect(self.diag_auto_poll)
        self.diag_status_label = QtWidgets.QLabel("未采集")
        self.diag_status_label.setWordWrap(True)
        cgrid.addWidget(self.diag_start_button, 0, 0)
        cgrid.addWidget(self.diag_stop_button, 0, 1)
        cgrid.addWidget(self.diag_rec_button, 1, 0)
        cgrid.addWidget(self.diag_load_button, 1, 1)
        cgrid.addWidget(self.diag_auto, 2, 0, 1, 2)
        cgrid.addWidget(self.diag_status_label, 3, 0, 1, 2)
        top.addWidget(ctrl, stretch=4)
        layout.addLayout(top)

        # ---- 两路链路级健康并排（读驱动内部统计，不是双源数值对比）----
        links = QtWidgets.QHBoxLayout()
        link = QtWidgets.QGroupBox("MT6701 链路健康（θ，SPI/SSI 帧层）")
        lform = QtWidgets.QFormLayout(link)
        self.mt6_mg_label = QtWidgets.QLabel("未查询")
        self.mt6_rate_label = QtWidgets.QLabel("未查询")
        self.mt6_err_label = QtWidgets.QLabel("未查询")
        self.mt6_state_label = QtWidgets.QLabel("未查询")
        self.mt6_angle_label = QtWidgets.QLabel("未查询")
        lform.addRow("磁场强度 Mg[1:0]", self.mt6_mg_label)
        lform.addRow("失跟踪 Mg[3]", self.mt6_state_label)
        lform.addRow("轮询帧率 / 坏帧率", self.mt6_rate_label)
        lform.addRow("累计 CRC 错 / Mg 告警", self.mt6_err_label)
        lform.addRow("最近原始角（14 位）", self.mt6_angle_label)
        lrow = QtWidgets.QHBoxLayout()
        self.mt6_query_button = QtWidgets.QPushButton("读取链路健康")
        self.mt6_query_button.setToolTip("连点两次用增量算帧率/坏帧率；"
                                         "或勾选上方自动读取")
        self.mt6_query_button.clicked.connect(self.mt6_query)
        self.mt6_check_button = QtWidgets.QPushButton("生成链路判定")
        self.mt6_check_button.clicked.connect(self.mt6_check)
        lrow.addWidget(self.mt6_query_button)
        lrow.addWidget(self.mt6_check_button)
        lrow.addStretch()
        lform.addRow(lrow)
        links.addWidget(link, stretch=1)

        link68 = QtWidgets.QGroupBox("MT6835 链路健康（摆杆 α，SPI mode3 帧层）")
        l68form = QtWidgets.QFormLayout(link68)
        self.mt68_status_label = QtWidgets.QLabel("未查询")
        self.mt68_rate_label = QtWidgets.QLabel("未查询")
        self.mt68_err_label = QtWidgets.QLabel("未查询")
        self.mt68_angle_label = QtWidgets.QLabel("未查询")
        self.mt68_held_label = QtWidgets.QLabel("未查询")
        l68form.addRow("器件状态位", self.mt68_status_label)
        l68form.addRow("轮询帧率 / 坏帧率", self.mt68_rate_label)
        l68form.addRow("累计 CRC 错 / 状态告警", self.mt68_err_label)
        l68form.addRow("最近原始角（21 位）", self.mt68_angle_label)
        l68form.addRow("α 保持字（遥测 adc）", self.mt68_held_label)
        l68row = QtWidgets.QHBoxLayout()
        self.mt68_query_button = QtWidgets.QPushButton("读取链路健康")
        self.mt68_query_button.setToolTip("连点两次用增量算帧率/坏帧率；"
                                          "或勾选上方自动读取")
        self.mt68_query_button.clicked.connect(self.mt68_query)
        self.mt68_check_button = QtWidgets.QPushButton("生成链路判定")
        self.mt68_check_button.clicked.connect(self.mt68_check)
        l68row.addWidget(self.mt68_query_button)
        l68row.addWidget(self.mt68_check_button)
        l68row.addStretch()
        l68form.addRow(l68row)
        links.addWidget(link68, stretch=1)
        layout.addLayout(links)

        # 实时双图并排：左图 θ（多圈）与 α 同轴看动作；右图原角一致性偏差
        # （原始 LSB），平坦贴 0 = 累加器没丢步，台阶/斜坡 = 丢步或方向反。
        plots = QtWidgets.QHBoxLayout()
        self.diag_plot = pg.PlotWidget()
        self.diag_plot.setMinimumHeight(210)
        self.diag_plot.addLegend(offset=(8, 8))
        self.diag_curve_theta = self.diag_plot.plot(
            pen=pg.mkPen("#2b7cd3", width=2), name="θ（MT6701 多圈）")
        self.diag_curve_hall = self.diag_plot.plot(
            pen=pg.mkPen("#d97b20", width=1, style=QtCore.Qt.PenStyle.DashLine),
            name="α（摆杆）")
        self.diag_plot.setLabel("left", "角度 (°)")
        self.diag_plot.setLabel("bottom", "t (s)")
        self.diag_plot.showGrid(x=True, y=True, alpha=0.25)
        plots.addWidget(self.diag_plot, stretch=5)
        self.diag_plot2 = pg.PlotWidget()
        self.diag_plot2.setMinimumHeight(210)
        self.diag_curve_delta = self.diag_plot2.plot(
            pen=pg.mkPen("#7a4fbf", width=2), name="Δ原角 − Δθ/9.31（LSB）")
        self.diag_plot2.setLabel("left", "原角一致性偏差 (LSB)")
        self.diag_plot2.setLabel("bottom", "t (s)")
        self.diag_plot2.showGrid(x=True, y=True, alpha=0.25)
        self.diag_plot2.addLegend(offset=(8, 8))
        plots.addWidget(self.diag_plot2, stretch=3)
        layout.addLayout(plots)

        self.diag_summary = QtWidgets.QLabel("记录结束后这里显示复盘摘要")
        self.diag_summary.setWordWrap(True)
        self.diag_summary.setStyleSheet("color:#334e68;")
        layout.addWidget(self.diag_summary)

        self.diag_table = QtWidgets.QTableWidget(0, 3)
        self.diag_table.setHorizontalHeaderLabels(["检查项", "结果", "说明"])
        self.diag_table.horizontalHeader().setSectionResizeMode(
            QtWidgets.QHeaderView.ResizeMode.Stretch)
        self.diag_table.setAlternatingRowColors(True)
        self.diag_table.verticalHeader().setVisible(False)
        self.diag_table.setEditTriggers(QtWidgets.QAbstractItemView.EditTrigger.NoEditTriggers)
        layout.addWidget(self.diag_table, stretch=1)
        self.diag_conclusion = QtWidgets.QLabel("检查单在采集后填入")
        self.diag_conclusion.setWordWrap(True)
        self.diag_conclusion.setStyleSheet("padding:8px;background:#eef3f9;border-radius:5px;")
        layout.addWidget(self.diag_conclusion)

    def diag_start(self):
        if not self.bridge or not self.bridge.connected or not self.bridge.latest:
            self.log("诊断：未连接或暂无遥测")
            return
        self.diag.reset()
        self.watch.reset()
        self._diag_tick = -1
        self.diag_capturing = True
        self._diag_start_tick = self.bridge.latest["tick"]
        self.diag_table.setRowCount(0)
        self.diag_conclusion.setText("采集中——按上方流程点动/静止，完成后点「结束并判定」")
        self.log("诊断采集开始（单源 θ：驱动-响应掉线 / 原角一致性 / 噪声）")

    def diag_finish(self):
        self.diag_capturing = False
        checks, conclusion = self.diag.verdict()
        self.diag_table.setRowCount(len(checks))
        for row, (item, ok, detail) in enumerate(checks):
            status = "—" if ok is None else ("✓ 通过" if ok else "✗ 未过")
            for col, text in enumerate((item, status, detail)):
                cell = QtWidgets.QTableWidgetItem(text)
                if ok is False and col == 1:
                    cell.setForeground(QtGui.QBrush(QtGui.QColor("#b42333")))
                self.diag_table.setItem(row, col, cell)
        self.diag_conclusion.setText(conclusion)
        self.log("诊断判定：" + conclusion)

    # ---------- 诊断记录（CSV） ----------
    def diag_record_toggle(self):
        """开始/停止把遥测逐拍写进诊断 CSV（与 JSONL 协议记录互补）。"""
        if self.recorder.active:
            rows = self.recorder.close()
            path = self.recorder.path
            self.diag_rec_button.setText("开始记录 CSV")
            self.log(f"诊断记录已保存：{path}（{rows} 行）")
            try:
                summary = summarize_csv(load_csv(path))
                self.diag_summary.setText(
                    f"记录复盘（{path}）：{self._fmt_summary(summary)}")
                self.log("记录复盘：" + self._fmt_summary(summary))
            except Exception as exc:
                self.log(f"诊断记录复盘失败：{exc}")
            return
        if not self.bridge or not self.bridge.connected:
            self.log("诊断记录：未连接设备")
            return
        out = ROOT / "logs"
        out.mkdir(exist_ok=True)
        path = out / (datetime.now().strftime("diag_%Y%m%d_%H%M%S") + ".csv")
        self.recorder.open_at(path)
        self.diag_rec_button.setText("停止记录 CSV")
        self.diag_summary.setText(f"正在记录 → {path}")
        self.log(f"诊断记录开始：{path}（每拍一行，含原角/一致性/掉线标记）")

    @staticmethod
    def _fmt_summary(s):
        if not s or not s.get("rows"):
            return "无有效样本"
        parts = [f"{s['rows']} 行 / {s['span_s']}s", f"行程 {s.get('motion_revs', 0):+.2f} 圈"]
        if s.get("drive_ms"):
            ratio = s.get("dropout_ratio")
            parts.append(f"驱动段 {s['drive_ms']:.0f} ms，掉线 {s.get('stuck_ms', 0):.0f} ms"
                         + (f"（{ratio*100:.1f}%）" if ratio is not None else ""))
        if s.get("raw_err_max") is not None:
            parts.append(f"原角偏差 均值 {s.get('raw_err_mean', 0):+.2f}/"
                         f"最大 {s['raw_err_max']:.2f} LSB")
        if s.get("quiet_theta_dot_sigma") is not None:
            parts.append(f"静止 θ̇σ {s['quiet_theta_dot_sigma']:.1f}")
        if s.get("fault_ticks"):
            parts.append(f"故障拍 {s['fault_ticks']}")
        return "；".join(parts)

    def diag_load_record(self):
        """载入一份已有诊断 CSV，画曲线并出摘要（复盘不用重连设备）。"""
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "载入诊断记录", str(ROOT / "logs"), "诊断 CSV (*.csv)")
        if not path:
            return
        try:
            rows = load_csv(path)
        except Exception as exc:
            self.log("载入诊断记录失败：" + str(exc))
            return
        if not rows:
            self.log("载入诊断记录：文件为空")
            return
        if "raw_err_lsb" not in rows[0]:
            self.log("载入诊断记录：这是旧的霍尔双源 CSV（缺 raw_err_lsb 列），"
                     "霍尔拆除后该记录只剩 θ/α 有效；请用新固件重录。")
        self._diag_plot_t.clear()
        self._diag_plot_theta.clear()
        self._diag_plot_hall.clear()
        self._diag_plot_delta.clear()
        t0 = rows[0]["tick_ms"]
        for r in rows:
            self._diag_plot_t.append((r["tick_ms"] - t0) / 1000.0)
            self._diag_plot_theta.append(r["theta_c01"] / 100.0)
            self._diag_plot_hall.append(r["alpha_c01"] / 100.0)
            self._diag_plot_delta.append(r.get("raw_err_lsb", 0.0))
        self.diag_curve_theta.setData(list(self._diag_plot_t), list(self._diag_plot_theta))
        self.diag_curve_hall.setData(list(self._diag_plot_t), list(self._diag_plot_hall))
        self.diag_curve_delta.setData(list(self._diag_plot_t), list(self._diag_plot_delta))
        summary = summarize_csv(rows)
        self.diag_summary.setText(f"复盘（{path}）：{self._fmt_summary(summary)}")
        self.log("载入诊断记录：" + self._fmt_summary(summary))

    def update_diag(self, latest):
        """本页可见时的实时面板刷新（累积计算在 refresh() 里常开进行）。

        `encoder` 在霍尔拆除后是 MT6701 原始 14 位角，与 θ 多圈累加器按**增量**
        互校（θ 上电清零、原角有任意零点，绝对域必然差一个常数偏置）。
        """
        raw = latest["encoder"] % RAW_FULL
        raw_c01 = raw * RAW_TO_C01
        self.diag_theta_label.setText(
            f"{latest['theta']/100:+9.2f}° / {raw_c01/100:8.2f}°")
        err_recent = self.diag.raw_err_recent
        if err_recent is None:
            self.diag_delta_label.setText("—（需角度事件）")
            self.diag_delta_label.setStyleSheet("")
        else:
            self.diag_delta_label.setText(
                f"{err_recent:+.2f} LSB（近 50 事件均值；容差 ±3）")
            self.diag_delta_label.setStyleSheet(
                "color:#b42333;font-weight:bold;" if abs(err_recent) > 3 else "")
        self.diag_rate_label.setText(
            f"{raw:5d} / 字 {latest['adc']:5d}   θ̇ {latest['theta_dot']/10:+.1f}°/s   "
            f"α̇ {latest['alpha_dot']/10:+.1f}°/s")
        driving = abs(latest["duty"]) >= 2500
        ratio = self.diag.dropout_ratio
        self.diag_source_label.setText(
            f"duty {latest['duty']:+5d}{'（真驱动区）' if driving else ''} · "
            f"掉线 {self.diag.dropout_events} 次 / {self.diag.dropout_ms:.0f} ms"
            f"（驱动段 {self.diag.drive_ms:.0f} ms，{ratio*100:.1f}%）")
        if ratio > 0.02:
            self.diag_source_label.setStyleSheet("color:#b42333;font-weight:bold;")
            self.diag_source_label.setToolTip(
                "驱动期 θ 明显不跟：MT6701 径向磁铁与电机霍尔磁铁叠装，大电流时"
                "磁场互相干扰（驱动 hold-last-good ⇒ θ 冻结、θ̇ 塌到 0）。"
                "起摆的加速/刹车已按时间开环兜住，但稳摆/定位仍靠这条链。")
        else:
            self.diag_source_label.setStyleSheet("")
            self.diag_source_label.setToolTip("")
        self._diag_plot_append(latest, latest["alpha"] / 100.0,
                               err_recent if err_recent is not None else 0.0)
        if self.diag_capturing:
            self.diag_status_label.setText(
                f"采集中 {(latest['tick']-self._diag_start_tick)/1000:.1f}s · "
                f"行程 {self.diag.motion_revs:+.2f} 圈 · "
                f"掉线 {self.diag.dropout_ms:.0f} ms"
                + (f" · 已记录 {self.recorder.rows} 行" if self.recorder.active else ""))

    def _diag_plot_append(self, latest, alpha_deg, err_lsb):
        """曲线只保留最近 12 秒，随遥测滚动。"""
        ts = latest["tick"] / 1000.0
        self._diag_plot_t.append(ts)
        self._diag_plot_theta.append(latest["theta"] / 100.0)
        self._diag_plot_hall.append(alpha_deg)
        self._diag_plot_delta.append(err_lsb)
        cutoff = ts - 12.0
        while self._diag_plot_t and self._diag_plot_t[0] < cutoff:
            self._diag_plot_t.popleft()
            self._diag_plot_theta.popleft()
            self._diag_plot_hall.popleft()
            self._diag_plot_delta.popleft()
        t = list(self._diag_plot_t)
        self.diag_curve_theta.setData(t, list(self._diag_plot_theta))
        self.diag_curve_hall.setData(t, list(self._diag_plot_hall))
        self.diag_curve_delta.setData(t, list(self._diag_plot_delta))

    def mt6_query(self):
        """读一次 STATUS 的 MT6701 健康字；连点两次用增量算帧率/坏帧率。"""
        if not self.bridge or not self.bridge.connected:
            self.log("MT6701：未连接")
            return
        try:
            status = self.bridge.submit("io_status", source="gui")
        except Exception as exc:
            self.log("MT6701 链路读取失败：" + str(exc))
            return
        tick = self.bridge.latest["tick"] if self.bridge.latest else None
        self._mt6_apply(status, tick)

    def _mt6_apply(self, status, tick):
        """把一次 io_status 结果刷进 MT6701 链路区块（手动查询与自动轮询共用）。"""
        rows, msgs = self.mt6.update(status, tick)
        if rows is None:
            self.mt6_mg_label.setText("旧固件无字段")
            self.mt6_state_label.setText("—")
            self.mt6_rate_label.setText("—")
            self.mt6_err_label.setText("—")
            self.mt6_angle_label.setText("—")
            for text in msgs:
                self.log(text)
            return None
        self.mt6_mg_label.setText(rows["mg_text"])
        self.mt6_mg_label.setStyleSheet(
            "" if rows["mg_text"] == "正常" else "color:#b42333;font-weight:bold;")
        self.mt6_state_label.setText("失跟踪（角度不可信）" if rows["lost_track"] else "正常")
        self.mt6_state_label.setStyleSheet(
            "color:#b42333;font-weight:bold;" if rows["lost_track"] else "")
        rate = rows.get("frame_rate")
        bad = rows.get("bad_rate")
        self.mt6_rate_label.setText(
            f"{rate:.0f} Hz / {bad:.2f}%" if rate is not None
            else f"再点一次算帧率（累计 {rows['frames']} 帧）")
        self.mt6_err_label.setText(
            f"{fmt_sat16(rows['crc_err'])} / {fmt_sat16(rows['mg_err'])}")
        self.mt6_angle_label.setText(
            f"{rows['angle_raw']}  ({rows['angle_deg']:.2f}° 电机轴)")
        self.mt6_state_label.setToolTip(
            "选中状态：" + ("MT6701 恒为 θ 源（霍尔编码器已拆除，TH_SRC_MT_ONLY=1）"
                            if rows["selected"]
                            else "旧 V5 跳线位流（A14 悬空），θ 用霍尔源"))
        for text in msgs:
            self.log(text)
        return rows

    def mt6_check(self):
        rows, _ = self.mt6.update(
            self.bridge.submit("io_status", source="gui")
            if self.bridge and self.bridge.connected else {"mt6701": None},
            self.bridge.latest["tick"] if self.bridge and self.bridge.latest else None)
        checks, conclusion = self.mt6.verdict(rows)
        self.diag_table.setRowCount(len(checks))
        for row, (item, ok, detail) in enumerate(checks):
            status = "—" if ok is None else ("✓ 通过" if ok else "✗ 未过")
            for col, text in enumerate((item, status, detail)):
                cell = QtWidgets.QTableWidgetItem(text)
                if ok is False and col == 1:
                    cell.setForeground(QtGui.QBrush(QtGui.QColor("#b42333")))
                self.diag_table.setItem(row, col, cell)
        self.diag_conclusion.setText(conclusion)
        self.log("MT6701 链路判定：" + conclusion)

    def mt68_query(self):
        """读一次 STATUS 的 MT6835 健康字；连点两次用增量算帧率/坏帧率。"""
        if not self.bridge or not self.bridge.connected:
            self.log("MT6835：未连接")
            return
        try:
            status = self.bridge.submit("io_status", source="gui")
        except Exception as exc:
            self.log("MT6835 链路读取失败：" + str(exc))
            return
        tick = self.bridge.latest["tick"] if self.bridge.latest else None
        self._mt68_apply(status, tick)

    def _mt68_apply(self, status, tick):
        """把一次 io_status 结果刷进 MT6835 链路区块（手动查询与自动轮询共用）。"""
        rows, msgs = self.mt68.update(status, tick)
        if rows is None:
            self.mt68_status_label.setText("旧固件无字段")
            self.mt68_rate_label.setText("—")
            self.mt68_err_label.setText("—")
            self.mt68_angle_label.setText("—")
            self.mt68_held_label.setText("—")
            for text in msgs:
                self.log(text)
            return None
        self.mt68_status_label.setText(rows["status_text"])
        self.mt68_status_label.setStyleSheet(
            "" if rows["status"] == 0 else "color:#b42333;font-weight:bold;")
        rate = rows.get("frame_rate")
        bad = rows.get("bad_rate")
        self.mt68_rate_label.setText(
            f"{rate:.0f} Hz / {bad:.2f}%" if rate is not None
            else f"再点一次算帧率（累计 {rows['frames']} 帧）")
        self.mt68_err_label.setText(
            f"{fmt_sat16(rows['crc_err'])} / {fmt_sat16(rows['status_err'])}")
        self.mt68_angle_label.setText(
            f"{rows['angle_raw']}  ({rows['angle_deg']:.3f}° 摆轴)"
            + ("  ← 断线签名" if rows["unwired"] else ""))
        self.mt68_angle_label.setStyleSheet(
            "color:#b42333;font-weight:bold;" if rows["unwired"] else "")
        latest = self.bridge.latest
        self.mt68_held_label.setText(
            f"{latest['adc']}" if latest else "—")
        self.mt68_held_label.setToolTip(
            "遥测 adc = 末好帧 angle21>>5（65536↔360°）；与原始角同步走=链路活，"
            "冻结在旧值=正在坏帧保持")
        for text in msgs:
            self.log(text)
        return rows

    def on_diag_auto_toggled(self, on):
        if on:
            self.diag_poll_timer.start()
        else:
            self.diag_poll_timer.stop()

    def diag_auto_poll(self):
        """本页可见时每秒读一次 STATUS，两路链路健康区块自动刷新。

        帧率/坏帧率靠两次读数的增量，人工"连点两次"总嫌麻烦——自动轮询天然
        连续读。走旁路来源，不抢 Agent 控制租约。
        """
        if not (self.on_page("传感诊断") and self.bridge and self.bridge.connected):
            return
        try:
            status = self.bridge.submit("io_status", source="状态轮询")
        except Exception:
            return      # 轮询失败静默：手动按钮的错误日志路径已覆盖
        tick = self.bridge.latest["tick"] if self.bridge.latest else None
        self._mt6_apply(status, tick)
        self._mt68_apply(status, tick)

    def mt68_check(self):
        rows, _ = self.mt68.update(
            self.bridge.submit("io_status", source="gui")
            if self.bridge and self.bridge.connected else {"mt6835": None},
            self.bridge.latest["tick"] if self.bridge and self.bridge.latest else None)
        checks, conclusion = self.mt68.verdict(rows)
        self.diag_table.setRowCount(len(checks))
        for row, (item, ok, detail) in enumerate(checks):
            status = "—" if ok is None else ("✓ 通过" if ok else "✗ 未过")
            for col, text in enumerate((item, status, detail)):
                cell = QtWidgets.QTableWidgetItem(text)
                if ok is False and col == 1:
                    cell.setForeground(QtGui.QBrush(QtGui.QColor("#b42333")))
                self.diag_table.setItem(row, col, cell)
        self.diag_conclusion.setText(conclusion)
        self.log("MT6835 链路判定：" + conclusion)

    def _log_link_counters(self, tag):
        """读一次 STATUS 把两路链路计数打进展日志（带相对上次快照的增量）。

        锁存现场沿触发 + 连接基线各一次：板复位后计数器单调累计，锁存瞬间的
        绝对值/增量直接指认元凶（CRC 爬=串扰、状态码 100=欠压、Mg 爬=磁场）。
        """
        if not self.bridge or not self.bridge.connected:
            return
        try:
            status = self.bridge.submit("io_status", source="gui")
        except Exception as exc:
            self.log(f"链路计数读取失败（{tag}）：{exc}")
            return
        mt, m8 = status.get("mt6701"), status.get("mt6835")
        if mt is None and m8 is None:
            self.log(f"链路计数（{tag}）：旧固件无健康字段（需 STATUS 57 字节位流）")
            self._link_prev = (None, None)
            return

        def delta_err(cur, old, key):
            # 只给错帧计数算增量：它们饱和不回绕，差值恒有效。帧/好帧计数
            # ~1.5s 回绕一次（40-50kHz 轮询 × 16 位），差值是回绕混叠垃圾。
            if cur is None or old is None:
                return ""
            d = cur[key] - old[key]
            return f"(+{d})" if d else ""

        prev6, prev8 = self._link_prev or (None, None)
        if mt:
            gap = mt.get("max_no_good")
            gap_text = f" 最长无好帧{gap}帧" if gap is not None else ""
            self.log(
                f"链路计数（{tag}）MT6701：帧{mt['frames']} "
                f"好{mt['good']} CRC{fmt_sat16(mt['crc_err'])}{delta_err(mt, prev6, 'crc_err')} "
                f"Mg{fmt_sat16(mt['mg_err'])}{delta_err(mt, prev6, 'mg_err')} "
                f"码{mt['mg_code']:03b} 原因={mt.get('cause', 0)}{gap_text}"
                + (" [θ源]" if mt["selected"] else "")
                + (" 电平FAULT" if mt["fault"] else ""))
        else:
            self.log(f"链路计数（{tag}）MT6701：旧固件无字段")
        if m8:
            self.log(
                f"链路计数（{tag}）MT6835：帧{m8['frames']} "
                f"好{m8['good']} CRC{fmt_sat16(m8['crc_err'])}{delta_err(m8, prev8, 'crc_err')} "
                f"状态告警{fmt_sat16(m8['status_err'])}{delta_err(m8, prev8, 'status_err')} "
                f"码{m8['status']:03b}"
                + (" 电平FAULT" if m8["fault"] else ""))
        else:
            self.log(f"链路计数（{tag}）MT6835：旧固件无字段")
        self._link_prev = (mt, m8)

    # ---------- 自动试验 ----------
    def build_trial(self):
        """25 秒全流程试验：起摆 → 8s 稳摆 → 相对定位 → 17s 观察（口径与 host 一致）。"""
        layout = self.page("自动试验")
        row = QtWidgets.QHBoxLayout()
        self.trial_delta = QtWidgets.QDoubleSpinBox()
        self.trial_delta.setRange(-60.0, 60.0)      # 固件 MOVE 限 ±60°
        self.trial_delta.setValue(30.0)
        self.trial_delta.setSingleStep(1.0)
        self.trial_delta.setSuffix(" °")
        row.addWidget(QtWidgets.QLabel("相对位移"))
        row.addWidget(self.trial_delta)
        row.addWidget(self.action("稳摆时执行定位", self.move_relative))
        row.addWidget(QtWidgets.QLabel("绝对方位"))
        self.abs_spin = QtWidgets.QDoubleSpinBox()
        self.abs_spin.setRange(0.0, 359.9)         # 0-360° 方位角（解回绕见 move_absolute）
        self.abs_spin.setValue(0.0)
        self.abs_spin.setSingleStep(5.0)
        self.abs_spin.setSuffix(" °")
        row.addWidget(self.abs_spin)
        row.addWidget(self.action("转到绝对方位", self.move_absolute))
        row.addStretch()
        layout.addLayout(row)
        layout.addWidget(self.action("启动 25 秒全流程试验", self.start_trial))
        hint = QtWidgets.QLabel(
            "从待机/安全状态开始：自动起摆 → 等待 8 秒 → 相对定位 → 观察 17 秒 → 停止。\n"
            "移动中摆杆峰值与末段误差写入日志并显示在下方；随时可点「停止」中断。\n"
            "锁步 HIL 后端不支持本流程（节拍由 STEP 驱动），请用 "
            "python -m host.acceptance --hil。")
        hint.setWordWrap(True)
        layout.addWidget(hint)
        self.trial_result = QtWidgets.QLabel("尚无试验结果")
        self.trial_result.setWordWrap(True)
        layout.addWidget(self.trial_result)
        layout.addStretch()

    def move_relative(self):
        delta = int(round(self.trial_delta.value() * 100))
        # delta=0 是合法请求：MOVE 0 会让固件把 pos_tgt 设为"当前 θ"再切 POS，
        # 即"原地进入定位"——K3/K4 渐变时不需要额外移动，是实测最安全的 POS 入场
        # 方式（2026-10-20：带 30° 位移入场时 K3 渐变与移动叠加，0.46Hz 振荡不收敛；
        # 原地入场可先验证渐变本身是否稳定）。旧版拒绝 0 反而堵死了这条路。
        # 2026-10-24：命令立即受理（固件内排队，状态门只影响执行时机）。
        self.async_submit("command", (p.MOVE, struct.pack("<i", delta)))

    def move_absolute(self):
        """绝对方位定位（0-360°）：算最短角差后**自动分 ≤55° 小步**走。

        2026-11-03 夜重写：本固件没有 MOVE_ABS（那是 champion 树的命令），
        只有相对 MOVE（受理限 ±60°）。所以绝对定位 = 最短路径换算 +
        分步 + 每步等到位（4°/s 斜坡，|te|<5° 或超时再下一步）。"""
        bridge = self.bridge
        if not bridge or not bridge.connected:
            self.log("定位失败：未连接设备")
            return
        last = bridge.latest
        if not last:
            self.log("定位失败：暂无遥测")
            return
        if last["fault"]:
            self.log("定位失败：设备存在故障，请先清除")
            return
        if last["mode"] not in (2, 3):
            self.log(f"定位失败：当前为{MODES.get(last['mode'], last['mode'])}，仅稳摆/定位时可定位")
            return
        az = float(self.abs_spin.value()) % 360.0
        theta = last["theta"] / 100.0
        delta = (az - theta + 180.0) % 360.0 - 180.0      # 最短角差
        chunks = []          # 绝对航点
        rem = delta; acc = 0.0
        while abs(rem) > 0.5:
            st = max(-55.0, min(55.0, rem))
            acc += st; chunks.append(theta + acc); rem -= st
        if not chunks:
            self.log("已在该方位附近（差 %.1f°）" % delta); return
        self._goto_q = chunks
        self._goto_target = az
        self._goto_trial = True
        self.log("绝对定位：方位 %.1f°，最短路径 %+.1f°，分 %d 步（每步 ≤55°）"
                 % (az, delta, len(chunks)))
        self._goto_send_next()

    def _goto_send_next(self):
        q = getattr(self, "_goto_q", [])
        if not q:
            msg = "已到 %.1f° 附近（绿廊内）" % getattr(self, "_goto_target", 0.0)
            if getattr(self, "_goto_trial", False):
                self.log("绝对定位完成：" + msg)
            else:
                self.pos_label.setText(msg)
            return
        wp = q.pop(0)                       # 绝对航点（°）
        try:
            self.bridge.submit("command",
                               (p.MOVE_ABS, struct.pack("<i", int(round(wp * 100)))),
                               source="gui", wait=False)
        except Exception as exc:
            self.log("定位失败：" + str(exc)); return
        self._goto_step = wp
        self._goto_verify = True            # ★首轮核对固件 target≈航点（帧失配自愈）
        self._goto_t0 = time.monotonic()
        self._goto_prev_tgt = None
        self._goto_prev_t = None
        QtCore.QTimer.singleShot(400, self._goto_watch)

    def _goto_watch(self):
        s = self.bridge.latest if self.bridge else None
        ok_modes = (2, 3) if getattr(self, "_goto_trial", False) else (3,)
        if not s or s.get("mode") not in ok_modes:
            self._goto_q = []
            msg = "已离开稳摆/定位态，定位中止"
            self.log(msg) if getattr(self, "_goto_trial", False) else self.pos_label.setText(msg)
            return
        # ★到位判据必须包含"目标已停止爬升"：MOVE 后目标从当前臂位起算，
        # 开头 te≈0 会骗过纯 te<5 判据（2026-11-03 实测：两步 1 秒内连发，
        # 第二步覆盖第一步目标 → 停在 351.8° 而不是 30°）。
        now = time.monotonic()
        # ★最小观察窗（11-4 实测：发步瞬间 te≈0 且 rate 未建立 → 同秒假"完成"）
        if now - getattr(self, "_goto_t0", now) < 1.2:
            QtCore.QTimer.singleShot(200, self._goto_watch); return
        # ★帧校验（11-4 定谳：旧 GUI 进程把航点锚在移动中重读的 θ 上，固件收到
        # −267.84° 错误绝对目标，臂反方向滑行 400°+）。首轮核对固件 target≈航点，
        # 不符则按当前臂位重锚重发一次。
        if getattr(self, "_goto_verify", False) and now - self._goto_t0 > 0.3:
            self._goto_verify = False
            fw_tgt = (s.get("target") or 0) / 100.0
            wp = getattr(self, "_goto_step", None)
            if wp is not None and abs(((fw_tgt - wp + 180) % 360) - 180) > 3.0:
                th_now = s.get("theta") / 100.0
                wp2 = th_now + ((wp - th_now + 180) % 360 - 180)
                self.log("定位校正：固件目标 %.1f°≠航点 %.1f°，按当前臂位 %.1f° 重锚为 %.1f° 重发"
                         % (fw_tgt % 360, wp % 360, th_now % 360, wp2 % 360))
                try:
                    self.bridge.submit("command",
                                       (p.MOVE_ABS, struct.pack("<i", int(round(wp2 * 100)))),
                                       source="gui", wait=False)
                    self._goto_step = wp2
                except Exception as exc:
                    self.log("定位重发失败：" + str(exc)); return
                self._goto_t0 = now
                self._goto_prev_tgt = None
                self._goto_prev_t = None
                QtCore.QTimer.singleShot(300, self._goto_watch)
                return
        tgt = (s.get("target") or 0) / 100.0
        prev = getattr(self, "_goto_prev_tgt", None)
        prev_t = getattr(self, "_goto_prev_t", None)
        rate = 0.0
        if prev is not None and prev_t is not None and now > prev_t:
            rate = (tgt - prev) / max(now - prev_t, 1e-6)
        self._goto_prev_tgt = tgt
        self._goto_prev_t = now
        te = abs(s.get("theta_e") or 0) / 100.0
        # 到位判据（2026-11-04）：优先用固件回传的 term_k3 = cmd−θ（真实"离目标方位还差多少"）
        k3 = s.get("term_k3")
        limit = abs(getattr(self, "_goto_step", 0.0)) / 4.0 + 1.5
        if k3 is not None and abs(k3) < 500 and now - self._goto_t0 > 1.2:
            self._goto_send_next(); return
        delivered = abs(rate) < 1.0
        if (delivered and te < 5.0) or now - self._goto_t0 > limit:
            self._goto_send_next()
        else:
            QtCore.QTimer.singleShot(200, self._goto_watch)

    def start_trial(self):
        if not self.bridge or not self.bridge.connected:
            self.log("试验未启动：未连接设备")
            return
        delta = int(round(self.trial_delta.value() * 100))
        try:
            self.bridge.run_trial("gui", delta)
        except Exception as exc:
            self.log("试验未启动：" + str(exc))
            return
        self.trial_result.setText(f"进行中：目标 Δ{delta/100:+.2f}°（约 25 秒，起摆→稳摆→定位→观察）")

    # ---------- 竞赛视频录制 ----------
    # ---------- 竞赛录制：分镜按钮 + 独立子窗口（2026-11-03）----------
    def build_record(self):
        """主页只放 4 个大按钮；点击弹出独立子窗口（大曲线+实时标注）。
        数据分析(α峰/误差最大/恢复时间)直接画在图上。"""
        layout = self.page("竞赛录制")
        grid = QtWidgets.QGridLayout()
        grid.setSpacing(14)

        def big_button(text, cb, color):
            b = QtWidgets.QPushButton(text)
            b.setMinimumHeight(72)
            b.setMinimumWidth(220)
            b.setStyleSheet(
                "QPushButton{font-size:17px;font-weight:bold;color:white;"
                "background:%s;border-radius:8px;}"
                "QPushButton:disabled{background:#9aa7b5;}" % color)
            b.clicked.connect(cb)
            self.action_widgets.append(b)
            return b

        grid.addWidget(big_button("起摆展示\n（自适应甩幅）", self.open_swing_window, "#2a7fbd"), 0, 0)
        grid.addWidget(big_button("抗扰展示\n（手推+恢复统计）", self.open_disturb_window, "#2e8b57"), 0, 1)
        grid.addWidget(big_button("定位步进\n（误差收敛）", self.open_pos_window, "#c47f17"), 1, 0)
        grid.addWidget(big_button("汇总成绩单\n（全部指标）", self.open_report_window, "#7d5ba6"), 1, 1)
        grid.addWidget(big_button("速度曲线\n（目标速率 vs 臂速）", self.open_vel_window, "#0b3d91"), 2, 0, 1, 2)
        layout.addLayout(grid)
        self.rec_state = QtWidgets.QLabel("待机（手机拍屏幕：子窗口内效果最清晰）")
        self.rec_state.setWordWrap(True)
        self.rec_state.setStyleSheet("font-size:14px;padding:6px;")
        layout.addWidget(self.rec_state)
        self.rec_windows = {}
        self._rec_t = collections.deque(maxlen=9000)
        self._rec_a = collections.deque(maxlen=9000)
        self._rec_e = collections.deque(maxlen=9000)
        self._rec_th = collections.deque(maxlen=9000)
        self._rec_marks = []
        self._rec_hook = None
        self._rec_stats = {"duty_rev": 0, "prev_duty": None}

    # ---- 共享采样钩子 ----
    def _record_sample_hook(self, s):
        try:
            self._rec_t.append(s["tick"] / 1000.0)
            self._rec_a.append(s["alpha"] / 100.0)
            self._rec_e.append((s.get("theta_e") or 0) / 100.0)
            self._rec_th.append(s["theta"] / 100.0)
            d = s.get("duty_pre_dz")
            st = self._rec_stats
            if d is not None and st["prev_duty"] is not None:
                if (d > 0) != (st["prev_duty"] > 0) and d != 0:
                    st["duty_rev"] += 1
            if d is not None:
                st["prev_duty"] = d
        except Exception:
            pass

    def _ensure_hook(self):
        if self._rec_hook is None and self.bridge:
            self._rec_hook = self.bridge.on_sample(self._record_sample_hook)

    @staticmethod
    def _mk_win(title, size=(860, 560)):
        w = QtWidgets.QWidget()
        w.setWindowTitle(title)
        w.resize(*size)
        return w

    @staticmethod
    def _big_label(init=""):
        lbl = QtWidgets.QLabel(init)
        lbl.setStyleSheet("font-size:20px;font-weight:bold;")
        lbl.setWordWrap(True)
        return lbl

    # ---- 镜头1 子窗口：起摆 ----
    def open_swing_window(self):
        self._ensure_hook()
        self._clear_buffers()
        win = self._mk_win("镜头1 · 起摆展示")
        v = QtWidgets.QVBoxLayout(win)
        self.sw_label = self._big_label("点下方按钮开始起摆")
        v.addWidget(self.sw_label)
        p1 = pg.PlotWidget(title="摆杆 α (°)——实时")
        self.sw_curve = p1.plot(pen=pg.mkPen("#2a7fbd", width=2))
        p1.addLine(y=3, pen=pg.mkPen("#3a3", penStyle=QtCore.Qt.DashLine))
        p1.addLine(y=-3, pen=pg.mkPen("#3a3", penStyle=QtCore.Qt.DashLine))
        p1.setLabel("left", "α", units="°")
        p1.setXRange(0, 10, padding=0)
        v.addWidget(p1, stretch=1)
        self.sw_peak_text = pg.TextItem("", color="#c22", anchor=(1, 0))
        self.sw_peak_text.setPos(0.1, 0.9)
        p1.addItem(self.sw_peak_text)
        r = QtWidgets.QHBoxLayout()
        self.sw_start_btn = self.action("开始起摆（甩幅自适应）", self.start_swing)
        self.sw_start_btn.setMinimumHeight(44)
        self.sw_stop_btn = self.action("■ 急停", lambda: self.async_submit("command", (p.STOP, b"")))
        self.sw_stop_btn.setMinimumHeight(44)
        r.addWidget(self.sw_start_btn)
        r.addWidget(self.sw_stop_btn)
        r.addStretch()
        v.addLayout(r)
        self.rec_windows["swing"] = (win, p1)
        self._hold_win = win
        win.show()
        # 记住图件供刷新
        self._sw_plot = p1
        self._sw_curve = self.sw_curve
        self._sw_peak = self.sw_peak_text

    # ---- 镜头2 子窗口：抗扰 ----
    def open_disturb_window(self):
        self._ensure_hook()
        win = self._mk_win("镜头2 · 抗扰展示（稳摆后用手轻推摆杆）")
        v = QtWidgets.QVBoxLayout(win)
        self.dz_label = self._big_label("先完成起摆并稳摆，再点「打标」后轻推摆杆")
        v.addWidget(self.dz_label)
        p1 = pg.PlotWidget(title="摆杆 α (°)——打标后自动量恢复时间")
        self.dz_curve = p1.plot(pen=pg.mkPen("#2e8b57", width=2))
        p1.addLine(y=3, pen=pg.mkPen("#888", penStyle=QtCore.Qt.DashLine))
        p1.addLine(y=-3, pen=pg.mkPen("#888", penStyle=QtCore.Qt.DashLine))
        p1.setLabel("left", "α", units="°")
        v.addWidget(p1, stretch=1)
        r = QtWidgets.QHBoxLayout()
        self.dz_mark_btn = self.action("📍 扰动打标", self.mark_disturb)
        self.dz_mark_btn.setMinimumHeight(44)
        r.addWidget(self.dz_mark_btn)
        r.addStretch()
        v.addLayout(r)
        self.rec_windows["disturb"] = (win, p1)
        self._dz_plot = p1
        self._dz_curve = self.dz_curve
        self._dz_mark_lines = []
        self._dz_marks = []
        self._hold_win = win
        win.show()

    # ---- 镜头3 子窗口：定位 ----
    def open_pos_window(self):
        self._ensure_hook()
        win = self._mk_win("镜头3 · 定位步进（位置/速度/误差）", size=(980, 780))
        v = QtWidgets.QVBoxLayout(win)
        self.pos_label = self._big_label("稳摆/定位态下点步进按钮")
        v.addWidget(self.pos_label)
        self._pos_ref = None          # 相对坐标原点（窗口打开时的臂位）
        # P1 位置：目标阶梯 + 臂角 + 误差填色 + ±5° 绿廊
        p1 = pg.PlotWidget(title="位置（相对窗口起点，°）— 橙虚线=目标，蓝=臂角，绿廊=±5°")
        p1.showGrid(x=True, y=True, alpha=0.25)
        p1.setLabel("left", "°")
        self._pos_cor_hi = p1.plot(pen=pg.mkPen(None))
        self._pos_cor_lo = p1.plot(pen=pg.mkPen(None))
        p1.addItem(pg.FillBetweenItem(self._pos_cor_hi, self._pos_cor_lo,
                                      brush=pg.mkBrush(60, 180, 90, 45)))
        self._pos_curve_tg = p1.plot(
            pen=pg.mkPen("#c47f17", width=2, style=QtCore.Qt.DashLine))
        self._pos_curve_th = p1.plot(pen=pg.mkPen("#2a7fbd", width=2))
        p1.addItem(pg.FillBetweenItem(self._pos_curve_th, self._pos_curve_tg,
                                      brush=pg.mkBrush(196, 127, 23, 45)))
        v.addWidget(p1, stretch=3)
        # P2 速度：目标速率（4°/s 平台） vs 臂速
        p2 = pg.PlotWidget(title="速度（°/s）— 橙虚线=目标速率（速度曲线），蓝=臂速")
        p2.showGrid(x=True, y=True, alpha=0.25)
        p2.setLabel("left", "°/s")
        self._pos_rate_tg = p2.plot(
            pen=pg.mkPen("#c47f17", width=2, style=QtCore.Qt.DashLine))
        self._pos_rate_arm = p2.plot(pen=pg.mkPen("#2a7fbd", width=1))
        # 臂速 200ms 平滑（看"平均速度"是否跟住目标速率，慢摆噪声被压掉）
        self._pos_rate_smooth = p2.plot(pen=pg.mkPen("#0b3d91", width=2))
        self._pos_rate_ref = p2.plot(
            pen=pg.mkPen("#2e8b57", width=2, style=QtCore.Qt.DashLine))
        p2.setXLink(p1)
        v.addWidget(p2, stretch=2)
        # P3 误差：θe + ±5° 验收绿廊
        p3 = pg.PlotWidget(title="位置误差 θe（°）— 绿廊 = ±5° 验收带")
        p3.showGrid(x=True, y=True, alpha=0.25)
        p3.setLabel("left", "°")
        self._pos_band_hi = p3.plot(pen=pg.mkPen(None))
        self._pos_band_lo = p3.plot(pen=pg.mkPen(None))
        p3.addItem(pg.FillBetweenItem(self._pos_band_hi, self._pos_band_lo,
                                      brush=pg.mkBrush(60, 180, 90, 55)))
        self.pos_curve_e = p3.plot(pen=pg.mkPen("#7d5ba6", width=2))
        p3.addLine(y=0, pen=pg.mkPen("#888", penStyle=QtCore.Qt.DashLine))
        p3.setXLink(p1)
        v.addWidget(p3, stretch=2)
        r = QtWidgets.QHBoxLayout()
        r.addWidget(QtWidgets.QLabel("步幅"))
        self.rec_step = QtWidgets.QDoubleSpinBox()
        self.rec_step.setRange(1.0, 60.0)
        self.rec_step.setValue(5.0)
        self.rec_step.setSuffix(" °")
        r.addWidget(self.rec_step)
        self.rec_slow = QtWidgets.QCheckBox("慢档 +1.6s")
        r.addWidget(self.rec_slow)
        self.pos_go_btn = self.action("步进 →", self.step_move)
        self.pos_go_btn.setMinimumHeight(44)
        self.pos_back_btn = self.action("← 反向", self.step_move_back)
        self.pos_back_btn.setMinimumHeight(44)
        r.addWidget(self.pos_go_btn)
        r.addWidget(self.pos_back_btn)
        r.addWidget(QtWidgets.QLabel("绝对目标"))
        self.pos_abs_spin = QtWidgets.QDoubleSpinBox()
        self.pos_abs_spin.setRange(0.0, 359.9)
        self.pos_abs_spin.setDecimals(1)
        self.pos_abs_spin.setSuffix(" °")
        self.pos_abs_spin.setSingleStep(5.0)
        r.addWidget(self.pos_abs_spin)
        self.pos_goto_btn = self.action("去 →", self.goto_abs_clicked)
        self.pos_goto_btn.setMinimumHeight(44)
        self.pos_goto_btn.setToolTip("按 0-360° 绝对角度走最短路径，自动分小步并在每步附近稳住")
        r.addWidget(self.pos_goto_btn)
        self.pos_vel_btn = self.action("▶ 速度曲线 30°/s", self.toggle_vel_demo)
        self.pos_vel_btn.setMinimumHeight(44)
        self.pos_vel_btn.setToolTip("定位态下按固件内生三角波速度曲线旋转（30°/s、30°/s²）")
        r.addWidget(self.pos_vel_btn)
        self.pos_force_btn = self.action("⚡ 强制进 POS", self.step_force_pos)
        self.pos_force_btn.setMinimumHeight(44)
        self.pos_force_btn.setToolTip("稳摆态下绕过入场门直接进定位（|α|<15° 安全界）")
        r.addWidget(self.pos_force_btn)
        r.addStretch()
        v.addLayout(r)
        self.rec_windows["pos"] = (win, p1)
        self._pos_plot_th = p1
        self._pos_plot_e = p3
        self.pos_curve_th = self._pos_curve_th   # 兼容旧引用
        self._pos_curve_e = self.pos_curve_e
        self._pos_err_peak = 0.0
        self._hold_win = win
        win.show()

    # ---- 镜头5 子窗口：速度曲线（目标速率 vs 臂速）----
    def open_vel_window(self):
        """速度曲线：目标速率（固件 slew 曲线斜率）vs 臂速（瞬时+200ms 平滑）。"""
        self._ensure_hook()
        win = self._mk_win("镜头5 · 速度曲线（目标速率 vs 臂速）", size=(980, 700))
        v = QtWidgets.QVBoxLayout(win)
        self.vel_label = self._big_label("目标速率 — · 臂速（平滑） —")
        v.addWidget(self.vel_label)
        p1 = pg.PlotWidget(title="速度（°/s）— 橙虚线=目标速率，深蓝=臂速(200ms平滑)，浅蓝=瞬时")
        p1.showGrid(x=True, y=True, alpha=0.25)
        p1.setLabel("left", "°/s")
        self._vel_rate_tg = p1.plot(pen=pg.mkPen("#c47f17", width=2, style=QtCore.Qt.DashLine))
        self._vel_rate_raw = p1.plot(pen=pg.mkPen("#2a7fbd", width=1))
        self._vel_rate_smooth = p1.plot(pen=pg.mkPen("#0b3d91", width=2))
        v.addWidget(p1, stretch=3)
        p2 = pg.PlotWidget(title="位置（多圈 °，不回绕）— 橙=目标，蓝=臂角")
        p2.showGrid(x=True, y=True, alpha=0.25)
        p2.setLabel("left", "°")
        self._vel_tgt = p2.plot(pen=pg.mkPen("#c47f17", width=2))
        self._vel_th = p2.plot(pen=pg.mkPen("#2a7fbd", width=1))
        p2.setXLink(p1)
        v.addWidget(p2, stretch=2)
        p3 = pg.PlotWidget(title="摆杆 α（°）")
        p3.showGrid(x=True, y=True, alpha=0.25)
        self._vel_alpha = p3.plot(pen=pg.mkPen("#2e8b57", width=1))
        p3.setXLink(p1)
        v.addWidget(p3, stretch=1)
        self._vel_rate_ref = p1.plot(pen=pg.mkPen("#2e8b57", width=2, style=QtCore.Qt.DashLine))
        r = QtWidgets.QHBoxLayout()
        self.vel_go_btn = self.action("▶ 启动定速旋转", self.vel_start)
        self.vel_go_btn.setMinimumHeight(44)
        r.addWidget(self.vel_go_btn)
        self.vel_stop_btn = self.action("■ 停止", self.vel_stop)
        self.vel_stop_btn.setMinimumHeight(44)
        r.addWidget(self.vel_stop_btn)
        r.addStretch()
        v.addLayout(r)
        self.rec_windows["vel"] = (win, p1)
        self._hold_win = win
        win.show()

    def vel_start(self):
        """启动定速旋转（VEL_DEMO）：固件内生三角波 v_ref，倾斜环跟踪速度。"""
        bridge = self.bridge
        last = bridge.latest if bridge else None
        if not last:
            self.vel_label.setText("未连接设备"); return
        if last.get("mode") != 3:
            self.vel_label.setText("需先进入定位态（⚡ 强制进 POS）"); return
        vv = 200      # 20°/s（只作显示参考，实际转速由倾斜环与摩擦的平衡点决定）
        aa = 1        # 10°/s²
        try:
            bridge.demo_vel("gui", True, vv, aa)
        except Exception as exc:
            self.vel_label.setText("启动失败：" + str(exc)); return
        self.vel_label.setText("")

    def vel_stop(self):
        try:
            self.bridge.demo_vel("gui", False, 100, 1)
        except Exception:
            pass
        self.vel_label.setText("")

    # ---- 镜头4 子窗口：成绩单 ----
    def open_report_window(self):
        win = self._mk_win("汇总成绩单", size=(720, 480))
        v = QtWidgets.QVBoxLayout(win)
        self.rep_text = QtWidgets.QPlainTextEdit()
        self.rep_text.setReadOnly(True)
        self.rep_text.setStyleSheet("font-size:15px;")
        v.addWidget(self.rep_text, stretch=1)
        r = QtWidgets.QHBoxLayout()
        b1 = self.action("📊 生成", self.rec_report)
        b2 = self.action("复制", self.rec_copy_report)
        r.addWidget(b1)
        r.addWidget(b2)
        r.addStretch()
        v.addLayout(r)
        self._hold_win = win
        win.show()
        self.rec_report_box = self.rep_text
        self.rec_report()

    # ---- 各镜头逻辑（保留原语义）----
    def _clear_buffers(self):
        self._rec_t.clear(); self._rec_a.clear(); self._rec_e.clear()
        self._rec_th.clear()
        self._rec_marks.clear()
        self._rec_stats.update(duty_rev=0, prev_duty=None)

    def start_swing(self):
        bridge = self.bridge
        if not bridge or not bridge.connected:
            self.sw_label.setText("未连接设备")
            return
        self._ensure_hook()
        self._clear_buffers()
        try:
            bridge.demo_swing("gui")
        except Exception as exc:
            self.sw_label.setText("起摆未启动：" + str(exc))
            return
        self.sw_label.setText("起摆中：甩幅 8800（失败自动 +250 重试≤4 次，电压自适应）")

    def mark_disturb(self):
        if not self._rec_t:
            self.dz_label.setText("没有采样流：先在镜头1完成起摆")
            return
        self._rec_marks.append(len(self._rec_t) - 1)
        self._dz_marks.append(len(self._rec_t) - 1)
        if self._dz_plot:
            ln = self._dz_plot.addLine(x=list(self._rec_t)[-1], pen=pg.mkPen("#c33", width=2))
            self._dz_mark_lines.append(ln)
        self.dz_label.setText("扰动已打标：轻推后看 α 回 ±3°（图上自动标恢复点）")

    def step_move(self):
        bridge = self.bridge
        last = bridge.latest if bridge else None
        if not last or last.get("mode") not in (2, 3):
            self.pos_label.setText("需稳摆/定位态（先完成起摆）")
            return
        delta = int(round(self.rec_step.value() * 100))
        bridge.demo_move("gui", delta)
        gap = "，慢档 +1.6s" if self.rec_slow.isChecked() else ""
        self.pos_label.setText("步进 %+d° 已发%s——看 θe 收敛" % (delta / 100, gap))
        self._pos_err_peak = 0.0

    def step_move_back(self):
        bridge = self.bridge
        last = bridge.latest if bridge else None
        if not last or last.get("mode") not in (2, 3):
            self.pos_label.setText("需稳摆/定位态（先完成起摆）")
            return
        delta = -int(round(self.rec_step.value() * 100))
        bridge.demo_move("gui", delta)
        self.pos_label.setText("步进 %+d° 已发——看 θe 收敛" % (delta / 100))
        self._pos_err_peak = 0.0

    def step_force_pos(self):
        """稳摆态强制切 POS：绕过 mode_fsm 的 pos_ready 门（|α|<5°/|α̇|<30°/臂慢速），
        固件侧仅保留 |α|<15° 安全界。delta=0 = 就地保持（不额外挪臂）。"""
        bridge = self.bridge
        last = bridge.latest if bridge else None
        if not last:
            self.pos_label.setText("未连接设备")
            return
        if last.get("mode") == 3:
            self.pos_label.setText("已在定位态")
            return
        if last.get("mode") not in (2, 4):
            self.pos_label.setText("需稳摆态（先完成起摆）；当前 mode=%s" % last.get("mode"))
            return
        try:
            bridge.demo_force_pos("gui", 0)
        except Exception as exc:
            self.pos_label.setText("强制进 POS 失败：" + str(exc))
            return
        self._pos_ref = None      # 新定位会话：相对坐标原点重取
        self.pos_label.setText("⚡ 已发强制进 POS（delta 0，就地保持）")

    def toggle_vel_demo(self):
        """速度曲线演示开关：POS 态下发 VEL_DEMO（30°/s、30°/s² 三角波）。"""
        bridge = self.bridge
        last = bridge.latest if bridge else None
        if not last:
            self.pos_label.setText("未连接设备")
            return
        on = not getattr(self, "_vel_on", False)
        if on and last.get("mode") != 3:
            self.pos_label.setText("需先进入定位态（⚡ 强制进 POS）")
            return
        try:
            bridge.demo_vel("gui", on, 300, 3)
        except Exception as exc:
            self.pos_label.setText("速度演示失败：" + str(exc))
            return
        self._vel_on = on
        self.pos_vel_btn.setText("■ 停速度曲线" if on else "▶ 速度曲线 30°/s")
        self.pos_label.setText("速度曲线演示：%s（目标 30°/s，斜率 30°/s²，绿虚线=目标速度）"
                               % ("开" if on else "关"))

    # ---- 0-360° 绝对定位（最短路径 + 小步多次）----
    def goto_abs_clicked(self):
        """按 0-360° 绝对目标走：最短路径换算 → 分 ≤30° 小步 → 每步等到位再下一步。"""
        bridge = self.bridge
        last = bridge.latest if bridge else None
        if not last:
            self.pos_label.setText("未连接设备"); return
        if last.get("mode") != 3:
            self.pos_label.setText("需先进入定位态（⚡ 强制进 POS）"); return
        tgt = float(self.pos_abs_spin.value()) % 360.0
        th = last["theta"] / 100.0
        delta = (tgt - th + 180.0) % 360.0 - 180.0     # 最短路径
        chunks = []          # 绝对航点（θ_click + 累计步长），逐点用 MOVE_ABS 下发
        rem = delta; acc = 0.0
        while abs(rem) > 0.5:
            st = max(-30.0, min(30.0, rem))
            acc += st; chunks.append(th + acc); rem -= st
        if not chunks:
            self.pos_label.setText("已在该角度附近（差 %.1f°）" % delta); return
        self._goto_q = chunks
        self._goto_target = tgt
        self._goto_total = delta
        self._goto_trial = False     # 走镜头3 的 pos_label 提示通道
        self.pos_label.setText("去 %.1f°：最短路径 %+.1f°，分 %d 步（每步 ≤30°，4°/s 斜坡）"
                               % (tgt, delta, len(chunks)))
        self._goto_send_next()

    def rec_report(self):
        t = list(self._rec_t); a = list(self._rec_a); e = list(self._rec_e)
        if len(t) < 100:
            self.rep_text.setPlainText("样本不足（先做镜头）")
            return
        import numpy as _np
        arr_a = _np.array(a); arr_e = _np.array(e) / 100.0; arr_t = _np.array(t)
        lines = ["== 录制成绩单 ==", ""]
        lines.append("总时长 %.1fs" % (t[-1] - t[0]))
        lines.append("摆杆：αRMS %.2f° / α最大 %.1f°" % (float(arr_a.std()), float(_np.abs(arr_a).max())))
        lines.append("定位：|θe|最大 %.1f° / 中位 %.2f°" % (float(_np.abs(arr_e).max()), float(_np.median(_np.abs(arr_e)))))
        st = self._rec_stats
        lines.append("输出：duty 换向 %d 次（%.1f 次/s）"
                     % (st["duty_rev"], st["duty_rev"] / max(t[-1] - t[0], 1)))
        for i, mk in enumerate(self._rec_marks):
            t_mark = t[mk]
            pre = arr_a[(arr_t >= t_mark - 2) & (arr_t < t_mark)]
            rec_t = None; run = 0
            for j in range(mk, len(arr_t)):
                if abs(a[j]) < 3.0:
                    run += 1
                    if run >= 500:
                        rec_t = arr_t[j] - t_mark - 1.0
                        break
                else:
                    run = 0
            lines.append("扰动#%d @%.1fs：本底 %.2f° → 恢复 %s"
                         % (i + 1, t_mark,
                            float(pre.std()) if len(pre) else 0.0,
                            ("%.1fs" % rec_t) if rec_t is not None else "未恢复"))
        self._report_text = "\n".join(lines)
        try:
            self.rep_text.setPlainText(self._report_text)
        except RuntimeError:
            pass   # 成绩单子窗口已关闭（重开窗口再点生成即可）

    def rec_copy_report(self):
        text = getattr(self, "_report_text", "") or ""
        if text:
            QtWidgets.QApplication.clipboard().setText(text)
            self.rep_text.setPlainText(text + "\n\n（已复制到剪贴板）")

    def _video_ui_update(self):
        """所有打开的镜头子窗口统一刷新（40ms 主定时器）。
        数据源=bridge.history（桥线程实时填的遥测环），不依赖采样钩子——
        钩子链路（on_sample→_record_sample_hook）在部分连接时序下不触发，
        轮询方案无条件工作。"""
        bridge = self.bridge
        if not bridge or not bridge.connected:
            return
        hist = list(bridge.history)[-4500:]
        if len(hist) < 10:
            return
        xs = [s["tick"] / 1000.0 for s in hist]
        ys_a = [s["alpha"] / 100.0 for s in hist]
        ys_e = [(s.get("theta_e") or 0) / 100.0 for s in hist]
        ys_th = [s["theta"] / 100.0 for s in hist]
        # 镜头1：α 曲线 + 峰值标注
        if "swing" in self.rec_windows and self.rec_windows["swing"][0].isVisible():
            self._sw_curve.setData(xs, ys_a)
            if xs:
                self._sw_plot.setXRange(max(0, xs[-1] - 10), xs[-1] + 0.5, padding=0)
                peak = max(ys_a, key=abs)
                self._sw_peak.setPos(xs[-1] - 9.5, 55)
                self._sw_peak.setText("α峰 %.1f°\n当前 %.1f°" % (peak, ys_a[-1]))
            bridge = self.bridge
            rec = getattr(bridge, "_swing_rec", None) if bridge else None
            if rec:
                self.sw_label.setText(
                    "起摆中：第 %d 次，甩幅 %d（自适应）" % (rec["attempt"], rec["sw_amp"]))
            elif bridge is not None and bridge.latest and bridge.latest.get("mode") == 2:
                self.sw_label.setText("✓ 稳摆中（α 当前 %.1f°）——可进镜头2/3" % ys_a[-1])
        # 镜头2：α + 打标线与恢复点
        if "disturb" in self.rec_windows and self.rec_windows["disturb"][0].isVisible():
            self._dz_curve.setData(xs, ys_a)
        # 镜头5：速度曲线（目标速率 vs 臂速）
        if "vel" in self.rec_windows and self.rec_windows["vel"][0].isVisible():
            import numpy as _np
            ds = 2
            x = _np.asarray(xs[::ds], float)
            tgv = _np.asarray([(s.get("target") or 0) / 100.0 for s in hist[::ds]], float)
            tdv = _np.asarray([(s.get("theta_dot") or 0) / 10.0 for s in hist[::ds]], float)
            alv = _np.asarray([s["alpha"] / 100.0 for s in hist[::ds]], float)
            thv = _np.asarray([s["theta"] / 100.0 for s in hist[::ds]], float)
            if len(x) > 20:
                k = max(1, int(0.10 * 500 / ds))
                idx = _np.arange(len(tgv))
                j0 = _np.maximum(0, idx - k); j1 = _np.minimum(len(tgv) - 1, idx + k)
                dtv = x[j1] - x[j0]
                rt = _np.where(dtv > 1e-6, (tgv[j1] - tgv[j0]) / _np.maximum(dtv, 1e-6), 0.0)
                _w = max(1, min(50, len(tdv)))
                if _w > 1:
                    _pad = _w // 2
                    _yp = _np.pad(tdv, (_pad, _w - 1 - _pad), mode="edge")
                    sm = _np.convolve(_yp, _np.ones(_w) / _w, mode="valid")[:len(tdv)]
                else:
                    sm = tdv
                self._vel_rate_tg.setData(x, rt)
                self._vel_rate_raw.setData(x, tdv)
                self._vel_rate_smooth.setData(x, sm)
                # 位置图用多圈原值（不回绕）：连续旋转时曲线是连续的斜坡，直观
                self._vel_tgt.setData(x, tgv)
                self._vel_th.setData(x, thv)
                self._vel_alpha.setData(x, alv)
                vr = _np.asarray([(s.get("term_k1") or 0) / 10.0 for s in hist[::ds]], float)
                self._vel_rate_ref.setData(x, vr)
                self.rec_windows["vel"][1].setXRange(max(x[0], x[-1] - 14.0), x[-1] + 0.3, padding=0)
                # 顶部只留运行状态（用户要求：数值判据看曲线本身，别在左上角刷）

        # 镜头3：三段式（位置相对坐标 / 速度曲线 / 误差 + ±5° 绿廊）
        if "pos" in self.rec_windows and self.rec_windows["pos"][0].isVisible():
            import numpy as _np
            ds = 2                       # 抽稀到 250Hz 保刷新流畅
            x = _np.asarray(xs[::ds], float)
            thv = _np.asarray(ys_th[::ds], float)
            tgv = _np.asarray([(s.get("target") or 0) / 100.0 for s in hist[::ds]], float)
            tdv = _np.asarray([(s.get("theta_dot") or 0) / 10.0 for s in hist[::ds]], float)
            ev = _np.asarray(ys_e[::ds], float)
            if len(x) > 10:
                if getattr(self, "_pos_ref", None) is None:
                    self._pos_ref = float(thv[0])
                ref = self._pos_ref
                # 绝对角度 0-360°（解回绕）；臂按"离目标最近的一圈"显示，避免 0/360 跨相
                tg_d = _np.mod(tgv, 360.0)
                th_d = tg_d + _np.mod(thv - tgv + 180.0, 360.0) - 180.0
                # 目标速率：±100ms 中心差分（4°/s 平台会被清楚画出来）
                k = max(1, int(0.10 * 500 / ds))
                idx = _np.arange(len(tgv))
                j0 = _np.maximum(0, idx - k); j1 = _np.minimum(len(tgv) - 1, idx + k)
                dtv = x[j1] - x[j0]
                rt = _np.where(dtv > 1e-6, (tgv[j1] - tgv[j0]) / _np.maximum(dtv, 1e-6), 0.0)
                self._pos_curve_th.setData(x, th_d)
                self._pos_plot_th.setXRange(max(x[0], x[-1] - 12.0), x[-1] + 0.3, padding=0)
                self._pos_rate_arm.setData(x, tdv)
                _w = max(1, min(50, len(tdv)))
                if _w > 1:
                    _pad = _w // 2
                    _yp = _np.pad(tdv, (_pad, _w - 1 - _pad), mode='edge')
                    _sm = _np.convolve(_yp, _np.ones(_w) / _w, mode='valid')[:len(tdv)]
                else:
                    _sm = tdv
                self._pos_rate_smooth.setData(x, _sm)
                vref = _np.asarray([(s.get("term_k1") or 0) / 10.0 for s in hist[::ds]], float)
                self._pos_rate_ref.setData(x, vref)
                if hist[-1].get("mode") != 3:
                    # 非定位态：target 是上次 POS 的陈旧值（可能差几千度），
                    # te 也可能是 s16 钳位值——一律不画，避免出现荒唐量程。
                    empty = _np.array([])
                    for cu in (self._pos_curve_tg, self._pos_cor_hi, self._pos_cor_lo,
                               self._pos_rate_tg, self._pos_rate_ref, self.pos_curve_e,
                               self._pos_band_hi, self._pos_band_lo):
                        cu.setData(empty, empty)
                    self.pos_label.setText(
                        "非定位态（mode=%s）— 稳摆后点「⚡ 强制进 POS」或「步进 →」"
                        % hist[-1].get("mode"))
                else:
                    self._pos_curve_tg.setData(x, tg_d)
                    self._pos_cor_hi.setData(x, tg_d + 5.0)
                    self._pos_cor_lo.setData(x, tg_d - 5.0)
                    self._pos_rate_tg.setData(x, rt)
                    self.pos_curve_e.setData(x, ev)
                    self._pos_band_hi.setData(x, _np.full_like(x, 5.0))
                    self._pos_band_lo.setData(x, _np.full_like(x, -5.0))
                    n3 = min(len(ev), int(3.0 * 500 / ds))
                    mean3 = float(_np.mean(_np.abs(ev[-n3:]))) if n3 else 0.0
                    ok = abs(float(ev[-1])) <= 5.0
                    self.pos_label.setText(
                        "目标 %.1f° · 臂 %.1f° · 误差 %+.1f° %s ｜ 近3s |误差| %.1f° · 峰 %.1f°"
                        % (_np.mod(tg_d[-1], 360.0), _np.mod(th_d[-1], 360.0), ev[-1],
                           "✓" if ok else "✗",
                           mean3, float(_np.max(_np.abs(ev)))))

    # ---------- 烧录 ----------
    def build_flash(self):
        """烧录页：T20K（Gowin programmer_cli）与 DE10（Quartus quartus_pgm）。

        两条现场纪律写死在流程里：① T20K 烧录前必须断开串口连接（CH340 句柄
        开着时 JTAG 会挂死）；② SRAM 烧录用 --run 2 不校验，成功与否以烧后
        读回的 UserCode 与 .fs 头期望值是否一致为准。
        """
        layout = self.page("烧录")
        warn = QtWidgets.QLabel(
            "烧录前请确认目标板已上电、JTAG/Type-C 已接好，机械处于安全位置。"
            "T20K 烧录会先断开串口连接（CH340 句柄打开时 JTAG 会挂死），烧完请重新连接。"
            "SRAM 断电即失；要断电保留请用 Flash 固化。运行时参数不随位流固化。")
        warn.setWordWrap(True)
        warn.setStyleSheet("color:#9b1c1c;padding:8px;")
        layout.addWidget(warn)

        form = QtWidgets.QFormLayout()
        self.flash_board = QtWidgets.QComboBox()
        for key, label in fl.BOARDS.items():
            self.flash_board.addItem(label, key)
        self.flash_board.currentIndexChanged.connect(self.on_flash_board_changed)
        form.addRow("目标板", self.flash_board)
        self.flash_tool_label = QtWidgets.QLabel("检测中…")
        self.flash_tool_label.setWordWrap(True)
        form.addRow("烧录工具", self.flash_tool_label)
        layout.addLayout(form)

        row = QtWidgets.QHBoxLayout()
        self.flash_source = QtWidgets.QComboBox()
        self.flash_source.setMinimumWidth(300)
        self.flash_source.currentIndexChanged.connect(
            lambda *_: self.refresh_bitstreams())
        row.addWidget(QtWidgets.QLabel("来源"))
        row.addWidget(self.flash_source, stretch=1)
        open_dir = QtWidgets.QPushButton("打开所在目录")
        open_dir.clicked.connect(self.open_bitstream_dir)
        row.addWidget(open_dir)
        layout.addLayout(row)

        row = QtWidgets.QHBoxLayout()
        self.flash_bitstream = QtWidgets.QComboBox()
        self.flash_bitstream.setMinimumWidth(520)
        self.flash_bitstream.currentIndexChanged.connect(self.update_flash_sha)
        row.addWidget(QtWidgets.QLabel("比特流"))
        row.addWidget(self.flash_bitstream, stretch=1)
        refresh = QtWidgets.QPushButton("刷新列表")
        refresh.clicked.connect(self.refresh_bitstreams)
        row.addWidget(refresh)
        browse = QtWidgets.QPushButton("浏览文件…")
        browse.clicked.connect(self.browse_bitstream)
        row.addWidget(browse)
        layout.addLayout(row)
        self.flash_sha_label = QtWidgets.QLabel("")
        self.flash_sha_label.setStyleSheet("color:#5f7d95;")
        self.flash_sha_label.setWordWrap(True)
        layout.addWidget(self.flash_sha_label)

        row = QtWidgets.QHBoxLayout()
        self.flash_cable = QtWidgets.QComboBox()
        self.flash_cable.setEditable(True)
        self.flash_cable.setMinimumWidth(300)
        self.flash_cable.lineEdit().setPlaceholderText("线缆（默认 cable-index 1 = FT2CH）")
        self.flash_cable.addItem("默认（cable-index 1）", None)
        row.addWidget(QtWidgets.QLabel("JTAG 线缆"))
        row.addWidget(self.flash_cable, stretch=1)
        scan = QtWidgets.QPushButton("扫描线缆")
        scan.clicked.connect(self.scan_cables)
        row.addWidget(scan)
        self.flash_verify = QtWidgets.QCheckBox("Flash 含校验")
        self.flash_verify.setChecked(True)
        self.flash_verify.setToolTip(
            "Gowin Flash 固化默认带校验（--run 9）。若桥回读校验报假失败，"
            "可取消勾选改用 --run 8 只烧不校验")
        row.addWidget(self.flash_verify)
        layout.addLayout(row)

        actions = QtWidgets.QHBoxLayout()
        self.flash_sram_button = QtWidgets.QPushButton("烧录 SRAM（掉电即失）")
        self.flash_sram_button.clicked.connect(lambda: self.start_flash("sram"))
        self.flash_flash_button = QtWidgets.QPushButton("固化 Flash（断电保留）")
        self.flash_flash_button.clicked.connect(lambda: self.start_flash("flash"))
        self.flash_flash_button.setToolTip(
            "烧外部 SPI Flash（W25Q128 档，断电保留）。等价于高云 Programmer GUI 的\n"
            "External Flash Mode → Program；CLI 无器件选择项，靠 JEDEC ID 自动\n"
            "探测——作业会先跑只读 Detect ID 并在日志显示实际颗粒（本机板载为\n"
            "XTX 8MB 兼容片，W25Q128 档可烧）。")
        self.flash_detect_button = QtWidgets.QPushButton("检测外部 Flash")
        self.flash_detect_button.clicked.connect(lambda: self.start_flash("detect_flash"))
        self.flash_detect_button.setToolTip(
            "只读探测外部 SPI Flash 的 JEDEC ID 并显示器件名（--run 51，不擦不写）。")
        self.flash_read_button = QtWidgets.QPushButton("读取板上 UserCode")
        self.flash_read_button.clicked.connect(lambda: self.start_flash("read_usercode"))
        self.flash_cancel_button = QtWidgets.QPushButton("取消")
        self.flash_cancel_button.clicked.connect(self.cancel_flash)
        self.flash_cancel_button.setEnabled(False)
        for button in (self.flash_sram_button, self.flash_flash_button,
                       self.flash_detect_button, self.flash_read_button):
            actions.addWidget(button)
        actions.addStretch()
        actions.addWidget(self.flash_cancel_button)
        layout.addLayout(actions)

        self.flash_status = QtWidgets.QLabel("就绪")
        self.flash_status.setStyleSheet("padding:6px;background:#eef4fa;border-radius:4px;")
        layout.addWidget(self.flash_status)

        self.flash_log = QtWidgets.QPlainTextEdit()
        self.flash_log.setReadOnly(True)
        self.flash_log.document().setMaximumBlockCount(3000)
        self.flash_log.setStyleSheet("font-family:Consolas,monospace;")
        layout.addWidget(self.flash_log, stretch=1)

        self.refresh_bitstreams()
        self.on_flash_board_changed()

    def flash_board_key(self):
        return self.flash_board.currentData()

    def on_flash_board_changed(self, *_):
        """切板卡：换比特流后缀、按目标板检测工具、更新按钮可用性。"""
        board = self.flash_board_key()
        tool = fl.detect_tool(board)
        if tool["ok"]:
            self.flash_tool_label.setText(f"✔ {tool['note']}\n{tool['path']}")
            self.flash_tool_label.setStyleSheet("color:#176b3a;")
        else:
            self.flash_tool_label.setText("✘ " + tool["note"])
            self.flash_tool_label.setStyleSheet("color:#9b1c1c;")
        is_t20k = board == "t20k"
        self.flash_cable.setEnabled(is_t20k)
        self.flash_flash_button.setVisible(is_t20k)
        self.flash_detect_button.setVisible(is_t20k)
        self.flash_verify.setVisible(is_t20k)
        self.flash_read_button.setVisible(is_t20k)
        self.flash_sram_button.setText("烧录 SRAM（掉电即失）" if is_t20k else "烧录 FPGA（JTAG）")
        self.refresh_bitstreams()

    def refresh_bitstreams(self):
        """后台扫描构建目录填充比特流下拉；保留当前手选的文件。

        来源下拉按工作树构建目录（t20k_build/<工作树>/audit_*）分组——同一份
        列表里可能混着多个实验分支与主线历史的构建，先选来源再选版本；
        根下散装 audit_* 归为「主线/历史」。

        2026-11-01 改后台线程 + 增量缓存：全量枚举后冷缓存首扫实测 40+s
        （480 份×4KB 头读取 + 目录树遍历），放 UI 线程就是"点刷新卡死/启动
        拖慢"。扫描在工作线程跑（flasher.list_bitstreams_cached，只有新增/
        变动的比特流才重新解析 usercode），完成后经信号回填；连点刷新/切
        板时按代数丢弃过期结果。
        """
        current = self.flash_bitstream.currentData()
        want = self.flash_source.currentData() \
            if hasattr(self, "flash_source") and self.flash_source.count() else None
        self._flash_pending_sel = (current, want)
        self._flash_scan_gen += 1
        gen = self._flash_scan_gen
        board = self.flash_board_key()
        self.flash_source.blockSignals(True)
        self.flash_source.clear()
        self.flash_source.addItem("（扫描构建目录…）", None)
        self.flash_source.blockSignals(False)
        self.flash_bitstream.clear()
        self._flash_populating = True
        self.flash_bitstream.addItem("（扫描构建目录…）", None)
        self._flash_populating = False

        def work():
            try:
                entries = fl.list_bitstreams_cached(board)
            except Exception as exc:      # 扫描失败不让界面残废，可手选文件
                entries = []
                self.flash_signals.line.emit(f"✘ 比特流扫描失败：{exc}")
            self.flash_scan_signals.done.emit(gen, entries)

        threading.Thread(target=work, daemon=True, name="flash-scan").start()

    def _on_bitstreams_scanned(self, gen, entries):
        """扫描线程回填（Qt 信号已marshaling 到 UI 线程）。过期代数直接丢弃。"""
        if gen != self._flash_scan_gen:
            return
        self.flash_bitstreams = entries
        current, want = self._flash_pending_sel
        sources = []
        for entry in self.flash_bitstreams:
            if entry["source"] not in sources:
                sources.append(entry["source"])
        self.flash_source.blockSignals(True)
        self.flash_source.clear()
        self.flash_source.addItem(f"全部来源（{len(self.flash_bitstreams)} 份）", None)
        for src in sources:
            n = sum(1 for e in self.flash_bitstreams if e["source"] == src)
            name = src if src else "主线/历史"
            self.flash_source.addItem(f"{name}（{n} 份）", src)
        if want is not None:
            index = self.flash_source.findData(want)
            if index >= 0:
                self.flash_source.setCurrentIndex(index)
        self.flash_source.blockSignals(False)
        self.flash_bitstream.clear()
        self._flash_populating = True
        for entry in self.flash_bitstreams:
            if want is not None and entry["source"] != want:
                continue
            usercode = f" · UserCode {entry['usercode']:#x}" if entry["usercode"] else ""
            when = datetime.fromtimestamp(entry["mtime"]).strftime("%m-%d %H:%M")
            where = (entry["source"] + "/" if entry["source"]
                     else ("（主线）/" if self.flash_board_key() == "t20k" else ""))
            label = f"{where}{entry['build']}{usercode} · {when} · {entry['size']//1024} KB"
            self.flash_bitstream.addItem(label, entry["path"])
            self.flash_bitstream.setItemData(
                self.flash_bitstream.count() - 1, entry["path"],
                QtCore.Qt.ItemDataRole.ToolTipRole)
        if current:
            index = self.flash_bitstream.findData(current)
            if index >= 0:
                self.flash_bitstream.setCurrentIndex(index)
        if not self.flash_bitstream.count():
            self.flash_bitstream.addItem("（未找到比特流，请点「浏览文件…」）", None)
        self._flash_populating = False
        self.update_flash_sha()

    def update_flash_sha(self):
        """选中位流的 SHA256 对账标签（audit 流程同款），完整值进 tooltip。"""
        if getattr(self, "_flash_populating", False):
            return      # 批量填充时跳过逐项计算，结束后统一算一次
        path = self.flash_bitstream.currentData()
        if not path or not Path(path).is_file():
            self.flash_sha_label.setText("")
            return
        try:
            digest = fl.sha256_of(path)
        except OSError as exc:
            self.flash_sha_label.setText(f"SHA256 计算失败：{exc}")
            return
        self.flash_sha_label.setText(
            f"SHA256 {digest[:24]}…（完整值见 tooltip）· {Path(path)}")
        self.flash_sha_label.setToolTip(digest)

    def open_bitstream_dir(self):
        path = self.flash_bitstream.currentData()
        if not path or not Path(path).is_file():
            self.flash_status.setText("请先选择一个真实存在的比特流文件")
            return
        QtGui.QDesktopServices.openUrl(
            QtCore.QUrl.fromLocalFile(str(Path(path).parent)))

    def browse_bitstream(self):
        suffix = "Gowin 比特流 (*.fs)" if self.flash_board_key() == "t20k" else "Quartus (*.sof *.jic)"
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "选择比特流", str(fl.workspace_root()), suffix)
        if not path:
            return
        self.flash_bitstream.addItem(f"[手选] {Path(path).name}", path)
        self.flash_bitstream.setCurrentIndex(self.flash_bitstream.count() - 1)

    def scan_cables(self):
        if self.flasher.running:
            self.log("烧录：已有任务在跑")
            return
        if self.flash_board_key() != "t20k":
            return
        tool = fl.detect_tool("t20k")
        if not tool["ok"]:
            self.flash_status.setText("扫描失败：" + tool["note"])
            return
        steps = fl.make_job("t20k", tool["path"], action="scan_cables")
        self.flasher.start(steps, tag="scan_cables")
        self.flash_status.setText("正在扫描 JTAG 线缆…")
        self.update_flash_buttons()

    def start_flash(self, action):
        """烧录/SRAM/读取的统一入口：先做飞行前检查，再断串口，最后起线程。"""
        if self.flasher.running:
            self.log("烧录：已有任务在跑，请先等待或取消")
            return
        board = self.flash_board_key()
        tool = fl.detect_tool(board)
        if not tool["ok"]:
            self.flash_status.setText("无法烧录：" + tool["note"])
            return
        bitstream = self.flash_bitstream.currentData()
        if action in ("sram", "flash", "program") and not bitstream:
            self.flash_status.setText("请先选择比特流文件")
            return
        cable_location = self.flash_cable.currentData()
        try:
            steps = fl.make_job(board, tool["path"], bitstream, action=action,
                                cable_location=cable_location,
                                flash_verify=self.flash_verify.isChecked())
        except ValueError as exc:
            self.flash_status.setText("烧录参数错误：" + str(exc))
            return

        # T20K 的 JTAG 与 CH340 串口抢同一颗桥：串口句柄开着时 programmer_cli 会挂死。
        # 这里自动断开（可随时点「连接」恢复），不因忘记断开而白跑一次烧录。
        self._flash_disconnected = False
        if board == "t20k" and self.bridge:
            self.log("烧录：先断开串口连接（T20K 的 JTAG 与串口共用桥，句柄未关会挂死）")
            self.close_bridge()
            self._flash_disconnected = True

        # 只有真正写板时才拿位流的 UserCode 当期望值复核；单纯"读取"只是汇报
        # 板上现状，拿它跟下拉框里恰好选中的那个位流比对只会误导。
        expected = fl.parse_fs_usercode(bitstream) if (
            bitstream and action in ("sram", "flash", "program")) else None
        self.flash_log.clear()
        self.flasher.start(steps, expected_usercode=expected, tag="burn")
        action_name = {"sram": "SRAM 烧录", "flash": "Flash 固化",
                       "detect_flash": "检测外部 Flash",
                       "read_usercode": "读取 UserCode", "program": "FPGA 烧录"}[action]
        target = Path(bitstream).name if bitstream else "板上现有位流"
        self.flash_status.setText(f"进行中：{action_name} · {target}")
        self.log(f"烧录：开始{action_name}（{fl.BOARDS[board]}）")
        self.update_flash_buttons()

    def cancel_flash(self):
        if self.flasher.running:
            self.flasher.cancel()
            self.flash_status.setText("正在取消…")

    def update_flash_buttons(self):
        """烧录中禁用一切触发按钮并亮出取消；结束后恢复。"""
        running = self.flasher.running
        for button in (self.flash_sram_button, self.flash_flash_button,
                       self.flash_read_button):
            button.setEnabled(not running)
        self.flash_cancel_button.setEnabled(running)
        self.flash_board.setEnabled(not running)
        self.flash_source.setEnabled(not running)
        self.flash_bitstream.setEnabled(not running)

    def on_flash_line(self, text):
        self.flash_log.appendPlainText(text)

    def on_flash_done(self, result):
        """烧录结束：把 UserCode 复核结果翻译成一句人话，并记进主日志。"""
        self.update_flash_buttons()
        if result.get("tag") == "scan_cables":
            cables = fl.parse_cables("\n".join(self.flasher.lines))
            self.flash_cables = cables
            self.flash_cable.clear()
            self.flash_cable.addItem("默认（cable-index 1）", None)
            for cable in cables:
                self.flash_cable.addItem(
                    f"{cable['name']} · location {cable['location']}", cable["location"])
            if cables:
                self.flash_cable.setCurrentIndex(1)     # 扫到了就默认用第一根
                note = f"扫到 {len(cables)} 根线缆，已选用 {cables[0]['name']}"
                self.log("烧录：" + note)
            else:
                note = "未扫到 Gowin 线缆：检查 Type-C 是否插好，必要时重插"
                self.log("烧录：" + note)
            self.flash_status.setText(note)
            return
        if result.get("cancelled"):
            self.flash_status.setText("已取消")
            self.log("烧录：已取消")
            return
        if not result.get("ok"):
            self.flash_status.setText("失败：" + str(result.get("error")))
            self.log(f"烧录失败：{result.get('error')}（详见「烧录」页日志）")
            return
        match = result.get("usercode_match")
        if match is True:
            text = f"成功 · 板上 UserCode {result['usercode']:#06x} 与位流期望一致"
        elif match is False:
            text = (f"已写入但复核不一致：板上 {result['usercode']:#06x} ≠ "
                    f"期望 {result['expected_usercode']:#06x}——确认烧的是目标板/线缆")
        elif result.get("usercode") is not None:
            text = f"完成 · 板上 UserCode {result['usercode']:#06x}"
        else:
            text = "完成"
        self.flash_status.setText(f"{text}（{result['elapsed']:.1f}s）")
        self.log(f"烧录：{text}")
        # 烧录前为 T20K 断开了串口；烧完提示重连，免得用户以为连接坏了
        if getattr(self, "_flash_disconnected", False):
            self.log("烧录：串口连接已断开，调试前请点「连接」重新握手")
            self.flash_status.setText(f"{text}（{result['elapsed']:.1f}s）· 记得点「连接」重新握手")
            self._flash_disconnected = False

    # ---------- 连接 ----------
    def refresh_ports(self):
        """枚举系统串口填充下拉框；保留当前选择或手动输入的值。"""
        current = str(self.port.currentData() or self.port.currentText()).strip()
        self.port.clear()
        for device, label in list_serial_ports():
            self.port.addItem(label, device)
        if not self.port.count():
            self.port.addItem("（未检测到串口，可直接输入）", "")
        index = self.port.findData(current)
        if index >= 0:
            self.port.setCurrentIndex(index)
        elif current:
            self.port.setEditText(current)

    def auto_refresh_ports(self):
        """未连接且后端需要串口时定时重扫串口；下拉展开时不打扰选择。"""
        if self.bridge or self.backend.currentText() not in ("实板", "锁步HIL"):
            return
        if self.port.view().isVisible():
            return
        self.refresh_ports()

    def on_backend_changed(self, _index):
        if self.backend.currentText() in ("实板", "锁步HIL"):
            self.refresh_ports()

    def toggle_connection(self):
        if self.bridge:
            self.close_bridge()
            return
        kind = self.backend.currentText()
        location = str(self.port.currentData() or self.port.currentText()).strip()
        if kind == "回放" and not location:
            location, _ = QtWidgets.QFileDialog.getOpenFileName(
                self, "选择记录", str(ROOT / "logs"), "记录 (*.csv *.jsonl)")
            if not location:
                return
        if kind in ("实板", "锁步HIL") and not location:
            dev = ch340_device()
            if dev:
                location = dev
                self.log(f"未选串口：自动选用 CH340 {dev}（T20K 板载串口）")
        if kind == "锁步HIL" and not location:
            self.log("锁步HIL 需要串口：请选择 USB-TTL 对应的 COM 口")
            return
        self.port.setEditText(location)
        self.readonly = kind == "回放"
        self.settings.setValue("connection/backend", kind)
        self.settings.setValue("connection/port", location)
        self.bridge = Bridge(kind, location)
        self.bridge.on_sample(self.signals.sample.emit)
        self.bridge.on_event(self.signals.event.emit)
        if self.agent:
            self.agent.set_bridge(self.bridge)
        self.bridge.start()
        self.connect_button.setText("断开")
        where = f" · {location}" if location else ""
        self.banner.setText(f"{kind}{where} · 正在握手{'（只读回放）' if self.readonly else '与回读参数'}")

    def close_bridge(self):
        if not self.bridge:
            return
        if self.recording:
            # 断开即收尾：Bridge.close() 会补一条 record_end 摘要。
            # 这里先把界面状态复位，免得按钮停在"停止记录"上。
            self.recording = False
            self.record_button.setText("开始记录")
            self.log("记录随断开自动停止")
        self.io_jog_timer.stop()      # 点动自动释放定时器随连接一起作废
        if self.agent:
            self.agent.set_bridge(_NullBridge())
        self.bridge.close()
        self.bridge = None
        self._hinted = False
        self.capabilities = 0
        self.encoder_mark = None
        self.io_status_data = None
        self.io_encoder_zero = None
        self.diag.reset()
        self.watch.reset()
        self.mt6.reset()
        self.mt68.reset()
        self._fault_edge_seen = False
        self._link_prev = None
        self._az_offset = None
        self._diag_tick = -1
        if self.recorder.active:
            rows = self.recorder.close()
            self.log(f"诊断记录已保存（断连自动停止）：{self.recorder.path}，{rows} 行")
            self.diag_rec_button.setText("开始记录 CSV")
        self.diag_capturing = False
        if hasattr(self, "diag_source_label"):
            self.diag_source_label.setText("未连接")
            self.diag_theta_label.setText("—")
            self.diag_delta_label.setText("—")
            self.diag_rate_label.setText("—")
            self.diag_status_label.setText("未采集")
        for label in ("mt6_mg_label", "mt6_state_label", "mt6_rate_label",
                      "mt6_err_label", "mt6_angle_label"):
            widget = getattr(self, label, None)
            if widget is not None:
                widget.setText("未查询")
        if hasattr(self, "io_encoder_label"):
            self.io_encoder_label.setText("未查询")
            self.io_adc_label.setText("未查询")
            self.io_output_label.setText("未查询")
        for curve in self.curves:      # 清掉上一段连接留下的轨迹，避免误读
            curve.setData([], [])
        self.banner.setText("未连接")
        self.banner.setStyleSheet(
            "background:#16334a;color:#edf6fc;padding:9px 12px;border-radius:5px;")
        self._banner_style = None
        self._connect_state = None
        if hasattr(self, "mode_card_val"):
            self.mode_card_val.setText("未连接")
            self.mode_card_val.setStyleSheet(
                "font-size:15px;font-weight:600;color:#7a8ba0;")
            self.mode_card_sub.setText("连接设备后显示实时状态")
            for label in (self.alpha_card_val, self.theta_card_val,
                          self.target_card_val, self.duty_card_val):
                label.setText("—")
                label.setStyleSheet(
                    "font-size:24px;font-weight:700;color:#b7c4d2;")
            self._value_colors = {}
            self.duty_bar.set_value(0)
        self.connect_button.setText("连接")
        self.model.reset_trail()
        self.action_state(False)

    def on_event(self, entry):
        kind = entry.get("type")
        if kind in ("message", "failed"):
            text = entry.get("text", "")
            self.log(("失败：" if kind == "failed" else "") + text)
            if kind == "failed":
                self.close_bridge()
            elif text.startswith("已连接"):
                self.capabilities = int(
                    ((self.bridge.identity if self.bridge else None) or {}).get("capabilities", 0))
                self.log_identity()
                if not self._hinted:
                    self._hinted = True
                    if self.model_3d:
                        self.log("提示：模型页可拖动摆臂端下发定位，F9 急停")
        elif kind == "swing_ok":
            self.log("镜头完成：" + entry.get("text", ""))
            self.rec_state.setText("✓ " + entry.get("text", ""))
        elif kind == "battery":
            self.log(("电量探针：" if entry.get("ok") else "电量探针无效：")
                     + entry.get("text", ""))
        elif kind == "failed" and "起摆" in entry.get("text", ""):
            self.log("失败：" + entry.get("text", ""))
            self.rec_state.setText("✗ " + entry.get("text", "") + "（检查电池/机械后重试）")
        elif kind == "trial":
            self.log("试验结果：" + json.dumps(entry, ensure_ascii=False))
            verdict = "通过" if entry.get("pass") else "未通过"
            self.trial_result.setText(
                f"最近一次结果（{verdict}）：Δ{entry.get('delta', 0)/100:+.2f}°  "
                f"移动中摆杆峰值={entry.get('moving_alpha')}°（判据 <3°）  "
                f"末段误差={entry.get('late_error')}°（判据 <2°）")

    def log_identity(self):
        """把设备自报的板标识与能力位记进日志——这是"这台设备会什么"的唯一依据。"""
        identity = (self.bridge.identity if self.bridge else None) or {}
        if not identity:
            return
        caps = identity.get("capabilities", 0)
        detail = "、".join(identity.get("cap_names") or []) or "无"
        # 构建时刻优先用真实时间戳（DEBUG_BUILD_EPOCH）；identity["date"] 是
        # 写死的常量，标出来免得被当成版本日期。
        built = identity.get("built")
        if built:
            age = identity.get("built_age_s")
            age_txt = ""
            if isinstance(age, (int, float)):
                age_txt = (f"，{age/60:.0f} 分钟前" if age >= 60
                           else f"，{age:.0f} 秒前")
            stamp = f"构建于 {built}{age_txt}"
        else:
            stamp = f"构建时间未知（固件未报，build 戳日期 {identity.get('date', '?')} 是常量）"
        self.log(f"设备：{identity.get('board', '未知板')} · {stamp} · "
                 f"能力 {caps:#010x}（{detail}）")
        missing = [label for bit, label in ((p.CAP_MOTION, "动作按钮"), (p.CAP_PARAMS, "参数按钮"),
                                            (p.CAP_TELEMETRY, "遥测/曲线"))
                   if not p.has_capability(caps, bit)]
        if missing:
            self.log("本设备不具备：" + "、".join(missing) + "——对应控件已禁用")

    # ---------- 命令 ----------
    def request_balance(self):
        """待机手扶直立直接稳摆，或从定位返回稳摆；固件会再次校验门限。"""
        bridge = self.bridge
        latest = bridge.latest if bridge and bridge.connected else None
        if latest is None:
            self.log("手扶稳摆失败：尚未连接或暂无遥测")
            return
        mode = latest["mode"]
        if latest["fault"]:
            self.log("手扶稳摆失败：设备存在故障，请先清除故障")
            return
        if mode == 2:
            self.log("设备已经处于稳摆状态")
            return
        if mode == 0:
            failed = []
            if abs(latest["alpha"]) >= 500:
                failed.append(f"|α|={abs(latest['alpha'])/100:.2f}°（需<5°）")
            if abs(latest["alpha_dot"]) >= 300:
                failed.append(f"|α̇|={abs(latest['alpha_dot'])/10:.1f}°/s（需<30°/s）")
            if abs(latest["theta_dot"]) >= 200:
                failed.append(f"|θ̇|={abs(latest['theta_dot'])/10:.1f}°/s（需<20°/s）")
            if failed:
                self.log("手扶稳摆尚未就绪：" + "；".join(failed))
                return
        elif mode != 3:
            self.log(f"稳摆命令不适用于当前状态：{MODES.get(mode, mode)}；请先停止并清除到待机")
            return
        self.command(p.BALANCE)

    def command(self, kind, data=b""):
        self.async_submit("command", (kind, data))

    def async_submit(self, name, value=None, source="gui"):
        if not self.bridge:
            self.log("尚未连接")
            return
        if self.readonly and name not in ("record",):
            self.log("回放只读")
            return
        try:
            result = self.bridge.submit(name, value, source=source)
        except Exception as exc:
            self.log("失败：" + str(exc))
            return
        if name in ("read", "apply", "defaults") and result is not None:
            self.params_updated(result)
        elif name == "io_status" and result is not None:
            self.io_status_received(result)

    def params_updated(self, values):
        for row, (param, value) in enumerate(zip(PARAMS, values)):
            if value is None:      # 设备拒绝该 id（固件参数表与上位机不一致）
                self.active_items[row].setText("设备拒绝读取")
                continue
            self.active_items[row].setText(f'{value/param["scale"]:.6g}  [{value}]')
            # 编辑框跟随设备回读值：固件可为每个板覆盖默认值（tools/
            # generate_debug_config.py 的 BOARD_DEFAULT_OVERRIDES —— T20K 就覆盖了
            # K2/K3/K4/死区/捕获角），而 host 参数表只有通用设计默认值。不同步的
            # 话"编辑值"列一直显示过期默认值，用户会误以为设备跑的是那些数；
            # 一旦提交还会把板级覆盖冲掉。回读即以设备为准。
            editor = self.editors[row]
            blocked = editor.blockSignals(True)
            editor.setValue(value / param["scale"])
            editor.blockSignals(blocked)
            self.pending_items[row].setText(str(value))

    def action_state(self, running):
        ready = bool(running and self.bridge and self.bridge.active_params and not self.readonly)
        caps = self.capabilities if ready else 0
        # 控件按设备自报的能力位启用：能力位是"这台设备会什么"的唯一依据，
        # 不看板名。缺哪一项就禁用哪一组，而不是让用户点了才报错。
        motion = ready and p.has_capability(caps, p.CAP_MOTION)
        params = ready and p.has_capability(caps, p.CAP_PARAMS)
        calib = ready and p.has_capability(caps, p.CAP_CALIB_DC)
        io_test = ready and p.has_capability(caps, p.CAP_IO_TEST)
        live = []
        for button in self.action_widgets:
            try:
                button.setEnabled(motion)
                live.append(button)
            except RuntimeError:
                pass          # C++ 对象已删（子窗口按钮等），剔除防刷屏
        self.action_widgets = live
        self.stop_button.setEnabled(ready)      # 停止是安全动作，不因能力位收窄
        for button in self.param_widgets:
            button.setEnabled(params)
        for button in self.calib_widgets:
            button.setEnabled(calib)
        for button in self.io_test_widgets:
            button.setEnabled(io_test)
        if hasattr(self, "io_capability"):
            self.io_capability.setText(
                "设备支持 I/O 联调" if io_test else "设备未声明 I/O 联调能力，请先更新 FPGA 固件")
            self.io_capability.setStyleSheet("color:#176b3a;" if io_test else "color:#9b1c1c;")

    def toggle_record(self):
        if not self.bridge:
            return
        if self.recording:
            self.async_submit("record", None)
            self.recording = False
        else:
            logs = Path(__file__).resolve().parents[1] / "logs"
            # 无扩展名基础名：桥写成 <基础名>.csv + <基础名>_events.jsonl 两个文件
            path = logs / datetime.now().strftime("%Y%m%d_%H%M%S")
            try:
                self.async_submit("record", str(path))
            except Exception as exc:
                self.log("失败：" + str(exc))
                return
            self.recording = True
        self.record_button.setText("停止记录" if self.recording else "开始记录")

    # ---------- Agent 接口 ----------
    def toggle_agent(self):
        if self.agent:
            self.agent.close()
            self.agent = None
            self.agent_button.setText(f"启动Agent接口(:{AGENT_DEFAULT_PORT})")
            self.log("Agent 接口已关闭")
            return
        self.agent = AgentServer(self.bridge or _NullBridge(), port=AGENT_DEFAULT_PORT)
        try:
            self.agent.start()
        except OSError as exc:
            self.log("Agent 接口启动失败：" + str(exc))
            self.agent = None
            return
        self.agent_button.setText(f"关闭Agent接口(:{self.agent.port})")
        self.log(f"Agent 接口已开启：127.0.0.1:{self.agent.port}（python -m j280wb.cli）")

    # ---------- 刷新 ----------
    def refresh(self):
        if self.bridge and not self.bridge.alive:
            self.close_bridge()      # 桥线程已退出（含命令致断），界面回到未连接，可一键重连
        if hasattr(self, '_video_ui_update'):
            try:
                self._video_ui_update()
            except Exception:
                pass
        running = bool(self.bridge and self.bridge.connected)
        self.action_state(running)
        if not self.bridge:
            state = "idle"
        elif running:
            state = "on"
        elif self.bridge.error is None:
            state = "wait"       # 握手期；点击=取消
        else:
            state = "idle"
        if state != getattr(self, "_connect_state", None):
            self._connect_state = state
            self.connect_button.setStyleSheet(
                "QPushButton{background:%s;color:white;font-weight:600;"
                "border:1px solid %s;border-radius:5px;padding:6px 18px;}"
                "QPushButton:hover{background:%s;}" % {
                    "idle": ("#0e7ea6", "#0c6e92", "#1487b3"),
                    "on": ("#2f8a54", "#277548", "#379a62"),
                    "wait": ("#b06a20", "#96591a", "#c07828"),
                }[state])
        self.connect_button.setText(
            "连接" if state == "idle" else ("断开" if state == "on" else "连接中…"))
        latest = self.bridge.latest if self.bridge else None
        if latest:
            # 传感诊断：监视消息常开入日志；θ 健康统计增量喂入（不因切页漏采）
            for text in self.watch.feed(latest):
                self.log(text)
            if self._link_prev is None:
                self._log_link_counters("连接基线")
            if latest["fault"] and not self._fault_edge_seen:
                self._log_link_counters("锁存现场")
            self._fault_edge_seen = bool(latest["fault"])
            history = self.bridge.history
            if history:
                samples = list(history)
                start = 0
                for idx in range(len(samples) - 1, -1, -1):
                    if samples[idx]["tick"] <= self._diag_tick:
                        start = idx + 1
                        break
                for s in samples[start:]:
                    self.diag.feed(s)
                    self.recorder.feed(s)
                    off = s.get("offset")
                    if off is not None and self._az_offset is not None \
                            and abs(off - self._az_offset) >= 5:
                        d = off - self._az_offset
                        self.log(
                            f"ℹ α 零位更新：{self._az_offset/100:+.2f}° → {off/100:+.2f}°"
                            f"（Δ{d/100:+.2f}°）@ θ={s['theta']/100:+.1f}° tick={s['tick']}"
                            f"——自校准提交或人工标定；(θ, 零位) 即全段 α 标定数据点")
                        self.record_cal_point(s, self._az_offset, off)
                    if off is not None:
                        self._az_offset = off
                    self._cal_prev_mode = s["mode"]
                self._diag_tick = samples[-1]["tick"]
            lease = self.bridge.lease()
            board = (self.bridge.identity or {}).get("board")
            self.banner.setText(
                f"{self.bridge.kind}" + (f" · {board}" if board else "")
                + f" · {MODES.get(latest['mode'],'?')}"
                f" · 配置 v{latest['revision']} · "
                + ("故障/待确认" if latest["fault"] else "无锁存故障")
                + (f" · Agent控制:{lease['owner']}({lease['remaining']:.0f}s)" if lease else "")
                + f" · t={latest['tick']/1000:.2f}s")
            banner_style = ("background:"
                            + ("#b42333" if latest["fault"]
                               else MODE_CHIP_BG.get(latest["mode"], "#16334a"))
                            + ";color:#ffffff;padding:9px 12px;border-radius:5px;"
                              "font-weight:600;")
            if banner_style != getattr(self, "_banner_style", None):
                self._banner_style = banner_style
                self.banner.setStyleSheet(banner_style)
            self._update_overview(latest)
            if self.on_page("标定"):
                self.sensor_label.setText(
                    f"ADC={latest['adc']}    编码器={latest['encoder']}    "
                    f"下垂零偏={latest['offset']/100:.2f}°    "
                    f"标定完成={bool(latest['calib_done'])}")
                self.calib_offset_label.setText(
                    f"{latest['offset']/100:+.2f}°（{latest['offset']} c01）"
                    f" · CAL 完成：{'是' if latest['calib_done'] else '否'}"
                    f" · 稳摆退出时自动修正（限幅 ±5°，断电回默认 8344）")
                if self.encoder_mark is not None:
                    self.encoder_label.setText(
                        f"从基准点转角差：{(latest['theta'] - self.encoder_mark)/100:+.2f}°"
                        f"（θ 口径 0.01°，MT6701 多圈累加）")
        if self.on_page("曲线"):
            self.update_curves()
        if latest and self.on_page("传感诊断"):
            self.update_diag(latest)

    def closeEvent(self, event):
        if self.agent:
            self.agent.close()
        if self.bridge:
            self.bridge.close()
        if self.flasher:
            self.flasher.close()      # 终止在跑的烧录子进程，避免留下孤儿进程
        event.accept()


class _NullBridge:
    """Agent 接口允许在连接前开启（可先 acquire 排队）；未连接时命令自然失败。"""
    def __init__(self):
        self.kind = "未连接"
        self.label = ""
        self.latest = None
        self.history = []
        self.readonly = True
        self.connected = False
        self.revision = 0
        self.identity = {}
        self._lock = __import__("threading").Lock()
        self._event_hooks = []

    def on_sample(self, callback):
        return lambda: None

    def lease(self):
        return None

    def acquire_lease(self, owner, seconds=30):
        return False

    def renew_lease(self, owner, seconds=30):
        return False

    def release_lease(self, owner):
        return True

    def submit(self, *args, **kwargs):
        raise ConnectionError("尚未连接设备")


def main(smoke_path=None):
    app = QtWidgets.QApplication([])
    window = MainWindow()
    window.show()
    if smoke_path:
        window.backend.setCurrentText("模拟")
        window.toggle_connection()

        def exercise():
            window.command(p.CLEAR)
            window.command(p.START)
            window.tabs.setCurrentWidget(window.tab_widgets["模型"])
        QtCore.QTimer.singleShot(800, exercise)

        def finish():
            from host.schema import ROOT
            out = Path(smoke_path)
            out.parent.mkdir(parents=True, exist_ok=True)
            extra = []
            # CH340 置顶排序：纯函数级验证（有 CH340 描述的排前组）
            from types import SimpleNamespace
            ch340 = SimpleNamespace(device="COM4", description="USB-SERIAL CH340")
            ftdi = SimpleNamespace(device="COM3", description="USB Serial Port")
            assert port_sort_key(ch340)[0] == 0 and port_sort_key(ftdi)[0] == 1
            assert port_sort_key(ch340)[0] < port_sort_key(ftdi)[0]

            def goto(title):
                # 按页名导航：页签顺序随功能插入变化，硬编码索引曾整段错位
                # （诊断页插入后 _trial/_flash 截图拍到的都是别的页）。
                window.tabs.setCurrentWidget(window.tab_widgets[title])
                window.refresh()

            # 曲线页取数：守住 2026-09-23 修掉的"建了图不喂数据"缺陷
            goto("曲线")
            xs, _ = window.curves[0].getData()
            assert xs is not None and len(xs) > 100, f"曲线页没有取到数据：{xs}"
            shot = out.with_name(out.stem + "_curve.png")
            window.grab().save(str(shot))
            extra.append(str(shot))
            # 标定页：零位自校准面板 + 原始传感器只读观察
            goto("标定")
            assert "ADC=" in window.sensor_label.text(), "标定页未刷新传感器读数"
            assert "c01" in window.calib_offset_label.text(), "标定页零位面板未刷新"
            assert window.cal_points_table.columnCount() == 7, "标定页数据点表缺失"
            shot = out.with_name(out.stem + "_calib.png")
            window.grab().save(str(shot))
            extra.append(str(shot))
            # 硬件联调页只截图；实板原始 I/O 读取由硬件验收手动执行
            goto("硬件联调")
            assert hasattr(window, "io_timed_button"), "硬件联调页缺定时点动"
            shot = out.with_name(out.stem + "_hardware.png")
            window.grab().save(str(shot))
            extra.append(str(shot))
            # 传感诊断页：滚动包裹布局 + 实时标签（页可见才会喂 update_diag）
            goto("传感诊断")
            assert window.diag_rate_label.text() != "—", "传感诊断页未刷新实时读数"
            assert hasattr(window, "diag_auto"), "传感诊断页缺自动链路轮询开关"
            shot = out.with_name(out.stem + "_diagnostic.png")
            window.grab().save(str(shot))
            extra.append(str(shot))
            # 自动试验页（此处只截图，不跑 25 秒流程；流程由 logs/trial_smoke.py 覆盖）
            goto("自动试验")
            shot = out.with_name(out.stem + "_trial.png")
            window.grab().save(str(shot))
            extra.append(str(shot))
            # 烧录页：只截图，不触发任何烧录（真实烧录需板子在位，绝不进冒烟）
            goto("烧录")
            assert window.flash_bitstream.count() > 0, "烧录页未扫描到比特流列表"
            assert window.flash_source.count() >= 1, "烧录页来源下拉未填充"
            assert window.flash_source.itemText(0).startswith("全部来源"), \
                "烧录页来源下拉首项应为全部来源"
            assert not window.flasher.running, "冒烟不应触发烧录任务"
            shot = out.with_name(out.stem + "_flash.png")
            window.grab().save(str(shot))
            extra.append(str(shot))
            goto("模型")      # 主截图仍留在模型页
            window.grab().save(str(out))
            last = window.bridge.latest if window.bridge else None
            assert last and last["mode"] in (2, 3), f"smoke 状态异常: {last}"
            print(json.dumps({"status": "PASS", "mode": last["mode"],
                              "curve_points": len(xs), "shot": str(out),
                              "extra_shots": extra}, ensure_ascii=True))
            window.close()
            app.quit()
        QtCore.QTimer.singleShot(6500, finish)
    app.exec()


if __name__ == "__main__":
    import sys
    main(sys.argv[1] if len(sys.argv) > 1 else None)
