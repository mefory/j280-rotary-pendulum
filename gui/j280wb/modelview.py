"""J280 实物模型实时动画：等轴测投影的伪 3D 绘制。

结构按套件实物（见 intro_p18.png）：底座电机（垂直长轴）→水平摆臂（θ，
俯仰平面旋转）→臂末端角位移传感器→摆杆（α，相对竖直方向）。附加元素：
臂端轨迹（地面投影）、POS 目标方向虚线、倒立参考线、占空比表、模式灯。
数据来源只有遥测（θ 连续角、α、duty、mode、target），不做任何控制假设。
"""
import math
from collections import deque

from PySide6 import QtCore, QtGui, QtWidgets

YAW = math.radians(33)
SIN_Y, COS_Y = math.sin(YAW), math.cos(YAW)
ELEV = math.radians(24)
SIN_E, COS_E = math.sin(ELEV), math.cos(ELEV)
ARM_L = 1.0
ROD_L = 1.18
BASE_R = 0.42
MOTOR_H = 0.30

MODE_COLORS = {0: "#5f7d95", 1: "#e08a3c", 2: "#63c178", 3: "#54b8dc", 4: "#d4545f"}
MODE_TEXT = {0: "待机", 1: "自动起摆", 2: "稳摆", 3: "定位", 4: "安全停止"}


def project(x, y, z):
    sx = x * COS_Y - y * SIN_Y
    sy = (x * SIN_Y + y * COS_Y) * SIN_E - z * COS_E
    return sx, -sy


class ModelView(QtWidgets.QWidget):
    """set_sample() 由 GUI 定时喂入；paintEvent 全量重绘。"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(520, 420)
        self.sample = None
        self.trail = deque(maxlen=360)
        self.setAutoFillBackground(True)
        pal = self.palette()
        pal.setColor(QtGui.QPalette.ColorRole.Window, QtGui.QColor("#101b28"))
        self.setPalette(pal)

    @QtCore.Slot(dict)
    def set_sample(self, sample):
        self.sample = sample
        self.update()

    def reset_trail(self):
        self.trail.clear()
        self.update()

    # ---------- 几何 ----------
    def _pt(self, x, y, z):
        sx, sy = project(x, y, z)          # sy 为“屏幕向上”分量
        return QtCore.QPointF(self._cx + sx * self._scale, self._cy - sy * self._scale)

    def _arm_end3(self, theta):
        return ARM_L * math.cos(theta), ARM_L * math.sin(theta), MOTOR_H

    def paintEvent(self, _):
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        rect = self.rect()
        self._cx = rect.center().x()
        self._cy = rect.height() * 0.62
        self._scale = min(rect.width() / 3.4, rect.height() / 2.7)
        bg = QtGui.QColor("#101b28")
        painter.fillRect(rect, bg)

        s = self.sample or {}
        theta = s.get("theta", 0) / 100 * math.pi / 180
        alpha = s.get("alpha", 0) / 100 * math.pi / 180
        target = s.get("target", 0) / 100 * math.pi / 180
        mode = s.get("mode", 0)
        fault = s.get("fault", 0)

        if s:
            ex, ey, _ = self._arm_end3(theta)
            self.trail.append((ex, ey))
        self._draw_ground(painter)
        self._draw_target(painter, target, mode == 3)
        self._draw_trail(painter)
        self._draw_base(painter)
        self._draw_arm_rod(painter, theta, alpha)
        self._draw_hud(painter, s, mode, fault)

    def _draw_ground(self, painter):
        pen = QtGui.QPen(QtGui.QColor(90, 116, 140, 60), 1)
        painter.setPen(pen)
        r = 1.9
        for gx in range(-2, 3):
            painter.drawLine(self._pt(gx * r / 2, -r, 0), self._pt(gx * r / 2, r, 0))
            painter.drawLine(self._pt(-r, gx * r / 2, 0), self._pt(r, gx * r / 2, 0))

    def _draw_trail(self, painter):
        if len(self.trail) < 2:
            return
        points = list(self.trail)
        step = max(1, len(points) // 90)
        for i in range(step, len(points), step):
            x0, y0 = points[i - step]
            x1, y1 = points[i]
            fade = i / len(points)
            pen = QtGui.QPen(QtGui.QColor(110, 220, 232, int(30 + 150 * fade)), 2)
            painter.setPen(pen)
            painter.drawLine(self._pt(x0, y0, 0.012), self._pt(x1, y1, 0.012))

    def _draw_target(self, painter, target, active):
        if not active:
            return
        pen = QtGui.QPen(QtGui.QColor("#ffbd69"), 2, QtCore.Qt.PenStyle.DashLine)
        painter.setPen(pen)
        painter.drawLine(self._pt(0, 0, 0.02),
                         self._pt(1.32 * math.cos(target), 1.32 * math.sin(target), 0.02))
        painter.setBrush(QtGui.QColor("#ffbd69"))
        painter.drawEllipse(self._pt(ARM_L * math.cos(target), ARM_L * math.sin(target), 0.02), 5, 5)

    def _draw_base(self, painter):
        painter.setPen(QtGui.QPen(QtGui.QColor("#22384d"), 2))
        painter.setBrush(QtGui.QColor("#1a2c3f"))
        c = self._pt(0, 0, 0)
        rx, ry = BASE_R * self._scale, BASE_R * self._scale * SIN_E
        painter.drawEllipse(c, rx, ry)
        painter.setBrush(QtGui.QColor("#243b52"))
        painter.drawEllipse(self._pt(0, 0, MOTOR_H), rx * 0.62, ry * 0.62)
        painter.setPen(QtGui.QPen(QtGui.QColor("#334f68"), 1))
        for dx, dy in ((-1, 0), (1, 0)):
            painter.drawLine(self._pt(dx * BASE_R * 0.62, dy * BASE_R * 0.62, 0),
                             self._pt(dx * BASE_R * 0.62, dy * BASE_R * 0.62, MOTOR_H))

    def _draw_arm_rod(self, painter, theta, alpha):
        ex, ey, ez = self._arm_end3(theta)
        end = self._pt(ex, ey, ez)
        # 倒立参考线（竖直虚线）
        painter.setPen(QtGui.QPen(QtGui.QColor(120, 140, 160, 110), 1,
                                  QtCore.Qt.PenStyle.DashLine))
        top = self._pt(ex, ey, ez + ROD_L)
        painter.drawLine(end, top)
        # 摆臂
        painter.setPen(QtGui.QPen(QtGui.QColor("#dfe9f2"), 7,
                                  QtCore.Qt.PenStyle.SolidLine, QtCore.Qt.PenCapStyle.RoundCap))
        painter.drawLine(self._pt(0, 0, MOTOR_H), end)
        # 臂末端角位移传感器（金色）
        painter.setPen(QtGui.QPen(QtGui.QColor("#8a6a1f"), 2))
        painter.setBrush(QtGui.QColor("#d9a93c"))
        painter.drawEllipse(end, 8, 8)
        # 摆杆：α=0 指向竖直上方；Furuta 铰（轴=臂方向），杆在垂直于臂的
        # 切向竖直平面内摆动——与 j280_plant_rhs.m / model3d 同口径
        # （2026-09-24 修复：此前误沿臂方向平面，转了 90°）。
        ux = -math.sin(alpha) * math.sin(theta)
        uy = math.sin(alpha) * math.cos(theta)
        uz = math.cos(alpha)
        tip = self._pt(ex + ROD_L * ux, ey + ROD_L * uy, ez + ROD_L * uz)
        painter.setPen(QtGui.QPen(QtGui.QColor("#f2f6f9"), 6,
                                  QtCore.Qt.PenStyle.SolidLine, QtCore.Qt.PenCapStyle.RoundCap))
        painter.drawLine(end, tip)
        painter.setPen(QtGui.QPen(QtGui.QColor("#8fa7bb"), 2))
        painter.setBrush(QtGui.QColor("#c2d2df"))
        painter.drawEllipse(tip, 4, 4)

    def _draw_hud(self, painter, s, mode, fault):
        painter.setPen(QtGui.QColor("#edf6fc"))
        painter.setFont(QtGui.QFont("Microsoft YaHei", 10))
        chip = MODE_COLORS.get(mode, "#5f7d95") if not fault else "#d4545f"
        painter.setPen(QtGui.QColor("#0e1723"))
        painter.setBrush(QtGui.QColor(chip))
        painter.drawRoundedRect(QtCore.QRectF(14, 14, 128, 28), 6, 6)
        painter.setPen(QtGui.QColor("#ffffff"))
        painter.drawText(QtCore.QRectF(14, 14, 128, 28),
                         QtCore.Qt.AlignmentFlag.AlignCenter,
                         ("故障 " if fault else "") + MODE_TEXT.get(mode, "?"))
        painter.setPen(QtGui.QColor("#cfe0ee"))
        if s:
            lines = [f"θ 摆臂  {s['theta']/100:+8.2f}°",
                     f"α 摆杆  {s['alpha']/100:+8.2f}°",
                     f"目标    {s['target']/100:+8.2f}°",
                     f"duty    {s['duty']/100:+8.2f}%"]
            for i, text in enumerate(lines):
                painter.drawText(16, 70 + i * 20, text)
            painter.drawText(16, 160, f"t = {s['tick']/1000:7.2f} s   配置 v{s['revision']}")
        else:
            painter.drawText(16, 70, "等待遥测…")
        # 占空比表
        duty = (s or {}).get("duty", 0) / 10000
        self._duty_bar(painter, duty)

    def _duty_bar(self, painter, duty):
        w, h = 190, 12
        x, y = self.width() - w - 18, self.height() - 30
        painter.setPen(QtGui.QPen(QtGui.QColor("#31485f"), 1))
        painter.setBrush(QtGui.QColor("#16283a"))
        painter.drawRoundedRect(QtCore.QRectF(x, y, w, h), 4, 4)
        mid = x + w / 2
        painter.setBrush(QtGui.QColor("#91d89b" if abs(duty) < 0.9 else "#d4545f"))
        painter.drawRect(QtCore.QRectF(mid, y + 1, (w / 2 - 1) * duty, h - 2))
        painter.setPen(QtGui.QColor("#5f7d95"))
        painter.setFont(QtGui.QFont("Microsoft YaHei", 8))
        painter.drawText(QtCore.QRectF(x, y + h + 2, w, 14),
                         QtCore.Qt.AlignmentFlag.AlignCenter, "输出占空比 −100% ～ +100%")
