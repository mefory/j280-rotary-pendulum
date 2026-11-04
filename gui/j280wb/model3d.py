"""J280 实物模型真 3D 视图：QOpenGLWidget + GL 3.3 着色器（无新增依赖）。

结构按套件实物：底座圆盘→电机（垂直轴）→水平摆臂（θ）→臂末端编码器→
摆杆（α，相对竖直，摆动面含臂方向）。角度约定与 2D 版一致：世界系 y 向上，
臂端 = (L·cosθ, HUB_Y, L·sinθ)，atan2(z, x) 即 θ。

视角：轨道相机——左键拖空白处旋转、滚轮缩放、双击复位。
交互控制：稳摆(2)/定位(3)时可抓取摆臂端或杆端，拖动时地面显示目标臂角
（幽灵臂+光柱），松手经 request_move 信号请求 app 下发 MOVE（最短角差、
夹紧固件上限 ±60°；定位中由 app 先 BALANCE 再 MOVE）。
数据来源只有遥测，不做任何控制假设。

PySide6 6.9.3 绑定坑（实测）：
- initializeGL 静默吞异常 → _init_gl 自行兜底转可见错误；
- QMatrix4x4*QVector4D 损坏（SyntaxError）→ 一律 .map()；
- 数组出参的 glGet*（GetIntegerv/GetUniformfv/GetProgramiv）绑定损坏 →
  返回全 0，不可用于调试，标量调用（glGetError/glDrawArrays）正常；
- QPainter 在 QOpenGLWidget 上不合成 → 文字 HUD 用透明子 QWidget。

红/青鬼影根因（已修复）：NVIDIA 581.08 驱动下，QOpenGLWidget FBO 带
alpha 通道时，α<1.0 的混合（读 dst）RGB 输出损坏（最小三角可复现，
α=1.0 不受影响）。修复 = QSurfaceFormat.setAlphaBufferSize(0)。
"""
import math
import time
from collections import deque

import numpy as np
from PySide6 import QtCore, QtGui, QtWidgets
from PySide6.QtOpenGL import (QOpenGLBuffer, QOpenGLShader, QOpenGLShaderProgram,
                              QOpenGLVertexArrayObject)
from PySide6.QtOpenGLWidgets import QOpenGLWidget

ARM_L = 1.0                     # 归一化基准 ↔ 真实 L_arm=0.156 m
ROD_L = 0.865                   # ↔ L_rod=0.135 m（杆/臂=0.135/0.156，硬件比例）
BASE_R = 0.42
BASE_H = 0.12
MOTOR_R = 0.15
MOTOR_H = 0.88                  # 立柱到臂挂高：HUB_Y=1.0 ≥ ROD_L，垂摆不穿地
HUB_Y = BASE_H + MOTOR_H

MODE_COLORS = {0: "#5f7d95", 1: "#e08a3c", 2: "#63c178", 3: "#54b8dc", 4: "#d4545f"}
MODE_TEXT = {0: "待机", 1: "自动起摆", 2: "稳摆", 3: "定位", 4: "安全停止"}

MOVE_LIMIT_CDEG = 6000          # 固件单次 MOVE 上限 ±60°

GL_DEPTH_TEST = 0x0B71
GL_LEQUAL = 0x0203
GL_BLEND = 0x0BE2
GL_SRC_ALPHA = 0x0302
GL_ONE_MINUS_SRC_ALPHA = 0x0303
GL_FLOAT = 0x1406
GL_TRIANGLES = 0x0004
GL_TRIANGLE_STRIP = 0x0005
GL_LINES = 0x0001
GL_COLOR_BUFFER_BIT = 0x4000
GL_DEPTH_BUFFER_BIT = 0x0100
GL_ARRAY_BUFFER = 0x8892
GL_DYNAMIC_DRAW = 0x88E8


# ---------- 纯映射函数（无 Qt 依赖，供 selftest 校验） ----------

def shortest_delta_deg(target_deg, current_deg):
    """当前臂角到目标臂角的最短角差，范围 [-180, 180)。"""
    return (target_deg - current_deg + 180.0) % 360.0 - 180.0


def clamp_delta_cdeg(delta_deg, limit_cdeg=MOVE_LIMIT_CDEG):
    """角差 → 定点 cdeg 并夹紧到固件单次上限。"""
    return max(-limit_cdeg, min(limit_cdeg, int(round(delta_deg * 100))))


def gl_supported():
    """预检能否创建 GL 3.3 Core 上下文；失败时 app 回退 2D 视图。"""
    global _GL_OK
    if _GL_OK is None:
        fmt = QtGui.QSurfaceFormat()
        fmt.setVersion(3, 3)
        fmt.setProfile(QtGui.QSurfaceFormat.OpenGLContextProfile.CoreProfile)
        fmt.setAlphaBufferSize(0)   # 与 Model3DView 保持一致（见其 __init__）
        surface = QtGui.QOffscreenSurface()
        surface.setFormat(fmt)
        surface.create()
        ctx = QtGui.QOpenGLContext()
        ctx.setFormat(fmt)
        ok = bool(ctx.create() and ctx.isValid())
        if ok:
            ok = ctx.makeCurrent(surface)
            if ok:
                ctx.doneCurrent()
        surface.destroy()
        _GL_OK = ok
    return _GL_OK


_GL_OK = None


# ---------- 几何生成（尺寸烘焙进顶点，模型矩阵只含旋转+平移） ----------

def cylinder_mesh(r, h, seg=48, axis="y"):
    """圆柱含端盖，轴沿 axis，底面中心在原点、高 h。返回 float32 (n,6)。"""
    u = np.array({"y": (0.0, 1.0, 0.0), "x": (1.0, 0.0, 0.0)}[axis])
    ref = np.array([0.0, 0.0, 1.0]) if abs(u[1]) > 0.9 else np.array([0.0, 1.0, 0.0])
    v = np.cross(u, ref)
    v /= np.linalg.norm(v)
    w = np.cross(u, v)
    ang = np.linspace(0, 2 * np.pi, seg, endpoint=False)
    ring = np.outer(np.cos(ang), v) * r + np.outer(np.sin(ang), w) * r
    P, N = [], []
    top, bot = h * u, np.zeros(3)

    def tri(p0, n0, p1, n1, p2, n2):
        for p, n in ((p0, n0), (p1, n1), (p2, n2)):
            P.append(p)
            N.append(n)

    for i in range(seg):
        p0, p1 = ring[i], ring[(i + 1) % seg]
        n0, n1 = p0 / r, p1 / r
        tri(p0, n0, p1, n1, p1 + top, n1)
        tri(p1 + top, n1, p0 + top, n0, p0, n0)
        tri(bot, -u, p1, -u, p0, -u)
        tri(top, u, p0 + top, u, p1 + top, u)
    return np.concatenate([np.array(P), np.array(N)], axis=1).astype(np.float32)


def box_mesh(w, h, d):
    hw, hh, hd = w / 2, h / 2, d / 2
    faces = [
        ((0, 0, 1), ((-hw, -hh, hd), (hw, -hh, hd), (hw, hh, hd), (-hw, hh, hd))),
        ((0, 0, -1), ((hw, -hh, -hd), (-hw, -hh, -hd), (-hw, hh, -hd), (hw, hh, -hd))),
        ((1, 0, 0), ((hw, -hh, hd), (hw, -hh, -hd), (hw, hh, -hd), (hw, hh, hd))),
        ((-1, 0, 0), ((-hw, -hh, -hd), (-hw, -hh, hd), (-hw, hh, hd), (-hw, hh, -hd))),
        ((0, 1, 0), ((-hw, hh, hd), (hw, hh, hd), (hw, hh, -hd), (-hw, hh, -hd))),
        ((0, -1, 0), ((-hw, -hh, -hd), (hw, -hh, -hd), (hw, -hh, hd), (-hw, -hh, hd))),
    ]
    P, N = [], []
    for n, quad in faces:
        n = np.array(n, dtype=float)
        for idx in (0, 1, 2, 0, 2, 3):
            P.append(np.array(quad[idx], dtype=float))
            N.append(n)
    return np.concatenate([np.array(P), np.array(N)], axis=1).astype(np.float32)


def sphere_mesh(r, lat=20, lon=32):
    a = np.linspace(0, np.pi, lat + 1)
    b = np.linspace(0, 2 * np.pi, lon + 1)
    A, B = np.meshgrid(a, b, indexing="ij")
    nx, ny, nz = np.sin(A) * np.cos(B), np.cos(A), np.sin(A) * np.sin(B)
    P = np.stack([nx, ny, nz], -1) * r
    N = np.stack([nx, ny, nz], -1)
    idx = []
    for i in range(lat):
        for j in range(lon):
            v00 = i * (lon + 1) + j
            v01, v10, v11 = v00 + 1, v00 + lon + 1, v00 + lon + 2
            if i > 0:
                idx += [v00, v10, v11]
            if i < lat - 1:
                idx += [v00, v11, v01]
    idx = np.array(idx)
    return np.concatenate([P.reshape(-1, 3)[idx], N.reshape(-1, 3)[idx]],
                          axis=1).astype(np.float32)


def disc_mesh(r, y=0.0, seg=72):
    a = np.linspace(0, 2 * np.pi, seg, endpoint=False)
    x, z = np.cos(a) * r, np.sin(a) * r
    P, N = [], []
    up = np.array([0.0, 1.0, 0.0])
    c = np.array([0.0, y, 0.0])
    for i in range(seg):
        j = (i + 1) % seg
        P += [c, np.array([x[i], y, z[i]]), np.array([x[j], y, z[j]])]
        N += [up, up, up]
    return np.concatenate([np.array(P), np.array(N)], axis=1).astype(np.float32)


# ---------- 着色器 ----------

MESH_VS = """#version 330
layout(location=0) in vec3 a_pos;
layout(location=1) in vec3 a_nrm;
uniform mat4 u_mvp;
uniform mat4 u_model;
out vec3 v_nrm;
out vec3 v_wpos;
void main() {
    vec4 wpos = u_model * vec4(a_pos, 1.0);
    v_wpos = wpos.xyz;
    v_nrm = mat3(u_model) * a_nrm;
    gl_Position = u_mvp * wpos;
}
"""

MESH_FS = """#version 330
in vec3 v_nrm;
in vec3 v_wpos;
uniform vec3 u_color;
uniform float u_alpha;
uniform vec3 u_eye;
uniform float u_spec;
uniform float u_shin;
uniform float u_unlit;
out vec4 frag;
vec3 shade(vec3 N, vec3 V, vec3 L, vec3 lc, float li) {
    float diff = max(dot(N, L), 0.0);
    vec3 H = normalize(L + V);
    float lit = step(0.0, dot(N, L));
    float spec = pow(max(dot(N, H), 0.0), u_shin) * u_spec * lit;
    return lc * li * (diff + 0.9 * spec);
}
void main() {
    if (u_unlit > 0.5) { frag = vec4(u_color, u_alpha); return; }
    vec3 N = normalize(v_nrm);
    if (!gl_FrontFacing) N = -N;
    vec3 V = normalize(u_eye - v_wpos);
    vec3 amb = mix(vec3(0.14, 0.17, 0.22), vec3(0.34, 0.40, 0.48), 0.5 + 0.5 * N.y);
    vec3 col = amb * u_color * 0.9;
    col += shade(N, V, normalize(vec3(0.45, 0.85, 0.30)), vec3(1.00, 0.96, 0.90), 1.0);
    col += shade(N, V, normalize(vec3(-0.60, 0.20, -0.55)), vec3(0.40, 0.52, 0.68), 0.5);
    float rim = pow(1.0 - max(dot(N, V), 0.0), 2.6);
    col += vec3(0.22, 0.42, 0.58) * rim * 0.55;
    frag = vec4(min(col, vec3(1.0)), u_alpha);
}
"""

LINE_VS = """#version 330
layout(location=0) in vec3 a_pos;
layout(location=1) in vec4 a_col;
uniform mat4 u_mvp;
out vec4 v_col;
void main() { v_col = a_col; gl_Position = u_mvp * vec4(a_pos, 1.0); }
"""

LINE_FS = """#version 330
in vec4 v_col;
out vec4 frag;
void main() { frag = v_col; }
"""


def m_translate(x, y, z):
    m = QtGui.QMatrix4x4()
    m.translate(x, y, z)
    return m


def m_rotate(deg, x, y, z):
    m = QtGui.QMatrix4x4()
    m.rotate(deg, QtGui.QVector3D(x, y, z))
    return m


def compose(*ms):
    r = QtGui.QMatrix4x4()
    for m in ms:
        r = r * m
    return r


FLAT_SHADOW = QtGui.QMatrix4x4([
    1, 0, 0, 0,
    0, 0, 0, 0.004,
    0, 0, 1, 0,
    0, 0, 0, 1])


class OrbitCamera:
    """轨道相机：yaw/pitch/distance 绕目标点旋转。"""

    def __init__(self):
        self.reset()

    def reset(self):
        self.yaw = 35.0
        self.pitch = 24.0
        self.dist = 3.6
        self.target = QtGui.QVector3D(0.0, 1.05, 0.0)

    def set_pose(self, yaw, pitch, dist):
        self.yaw, self.pitch, self.dist = yaw, pitch, dist

    def orbit(self, dx, dy):
        self.yaw = (self.yaw + dx * 0.45) % 360.0
        self.pitch = min(80.0, max(8.0, self.pitch + dy * 0.35))

    def zoom(self, factor):
        self.dist = min(10.0, max(1.8, self.dist * factor))

    def eye(self):
        yr, pr = math.radians(self.yaw), math.radians(self.pitch)
        return self.target + QtGui.QVector3D(
            self.dist * math.cos(pr) * math.sin(yr),
            self.dist * math.sin(pr),
            self.dist * math.cos(pr) * math.cos(yr))

    def view(self):
        m = QtGui.QMatrix4x4()
        m.lookAt(self.eye(), self.target, QtGui.QVector3D(0, 1, 0))
        return m


class _HudOverlay(QtWidgets.QWidget):
    """透明子控件承载文字 HUD。

    本 PySide6 构建的 QOpenGLWidget 无法用 QPainter 合成（paintGL 内与
    paintEvent 叠加均不出图），故文字层用普通子控件绘制。
    """

    def __init__(self, owner):
        super().__init__(owner)
        self._owner = owner
        self.setAttribute(QtCore.Qt.WidgetAttribute.WA_TransparentForMouseEvents)
        self.setAttribute(QtCore.Qt.WidgetAttribute.WA_NoSystemBackground)

    def paintEvent(self, _event):
        owner = self._owner
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        s = owner.sample or {}
        mode = s.get("mode", 0)
        fault = s.get("fault", 0)

        # 模式徽章（左上）：故障红色优先
        chip_color = QtGui.QColor("#d4545f") if fault else QtGui.QColor(
            MODE_COLORS.get(mode, "#5f7d95"))
        painter.setPen(QtGui.QColor("#0e1723"))
        painter.setBrush(chip_color)
        painter.drawRoundedRect(QtCore.QRectF(14, 14, 132, 30), 6, 6)
        painter.setPen(QtGui.QColor("#ffffff"))
        font = QtGui.QFont("Microsoft YaHei", 10)
        font.setBold(True)
        painter.setFont(font)
        painter.drawText(QtCore.QRectF(14, 14, 132, 30), QtCore.Qt.AlignmentFlag.AlignCenter,
                         ("故障 " if fault else "") + MODE_TEXT.get(mode, "?"))

        # 遥测数据面板：半透明圆角底板 + 标签/等宽数值双列；Δ→目标 一眼看出
        # 还差多少到位（POS/拖动定位时最有用）
        rows, foot = [], ""
        if s:
            theta_deg = s["theta"] / 100
            target_deg = s["target"] / 100
            duty = s["duty"]
            rows = [
                ("θ 摆臂", f"{theta_deg:+8.2f}°", "#dfe9f2"),
                ("α 摆杆", f"{s['alpha']/100:+8.2f}°", "#6edce8"),
                ("目标", f"{target_deg:+8.2f}°", "#ffbd69"),
                ("Δ→目标", f"{shortest_delta_deg(target_deg, theta_deg):+8.2f}°",
                 "#9fc6dd"),
                ("duty", f"{duty/100:+8.2f}%",
                 "#d4545f" if abs(duty) >= 9000 else "#91d89b"),
            ]
            foot = f"t = {s['tick']/1000:7.2f}s   配置 v{s['revision']}"
        panel = QtCore.QRectF(14, 54, 196,
                              24 * max(1, len(rows)) + (26 if s else 18) + 10)
        painter.setPen(QtCore.Qt.PenStyle.NoPen)
        painter.setBrush(QtGui.QColor(8, 16, 26, 150))
        painter.drawRoundedRect(panel, 8, 8)
        label_font = QtGui.QFont("Microsoft YaHei", 9)
        value_font = QtGui.QFont("Consolas", 10)
        value_font.setBold(True)
        y = panel.top() + 7
        for label, value, color in rows:
            painter.setPen(QtGui.QColor("#8fa7bb"))
            painter.setFont(label_font)
            painter.drawText(26, int(y) + 14, label)
            painter.setPen(QtGui.QColor(color))
            painter.setFont(value_font)
            painter.drawText(QtCore.QRectF(64, y, 136, 18),
                             QtCore.Qt.AlignmentFlag.AlignRight, value)
            y += 24
        painter.setFont(QtGui.QFont("Consolas", 8))
        if s:
            painter.setPen(QtGui.QColor("#6f879c"))
            painter.drawText(26, int(y) + 14, foot)
        else:
            painter.setPen(QtGui.QColor("#8fa7bb"))
            painter.setFont(QtGui.QFont("Microsoft YaHei", 9))
            painter.drawText(26, int(y) + 14, "等待遥测…")

        self._draw_compass(painter, owner, s)

        if owner._preview is not None:
            painter.setPen(QtGui.QColor("#6edce8"))
            font = QtGui.QFont("Microsoft YaHei", 11)
            font.setBold(True)
            painter.setFont(font)
            painter.drawText(16, int(panel.bottom()) + 22,
                             f"拖动中：目标臂角 {math.degrees(owner._preview):+.1f}°"
                             "（松手发送定位）")

        age = time.monotonic() - owner._note[1]
        if owner._note[0] and age < 3.0:
            painter.setPen(QtGui.QColor(110, 220, 232, int(255 * (1 - age / 3))))
            painter.setFont(QtGui.QFont("Microsoft YaHei", 10))
            painter.drawText(QtCore.QRectF(0, 22, self.width(), 24),
                             QtCore.Qt.AlignmentFlag.AlignCenter, owner._note[0])

        painter.setPen(QtGui.QColor("#5f7d95"))
        painter.setFont(QtGui.QFont("Microsoft YaHei", 8))
        painter.drawText(16, self.height() - 14,
                         "左键拖空白处=旋转视角 · 滚轮=缩放 · 双击=复位"
                         + (" · 拖摆臂端/杆端=定位（稳摆/定位时）" if owner.drag_enabled else ""))

        self._draw_duty_bar(painter, s)
        painter.end()

    def _draw_compass(self, painter, owner, s):
        """右上角方位罗盘：0° 在正上方、顺时针为正（与俯视图一致）。
        白实针=当前臂向，橙虚针=POS 目标方位，青针=拖动预览——
        “臂在哪儿、要去哪儿、还差多少”不用再心算角度。"""
        if not s:
            return
        r = 52.0
        cx = owner.width() - r - 26
        cy = r + 26
        painter.setPen(QtGui.QPen(QtGui.QColor("#3a5570"), 1.5))
        painter.setBrush(QtGui.QColor(8, 16, 26, 140))
        painter.drawEllipse(QtCore.QPointF(cx, cy), r, r)
        painter.setFont(QtGui.QFont("Consolas", 7))
        for deg in range(0, 360, 30):
            a = math.radians(deg)
            sin_a, cos_a = math.sin(a), math.cos(a)
            major = deg % 90 == 0
            r0, r1 = (r - 8, r - 2) if major else (r - 5, r - 2)
            painter.setPen(QtGui.QPen(QtGui.QColor("#31485f"), 1))
            painter.drawLine(QtCore.QPointF(cx + r0 * sin_a, cy - r0 * cos_a),
                             QtCore.QPointF(cx + r1 * sin_a, cy - r1 * cos_a))
            if major:
                painter.setPen(QtGui.QColor("#5f7d95"))
                painter.drawText(
                    QtCore.QRectF(cx + (r + 2) * sin_a - 14,
                                  cy - (r + 2) * cos_a - 7, 28, 14),
                    QtCore.Qt.AlignmentFlag.AlignCenter, str(deg))

        def needle(deg, color, width, dashed=False):
            a = math.radians(deg)
            pen = QtGui.QPen(QtGui.QColor(color), width)
            if dashed:
                pen.setDashPattern([4, 3])
            painter.setPen(pen)
            painter.drawLine(
                QtCore.QPointF(cx - 7 * math.sin(a), cy + 7 * math.cos(a)),
                QtCore.QPointF(cx + (r - 13) * math.sin(a),
                               cy - (r - 13) * math.cos(a)))

        if s.get("mode") == 3:
            needle(s["target"] / 100, "#ffbd69", 1.6, dashed=True)
        if owner._preview is not None:
            needle(math.degrees(owner._preview), "#6edce8", 1.8)
        needle(s["theta"] / 100, "#dfe9f2", 2.4)
        painter.setPen(QtCore.Qt.PenStyle.NoPen)
        painter.setBrush(QtGui.QColor("#8fa7bb"))
        painter.drawEllipse(QtCore.QPointF(cx, cy), 3, 3)
        painter.setPen(QtGui.QColor("#5f7d95"))
        painter.setFont(QtGui.QFont("Microsoft YaHei", 7))
        painter.drawText(QtCore.QRectF(cx - r, cy + r + 14, 2 * r, 14),
                         QtCore.Qt.AlignmentFlag.AlignCenter, "臂方位 θ")

    def _draw_duty_bar(self, painter, s):
        """右下角占空比条：加 ±50% 刻度与两端量程标注，中线=零输出。"""
        duty = (s or {}).get("duty", 0) / 10000
        w, h = 190, 12
        x, y = self.width() - w - 18, self.height() - 30
        painter.setPen(QtGui.QPen(QtGui.QColor("#31485f"), 1))
        painter.setBrush(QtGui.QColor("#16283a"))
        painter.drawRoundedRect(QtCore.QRectF(x, y, w, h), 4, 4)
        mid = x + w / 2
        painter.setPen(QtGui.QPen(QtGui.QColor("#31485f"), 1))
        painter.setBrush(QtCore.Qt.BrushStyle.NoBrush)
        for frac in (-0.5, 0.5):
            tx = x + w * (0.5 + frac)
            painter.drawLine(QtCore.QPointF(tx, y + 2), QtCore.QPointF(tx, y + h - 2))
        painter.setPen(QtCore.Qt.PenStyle.NoPen)
        painter.setBrush(QtGui.QColor("#91d89b" if abs(duty) < 0.9 else "#d4545f"))
        painter.drawRect(QtCore.QRectF(mid, y + 1, (w / 2 - 1) * duty, h - 2))
        painter.setPen(QtGui.QColor("#5f7d95"))
        painter.setFont(QtGui.QFont("Microsoft YaHei", 8))
        painter.drawText(QtCore.QRectF(x, y + h + 2, 44, 14),
                         QtCore.Qt.AlignmentFlag.AlignLeft, "−100%")
        painter.drawText(QtCore.QRectF(x, y + h + 2, w, 14),
                         QtCore.Qt.AlignmentFlag.AlignCenter, "0")
        painter.drawText(QtCore.QRectF(x + w - 44, y + h + 2, 44, 14),
                         QtCore.Qt.AlignmentFlag.AlignRight, "+100%")


class Model3DView(QOpenGLWidget):
    """set_sample() 由 GUI 定时喂入；request_move 在拖动松手时发射。

    request_move 发的是**相对位移**（弧度），不是绝对目标角：
    设备 θ 是多圈累积值（可达 ±1000°），而屏幕拖动只能算出 atan2 范围内的
    绝对方位角。若直接把绝对角当目标，绕圈数与拖动手感完全对不上
    （实测 θ=−434° 时拖 10° 会算出 −130°）。改为：松手时用"拖动预览角
    相对按下瞬间的显示角"的差值作为位移，和手在屏幕上移动的量严格对应。
    """

    request_move = QtCore.Signal(float)     # 相对位移（弧度，正=逆时针）

    def __init__(self, parent=None):
        super().__init__(parent)
        fmt = QtGui.QSurfaceFormat()
        fmt.setVersion(3, 3)
        fmt.setProfile(QtGui.QSurfaceFormat.OpenGLContextProfile.CoreProfile)
        fmt.setSamples(8)
        fmt.setDepthBufferSize(24)
        # 关键修复：不要 alpha 通道。NVIDIA 581.08 + Qt 6.9.3 下，带 alpha
        # 的 QOpenGLWidget FBO 做 α<1.0 混合（读 dst）时 RGB 输出损坏
        # （实测最小三角复现，α=1.0 不受影响）。场景背景本就不透明，
        # 文字 HUD 是 raster 子控件，均不依赖 GL alpha。
        fmt.setAlphaBufferSize(0)
        self.setFormat(fmt)
        self.setMinimumSize(520, 420)
        self.setMouseTracking(True)
        self.sample = None
        self.trail = deque(maxlen=720)
        self.cam = OrbitCamera()
        self.drag_enabled = True
        self._mode = None            # None=空闲, "orbit"=转视角, "cmd"=拖动定位
        self._last_pos = None
        self._preview = None         # 拖动中的目标臂角（弧度）
        self._drag_ref = None        # 按下瞬间的显示角（相对位移基准）
        self._note = ("", 0.0)
        self._gl_error = None
        self._hud = _HudOverlay(self)
        self._error_label = QtWidgets.QLabel(self._hud)
        self._error_label.setWordWrap(True)
        self._error_label.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self._error_label.setStyleSheet(
            "color:#ffb9c0;background:#3a1720;border-radius:6px;padding:14px;")
        self._error_label.hide()
        self._prog_mesh = None
        self._prog_line = None
        self._meshes = {}
        self._mvp = QtGui.QMatrix4x4()
        self._eye = QtGui.QVector3D()
        self._lines = []             # 本帧线段 [x,y,z,r,g,b,a] * 2n

    # ---------- 对外接口 ----------
    @QtCore.Slot(dict)
    def set_sample(self, sample):
        self.sample = sample
        if sample:            # 无遥测不记轨迹（否则会在 0° 记假点）
            theta = sample.get("theta", 0) / 100 * math.pi / 180
            self.trail.append((math.cos(theta) * ARM_L, math.sin(theta) * ARM_L))
        self.update()
        self._hud.update()

    def reset_trail(self):
        self.trail.clear()
        self.update()

    def set_drag_enabled(self, on):
        self.drag_enabled = bool(on)
        self._hud.update()

    def set_note(self, text):
        self._note = (text, time.monotonic())
        self._hud.update()

    def reset_camera(self):
        self.cam.reset()
        self.update()

    def set_camera(self, yaw, pitch, dist):
        self.cam.set_pose(yaw, pitch, dist)
        self.update()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._hud.setGeometry(self.rect())
        self._error_label.setGeometry(self._hud.rect().adjusted(60, 60, -60, -60))

    # ---------- 相机/拾取几何 ----------
    def _projection(self):
        m = QtGui.QMatrix4x4()
        m.perspective(38.0, max(0.2, self.width() / max(1, self.height())), 0.05, 60.0)
        return m

    def world_to_screen(self, x, y, z):
        """世界坐标 → 本 widget 逻辑像素；在相机背后时返回 None。"""
        clip = self._mvp.map(QtGui.QVector4D(x, y, z, 1.0))
        if clip.w() <= 1e-6:
            return None
        ndc = clip.toVector3DAffine()
        return QtCore.QPointF((ndc.x() + 1) / 2 * self.width(),
                              (1 - (ndc.y() + 1) / 2) * self.height())

    def screen_to_arm_angle(self, pos):
        """鼠标位置 → 与臂端所在平面(y=HUB_Y)交点的方位角 atan2(z,x)。

        在臂端平面上取交点可消除视差：拖动时臂端精确跟随鼠标。
        """
        inv, ok = (self._projection() * self.cam.view()).inverted()
        if not ok:
            return None
        ndc_x = pos.x() / self.width() * 2 - 1
        ndc_y = 1 - pos.y() / self.height() * 2
        p0 = inv.map(QtGui.QVector4D(ndc_x, ndc_y, -1.0, 1.0)).toVector3DAffine()
        p1 = inv.map(QtGui.QVector4D(ndc_x, ndc_y, 1.0, 1.0)).toVector3DAffine()
        d = p1 - p0
        if abs(d.y()) < 1e-9:
            return None
        t = (HUB_Y - p0.y()) / d.y()
        if t < 0:
            return None
        hit = p0 + d * t
        return math.atan2(hit.z(), hit.x())

    def _sample_angles(self):
        s = self.sample or {}
        theta = s.get("theta", 0) / 100 * math.pi / 180
        alpha = s.get("alpha", 0) / 100 * math.pi / 180
        return theta, alpha

    def _display_angle(self):
        """当前臂的显示方位角（atan2 域，[−π, π)）。θ 多圈累积需先归一。"""
        theta, _ = self._sample_angles()
        return math.atan2(math.sin(theta), math.cos(theta))

    def arm_end_world(self):
        theta, _ = self._sample_angles()
        return QtGui.QVector3D(ARM_L * math.cos(theta), HUB_Y, ARM_L * math.sin(theta))

    def rod_tip_world(self):
        theta, alpha = self._sample_angles()
        # Furuta 铰（轴=臂方向）：杆尖 = 臂端 + ROD·(sinα·t̂ + cosα·up)，
        # t̂=切向单位矢量——与 paintGL 的 rod_m 同口径。
        return QtGui.QVector3D(
            ARM_L * math.cos(theta) - ROD_L * math.sin(alpha) * math.sin(theta),
            HUB_Y + ROD_L * math.cos(alpha),
            ARM_L * math.sin(theta) + ROD_L * math.sin(alpha) * math.cos(theta))

    def _hit_handle(self, pos):
        """鼠标是否抓到摆臂端/杆端（26px 内）；可抓时返回 True。"""
        if self._gl_error or not self.sample:
            return False      # GL 失败时 _mvp 停在单位阵，拾取结果无意义
        for p in (self.arm_end_world(), self.rod_tip_world()):
            sp = self.world_to_screen(p.x(), p.y(), p.z())
            if sp is not None and (sp - pos).x() ** 2 + (sp - pos).y() ** 2 < 26.0 ** 2:
                return True
        return False

    # ---------- 鼠标交互 ----------
    def mousePressEvent(self, e):
        if e.button() == QtCore.Qt.MouseButton.LeftButton:
            mode = (self.sample or {}).get("mode", 0)
            if (self.drag_enabled and mode in (2, 3) and self._hit_handle(e.position())):
                self._mode = "cmd"
                ang = self.screen_to_arm_angle(e.position())
                # 按下瞬间的"显示角"作为相对位移基准：θ 本身是多圈累积值，
                # 而屏幕拾取只能给出 atan2 方位角，两者必须统一到同一参考
                # 才能让"拖多少转多少"。显示角 = 当前 θ 归一到 atan2 域。
                self._drag_ref = self._display_angle() if ang is None else ang
                self._preview = ang
                self.setCursor(QtCore.Qt.CursorShape.ClosedHandCursor)
                self.update()
                return
            self._mode = "orbit"
            self._last_pos = e.position()

    def mouseMoveEvent(self, e):
        if self._mode == "cmd":
            ang = self.screen_to_arm_angle(e.position())
            if ang is not None:
                self._preview = ang
            self._hud.update()
            return
        if self._mode == "orbit":
            d = e.position() - self._last_pos
            self._last_pos = e.position()
            self.cam.orbit(d.x(), d.y())
            self.update()
            return
        mode = (self.sample or {}).get("mode", 0)
        hover = (self.drag_enabled and mode in (2, 3) and self._hit_handle(e.position()))
        self.setCursor(QtCore.Qt.CursorShape.PointingHandCursor if hover
                       else QtCore.Qt.CursorShape.ArrowCursor)

    def mouseReleaseEvent(self, e):
        if self._mode == "cmd":
            self._mode = None
            ang, self._preview = self._preview, None
            ref, self._drag_ref = getattr(self, "_drag_ref", None), None
            self.setCursor(QtCore.Qt.CursorShape.ArrowCursor)
            self._hud.update()
            if ang is not None and ref is not None:
                # 发相对位移而非绝对目标：拖动预览角 - 按下基准角，取最短环绕
                delta_rad = math.radians(shortest_delta_deg(math.degrees(ang),
                                                           math.degrees(ref)))
                if abs(delta_rad) > 1e-6:
                    self.request_move.emit(delta_rad)
        elif self._mode == "orbit":
            self._mode = None

    def mouseDoubleClickEvent(self, e):
        if e.button() == QtCore.Qt.MouseButton.LeftButton:
            self.cam.reset()
            self.update()

    def wheelEvent(self, e):
        self.cam.zoom(math.exp(-e.angleDelta().y() * 0.0012))
        self.update()

    # ---------- GL ----------
    def initializeGL(self):
        # PySide 会静默吞掉此虚函数里的异常，必须自己兜住并转成可见错误
        try:
            self._init_gl()
        except Exception as exc:
            self._gl_error = f"{type(exc).__name__}: {exc}"
            self._error_label.setText("3D 视图初始化失败：" + self._gl_error)
            self._error_label.setGeometry(self._hud.rect().adjusted(60, 60, -60, -60))
            self._error_label.show()

    def _init_gl(self):
        self._prog_mesh = self._build_program(MESH_VS, MESH_FS)
        self._prog_line = self._build_program(LINE_VS, LINE_FS)
        # PySide6 6.9.3 绑定坑（实测，详见模块 docstring）：
        # QMatrix4x4*QVector4D 损坏 → 用 .map()；数组出参的 glGet* 绑定
        # 损坏 → 不可用于调试。拖动帧红/青鬼影的真因是 QOpenGLWidget
        # FBO 带 alpha 通道时 α<1.0 混合损坏，已在 __init__ 修复。
        fns = self._gl_fns()
        self._fns = fns
        meshes = {
            "base": cylinder_mesh(BASE_R, BASE_H),
            "motor": cylinder_mesh(MOTOR_R, MOTOR_H),
            "collar": cylinder_mesh(MOTOR_R * 0.72, 0.05),
            "arm": cylinder_mesh(0.045, ARM_L, axis="x"),
            "hub": cylinder_mesh(0.085, 0.09),
            "rod": cylinder_mesh(0.028, ROD_L),
            "pivot": sphere_mesh(0.065),
            "joint": sphere_mesh(0.05),
            "tip": sphere_mesh(0.045),
            "ground": disc_mesh(2.3),
            "contact": disc_mesh(BASE_R * 1.18, y=0.002),
            "mark": disc_mesh(0.11, y=0.006),
            "pillar": cylinder_mesh(0.032, 1.35),
        }
        self._meshes = {name: self._make_mesh(arr) for name, arr in meshes.items()}
        self._bg_vao, self._bg_vbo = self._make_line_buffer(
            np.array([[-1, -1, 0, 0.040, 0.070, 0.118, 1],
                      [1, -1, 0, 0.040, 0.070, 0.118, 1],
                      [1, 1, 0, 0.062, 0.100, 0.160, 1],
                      [-1, 1, 0, 0.062, 0.100, 0.160, 1]], dtype=np.float32))
        self._line_vao, self._line_vbo = self._make_line_buffer(
            np.zeros((2, 7), dtype=np.float32))

    def _gl_fns(self):
        """缓存的 QOpenGLFunctions（glDrawArrays 等标量调用用）。"""
        return self.context().functions()

    def _build_program(self, vs, fs):
        prog = QOpenGLShaderProgram(self)
        if not prog.addShaderFromSourceCode(QOpenGLShader.ShaderTypeBit.Vertex, vs):
            raise RuntimeError("顶点着色器编译失败：" + prog.log())
        if not prog.addShaderFromSourceCode(QOpenGLShader.ShaderTypeBit.Fragment, fs):
            raise RuntimeError("片元着色器编译失败：" + prog.log())
        if not prog.link():
            raise RuntimeError("着色器链接失败：" + prog.log())
        return prog

    def _make_mesh(self, array):
        vao = QOpenGLVertexArrayObject(self)
        vao.create()
        vao.bind()
        vbo = QOpenGLBuffer(QOpenGLBuffer.Type.VertexBuffer)
        vbo.create()
        vbo.bind()
        vbo.allocate(array.tobytes(), array.nbytes)
        self._prog_mesh.bind()
        self._prog_mesh.enableAttributeArray(0)
        self._prog_mesh.setAttributeBuffer(0, GL_FLOAT, 0, 3, 24)
        self._prog_mesh.enableAttributeArray(1)
        self._prog_mesh.setAttributeBuffer(1, GL_FLOAT, 12, 3, 24)
        self._prog_mesh.release()
        vbo.release()
        vao.release()
        return (vao, vbo, array.shape[0])

    def _make_line_buffer(self, array):
        vao = QOpenGLVertexArrayObject(self)
        vao.create()
        vao.bind()
        vbo = QOpenGLBuffer(QOpenGLBuffer.Type.VertexBuffer)
        vbo.create()
        vbo.bind()
        vbo.allocate(array.tobytes(), array.nbytes)
        self._prog_line.bind()
        self._prog_line.enableAttributeArray(0)
        self._prog_line.setAttributeBuffer(0, GL_FLOAT, 0, 3, 28)
        self._prog_line.enableAttributeArray(1)
        self._prog_line.setAttributeBuffer(1, GL_FLOAT, 12, 4, 28)
        self._prog_line.release()
        vbo.release()
        vao.release()
        return (vao, vbo)

    def _draw_mesh(self, name, model, color, alpha=1.0, spec=0.5, shin=40.0, unlit=0.0):
        vao, _vbo, count = self._meshes[name]
        prog = self._prog_mesh
        prog.bind()
        fns = self._fns
        prog.setUniformValue("u_mvp", self._mvp)
        prog.setUniformValue("u_model", model)
        prog.setUniformValue("u_color", QtGui.QVector3D(*color))
        prog.setUniformValue1f("u_alpha", float(alpha))
        prog.setUniformValue("u_eye", self._eye)
        prog.setUniformValue1f("u_spec", float(spec))
        prog.setUniformValue1f("u_shin", float(shin))
        prog.setUniformValue1f("u_unlit", float(unlit))
        vao.bind()
        fns.glDrawArrays(GL_TRIANGLES, 0, count)
        vao.release()
        prog.release()

    def _line3(self, p0, p1, color, alpha):
        """p0/p1 为 (x, y, z) 世界坐标；追加一条线段到本帧线缓冲。"""
        c = QtGui.QColor(color)
        rgb = (c.redF(), c.greenF(), c.blueF(), alpha)
        self._lines += [[*p0, *rgb], [*p1, *rgb]]

    def _line(self, p0, p1, color, alpha, y=0.0):
        self._line3((p0[0], y, p0[1]), (p1[0], y, p1[1]), color, alpha)

    def _dash_line3(self, p0, p1, color, alpha, dash=0.07, gap=0.05):
        d = [p1[i] - p0[i] for i in range(3)]
        length = math.sqrt(sum(v * v for v in d))
        if length < 1e-6:
            return
        n = max(1, int(length / (dash + gap)))
        u = [v / length for v in d]
        for i in range(n):
            a = i * (dash + gap)
            b = min(a + dash, length)
            self._line3(tuple(p0[k] + u[k] * a for k in range(3)),
                        tuple(p0[k] + u[k] * b for k in range(3)), color, alpha)

    def _dash_line(self, p0, p1, color, alpha, y=0.0):
        self._dash_line3((p0[0], y, p0[1]), (p1[0], y, p1[1]), color, alpha)

    def _circle(self, cx, cz, r, color, alpha, y=0.0, seg=96):
        for i in range(seg):
            a0 = 2 * math.pi * i / seg
            a1 = 2 * math.pi * (i + 1) / seg
            self._line((cx + r * math.cos(a0), cz + r * math.sin(a0)),
                       (cx + r * math.cos(a1), cz + r * math.sin(a1)), color, alpha, y)

    def _flush_lines(self, f):
        arr = np.array(self._lines, dtype=np.float32)
        self._lines = []
        if arr.shape[0] == 0:
            return
        self._prog_line.bind()
        self._line_vao.bind()
        self._line_vbo.bind()
        self._line_vbo.allocate(arr.tobytes(), arr.nbytes)
        self._prog_line.setUniformValue("u_mvp", self._mvp)
        f.glLineWidth(2.0)
        f.glDrawArrays(GL_LINES, 0, arr.shape[0])
        self._line_vbo.release()
        self._line_vao.release()
        self._prog_line.release()

    def paintGL(self):
        if self._gl_error:
            return
        f = self.context().functions()
        dpr = self.devicePixelRatioF()
        f.glViewport(0, 0, int(self.width() * dpr), int(self.height() * dpr))
        f.glEnable(GL_DEPTH_TEST)
        f.glDepthFunc(GL_LEQUAL)
        f.glEnable(GL_BLEND)
        f.glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        f.glClearColor(0.043, 0.075, 0.125, 1.0)
        f.glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        self._mvp = self._projection() * self.cam.view()
        self._eye = self.cam.eye()
        s = self.sample or {}
        theta, alpha = self._sample_angles()
        tdeg, adeg = math.degrees(theta), math.degrees(alpha)

        # 1) 背景渐变（关深度）
        f.glDepthMask(False)
        f.glDisable(GL_DEPTH_TEST)
        self._prog_line.bind()
        self._bg_vao.bind()
        self._prog_line.setUniformValue("u_mvp", QtGui.QMatrix4x4())
        f.glDrawArrays(GL_TRIANGLE_STRIP, 0, 4)
        self._bg_vao.release()
        self._prog_line.release()
        f.glEnable(GL_DEPTH_TEST)
        f.glDepthMask(True)

        # 2) 地面与接触阴影（不写深度）
        f.glDepthMask(False)
        self._draw_mesh("ground", QtGui.QMatrix4x4(), (0.050, 0.078, 0.116), unlit=1.0)
        if s:
            self._draw_mesh("contact", QtGui.QMatrix4x4(), (0, 0, 0), alpha=0.35, unlit=1.0)
        f.glDepthMask(True)

        # 3) 静态机构
        self._draw_mesh("base", m_translate(0, 0, 0), (0.11, 0.135, 0.17), spec=0.3, shin=24)
        self._draw_mesh("motor", m_translate(0, BASE_H, 0), (0.16, 0.20, 0.26), spec=0.5, shin=42)
        self._draw_mesh("collar", m_translate(0, BASE_H + MOTOR_H, 0),
                        (0.62, 0.48, 0.19), spec=0.9, shin=64)

        if s:
            arm_rot = m_rotate(-tdeg, 0, 1, 0)
            arm_m = compose(m_translate(0, HUB_Y, 0), arm_rot)
            hub_m = arm_m * m_translate(ARM_L, 0, 0)
            # Furuta 铰：铰轴=臂方向（局部 X），杆在垂直于臂的切向竖直平面内
            # 摆动——与 j280_plant_rhs.m 的质量阵耦合 M12=C·cosα 同口径
            #（2026-09-24 修复：此前误绕局部 Z 轴，摆动平面转了 90°）。
            rod_m = compose(arm_m, m_translate(ARM_L, 0.045, 0), m_rotate(adeg, 1, 0, 0))
            # 阴影（正投影到地面）
            f.glDepthMask(False)
            for name, m in (("arm", arm_m), ("hub", hub_m), ("rod", rod_m),
                            ("tip", rod_m * m_translate(0, ROD_L, 0))):
                self._draw_mesh(name, FLAT_SHADOW * m, (0, 0, 0), alpha=0.22, unlit=1.0)
            f.glDepthMask(True)

            self._draw_mesh("arm", arm_m, (0.78, 0.81, 0.86), spec=0.7, shin=52)
            self._draw_mesh("pivot", m_translate(0, HUB_Y, 0), (0.55, 0.60, 0.66), spec=0.6)
            self._draw_mesh("hub", hub_m, (0.86, 0.68, 0.22), spec=0.9, shin=70)
            self._draw_mesh("joint", hub_m * m_translate(0, 0.09, 0), (0.30, 0.34, 0.40), spec=0.8)
            self._draw_mesh("rod", rod_m, (0.30, 0.34, 0.40), spec=1.0, shin=90)
            self._draw_mesh("tip", rod_m * m_translate(0, ROD_L, 0),
                            (0.30, 0.75, 0.80), spec=0.9, shin=64)

        # 4) 线层：网格/可达圆/轨迹/参考线/目标方向（测试深度、不写深度）
        f.glDepthMask(False)
        for i in range(-4, 5):
            c = i / 2
            major = (i % 2 == 0)
            col = "#5a748c"
            a = 0.16 if major else 0.07
            self._line((-2.2, c), (2.2, c), col, a)
            self._line((c, -2.2), (c, 2.2), col, a)
        self._circle(0, 0, ARM_L, "#6edce8", 0.28)
        self._circle(0, 0, 1.65, "#5a748c", 0.10)
        if s:
            pts = list(self.trail)
            for i in range(1, len(pts)):
                fade = (i / len(pts)) ** 1.5
                self._line(pts[i - 1], pts[i], "#6edce8", 0.5 * fade, y=0.008)
            ex, ez = ARM_L * math.cos(theta), ARM_L * math.sin(theta)
            self._dash_line((0, 0), (ex, ez), "#8fa7bb", 0.10, y=0.01)
            # 倒立参考线（臂端竖直向上的虚线）
            self._dash_line3((ex, HUB_Y + 0.04, ez),
                             (ex, HUB_Y + ROD_L, ez), "#8fa7bb", 0.4)
        self._flush_lines(f)

        # 5) 目标/预览标记：光柱 + 地面标记 + 方向虚线（半透明、不写深度）
        if s:
            target = s.get("target", 0) / 100 * math.pi / 180
            if s.get("mode") == 3:
                self._marker(target, "#ffbd69")
            if self._preview is not None:
                self._marker(self._preview, "#6edce8")
                self._draw_ghost_arm(self._preview)
            self._flush_lines(f)
        f.glDepthMask(True)

    def _marker(self, phi, color):
        px, pz = ARM_L * math.cos(phi), ARM_L * math.sin(phi)
        self._draw_mesh("pillar", m_translate(px, 0, pz), self._qcolor(color),
                        alpha=0.12, unlit=1.0)
        self._draw_mesh("mark", m_translate(px, 0, pz), self._qcolor(color),
                        alpha=0.35, unlit=1.0)
        self._dash_line((0, 0), (px, pz), color, 0.55, y=0.012)
        self._circle(px, pz, 0.09, color, 0.9, y=0.014, seg=24)

    def _draw_ghost_arm(self, phi):
        arm_m = compose(m_translate(0, HUB_Y, 0), m_rotate(-math.degrees(phi), 0, 1, 0))
        f = self.context().functions()
        f.glDepthMask(False)
        self._draw_mesh("arm", arm_m, (0.43, 0.86, 0.91), alpha=0.4, unlit=1.0)
        self._draw_mesh("tip", arm_m * m_translate(ARM_L, 0.1, 0),
                        (0.43, 0.86, 0.91), alpha=0.5, unlit=1.0)
        f.glDepthMask(True)

    @staticmethod
    def _qcolor(name):
        c = QtGui.QColor(name)
        return (c.redF(), c.greenF(), c.blueF())
