#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
障碍物场景可视化编辑器（2D 俯视）。

功能：
  - 编辑静态障碍物：立方体 / 圆柱（位置、大小、形状）
  - 编辑动态障碍物：圆柱 / 球体（位置、大小、形状、轨迹方向与长度；
    仅直线往复运动，最大速度默认 1.5 m/s、可编辑）
  - 画布上点击/拖拽选择与移动障碍物，属性面板精确编辑
  - 画布框选 / Ctrl 点选多选，右侧列表 Shift 范围 / Ctrl 逐个多选；
    多选障碍物可整体拖动或组编辑共有参数
  - 时间滑条 + 播放：预览动态障碍物直线往复运动与扫掠区域
  - 冲突检查：静态-静态重叠、动态扫掠与静态相交、动态-动态扫掠重叠红色高亮
  - 保存/加载 YAML 场景文件（与 obs_generator 节点共用格式）
  - 记住上次窗口大小、左右区域占比与最近打开的文件，重启自动恢复
  - 数值输入框输入完成后按 Enter（或失焦）才生效，不随按键即时修改
  - 可选 --publish：以 ROS 节点方式实时发布当前场景（需先 source devel/setup.bash）

用法：
  python3 scripts/scene_editor.py                          # 编辑模式
  python3 scripts/scene_editor.py --publish                # 编辑 + 实时发布到 RViz
  python3 scripts/scene_editor.py --scene path/to/x.yaml   # 打开指定场景
"""

import argparse
import math
import os
import struct
import sys

from PyQt5.QtCore import QLineF, QPointF, QRect, QRectF, QSettings, Qt, QTimer
from PyQt5.QtGui import QBrush, QColor, QPainter, QPen
from PyQt5.QtWidgets import (
    QAbstractItemView,
    QAbstractSpinBox,
    QAction,
    QApplication,
    QComboBox,
    QDoubleSpinBox,
    QFileDialog,
    QFormLayout,
    QGridLayout,
    QGraphicsEllipseItem,
    QGraphicsItem,
    QGraphicsLineItem,
    QGraphicsRectItem,
    QGraphicsScene,
    QGraphicsSimpleTextItem,
    QGraphicsView,
    QHBoxLayout,
    QLabel,
    QListWidget,
    QListWidgetItem,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QRubberBand,
    QSlider,
    QSplitter,
    QVBoxLayout,
    QWidget,
)

try:
    import yaml
except ImportError:
    yaml = None

SPEED_MAX_DEFAULT = 1.5  # 动态障碍物最大速度默认值 (m/s)


# ============================== 数据模型 ==============================
class StaticObstacle:
    def __init__(self, oid=0, shape="cuboid", x=0.0, y=0.0,
                 lx=1.0, ly=1.0, h=1.0, radius=0.5):
        self.oid = oid
        self.shape = shape          # cuboid | cylinder
        self.x = x
        self.y = y
        self.lx = lx                # cuboid 底面长
        self.ly = ly                # cuboid 底面宽
        self.h = h                  # 高度
        self.radius = radius        # cylinder 半径

    def to_dict(self):
        d = {"id": self.oid, "shape": self.shape, "x": self.x, "y": self.y}
        if self.shape == "cylinder":
            d["radius"] = self.radius
            d["height"] = self.h
        else:
            d["size"] = [self.lx, self.ly, self.h]
        return d

    @staticmethod
    def from_dict(d):
        shape = d.get("shape", "cuboid")
        s = StaticObstacle(
            oid=d.get("id", 0), shape=shape,
            x=float(d.get("x", 0.0)), y=float(d.get("y", 0.0)))
        if shape == "cylinder":
            s.radius = float(d.get("radius", 0.5))
            s.h = float(d.get("height", 1.0))
        elif "size" in d and len(d["size"]) == 3:
            s.lx, s.ly, s.h = [float(v) for v in d["size"]]
        else:
            s.lx = float(d.get("lx", 1.0))
            s.ly = float(d.get("ly", 1.0))
            s.h = float(d.get("h", 1.0))
        return s

    def aabb(self):
        """返回 (minx, maxx, miny, maxy, minz, maxz)"""
        if self.shape == "cylinder":
            return (self.x - self.radius, self.x + self.radius,
                    self.y - self.radius, self.y + self.radius, 0.0, self.h)
        return (self.x - self.lx / 2.0, self.x + self.lx / 2.0,
                self.y - self.ly / 2.0, self.y + self.ly / 2.0, 0.0, self.h)


class DynamicObstacle:
    def __init__(self, oid=0, shape="cylinder", radius=0.3, height=2.0,
                 z_center=1.0, cx=0.0, cy=0.0, direction_deg=90.0,
                 length=3.0, phase=0.0):
        self.oid = oid
        self.shape = shape          # cylinder | sphere
        self.radius = radius
        self.height = height if shape == "cylinder" else 0.0
        self.z_center = z_center
        self.cx = cx
        self.cy = cy
        self.direction_deg = direction_deg
        self.length = length        # 轨迹峰峰长度，振幅 = length/2
        self.speed_max = SPEED_MAX_DEFAULT
        self.phase = phase

    @property
    def amplitude(self):
        return self.length / 2.0

    @property
    def omega(self):
        return self.speed_max / self.amplitude if self.amplitude > 0 else 0.0

    @property
    def period(self):
        return 2.0 * math.pi / self.omega if self.omega > 0 else 10.0

    def to_dict(self):
        return {
            "id": self.oid, "shape": self.shape, "radius": self.radius,
            "height": self.height if self.shape == "cylinder" else 0.0,
            "z_center": self.z_center, "center": [self.cx, self.cy],
            "direction_deg": self.direction_deg, "length": self.length,
            "speed_max": self.speed_max, "phase": self.phase,
        }

    @staticmethod
    def from_dict(d):
        shape = d.get("shape", "cylinder")
        o = DynamicObstacle(
            oid=d.get("id", 0), shape=shape,
            radius=float(d.get("radius", 0.3)),
            height=float(d.get("height", 2.0)),
            z_center=float(d.get("z_center", 1.0)),
            direction_deg=float(d.get("direction_deg", 90.0)),
            length=float(d.get("length", 3.0)),
            phase=float(d.get("phase", 0.0)))
        if "center" in d and len(d["center"]) == 2:
            o.cx, o.cy = [float(v) for v in d["center"]]
        o.speed_max = float(d.get("speed_max", SPEED_MAX_DEFAULT))
        return o

    def position(self, t):
        """世界系平面位置 (x, y)"""
        s = self.amplitude * math.sin(self.omega * t + self.phase)
        rad = math.radians(self.direction_deg)
        return (self.cx + s * math.cos(rad), self.cy + s * math.sin(rad))

    def swept_aabb(self):
        """扫掠区域 AABB（含半径膨胀），返回 (minx, maxx, miny, maxy, minz, maxz)"""
        rad = math.radians(self.direction_deg)
        ax = math.cos(rad) * self.amplitude
        ay = math.sin(rad) * self.amplitude
        minx = min(self.cx - ax, self.cx + ax) - self.radius
        maxx = max(self.cx - ax, self.cx + ax) + self.radius
        miny = min(self.cy - ay, self.cy + ay) - self.radius
        maxy = max(self.cy - ay, self.cy + ay) + self.radius
        if self.shape == "cylinder":
            minz, maxz = 0.0, self.height
        else:
            minz, maxz = self.z_center - self.radius, self.z_center + self.radius
        return (minx, maxx, miny, maxy, minz, maxz)


def boxes_overlap(a, b):
    return (a[0] < b[1] and a[1] > b[0] and
            a[2] < b[3] and a[3] > b[2] and
            a[4] < b[5] and a[5] > b[4])


class SceneModel:
    """场景模型：YAML 读写 + 障碍物增删改 + 冲突检查"""

    def __init__(self):
        self.frame_id = "world"
        self.scene = {
            "left_x": -20.0, "center_y": 0.0, "size_x": 20.0,
            "size_y": 20.0, "size_z": 3.0, "resolution": 0.1,
        }
        self.statics = []
        self.dynamics = []

    # ---- 序列化 ----
    def to_dict(self):
        return {
            "version": 1,
            "frame_id": self.frame_id,
            "scene": self.scene,
            "static_obstacles": [s.to_dict() for s in self.statics],
            "dynamic_obstacles": [d.to_dict() for d in self.dynamics],
        }

    def load_dict(self, data):
        self.frame_id = data.get("frame_id", "world")
        if "scene" in data:
            for k, v in data["scene"].items():
                self.scene[k] = float(v)
        self.statics = [StaticObstacle.from_dict(d)
                        for d in data.get("static_obstacles", [])]
        self.dynamics = [DynamicObstacle.from_dict(d)
                         for d in data.get("dynamic_obstacles", [])]

    def save(self, path):
        if yaml is None:
            raise RuntimeError("缺少 pyyaml，无法保存")
        with open(path, "w", encoding="utf-8") as f:
            yaml.safe_dump(self.to_dict(), f, allow_unicode=True,
                           sort_keys=False, default_flow_style=False)

    def load(self, path):
        if yaml is None:
            raise RuntimeError("缺少 pyyaml，无法加载")
        with open(path, "r", encoding="utf-8") as f:
            self.load_dict(yaml.safe_load(f))

    # ---- 增删 ----
    def next_id(self, items):
        return max([it.oid for it in items], default=-1) + 1

    def add_static(self, shape="cuboid"):
        s = StaticObstacle(oid=self.next_id(self.statics), shape=shape)
        s.x = self.scene["left_x"] + self.scene["size_x"] * 0.4
        s.y = self.scene["center_y"]
        self.statics.append(s)
        return s

    def add_dynamic(self, shape="cylinder"):
        d = DynamicObstacle(oid=self.next_id(self.dynamics), shape=shape)
        d.cx = self.scene["left_x"] + self.scene["size_x"] * 0.6
        d.cy = self.scene["center_y"]
        self.dynamics.append(d)
        return d

    # ---- 冲突检查 ----
    def check_collisions(self):
        """返回 [(key1, key2, 描述), ...]，key = ("static"/"dynamic", oid)"""
        warns = []
        statics_box = [(s.oid, s.aabb()) for s in self.statics]
        for i in range(len(statics_box)):
            for j in range(i + 1, len(statics_box)):
                if boxes_overlap(statics_box[i][1], statics_box[j][1]):
                    warns.append((("static", statics_box[i][0]),
                                  ("static", statics_box[j][0]),
                                  "静态障碍物 #%d 与 #%d 重叠" %
                                  (statics_box[i][0], statics_box[j][0])))
        dyn_box = [(d.oid, d.swept_aabb()) for d in self.dynamics]
        for d_oid, dbox in dyn_box:
            for s_oid, sbox in statics_box:
                if boxes_overlap(dbox, sbox):
                    warns.append((("dynamic", d_oid), ("static", s_oid),
                                  "动态障碍物 #%d 扫掠区域与静态障碍物 #%d 相交" %
                                  (d_oid, s_oid)))
        for i in range(len(dyn_box)):
            for j in range(i + 1, len(dyn_box)):
                if boxes_overlap(dyn_box[i][1], dyn_box[j][1]):
                    warns.append((("dynamic", dyn_box[i][0]),
                                  ("dynamic", dyn_box[j][0]),
                                  "动态障碍物 #%d 与 #%d 扫掠区域相交" %
                                  (dyn_box[i][0], dyn_box[j][0])))
        return warns


# ============================== 画布 ==============================
COLOR_STATIC = QColor(80, 160, 220)
COLOR_DYN = QColor(240, 150, 60)
COLOR_COLLIDE = QColor(220, 40, 40)
COLOR_SELECT = QColor(30, 160, 80)  # 多选高亮描边
COLOR_SWEEP = QColor(240, 150, 60, 90)
HANDLE_SIZE = 10.0  # 缩放手柄屏幕像素尺寸


class CommitSpin(QDoubleSpinBox):
    """数值输入框：输入过程中不即时写模型，按 Enter / 失焦 / 步进按钮才提交。
    keyboardTracking=False：输入数字期间内部值不变，Enter 或焦点离开时才
    valueChanged 一次；箭头/滚轮属于显式操作，变化即提交。"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setKeyboardTracking(False)
        self._mixed = False  # 组编辑中该字段所选值不一致，显示“多值”占位

    def set_mixed(self, mixed):
        """组编辑：各选中项取值不一致时清空文本并显示占位符（不显示误导数值）"""
        self._mixed = bool(mixed)
        le = self.lineEdit()
        if self._mixed:
            le.setPlaceholderText("多值")
            le.setText("")
            self.setButtonSymbols(QAbstractSpinBox.NoButtons)  # 多值时禁用箭头防误改
        else:
            le.setPlaceholderText("")
            self.setButtonSymbols(QAbstractSpinBox.UpDownArrows)

    def set_value_silent(self, value):
        """程序回填数值，不触发提交回调"""
        blocked = self.blockSignals(True)
        self.setValue(float(value))
        self.set_mixed(False)
        self.blockSignals(blocked)


class SceneCanvas(QGraphicsView):
    def __init__(self, app):
        super().__init__()
        self.app = app
        self.auto_fit = True  # 视图尺寸变化时自动重新适配场景
        self._scene = QGraphicsScene(self)
        self.setScene(self._scene)
        self.setRenderHint(QPainter.Antialiasing)
        self.setDragMode(QGraphicsView.NoDrag)
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.setResizeAnchor(QGraphicsView.AnchorViewCenter)
        self.items_by_key = {}
        # 鼠标编辑状态
        self._edit_mode = None  # None | "move" | "resize"
        self._edit_key = None
        self._start_cursor_world = (0.0, 0.0)
        self._move_starts = []  # [(key, (x0, y0)), ...] 多选整体平移的每项起点
        # 框选（矩形多选）状态
        self._rubber = None
        self._rubber_start_vp = None  # 按下时视口坐标
        self._rubber_add = False      # True=追加到现有选择（Ctrl/Shift 框选）
        self._rubber_active = False   # 是否已拖出有效选框
        self._resize_corner = 0
        self._resize_anchor = (0.0, 0.0)
        self._resize_center = (0.0, 0.0)

    # ---- 坐标映射（Qt y 向下，世界 y 向上）----
    def world_to_scene(self, x, y):
        return QPointF(x, -y)

    def scene_to_world(self, p):
        return (p.x(), -p.y())

    def _to_world(self, viewport_pos):
        return self.scene_to_world(self.mapToScene(viewport_pos))

    def fit_scene(self):
        sc = self.app.model.scene
        rect = QRectF(sc["left_x"],
                      -(sc["center_y"] + sc["size_y"] / 2.0),
                      sc["size_x"], sc["size_y"])
        self.setSceneRect(rect)
        if self.viewport().width() < 10 or self.viewport().height() < 10:
            return  # 视图尚未完成布局，等 showEvent/resizeEvent 再适配
        self.fitInView(rect, Qt.KeepAspectRatio)
        print("[scene_editor] 视图适配: %.1f px/m (视口 %dx%d)" %
              (self.transform().m11(), self.viewport().width(),
               self.viewport().height()))

    def showEvent(self, event):
        super().showEvent(event)
        if self.auto_fit:
            self.fit_scene()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        if self.auto_fit:
            self.fit_scene()

    def wheelEvent(self, event):
        # 滚轮缩放（以鼠标位置为中心），随时可手动修正显示比例
        factor = 1.2 if event.angleDelta().y() > 0 else 1.0 / 1.2
        self.scale(factor, factor)
        self.refresh()  # 重新居中恒定像素大小的标签

    def refresh(self):
        """全量重绘画布（含网格、静态/动态障碍物、扫掠区域）"""
        self._scene.clear()
        self.items_by_key.clear()
        sc = self.app.model.scene
        # 场景边界 + 网格
        x0 = sc["left_x"]
        x1 = sc["left_x"] + sc["size_x"]
        y0 = sc["center_y"] - sc["size_y"] / 2.0
        y1 = sc["center_y"] + sc["size_y"] / 2.0
        grid = 1.0 if sc["size_x"] < 40 else 2.0
        pen_grid = QPen(QColor(0, 0, 0, 25), 0, Qt.DashLine)
        pen_grid.setCosmetic(True)
        x = math.ceil(x0 / grid) * grid
        while x < x1:
            self._scene.addLine(QLineF(self.world_to_scene(x, y0),
                                       self.world_to_scene(x, y1)), pen_grid)
            x += grid
        y = math.ceil(y0 / grid) * grid
        while y < y1:
            self._scene.addLine(QLineF(self.world_to_scene(x0, y),
                                       self.world_to_scene(x1, y)), pen_grid)
            y += grid
        border = QPen(QColor(0, 0, 0, 120))
        border.setCosmetic(True)
        self._scene.addRect(QRectF(x0, -y1, x1 - x0, y1 - y0), border)

        colliding = self.app.colliding_keys()
        for idx, s in enumerate(self.app.model.statics):
            key = ("static", s.oid)
            bad = key in colliding
            sel = key in self.app.selected_keys
            color = COLOR_COLLIDE if bad else (COLOR_SELECT if sel
                                               else COLOR_STATIC)
            pen = QPen(color, 3.0 if (bad or sel) else 1.5)
            pen.setCosmetic(True)
            brush = QBrush(color.lighter(150))
            if s.shape == "cylinder":
                r = s.radius
                item = QGraphicsEllipseItem(
                    QRectF(self.world_to_scene(s.x - r, s.y + r),
                           self.world_to_scene(s.x + r, s.y - r)))
            else:
                item = QGraphicsRectItem(
                    QRectF(self.world_to_scene(s.x - s.lx / 2.0,
                                               s.y + s.ly / 2.0),
                           self.world_to_scene(s.x + s.lx / 2.0,
                                               s.y - s.ly / 2.0)))
            item.setPen(pen)
            item.setBrush(brush)
            item.setFlag(QGraphicsItem.ItemIsSelectable, True)
            item.setData(0, key)
            self._scene.addItem(item)
            self.items_by_key[key] = item
            label = QGraphicsSimpleTextItem("S%d" % s.oid)
            label.setBrush(QColor(30, 60, 100))
            # 默认字体像素尺寸会被解释为场景单位（约 12 米高），会盖住整幅
            # 地图；忽略变换后标签恒为屏幕像素大小，位置按当前缩放换算居中。
            label.setFlag(QGraphicsItem.ItemIgnoresTransformations, True)
            self._place_label(label, s.x, s.y)
            label.setData(0, key)
            self._scene.addItem(label)
            if sel and len(self.app.selected_keys) == 1:
                self._add_handles(key)

        t = self.app.time
        for d in self.app.model.dynamics:
            key = ("dynamic", d.oid)
            bad = key in colliding
            sel = key in self.app.selected_keys
            color = COLOR_COLLIDE if bad else (COLOR_SELECT if sel
                                               else COLOR_DYN)
            # 扫掠线段（半透明）
            rad = math.radians(d.direction_deg)
            ax = d.amplitude * math.cos(rad)
            ay = d.amplitude * math.sin(rad)
            p1 = self.world_to_scene(d.cx - ax, d.cy - ay)
            p2 = self.world_to_scene(d.cx + ax, d.cy + ay)
            sweep = QGraphicsLineItem(QLineF(p1, p2))
            pen_sweep = QPen(COLOR_SWEEP, 8.0, Qt.SolidLine)
            pen_sweep.setCosmetic(True)
            sweep.setPen(pen_sweep)
            sweep.setData(0, key)
            self._scene.addItem(sweep)
            # 当前时刻位置（圆形轮廓 + 填充）
            px, py = d.position(t)
            r = d.radius
            item = QGraphicsEllipseItem(
                QRectF(self.world_to_scene(px - r, py + r),
                       self.world_to_scene(px + r, py - r)))
            pen_dyn = QPen(color, 3.0 if (bad or sel) else 1.8)
            pen_dyn.setCosmetic(True)
            item.setPen(pen_dyn)
            item.setBrush(QBrush(color.lighter(160)))
            item.setFlag(QGraphicsItem.ItemIsSelectable, True)
            item.setData(0, key)
            self._scene.addItem(item)
            self.items_by_key[key] = item
            label = QGraphicsSimpleTextItem("D%d" % d.oid)
            label.setBrush(QColor(120, 70, 20))
            label.setFlag(QGraphicsItem.ItemIgnoresTransformations, True)
            self._place_label(label, px, py)
            label.setData(0, key)
            self._scene.addItem(label)
            if sel and len(self.app.selected_keys) == 1:
                self._add_handles(key)

    def _place_label(self, label, wx, wy):
        """把恒定像素大小的标签放在世界坐标 (wx, wy) 中心"""
        br = label.boundingRect()
        scale = max(abs(self.transform().m11()), abs(self.transform().m22()),
                    1e-6)
        label.setPos(self.world_to_scene(wx, wy) -
                     QPointF(br.width() / (2.0 * scale),
                             br.height() / (2.0 * scale)))

    # ---- 选中障碍物的几何辅助 ----
    def _display_center(self, key):
        """障碍物在世界系中的显示中心（动态障碍物取当前时刻位置）"""
        kind, oid = key
        if kind == "static":
            s = self.app.find_static(oid)
            return (s.x, s.y)
        d = self.app.find_dynamic(oid)
        return d.position(self.app.time)

    def _shape_half_extents(self, key):
        """返回 (hx, hy)：矩形半长宽 / 圆形半径"""
        kind, oid = key
        if kind == "static":
            s = self.app.find_static(oid)
            if s.shape == "cylinder":
                return (s.radius, s.radius)
            return (s.lx / 2.0, s.ly / 2.0)
        d = self.app.find_dynamic(oid)
        return (d.radius, d.radius)

    def _corner_world(self, key, corner):
        """矩形/圆形外接矩形 4 角世界坐标；corner: 0=TL,1=TR,2=BL,3=BR"""
        cx, cy = self._display_center(key)
        hx, hy = self._shape_half_extents(key)
        corners = [(cx - hx, cy + hy), (cx + hx, cy + hy),
                   (cx - hx, cy - hy), (cx + hx, cy - hy)]
        return corners[corner]

    def _add_handles(self, key):
        """在选中障碍物外接矩形的 4 角画缩放手柄（恒定像素大小）"""
        pen = QPen(QColor(20, 20, 20), 1)
        pen.setCosmetic(True)
        brush = QBrush(QColor(255, 255, 255))
        for corner in range(4):
            wx, wy = self._corner_world(key, corner)
            handle = QGraphicsRectItem(-HANDLE_SIZE / 2.0, -HANDLE_SIZE / 2.0,
                                       HANDLE_SIZE, HANDLE_SIZE)
            handle.setFlag(QGraphicsItem.ItemIgnoresTransformations, True)
            handle.setPen(pen)
            handle.setBrush(brush)
            handle.setPos(self.world_to_scene(wx, wy))
            handle.setData(0, ("handle", key, corner))
            handle.setZValue(20)
            self._scene.addItem(handle)

    # ---- 鼠标编辑：拖动移动 / 顶角缩放 ----
    def _pick(self, event):
        """几何拾取：优先手柄，其次障碍物本体（后绘制者在上）。
        返回 ("handle", key, corner) / key / None。
        不用 itemAt：cosmetic 笔宽会让线段命中区被放大数米。"""
        wx, wy = self._to_world(event.pos())
        scale = max(abs(self.transform().m11()), 1e-6)
        # 只有单选时才绘制/响应缩放手柄；多选整体编辑尺寸走属性面板
        if len(self.app.selected_keys) == 1:
            key = self.app.selected_keys[0]
            thr = (HANDLE_SIZE / 2.0 + 2.0) / scale
            for corner in range(4):
                hx, hy = self._corner_world(key, corner)
                if math.hypot(wx - hx, wy - hy) <= thr:
                    return ("handle", key, corner)
        hit = None
        for s in self.app.model.statics:
            key = ("static", s.oid)
            if s.shape == "cylinder":
                inside = math.hypot(wx - s.x, wy - s.y) <= s.radius
            else:
                inside = (abs(wx - s.x) <= s.lx / 2.0 and
                          abs(wy - s.y) <= s.ly / 2.0)
            if inside:
                hit = key
        for d in self.app.model.dynamics:
            key = ("dynamic", d.oid)
            px, py = d.position(self.app.time)
            if math.hypot(wx - px, wy - py) <= d.radius:
                hit = key
        return hit

    def _begin_move(self, event):
        """进入整体平移模式：记录当前所有选中项的各自起点"""
        self._edit_mode = "move"
        self._edit_key = None
        self._start_cursor_world = self._to_world(event.pos())
        self._move_starts = []
        for key in self.app.selected_keys:
            kind, oid = key
            if kind == "static":
                s = self.app.find_static(oid)
                self._move_starts.append((key, (s.x, s.y)))
            else:
                d = self.app.find_dynamic(oid)
                self._move_starts.append((key, (d.cx, d.cy)))

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.app.panel.commit_pending()
            pick = self._pick(event)
            if pick is not None and pick[0] == "handle":
                _, key, corner = pick
                self.app.select_only(key)
                self.app.panel.refresh()
                self.refresh()
                self._edit_mode = "resize"
                self._edit_key = key
                self._resize_corner = corner
                self._start_cursor_world = self._to_world(event.pos())
                self._store_resize_origin(key, corner)
                event.accept()
                return
            if pick is not None:
                key = pick
                if event.modifiers() & Qt.ControlModifier:
                    # Ctrl+点击：逐个切换多选（点在未选中项上再拖动=整组移动）
                    toggled_off = key in self.app.selected_keys
                    if toggled_off:
                        self.app.remove_from_selection(key)
                    else:
                        self.app.add_to_selection(key)
                    self.app.refresh_all()
                    if not toggled_off:
                        self._begin_move(event)
                else:
                    # 普通点击：点中已选中项保持整组选择，否则替换为单选，
                    # 随后都可整体拖动
                    if key not in self.app.selected_keys:
                        self.app.select_only(key)
                        self.app.refresh_all()
                    self._begin_move(event)
                event.accept()
                return
            # 空白处：无修饰键先取消选择；按住 Ctrl/Shift 准备追加框选
            mods = event.modifiers()
            self._rubber_add = bool(mods & (Qt.ControlModifier |
                                            Qt.ShiftModifier))
            if not self._rubber_add and self.app.selected_keys:
                self.app.clear_selection()
                self.app.panel.refresh()
                self.refresh()
            self._rubber_start_vp = event.pos()
            self._rubber_active = False
            event.accept()
            return
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        if self._edit_mode == "move" and event.buttons() & Qt.LeftButton:
            wx, wy = self._to_world(event.pos())
            dx = wx - self._start_cursor_world[0]
            dy = wy - self._start_cursor_world[1]
            for key, (x0, y0) in self._move_starts:
                self.app.set_obstacle_center(key, x0 + dx, y0 + dy)
            # 单选时同步面板坐标；多选拖动期间面板保持组编辑状态
            if len(self._move_starts) == 1:
                key, (x0, y0) = self._move_starts[0]
                if key[0] == "static":
                    self.app.panel.set_field("x", x0 + dx)
                    self.app.panel.set_field("y", y0 + dy)
                else:
                    self.app.panel.set_field("cx", x0 + dx)
                    self.app.panel.set_field("cy", y0 + dy)
            self.refresh()
            self.app.refresh_list()
            self.app.update_collisions()
            event.accept()
            return
        if self._edit_mode == "resize" and event.buttons() & Qt.LeftButton:
            wx, wy = self._to_world(event.pos())
            self._apply_resize(wx, wy)
            self.refresh()
            event.accept()
            return
        # 空白处拖出矩形选框
        if (self._rubber_start_vp is not None and
                event.buttons() & Qt.LeftButton):
            cur = event.pos()
            if not self._rubber_active:
                if (cur - self._rubber_start_vp).manhattanLength() >= 6:
                    self._rubber_active = True
                    self._rubber = QRubberBand(QRubberBand.Rectangle,
                                               self.viewport())
                    self._rubber.setGeometry(
                        QRect(self._rubber_start_vp, cur).normalized())
                    self._rubber.show()
            else:
                self._rubber.setGeometry(
                    QRect(self._rubber_start_vp, cur).normalized())
            event.accept()
            return
        # 悬停光标提示（几何拾取）
        pick = self._pick(event)
        if pick is not None and pick[0] == "handle":
            self.setCursor(Qt.SizeFDiagCursor)
        elif pick is not None:
            self.setCursor(Qt.SizeAllCursor)
        else:
            self.setCursor(Qt.ArrowCursor)
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.LeftButton and \
                self._rubber_start_vp is not None:
            if self._rubber_active and self._rubber is not None:
                keys = self._keys_in_viewport_rect(self._rubber.geometry())
                if self._rubber_add:
                    merged = list(self.app.selected_keys)
                    for k in keys:
                        if k not in merged:
                            merged.append(k)
                    self.app.selected_keys = merged
                else:
                    self.app.selected_keys = keys
                self.app.refresh_all()
            elif self._rubber_add:
                # 空白处单击（无拖动）：取消选择
                self.app.clear_selection()
                self.app.panel.refresh()
                self.refresh()
            if self._rubber is not None:
                self._rubber.hide()
                self._rubber.deleteLater()
                self._rubber = None
            self._rubber_start_vp = None
            self._rubber_active = False
            self._rubber_add = False
            event.accept()
            return
        super().mouseReleaseEvent(event)
        if self._edit_mode is not None:
            self._edit_mode = None
            self.app.refresh_all()

    def _keys_in_viewport_rect(self, vp_rect):
        """矩形框选：返回与世界系矩形相交的障碍物 key（按模型顺序）"""
        w1 = self.scene_to_world(self.mapToScene(vp_rect.topLeft()))
        w2 = self.scene_to_world(self.mapToScene(vp_rect.bottomRight()))
        x0, x1 = sorted((w1[0], w2[0]))
        y0, y1 = sorted((w1[1], w2[1]))
        hits = []
        for s in self.app.model.statics:
            key = ("static", s.oid)
            if s.shape == "cylinder":
                if self._circle_hit_rect(s.x, s.y, s.radius, x0, x1, y0, y1):
                    hits.append(key)
            elif not (s.x + s.lx / 2.0 < x0 or s.x - s.lx / 2.0 > x1 or
                      s.y + s.ly / 2.0 < y0 or s.y - s.ly / 2.0 > y1):
                hits.append(key)
        for d in self.app.model.dynamics:
            key = ("dynamic", d.oid)
            px, py = d.position(self.app.time)
            if self._circle_hit_rect(px, py, d.radius, x0, x1, y0, y1):
                hits.append(key)
        return hits

    @staticmethod
    def _circle_hit_rect(cx, cy, r, x0, x1, y0, y1):
        """圆心 (cx, cy) 半径 r 的圆与世界系轴对齐矩形是否相交"""
        nx = min(max(cx, x0), x1)
        ny = min(max(cy, y0), y1)
        return (cx - nx) ** 2 + (cy - ny) ** 2 <= r * r

    def _store_resize_origin(self, key, corner):
        """记录缩放基准：矩形用对侧角锚点，圆形用圆心"""
        kind, oid = key
        if kind == "static":
            s = self.app.find_static(oid)
            if s.shape == "cylinder":
                self._resize_center = (s.x, s.y)
            else:
                hx, hy = s.lx / 2.0, s.ly / 2.0
                opposite = {0: 3, 1: 2, 2: 1, 3: 0}
                self._resize_anchor = self._corner_world(
                    key, opposite[corner])
                self._resize_center = (s.x, s.y)
        else:
            d = self.app.find_dynamic(oid)
            self._resize_center = d.position(self.app.time)

    def _apply_resize(self, wx, wy):
        """根据鼠标世界坐标更新选中障碍物的 xy 尺寸"""
        kind, oid = self._edit_key
        if kind == "static":
            s = self.app.find_static(oid)
            if s.shape == "cylinder":
                r = math.hypot(wx - self._resize_center[0],
                               wy - self._resize_center[1]) / math.sqrt(2.0)
                s.radius = max(0.1, r)
                self.app.panel.set_field("radius", s.radius)
            else:
                ax, ay = self._resize_anchor
                s.lx = max(0.2, abs(wx - ax))
                s.ly = max(0.2, abs(wy - ay))
                s.x = (ax + wx) / 2.0
                s.y = (ay + wy) / 2.0
                self.app.panel.set_field("lx", s.lx)
                self.app.panel.set_field("ly", s.ly)
                self.app.panel.set_field("x", s.x)
                self.app.panel.set_field("y", s.y)
        else:
            d = self.app.find_dynamic(oid)
            r = math.hypot(wx - self._resize_center[0],
                           wy - self._resize_center[1]) / math.sqrt(2.0)
            d.radius = max(0.1, r)
            self.app.panel.set_field("radius", d.radius)
        self.app.refresh_list()
        self.app.update_collisions()


# ============================== 属性面板 ==============================
class PropertyPanel(QWidget):
    def __init__(self, app):
        super().__init__()
        self.app = app
        self._layout = QVBoxLayout(self)
        self._form = QFormLayout()
        self._title = QLabel("未选中")
        self._layout.addWidget(self._title)
        self._layout.addLayout(self._form)
        self._layout.addStretch(1)
        self._widgets = {}

    def _clear(self):
        while self._form.count():
            item = self._form.takeAt(0)
            w = item.widget()
            if w is not None:
                w.deleteLater()
        self._widgets.clear()

    def commit_pending(self):
        """提交仍处于编辑态（已输入但未回车）的数值框，避免被重建丢弃"""
        for w in list(self._widgets.values()):
            if isinstance(w, CommitSpin) and w.lineEdit().isModified():
                w.interpretText()
                return True
        return False

    def _add_spin(self, label, key, value, lo, hi, step=0.1, readonly=False,
                  mixed=False, commit=None):
        spin = CommitSpin()
        spin.setRange(lo, hi)
        spin.setDecimals(3)
        spin.setSingleStep(step)
        spin.setValue(value)
        if mixed:
            spin.set_mixed(True)
        spin.setReadOnly(readonly)
        cb = commit if commit is not None else \
            (lambda v: self.app.on_property_changed(key, v))
        spin.valueChanged.connect(cb)
        self._form.addRow(label, spin)
        self._widgets[key] = spin

    def _add_combo(self, label, key, value, choices):
        combo = QComboBox()
        combo.addItems(choices)
        combo.setCurrentText(value)
        combo.currentTextChanged.connect(
            lambda v: self.app.on_property_changed(key, v))
        self._form.addRow(label, combo)
        self._widgets[key] = combo

    def refresh(self):
        self._clear()
        app = self.app
        keys = list(app.selected_keys)
        if not keys:
            self._title.setText("场景设置")
            sc = app.model.scene
            self._add_spin("左边缘 x", "left_x", sc["left_x"], -500, 500, 1.0)
            self._add_spin("中心 y", "center_y", sc["center_y"], -500, 500, 1.0)
            self._add_spin("尺寸 x", "size_x", sc["size_x"], 1, 500, 1.0)
            self._add_spin("尺寸 y", "size_y", sc["size_y"], 1, 500, 1.0)
            self._add_spin("尺寸 z", "size_z", sc["size_z"], 0.1, 100, 0.5)
            self._add_spin("分辨率", "resolution", sc["resolution"], 0.05,
                           1.0, 0.05)
            return

        if len(keys) > 1:
            self._refresh_group(keys)
            return

        key = keys[0]
        kind, oid = key
        if kind == "static":
            s = app.find_static(oid)
            self._title.setText("静态障碍物 #%d" % oid)
            self._add_combo("形状", "shape", s.shape, ["cuboid", "cylinder"])
            self._add_spin("中心 x", "x", s.x, -500, 500, 0.1)
            self._add_spin("中心 y", "y", s.y, -500, 500, 0.1)
            if s.shape == "cylinder":
                self._add_spin("半径", "radius", s.radius, 0.05, 20, 0.05)
                self._add_spin("高度", "h", s.h, 0.05, 50, 0.1)
            else:
                self._add_spin("长度 lx", "lx", s.lx, 0.1, 50, 0.1)
                self._add_spin("宽度 ly", "ly", s.ly, 0.1, 50, 0.1)
                self._add_spin("高度 h", "h", s.h, 0.05, 50, 0.1)
        else:
            d = app.find_dynamic(oid)
            self._title.setText("动态障碍物 #%d（直线往复，v_max=%.1f m/s）"
                                % (oid, d.speed_max))
            self._add_combo("形状", "shape", d.shape, ["cylinder", "sphere"])
            self._add_spin("半径", "radius", d.radius, 0.05, 10, 0.05)
            if d.shape == "cylinder":
                self._add_spin("圆柱高度", "height", d.height, 0.1, 50, 0.1)
            self._add_spin("球心高度 z", "z_center", d.z_center, 0.0, 50, 0.1)
            self._add_spin("中心 x", "cx", d.cx, -500, 500, 0.1)
            self._add_spin("中心 y", "cy", d.cy, -500, 500, 0.1)
            self._add_spin("方向 (度)", "direction_deg", d.direction_deg,
                           0, 360, 1.0)
            self._add_spin("轨迹长度", "length", d.length, 0.2, 50, 0.1)
            self._add_spin("最大速度", "speed", d.speed_max, 0.1, 10, 0.1)

    # ---- 多选组编辑：只显示全部选中项共有的字段 ----
    def _refresh_group(self, keys):
        app = self.app
        kinds = {k[0] for k in keys}
        n_static = sum(1 for k in keys if k[0] == "static")
        self._title.setText("组编辑：已选 %d 个（%d 静 %d 动）" %
                            (len(keys), n_static, len(keys) - n_static))

        def add_row(label, key, getter, setter, lo, hi, step=0.1):
            vals = [getter(k) for k in keys]
            equal = all(abs(v - vals[0]) < 1e-9 for v in vals[1:])
            self._add_spin(label, key, vals[0] if equal else 0.0, lo, hi,
                           step, mixed=not equal,
                           commit=lambda v: self._apply_group(keys, setter, v))

        # 位置：静态中心 x/y 与动态运动中心 cx/cy 统一为“中心 x/y”
        def get_x(k):
            if k[0] == "static":
                return app.find_static(k[1]).x
            return app.find_dynamic(k[1]).cx

        def set_x(k, v):
            if k[0] == "static":
                app.find_static(k[1]).x = v
            else:
                app.find_dynamic(k[1]).cx = v

        def get_y(k):
            if k[0] == "static":
                return app.find_static(k[1]).y
            return app.find_dynamic(k[1]).cy

        def set_y(k, v):
            if k[0] == "static":
                app.find_static(k[1]).y = v
            else:
                app.find_dynamic(k[1]).cy = v

        add_row("中心 x", "x", get_x, set_x, -500, 500, 0.1)
        add_row("中心 y", "y", get_y, set_y, -500, 500, 0.1)

        if kinds == {"static"}:
            shapes = {app.find_static(k[1]).shape for k in keys}
            if shapes == {"cuboid"}:
                add_row("长度 lx", "lx",
                        lambda k: app.find_static(k[1]).lx,
                        lambda k, v: setattr(app.find_static(k[1]), "lx", v),
                        0.1, 50, 0.1)
                add_row("宽度 ly", "ly",
                        lambda k: app.find_static(k[1]).ly,
                        lambda k, v: setattr(app.find_static(k[1]), "ly", v),
                        0.1, 50, 0.1)
            elif shapes == {"cylinder"}:
                add_row("半径", "radius",
                        lambda k: app.find_static(k[1]).radius,
                        lambda k, v: setattr(app.find_static(k[1]), "radius",
                                             v),
                        0.05, 20, 0.05)
            if len(shapes) == 1:
                add_row("高度", "h",
                        lambda k: app.find_static(k[1]).h,
                        lambda k, v: setattr(app.find_static(k[1]), "h", v),
                        0.05, 50, 0.1)
        elif kinds == {"dynamic"}:
            add_row("半径", "radius",
                    lambda k: app.find_dynamic(k[1]).radius,
                    lambda k, v: setattr(app.find_dynamic(k[1]), "radius", v),
                    0.05, 10, 0.05)
            shapes = {app.find_dynamic(k[1]).shape for k in keys}
            if shapes == {"cylinder"}:
                add_row("圆柱高度", "height",
                        lambda k: app.find_dynamic(k[1]).height,
                        lambda k, v: setattr(app.find_dynamic(k[1]), "height",
                                             v),
                        0.1, 50, 0.1)
            add_row("球心高度 z", "z_center",
                    lambda k: app.find_dynamic(k[1]).z_center,
                    lambda k, v: setattr(app.find_dynamic(k[1]), "z_center",
                                         v),
                    0.0, 50, 0.1)
            add_row("方向 (度)", "direction_deg",
                    lambda k: app.find_dynamic(k[1]).direction_deg,
                    lambda k, v: setattr(app.find_dynamic(k[1]),
                                         "direction_deg", v % 360.0),
                    0, 360, 1.0)
            add_row("轨迹长度", "length",
                    lambda k: app.find_dynamic(k[1]).length,
                    lambda k, v: setattr(app.find_dynamic(k[1]), "length",
                                         max(0.2, v)),
                    0.2, 50, 0.1)
            add_row("最大速度", "speed",
                    lambda k: app.find_dynamic(k[1]).speed_max,
                    lambda k, v: setattr(app.find_dynamic(k[1]), "speed_max",
                                         max(0.1, v)),
                    0.1, 10, 0.1)

    def _apply_group(self, keys, setter, value):
        """组编辑提交：把同一数值绝对赋值给所有选中障碍物"""
        for k in keys:
            setter(k, value)
        self.app.refresh_all()

    def set_field(self, key, value):
        w = self._widgets.get(key)
        if w is not None:
            w.blockSignals(True)
            if isinstance(w, CommitSpin):
                w.set_value_silent(float(value))
            else:
                w.setCurrentText(str(value))
            w.blockSignals(False)


# ============================== 主窗口 ==============================
class MainWindow(QMainWindow):
    @property
    def selected_key(self):
        """兼容旧代码的“焦点项”：多选时返回最后选中项"""
        return self.selected_keys[-1] if self.selected_keys else None

    @selected_key.setter
    def selected_key(self, key):
        self.selected_keys = [key] if key is not None else []

    def __init__(self, model, scene_path=None, publish=False):
        super().__init__()
        self.model = model
        self.selected_keys = []
        self._syncing_list = False
        self.time = 0.0
        self.playing = False
        self.publish = publish
        self.scene_path = scene_path
        self._settings = QSettings("form_nav", "scene_editor")

        self.setWindowTitle("障碍物场景编辑器 (2D)")
        self.resize(1280, 760)

        # 画布 + 右侧面板
        self.canvas = SceneCanvas(self)
        self.panel = PropertyPanel(self)
        self.list_widget = QListWidget()
        self.list_widget.setSelectionMode(
            QAbstractItemView.ExtendedSelection)  # Shift 范围 / Ctrl 逐个多选
        self.list_widget.itemSelectionChanged.connect(
            self.on_list_selection_changed)
        right = QWidget()
        rl = QVBoxLayout(right)
        rl.addWidget(QLabel("障碍物列表"))
        rl.addWidget(self.list_widget)
        rl.addWidget(self.panel)
        rl.addWidget(self._build_edit_buttons())

        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(self.canvas)
        splitter.addWidget(right)
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 1)
        self.splitter = splitter  # 用于记住左右区域占比
        self.setCentralWidget(splitter)

        # 底部：时间控制
        bottom = QWidget()
        bl = QHBoxLayout(bottom)
        self.play_btn = QPushButton("播放")
        self.play_btn.clicked.connect(self.toggle_play)
        self.time_slider = QSlider(Qt.Horizontal)
        self.time_slider.setRange(0, 1000)
        self.time_slider.valueChanged.connect(self.on_slider)
        self.time_label = QLabel("t=0.0s")
        self.collision_label = QLabel("")
        self.count_label = QLabel("")
        bl.addWidget(self.play_btn)
        bl.addWidget(QLabel("时间"))
        bl.addWidget(self.time_slider, 1)
        bl.addWidget(self.time_label)
        bl.addWidget(self.collision_label, 1)
        bl.addWidget(self.count_label)
        self.statusBar().addWidget(bottom)

        # 快捷键：Ctrl+0 适应视图
        fit_act = QAction("适应视图", self)
        fit_act.setShortcut("Ctrl+0")
        fit_act.triggered.connect(self.reset_view)
        self.addAction(fit_act)
        self._anim_timer = QTimer(self)
        self._anim_timer.timeout.connect(self.on_tick)
        self._anim_timer.start(30)

        # ROS 发布（可选）
        self._ros_pubs = None
        self._ros_start = None
        self._ros_timer = None
        if self.publish:
            self._init_ros()

        self.refresh_all()

    # ---- 编辑按钮（右侧属性面板下方）----
    def _build_edit_buttons(self):
        box = QWidget()
        lay = QGridLayout(box)
        lay.setContentsMargins(0, 6, 0, 0)
        buttons = [
            ("新建", self.new_scene),
            ("打开...", self.open_scene),
            ("保存", self.save_scene),
            ("另存为...", self.save_scene_as),
            ("+静态立方体", lambda: self.add_obstacle("static", "cuboid")),
            ("+静态圆柱", lambda: self.add_obstacle("static", "cylinder")),
            ("+动态圆柱", lambda: self.add_obstacle("dynamic", "cylinder")),
            ("+动态球体", lambda: self.add_obstacle("dynamic", "sphere")),
            ("删除", self.delete_selected),
            ("复制", self.duplicate_selected),
            ("适应视图 (Ctrl+0)", self.reset_view),
        ]
        for i, (text, fn) in enumerate(buttons):
            btn = QPushButton(text)
            btn.clicked.connect(fn)
            lay.addWidget(btn, i // 2, i % 2)
        return box

    def reset_view(self):
        """恢复自动适配并重新铺满视口"""
        self.canvas.auto_fit = True
        self.canvas.fit_scene()

    # ---- 刷新 ----
    def refresh_all(self):
        self.canvas.refresh()
        self.refresh_list()
        self.panel.refresh()
        self.update_collisions()

    def refresh_list(self):
        self.list_widget.blockSignals(True)
        self.list_widget.clear()
        for s in self.model.statics:
            item = QListWidgetItem(
                "静态 #%d  %s  (%.2f, %.2f)" %
                (s.oid, s.shape, s.x, s.y))
            item.setData(Qt.UserRole, ("static", s.oid))
            self.list_widget.addItem(item)
        for d in self.model.dynamics:
            item = QListWidgetItem(
                "动态 #%d  %s  (%.2f, %.2f)  dir=%.0f° len=%.2f" %
                (d.oid, d.shape, d.cx, d.cy, d.direction_deg, d.length))
            item.setData(Qt.UserRole, ("dynamic", d.oid))
            self.list_widget.addItem(item)
        # 按多选集恢复列表选中行
        first = None
        for i in range(self.list_widget.count()):
            item = self.list_widget.item(i)
            if item.data(Qt.UserRole) in self.selected_keys:
                item.setSelected(True)
                if first is None:
                    first = item
        if first is not None:
            self.list_widget.setCurrentItem(first)
        self.list_widget.blockSignals(False)

    def update_collisions(self):
        warns = self.model.check_collisions()
        if warns:
            self.collision_label.setStyleSheet("color: red;")
            self.collision_label.setText(
                "冲突: %s" % "; ".join(w[2] for w in warns[:3]))
        else:
            self.collision_label.setStyleSheet("color: green;")
            self.collision_label.setText("无冲突")
        self.count_label.setText("静态 %d | 动态 %d | 已选 %d" %
                                 (len(self.model.statics),
                                  len(self.model.dynamics),
                                  len(self.selected_keys)))

    def colliding_keys(self):
        keys = set()
        for k1, k2, _msg in self.model.check_collisions():
            keys.add(k1)
            keys.add(k2)
        return keys

    # ---- 查找 ----
    def find_static(self, oid):
        return next(s for s in self.model.statics if s.oid == oid)

    def find_dynamic(self, oid):
        return next(d for d in self.model.dynamics if d.oid == oid)

    # ---- 选择管理 ----
    def select_key(self, key):
        self.selected_keys = [key] if key is not None else []
        self.panel.refresh()
        self.refresh_list()

    def select_only(self, key):
        self.selected_keys = [key] if key is not None else []

    def clear_selection(self):
        self.selected_keys = []

    def add_to_selection(self, key):
        if key not in self.selected_keys:
            self.selected_keys.append(key)

    def remove_from_selection(self, key):
        if key in self.selected_keys:
            self.selected_keys.remove(key)

    def toggle_selection(self, key):
        if key in self.selected_keys:
            self.remove_from_selection(key)
        else:
            self.add_to_selection(key)

    # ---- 编辑操作 ----
    def add_obstacle(self, kind, shape):
        if kind == "static":
            obj = self.model.add_static(shape)
            key = ("static", obj.oid)
        else:
            obj = self.model.add_dynamic(shape)
            key = ("dynamic", obj.oid)
        self.selected_keys = [key]
        self.refresh_all()

    def delete_selected(self):
        if not self.selected_keys:
            return
        del_static = {oid for kind, oid in self.selected_keys
                      if kind == "static"}
        del_dynamic = {oid for kind, oid in self.selected_keys
                       if kind == "dynamic"}
        self.model.statics = [s for s in self.model.statics
                              if s.oid not in del_static]
        self.model.dynamics = [d for d in self.model.dynamics
                               if d.oid not in del_dynamic]
        self.selected_keys = []
        self.refresh_all()

    def duplicate_selected(self):
        if not self.selected_keys:
            return
        new_keys = []
        for key in list(self.selected_keys):
            kind, oid = key
            if kind == "static":
                src = self.find_static(oid)
                cp = StaticObstacle.from_dict(src.to_dict())
                cp.oid = self.model.next_id(self.model.statics)
                cp.x += 1.0
                self.model.statics.append(cp)
                new_keys.append(("static", cp.oid))
            else:
                src = self.find_dynamic(oid)
                cp = DynamicObstacle.from_dict(src.to_dict())
                cp.oid = self.model.next_id(self.model.dynamics)
                cp.cx += 1.0
                self.model.dynamics.append(cp)
                new_keys.append(("dynamic", cp.oid))
        self.selected_keys = new_keys
        self.refresh_all()

    # ---- 属性/画布回调 ----
    def on_property_changed(self, key, value):
        if self.selected_key is None:
            if key in self.model.scene:
                self.model.scene[key] = value
                self.canvas.fit_scene()
            self.canvas.refresh()
            return
        kind, oid = self.selected_key
        if kind == "static":
            s = self.find_static(oid)
            if key == "shape":
                s.shape = str(value)
            elif key == "x":
                s.x = value
            elif key == "y":
                s.y = value
            elif key == "lx":
                s.lx = value
            elif key == "ly":
                s.ly = value
            elif key == "h":
                s.h = value
            elif key == "radius":
                s.radius = value
        else:
            d = self.find_dynamic(oid)
            if key == "shape":
                d.shape = str(value)
                d.height = d.height if d.shape == "cylinder" else 0.0
            elif key == "radius":
                d.radius = value
            elif key == "height":
                d.height = value
            elif key == "z_center":
                d.z_center = value
            elif key == "cx":
                d.cx = value
            elif key == "cy":
                d.cy = value
            elif key == "direction_deg":
                d.direction_deg = value % 360.0
            elif key == "length":
                d.length = max(0.2, value)
            elif key == "speed":
                d.speed_max = max(0.1, value)
        self.refresh_all()

    def set_obstacle_center(self, key, x, y):
        """只更新模型位置（画布整体平移使用，不触发面板重建）"""
        kind, oid = key
        if kind == "static":
            s = self.find_static(oid)
            s.x, s.y = x, y
        else:
            d = self.find_dynamic(oid)
            d.cx, d.cy = x, y

    # ---- 列表回调 ----
    def on_list_selection_changed(self):
        """列表 Shift/Ctrl 多选变化 -> 同步画布与属性面板"""
        if self._syncing_list:
            return
        self._syncing_list = True
        keys = []
        for i in range(self.list_widget.count()):
            item = self.list_widget.item(i)
            if item.isSelected():
                keys.append(item.data(Qt.UserRole))
        self.selected_keys = keys
        self.canvas.refresh()
        self.panel.refresh()
        self.update_collisions()
        self._syncing_list = False

    # ---- 时间动画 ----
    def toggle_play(self):
        self.playing = not self.playing
        self.play_btn.setText("暂停" if self.playing else "播放")

    def on_slider(self, v):
        self.time = v / 1000.0 * 20.0  # 0..20s
        self.time_label.setText("t=%.1fs" % self.time)
        self.canvas.refresh()

    def on_tick(self):
        if self.playing:
            self.time = (self.time + 0.03) % 20.0
            self.time_slider.blockSignals(True)
            self.time_slider.setValue(int(self.time / 20.0 * 1000))
            self.time_slider.blockSignals(False)
            self.time_label.setText("t=%.1fs" % self.time)
            self.canvas.refresh()
        if self._ros_timer is not None and self._ros_pubs is not None:
            self._publish_ros()

    # ---- 文件 ----
    def new_scene(self):
        self.model = SceneModel()
        self.selected_keys = []
        self.scene_path = None
        self.refresh_all()
        self.canvas.fit_scene()
        self._remember_scene()

    def open_scene(self):
        start_dir = self._settings.value("last/dir", "") or ""
        path, _ = QFileDialog.getOpenFileName(
            self, "打开场景", str(start_dir),
            "YAML 场景 (*.yaml *.yml);;所有文件 (*)")
        if not path:
            return
        try:
            self.model.load(path)
        except Exception as e:
            QMessageBox.critical(self, "加载失败", str(e))
            return
        self.scene_path = path
        self.selected_keys = []
        self.refresh_all()
        self.canvas.fit_scene()
        self._remember_scene()

    def save_scene(self):
        if self.scene_path is None:
            self.save_scene_as()
            return
        try:
            self.model.save(self.scene_path)
            self.count_label.setText("已保存: %s" % self.scene_path)
        except Exception as e:
            QMessageBox.critical(self, "保存失败", str(e))

    def save_scene_as(self):
        path, _ = QFileDialog.getSaveFileName(
            self, "保存场景",
            self.scene_path or
            os.path.join(str(self._settings.value("last/dir", "") or ""),
                         "scene.yaml"),
            "YAML 场景 (*.yaml *.yml)")
        if not path:
            return
        self.scene_path = path
        self.save_scene()
        self._remember_scene()

    def _remember_scene(self):
        """记住当前文件路径与所在目录，便于下次启动自动恢复"""
        self._settings.setValue("last/scene_path", self.scene_path or "")
        if self.scene_path:
            self._settings.setValue("last/dir",
                                    os.path.dirname(self.scene_path))

    # ---- 窗口状态记忆（大小 / 左右占比）----
    def restore_ui_settings(self):
        geometry = self._settings.value("window/geometry")
        if geometry is not None:
            self.restoreGeometry(geometry)
        splitter_state = self._settings.value("window/splitter")
        if splitter_state is not None:
            self.splitter.restoreState(splitter_state)

    def closeEvent(self, event):
        self._settings.setValue("window/geometry", self.saveGeometry())
        self._settings.setValue("window/splitter", self.splitter.saveState())
        self._remember_scene()
        super().closeEvent(event)

    # ---- 可选 ROS 实时发布 ----
    def _init_ros(self):
        try:
            import rospy
            from sensor_msgs.msg import PointCloud2
            from form_nav.msg import DynamicObstacleTrajectory
        except ImportError as e:
            QMessageBox.critical(
                self, "ROS 导入失败",
                "%s\n请先 source devel/setup.bash 再运行 --publish。" % e)
            sys.exit(1)
        self._PointCloud2 = PointCloud2
        self._DynTraj = DynamicObstacleTrajectory
        rospy.init_node("scene_editor", anonymous=True)
        self._ros_pubs = {
            "global": rospy.Publisher("/map_generator/global_cloud",
                                      PointCloud2, queue_size=1),
            "local": rospy.Publisher("/map_generator/local_cloud",
                                     PointCloud2, queue_size=1),
            "dynamic": rospy.Publisher("/map_generator/dynamic_cloud",
                                       PointCloud2, queue_size=1),
            "traj": rospy.Publisher("/dynamic_obstacles/trajectory",
                                    DynamicObstacleTrajectory, queue_size=10),
        }
        import rospy as _rp
        self._ros_start = _rp.Time.now().to_sec()
        self._ros_timer = QTimer(self)
        self._ros_timer.timeout.connect(self._publish_ros)
        self._ros_timer.start(100)

    @staticmethod
    def _make_cloud(msg_type, frame_id, stamp, points):
        from sensor_msgs.msg import PointField
        fields = [
            PointField("x", 0, PointField.FLOAT32, 1),
            PointField("y", 4, PointField.FLOAT32, 1),
            PointField("z", 8, PointField.FLOAT32, 1),
            PointField("pad", 12, PointField.FLOAT32, 1),
        ]
        data = b"".join(struct.pack("<ffff", x, y, z, 0.0)
                        for x, y, z in points)
        msg = msg_type()
        msg.header.frame_id = frame_id
        msg.header.stamp = stamp
        msg.height = 1
        msg.width = len(points)
        msg.fields = fields
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = 16 * len(points)
        msg.is_dense = True
        msg.data = data
        return msg

    def _voxelize_static(self, s):
        """静态障碍物表面点云（侧面 + 顶面 + 底边一圈，内部挖空），
        与 obs_generator 节点保持一致的表面层生成逻辑。"""
        res = self.model.scene["resolution"]
        pts = []
        if s.shape == "cylinder":
            ix0 = int(math.floor((s.x - s.radius) / res))
            ix1 = int(math.ceil((s.x + s.radius) / res))
            iy0 = int(math.floor((s.y - s.radius) / res))
            iy1 = int(math.ceil((s.y + s.radius) / res))
            iz1 = int(math.ceil(s.h / res))

            def inside(ix, iy, iz):
                if iz < 0 or iz >= iz1:
                    return False
                dx = (ix + 0.5) * res - s.x
                dy = (iy + 0.5) * res - s.y
                return dx * dx + dy * dy <= s.radius ** 2

            for ix in range(ix0, ix1):
                for iy in range(iy0, iy1):
                    for iz in range(iz1):
                        if not inside(ix, iy, iz):
                            continue
                        if (not inside(ix - 1, iy, iz) or
                                not inside(ix + 1, iy, iz) or
                                not inside(ix, iy - 1, iz) or
                                not inside(ix, iy + 1, iz) or
                                not inside(ix, iy, iz + 1)):
                            pts.append(((ix + 0.5) * res,
                                        (iy + 0.5) * res,
                                        (iz + 0.5) * res))
        else:
            x0, x1 = s.x - s.lx / 2.0, s.x + s.lx / 2.0
            y0, y1 = s.y - s.ly / 2.0, s.y + s.ly / 2.0
            ix0, ix1 = int(math.floor(x0 / res)), int(math.ceil(x1 / res))
            iy0, iy1 = int(math.floor(y0 / res)), int(math.ceil(y1 / res))
            iz1 = int(math.ceil(s.h / res))
            for ix in range(ix0, ix1):
                for iy in range(iy0, iy1):
                    for iz in range(iz1):
                        if (ix == ix0 or ix == ix1 - 1 or
                                iy == iy0 or iy == iy1 - 1 or
                                iz == iz1 - 1):
                            pts.append(((ix + 0.5) * res,
                                        (iy + 0.5) * res,
                                        (iz + 0.5) * res))
        return pts

    def _dyn_surface_points(self, d, px, py, res):
        pts = []
        if d.shape == "cylinder":
            hei = int(math.ceil(d.height / res))
            ang = max(8, int(math.ceil(2 * math.pi * d.radius / res)))
            for t in range(hei):
                for a in range(ang):
                    th = 2 * math.pi * a / ang
                    pts.append((px + d.radius * math.cos(th),
                                py + d.radius * math.sin(th),
                                (t + 0.5) * res + 1e-2))
            rad = int(math.ceil(d.radius / res))
            for i in range(-rad, rad + 1):
                for j in range(-rad, rad + 1):
                    x = px + (i + 0.5) * res
                    y = py + (j + 0.5) * res
                    if (x - px) ** 2 + (y - py) ** 2 <= d.radius ** 2:
                        pts.append((x, y, d.height + 1e-2))
        else:
            ring = max(4, int(math.ceil(math.pi * d.radius / res)))
            for i in range(ring + 1):
                phi = math.pi * i / ring
                z = d.z_center + d.radius * math.cos(phi)
                rr = d.radius * math.sin(phi)
                ang = max(4, int(math.ceil(2 * math.pi * rr / res)))
                for a in range(ang):
                    th = 2 * math.pi * a / ang
                    pts.append((px + rr * math.cos(th),
                                py + rr * math.sin(th), z))
        return pts

    def _publish_ros(self):
        import rospy
        now = rospy.Time.now()
        t = now.to_sec() - self._ros_start
        static_pts = []
        for s in self.model.statics:
            static_pts.extend(self._voxelize_static(s))
        cloud = self._make_cloud(self._PointCloud2, self.model.frame_id,
                                 now, static_pts)
        self._ros_pubs["global"].publish(cloud)
        self._ros_pubs["local"].publish(cloud)

        dyn_pts = []
        for d in self.model.dynamics:
            px, py = d.position(t)
            dyn_pts.extend(self._dyn_surface_points(
                d, px, py, self.model.scene["resolution"]))
        if self.model.dynamics:
            self._ros_pubs["dynamic"].publish(
                self._make_cloud(self._PointCloud2, self.model.frame_id,
                                 now, dyn_pts))
        for i, d in enumerate(self.model.dynamics):
            msg = self._DynTraj()
            msg.header.stamp = now
            msg.header.frame_id = self.model.frame_id
            msg.obstacle_id = i
            msg.shape = 1 if d.shape == "cylinder" else 0
            msg.radius = d.radius
            msg.height = d.height
            msg.dt = 0.05
            msg.steps = 21
            h = 0.01
            for k in range(msg.steps):
                tk = t + k * msg.dt
                px, py = d.position(tk)
                pmx, pmy = d.position(tk - h)
                ppx, ppy = d.position(tk + h)
                vx = (ppx - pmx) / (2 * h)
                vy = (ppy - pmy) / (2 * h)
                msg.states.extend([px, py, d.z_center, 0.0, vx, vy, 0.0, 0.0])
            self._ros_pubs["traj"].publish(msg)


def main():
    parser = argparse.ArgumentParser(description="障碍物场景编辑器 (2D)")
    parser.add_argument("--scene", default=None,
                        help="YAML 场景文件路径")
    parser.add_argument("--publish", action="store_true",
                        help="以 ROS 节点方式实时发布场景（需已 source ROS）")
    args = parser.parse_args()

    default_scene = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        os.pardir, "config", "scenes", "fixed_scene.yaml")
    path = args.scene
    if path is None:
        # 记忆上次打开的文件；不存在或缺失时回退默认场景
        remembered = QSettings("form_nav", "scene_editor").value(
            "last/scene_path", "")
        if remembered and os.path.exists(str(remembered)):
            path = str(remembered)
    path = path or default_scene
    model = SceneModel()
    if os.path.exists(path):
        try:
            model.load(path)
        except Exception as e:
            print("场景加载失败: %s" % e, file=sys.stderr)
    else:
        print("场景文件不存在: %s" % path, file=sys.stderr)

    app = QApplication(sys.argv)
    win = MainWindow(model, scene_path=path, publish=args.publish)
    win.show()
    win.restore_ui_settings()
    win.canvas.fit_scene()
    # 窗口布局完全就绪后再适配一次，避免首次 fitInView 拿到未就绪的视口尺寸
    QTimer.singleShot(0, win.canvas.fit_scene)
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
