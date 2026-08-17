#!/usr/bin/env python3
"""PCD-Image Alignment Tool — LiDAR projection onto main camera image"""

import sys
import os
import numpy as np

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QLineEdit, QGroupBox, QGridLayout,
    QFileDialog, QScrollArea, QStatusBar, QSizePolicy,
    QCheckBox, QDoubleSpinBox, QSlider, QSplitter, QComboBox,
)
from PyQt5.QtCore import Qt, pyqtSignal, QPointF
from PyQt5.QtGui import QPixmap, QImage, QFont, QPalette, QColor, QPainter

import cv2
from PIL import Image as PILImage
from matplotlib import cm as mpl_cm


# ──────────────────────────────────────────────────────────────────────────────
# PCD reader
# ──────────────────────────────────────────────────────────────────────────────

def read_pcd(filepath: str) -> np.ndarray:
    """Return Nx3 float32 (x, y, z) from binary or ascii PCD."""
    with open(filepath, 'rb') as f:
        header = {}
        while True:
            raw = f.readline()
            if not raw:
                break
            line = raw.decode('utf-8', errors='ignore').strip()
            if line.startswith('#') or not line:
                continue
            parts = line.split()
            key = parts[0].upper()
            header[key] = parts[1:]
            if key == 'DATA':
                offset = f.tell()
                break

        dt_str  = header.get('DATA', ['ascii'])[0].lower()
        fields  = header.get('FIELDS', ['x', 'y', 'z'])
        sizes   = [int(s) for s in header.get('SIZE',  ['4'] * len(fields))]
        types   = header.get('TYPE',  ['F'] * len(fields))
        num_pts = int((header.get('POINTS') or header.get('WIDTH') or ['0'])[0])

        if dt_str == 'ascii':
            f.seek(offset)
            data = np.loadtxt(f, max_rows=num_pts or None)
            if data.ndim == 1:
                data = data.reshape(1, -1)
            xi, yi, zi = fields.index('x'), fields.index('y'), fields.index('z')
            return data[:, [xi, yi, zi]].astype(np.float32)

        elif dt_str == 'binary':
            tmap = {'F': 'f', 'I': 'i', 'U': 'u'}
            dt = np.dtype([(fn, tmap.get(t, 'f') + str(sz))
                           for fn, sz, t in zip(fields, sizes, types)])
            f.seek(offset)
            data = np.frombuffer(f.read(num_pts * dt.itemsize), dtype=dt)
            return np.column_stack([
                data['x'].astype(np.float32),
                data['y'].astype(np.float32),
                data['z'].astype(np.float32),
            ])

    return np.zeros((0, 3), np.float32)


# ──────────────────────────────────────────────────────────────────────────────
# Config I/O  (Python-dict format with numpy arrays)
# ──────────────────────────────────────────────────────────────────────────────

def load_config(path: str) -> dict:
    """exec() the Python-dict config file, return CAM_CONFIGS dict."""
    ns = {'np': np}
    with open(path, 'r', encoding='utf-8') as f:
        exec(f.read(), ns)
    return ns['CAM_CONFIGS']


def _fmt_row(row) -> str:
    return '[' + ', '.join(f'{v:14.8f}' for v in row) + ']'


def save_config(path: str, cfg: dict):
    """Regenerate the Python-dict config file from the in-memory dict."""
    lines = ['CAM_CONFIGS = {\n']
    cam_items = list(cfg.items())
    for ci, (name, cam) in enumerate(cam_items):
        trailing = ',' if ci < len(cam_items) - 1 else ''
        lines.append(f"    '{name}': {{\n")
        lines.append(f"        'type': '{cam['type']}',\n")

        # scale — 選用(sideL/sideR 等 pinhole 沒有 scale 鍵就跳過)
        if 'scale' in cam:
            scale = float(cam['scale'])
            if abs(scale - 2560 / 3840) < 1e-9:
                lines.append("        'scale': 2560 / 3840,\n")
            elif abs(scale - 2 / 3) < 1e-9:
                lines.append("        'scale': 2/3,\n")
            else:
                lines.append(f"        'scale': {scale},\n")

        # K  (standard cameras)
        if 'K' in cam:
            K = np.array(cam['K'])
            lines.append("        'K': np.array([\n")
            for row in K.tolist():
                lines.append(f"            {_fmt_row(row)},\n")
            lines.append(f"        ], dtype=np.float32),\n")

        # K_native  (pinhole sideL/sideR,原生 1920×1536)
        if 'K_native' in cam:
            Kn = np.array(cam['K_native'])
            lines.append("        'K_native': np.array([\n")
            for row in Kn.tolist():
                lines.append(f"            {_fmt_row(row)},\n")
            lines.append(f"        ], dtype=np.float32),\n")

        # K_base  (fisheye cameras)
        if 'K_base' in cam:
            lines.append(f"        'K_base': {list(cam['K_base'])},\n")

        # fov_half_deg  (選用,pinhole 過濾角)
        if 'fov_half_deg' in cam:
            lines.append(f"        'fov_half_deg': {float(cam['fov_half_deg'])},\n")

        # D
        D = cam['D']
        if isinstance(D, np.ndarray):
            lines.append(f"        'D': np.array({D.tolist()}, dtype=np.float32),\n")
        else:
            lines.append(f"        'D': {D},\n")

        # T  (4×4)
        T = np.array(cam['T'])
        lines.append("        'T': np.array([\n")
        for row in T.tolist():
            lines.append(f"            {_fmt_row(row)},\n")
        lines.append(f"        ], dtype=np.float32)\n")

        lines.append(f"    }}{trailing}\n")

    lines.append('}\n')
    with open(path, 'w', encoding='utf-8') as f:
        f.writelines(lines)


# ──────────────────────────────────────────────────────────────────────────────
# RPY ↔ rotation matrix  (ZYX convention, degrees)
# ──────────────────────────────────────────────────────────────────────────────

def _rpy_to_rot(roll_deg, pitch_deg, yaw_deg) -> np.ndarray:
    r, p, y = np.deg2rad(roll_deg), np.deg2rad(pitch_deg), np.deg2rad(yaw_deg)
    Rx = np.array([[1, 0, 0], [0, np.cos(r), -np.sin(r)], [0, np.sin(r), np.cos(r)]])
    Ry = np.array([[np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]])
    Rz = np.array([[np.cos(y), -np.sin(y), 0], [np.sin(y), np.cos(y), 0], [0, 0, 1]])
    return (Rz @ Ry @ Rx).astype(np.float32)


def _rot_to_rpy(R: np.ndarray):
    """Return (roll, pitch, yaw) in degrees from a 3×3 rotation matrix (ZYX)."""
    pitch = np.arctan2(-R[2, 0], np.sqrt(R[0, 0]**2 + R[1, 0]**2))
    cp = np.cos(pitch)
    if abs(cp) > 1e-6:
        roll = np.arctan2(R[2, 1] / cp, R[2, 2] / cp)
        yaw  = np.arctan2(R[1, 0] / cp, R[0, 0] / cp)
    else:
        roll = np.arctan2(-R[1, 2], R[1, 1])
        yaw  = 0.0
    return float(np.degrees(roll)), float(np.degrees(pitch)), float(np.degrees(yaw))


# ──────────────────────────────────────────────────────────────────────────────
# Projection
# ──────────────────────────────────────────────────────────────────────────────

def project_and_draw(
    pts: np.ndarray,
    img_path: str,
    K: np.ndarray,
    D: np.ndarray,
    T: np.ndarray,
    scale: float,
    min_depth: float = 0.5,
    max_depth: float = 80.0,
    point_r: int = 2,
    cam_type: str = 'standard',
    opacity: float = 1.0,
    fov_half_deg: float = 0.0,    # >0 啟用光軸夾角過濾(針孔相機建議用)
    k_native_size: tuple = None,  # (native_W, native_H) — sideL/sideR(K_native) 專用; 設定後改用 scale_x/scale_y 各別縮放
) -> QPixmap:
    """
    Project LiDAR points onto image. 支援 fisheye / pinhole 兩種模型。
      K            : 3×3 intrinsic (already in capture-resolution coords if scale==1)
      D            : pinhole → [k1,k2,p1,p2,k3];  fisheye → [k1,k2,k3,k4, 0]
      T            : 4×4 rigid transform (LiDAR → camera), translation in metres
      scale        : K is multiplied by this to get pixel coords at capture resolution
      cam_type     : 'pinhole' | 'standard' | 'fisheye'
      fov_half_deg : 光軸夾角過濾(度);0 = 不過濾
    """
    R = T[:3, :3]
    t = T[:3,  3]

    pts_c = (R @ pts.T).T + t
    if cam_type == 'fisheye':
        # 魚眼(尤其環景 ~183°):以「到相機距離」而非 z 做深度過濾,並用入射角(atan2)上限
        # 保留 θ>90° 的外圈點(z 過濾會把它們誤刪);θ 上限 93° 濾掉鏡頭視野外(背向)的點。
        _rng = np.linalg.norm(pts_c, axis=1)
        _thd = np.degrees(np.arctan2(np.hypot(pts_c[:, 0], pts_c[:, 1]), pts_c[:, 2]))
        mask = (_rng > min_depth) & (_rng < max_depth) & (_thd < 93.0)
    else:
        mask = (pts_c[:, 2] > min_depth) & (pts_c[:, 2] < max_depth)

    # 光軸夾角過濾(針孔大角度會數值摺疊,sideL/sideR 必須過濾)
    if fov_half_deg > 0:
        z = np.maximum(pts_c[:, 2], 1e-6)
        r_norm = np.sqrt(pts_c[:, 0]**2 + pts_c[:, 1]**2) / z
        ang = np.degrees(np.arctan(r_norm))
        mask = mask & (ang < fov_half_deg)

    pts_c = pts_c[mask]
    if len(pts_c) == 0:
        return QPixmap(img_path)

    depth = pts_c[:, 2]

    # ── scaled intrinsics ────────────────────────────────────────────────
    # sideL / sideR (K_native, 原生 1920x1536): 用實際影像解析度推導 scale_x, scale_y
    if k_native_size is not None:
        _peek = PILImage.open(img_path)
        _W, _H = _peek.size; _peek.close()
        sx = _W / float(k_native_size[0])
        sy = _H / float(k_native_size[1])
        Ks = np.array([
            [K[0, 0] * sx, 0,            K[0, 2] * sx],
            [0,            K[1, 1] * sy, K[1, 2] * sy],
            [0,            0,            1            ],
        ], dtype=np.float64)
    else:
        # 一般情況(main / 環景魚眼): 依實際影像解析度自動縮放 K。
        # K 之標定寬由 cx 推得(main≈2560、環景≈1280),取最接近者;
        # 影像若等於標定解析度(如 2560),res_scale=1.0 行為與原本相同(向後相容);
        # 若為 4K(3840),res_scale=1.5 自動放大。scale 滑桿仍可在其上微調。
        _peek = PILImage.open(img_path); _W, _H = _peek.size; _peek.close()
        _ref_w = min((2560, 1280), key=lambda w: abs(w - 2.0 * K[0, 2]))
        _res_scale = _W / float(_ref_w)
        s = scale * _res_scale
        Ks = np.array([
            [K[0, 0] * s, 0,           K[0, 2] * s],
            [0,           K[1, 1] * s, K[1, 2] * s],
            [0,           0,           1          ],
        ], dtype=np.float64)

    if cam_type == 'fisheye':
        # KB 魚眼投影,θ 以 atan2 計算(可達 180°)。cv2.fisheye.projectPoints 內部用
        # atan(r/z),在 θ>90°(z<0)會把外圈點摺回影像內側;環景 ~183° 魚眼必須自行實作。
        _d = D[:4].astype(np.float64).flatten()
        _x = pts_c[:, 0]; _y = pts_c[:, 1]; _z = pts_c[:, 2]
        _rxy = np.hypot(_x, _y)
        _th = np.arctan2(_rxy, _z)
        _r = _th + _d[0]*_th**3 + _d[1]*_th**5 + _d[2]*_th**7 + _d[3]*_th**9
        _sp = np.where(_rxy > 1e-9, _r / np.maximum(_rxy, 1e-9), 0.0)
        u = Ks[0, 0] * _sp * _x + Ks[0, 2]
        v = Ks[1, 1] * _sp * _y + Ks[1, 2]
    else:
        # pinhole / standard:套 OpenCV 官方 Brown-Conrady (對應 sideL/sideR tuner)
        D5 = np.zeros(5, dtype=np.float64)
        D5[:min(5, len(D))] = D[:5].astype(np.float64)
        img_pts, _ = cv2.projectPoints(
            pts_c.astype(np.float64), np.zeros(3), np.zeros(3),
            Ks, D5)
        u = img_pts.reshape(-1, 2)[:, 0]
        v = img_pts.reshape(-1, 2)[:, 1]

    # 讀影像並以 BGR 處理(走 OpenCV pipeline,跟 tuner 一致)
    img_bgr = cv2.imdecode(np.fromfile(img_path, dtype=np.uint8), cv2.IMREAD_COLOR)
    if img_bgr is None:
        img_pil = PILImage.open(img_path).convert('RGB')
        img_bgr = cv2.cvtColor(np.array(img_pil, dtype=np.uint8), cv2.COLOR_RGB2BGR)
    H, W = img_bgr.shape[:2]

    valid = (u >= 0) & (u < W) & (v >= 0) & (v < H) & np.isfinite(u) & np.isfinite(v)
    u = u[valid].astype(np.int32)
    v = v[valid].astype(np.int32)
    if len(u) == 0:
        return QPixmap(img_path)

    # 上色:pinhole(sideL/sideR)走 tuner 風格(JET + 3D 距離 / 30 m),fisheye 走 plasma + depth
    if cam_type == 'fisheye':
        d = depth[valid]
        d_n = np.clip((d - d.min()) / ((d.max() - d.min()) + 1e-6), 0.0, 1.0)
        colors_rgb = (mpl_cm.plasma(d_n)[:, :3] * 255).astype(np.uint8)
        colors_bgr = colors_rgb[:, ::-1]
    else:
        dist3d = np.linalg.norm(pts_c[valid], axis=1)
        col_vals = (np.clip(dist3d / 30.0, 0.0, 1.0) * 255).astype(np.uint8)
        colors_bgr = cv2.applyColorMap(col_vals.reshape(-1, 1), cv2.COLORMAP_JET).reshape(-1, 3)

    alpha = float(np.clip(opacity, 0.0, 1.0))
    if alpha >= 0.999:
        # 完全不透明:用 cv2.circle 畫圓(與 tuner 同)
        for i in range(len(u)):
            cv2.circle(img_bgr, (int(u[i]), int(v[i])), int(point_r),
                       colors_bgr[i].tolist(), -1)
    else:
        # 透明 blending:沿用 dx/dy 方塊填(維持 alpha 控制)
        for dy in range(-point_r, point_r + 1):
            for dx in range(-point_r, point_r + 1):
                if dx * dx + dy * dy <= point_r * point_r:
                    pv = np.clip(v + dy, 0, H - 1)
                    pu = np.clip(u + dx, 0, W - 1)
                    bg = img_bgr[pv, pu].astype(np.float32)
                    img_bgr[pv, pu] = (bg * (1.0 - alpha) + colors_bgr.astype(np.float32) * alpha).astype(np.uint8)

    # BGR → RGB → QPixmap
    img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
    data = img_rgb.tobytes()
    return QPixmap.fromImage(QImage(data, W, H, 3 * W, QImage.Format_RGB888))


# ──────────────────────────────────────────────────────────────────────────────
# MatrixEdit  — editable grid widget for numpy matrices
# ──────────────────────────────────────────────────────────────────────────────

class MatrixEdit(QGroupBox):
    def __init__(self, title: str, rows: int, cols: int, cell_w: int = 90):
        super().__init__(title)
        grid = QGridLayout(self)
        grid.setSpacing(2)
        grid.setContentsMargins(4, 4, 4, 4)
        self._edits: list[list[QLineEdit]] = []
        for i in range(rows):
            row_edits = []
            for j in range(cols):
                e = QLineEdit('0')
                e.setFixedWidth(cell_w)
                e.setAlignment(Qt.AlignCenter)
                grid.addWidget(e, i, j)
                row_edits.append(e)
            self._edits.append(row_edits)

    def set(self, mat):
        arr = np.asarray(mat, dtype=float)
        for i, row in enumerate(arr):
            for j, v in enumerate(row):
                self._edits[i][j].setText(f'{v:.7g}')

    def get(self) -> np.ndarray:
        return np.array(
            [[float(self._edits[i][j].text())
              for j in range(len(self._edits[0]))]
             for i in range(len(self._edits))],
            dtype=np.float32)


class VectorEdit(QGroupBox):
    """Single-row editable widget for a 1-D array."""
    def __init__(self, title: str, labels: list[str], cell_w: int = 100):
        super().__init__(title)
        grid = QGridLayout(self)
        grid.setSpacing(2)
        grid.setContentsMargins(4, 4, 4, 4)
        self._edits: list[QLineEdit] = []
        for j, lbl in enumerate(labels):
            grid.addWidget(QLabel(lbl + ':'), 0, j * 2)
            e = QLineEdit('0')
            e.setFixedWidth(cell_w)
            e.setAlignment(Qt.AlignCenter)
            grid.addWidget(e, 0, j * 2 + 1)
            self._edits.append(e)

    def set(self, vec):
        arr = np.asarray(vec, dtype=float).flatten()
        for i, v in enumerate(arr):
            self._edits[i].setText(f'{v:.7g}')

    def get(self) -> np.ndarray:
        return np.array([float(e.text()) for e in self._edits], dtype=np.float32)


# ──────────────────────────────────────────────────────────────────────────────
# ZoomableImageLabel — 滾輪縮放(以游標為中心) + 拖移平移 + 雙擊還原
# ──────────────────────────────────────────────────────────────────────────────

class ZoomableImageLabel(QLabel):
    """投影顯示用 QLabel:支援滾輪縮放、拖移平移、雙擊回到 fit-to-window。"""
    ZOOM_STEP = 1.15
    ZOOM_MIN  = 0.1
    ZOOM_MAX  = 30.0

    def __init__(self, parent=None):
        super().__init__(parent)
        self._orig = QPixmap()
        self._zoom = 1.0
        self._pan  = QPointF(0.0, 0.0)
        self._panning = False
        self._last_pos = QPointF(0.0, 0.0)
        self.setAlignment(Qt.AlignCenter)
        self.setStyleSheet('background:#0d0d1a;')
        self.setMouseTracking(True)
        self.setFocusPolicy(Qt.StrongFocus)
        self.setToolTip('滾輪縮放(以游標為中心)・左鍵拖移平移・雙擊還原・R 還原')

    def set_pixmap(self, pix: QPixmap):
        if pix is None or pix.isNull():
            return
        self._orig = pix
        self.update()

    def reset_view(self):
        self._zoom = 1.0
        self._pan = QPointF(0.0, 0.0)
        self.update()

    def wheelEvent(self, e):
        if self._orig.isNull():
            return
        dy = e.angleDelta().y()
        if dy == 0:
            return
        factor = self.ZOOM_STEP if dy > 0 else 1.0 / self.ZOOM_STEP
        old = self._zoom
        new = max(self.ZOOM_MIN, min(self.ZOOM_MAX, old * factor))
        if new == old:
            return
        # 以游標為錨點縮放:讓游標下方那塊影像保持原位
        try:
            cursor = QPointF(e.position())     # Qt ≥ 5.14
        except (AttributeError, TypeError):
            cursor = QPointF(e.x(), e.y())
        center = QPointF(self.width() / 2.0, self.height() / 2.0)
        ratio = new / old
        self._pan = self._pan + (cursor - center - self._pan) * (1.0 - ratio)
        self._zoom = new
        self.update()
        e.accept()

    def mousePressEvent(self, e):
        if e.button() in (Qt.LeftButton, Qt.MiddleButton):
            self._panning = True
            self._last_pos = QPointF(e.pos())
            self.setCursor(Qt.ClosedHandCursor)
            e.accept()

    def mouseMoveEvent(self, e):
        if self._panning:
            cur = QPointF(e.pos())
            self._pan += cur - self._last_pos
            self._last_pos = cur
            self.update()
            e.accept()

    def mouseReleaseEvent(self, e):
        if self._panning:
            self._panning = False
            self.unsetCursor()
            e.accept()

    def mouseDoubleClickEvent(self, e):
        self.reset_view()
        e.accept()

    def keyPressEvent(self, e):
        if e.key() == Qt.Key_R:
            self.reset_view(); e.accept()
        elif e.key() in (Qt.Key_Plus, Qt.Key_Equal):
            self._zoom = min(self.ZOOM_MAX, self._zoom * self.ZOOM_STEP)
            self.update(); e.accept()
        elif e.key() == Qt.Key_Minus:
            self._zoom = max(self.ZOOM_MIN, self._zoom / self.ZOOM_STEP)
            self.update(); e.accept()
        else:
            super().keyPressEvent(e)

    def paintEvent(self, e):
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor(13, 13, 26))
        if self._orig.isNull():
            painter.end(); return
        ow, oh = self._orig.width(), self._orig.height()
        if ow <= 0 or oh <= 0:
            painter.end(); return
        sw, sh = self.width(), self.height()
        fit_scale = min(sw / ow, sh / oh)
        s = fit_scale * self._zoom
        painter.setRenderHint(QPainter.SmoothPixmapTransform, True)
        painter.translate(sw / 2.0 + self._pan.x(), sh / 2.0 + self._pan.y())
        painter.scale(s, s)
        painter.translate(-ow / 2.0, -oh / 2.0)
        painter.drawPixmap(0, 0, self._orig)
        painter.end()


# ──────────────────────────────────────────────────────────────────────────────
# SliderParam — label + slider + value edit for one float parameter
# ──────────────────────────────────────────────────────────────────────────────

class SliderParam(QWidget):
    released = pyqtSignal()  # emitted when slider is released or Enter pressed

    STEPS = 2000  # slider integer range: 0 … STEPS

    def __init__(self, label: str, lo: float, hi: float,
                 value: float = 0.0, decimals: int = 4, label_w: int = 44):
        super().__init__()
        self._lo       = lo
        self._hi       = hi
        self._decimals = decimals
        self._locked   = False  # prevent recursive signal loops

        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 1, 0, 1)
        layout.setSpacing(4)

        lbl = QLabel(label + ':')
        lbl.setFixedWidth(label_w)
        lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        layout.addWidget(lbl)

        self.slider = QSlider(Qt.Horizontal)
        self.slider.setRange(0, self.STEPS)
        self.slider.setValue(self._to_int(value))
        layout.addWidget(self.slider, stretch=1)

        self.val_edit = QLineEdit(f'{value:.{decimals}f}')
        self.val_edit.setFixedWidth(82)
        self.val_edit.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.val_edit)

        self.slider.valueChanged.connect(self._slider_moved)
        self.slider.sliderReleased.connect(self.released.emit)
        self.val_edit.returnPressed.connect(self._edit_committed)

        # keyboard arrow keys don't trigger sliderReleased — emit manually
        _orig_key = self.slider.keyReleaseEvent
        def _key_release(ev, _orig=_orig_key):
            _orig(ev)
            if ev.key() in (Qt.Key_Left, Qt.Key_Right,
                            Qt.Key_Up,   Qt.Key_Down,
                            Qt.Key_PageUp, Qt.Key_PageDown):
                self.released.emit()
        self.slider.keyReleaseEvent = _key_release

    # ── helpers ──────────────────────────────────────────────────────────

    def _to_int(self, val: float) -> int:
        ratio = (float(val) - self._lo) / (self._hi - self._lo)
        return int(np.clip(ratio * self.STEPS, 0, self.STEPS))

    def _to_float(self, i: int) -> float:
        return self._lo + (i / self.STEPS) * (self._hi - self._lo)

    def _slider_moved(self, i: int):
        if self._locked:
            return
        self._locked = True
        self.val_edit.setText(f'{self._to_float(i):.{self._decimals}f}')
        self._locked = False

    def _edit_committed(self):
        try:
            v = np.clip(float(self.val_edit.text()), self._lo, self._hi)
        except ValueError:
            return
        self._locked = True
        self.slider.setValue(self._to_int(v))
        self.val_edit.setText(f'{v:.{self._decimals}f}')
        self._locked = False
        self.released.emit()

    # ── public API ────────────────────────────────────────────────────────

    def get(self) -> float:
        try:
            return float(self.val_edit.text())
        except ValueError:
            return self._to_float(self.slider.value())

    def set(self, val: float):
        val = float(np.clip(val, self._lo, self._hi))
        self._locked = True
        self.slider.setValue(self._to_int(val))
        self.val_edit.setText(f'{val:.{self._decimals}f}')
        self._locked = False


# ──────────────────────────────────────────────────────────────────────────────
# Main window
# ──────────────────────────────────────────────────────────────────────────────

MODE_PCD  = 'pcd'
MODE_BOTH = 'both'
MODE_IMG  = 'img'


_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


class AlignmentTool(QMainWindow):
    # 預設指向 alignment_tool.py 同一層的 data/(跟著腳本走,不寫死磁碟代號)
    DEFAULT_DATA_ROOT = os.path.join(_SCRIPT_DIR, 'data')
    CAMERAS = ['main', 'rear', 'left', 'right', 'sideL', 'sideR']
    IMG_EXTS = ('.png', '.jpg', '.jpeg', '.bmp', '.tiff')
    # 各相機 FOV 半角過濾預設(僅顯示用,不屬標定參數,故不寫入 config)。
    # config 沒有 fov_half_deg 時用此兜底;pinhole 側相機幾何上必須過濾。
    DEFAULT_FOV = {'main': 85.0, 'sideL': 37.0, 'sideR': 37.0}

    def __init__(self):
        super().__init__()
        self.setWindowTitle('PCD-Image Alignment Tool')
        self.resize(1400, 940)

        self.pcd_files: list = []
        self.img_files: list = []
        self.config:    dict = {}
        self.pcd_idx:   int  = 0
        self.img_idx:   int  = 0
        self.nav_mode:  str  = MODE_BOTH
        self._pts_cache      = None
        self._pcd_dir:  str  = ''
        self._img_dir:  str  = ''

        self._build_ui()
        self._prefill_session()
        self._load_all()

    # ── UI construction ───────────────────────────────────────────────────

    def _build_ui(self):
        root_w = QWidget()
        self.setCentralWidget(root_w)
        root = QVBoxLayout(root_w)
        root.setSpacing(6)
        root.setContentsMargins(8, 8, 8, 8)

        root.addWidget(self._make_path_panel())
        root.addWidget(self._make_nav_bar())
        root.addWidget(self._make_viewer(), stretch=1)
        root.addWidget(self._make_param_panel())

        self.setStatusBar(QStatusBar())

    def _make_path_panel(self) -> QGroupBox:
        box = QGroupBox('Session')
        row = QHBoxLayout(box)
        row.setSpacing(6)

        row.addWidget(QLabel('Data Root:'))
        self.root_edit = QLineEdit()
        self.root_edit.setFixedWidth(300)
        row.addWidget(self.root_edit)

        browse_btn = QPushButton('Browse')
        browse_btn.setFixedWidth(70)
        browse_btn.clicked.connect(self._browse_root)
        row.addWidget(browse_btn)

        refresh_btn = QPushButton('↺')
        refresh_btn.setFixedWidth(30)
        refresh_btn.setToolTip('Rescan sessions')
        refresh_btn.clicked.connect(self._refresh_sessions)
        row.addWidget(refresh_btn)

        row.addSpacing(10)
        row.addWidget(QLabel('Session:'))
        self.session_combo = QComboBox()
        self.session_combo.setMinimumWidth(280)
        self.session_combo.setSizeAdjustPolicy(QComboBox.AdjustToContents)
        self.session_combo.currentTextChanged.connect(self._on_session_changed)
        row.addWidget(self.session_combo)

        row.addSpacing(10)
        row.addWidget(QLabel('Camera:'))
        self.cam_combo = QComboBox()
        self.cam_combo.addItems(self.CAMERAS)
        self.cam_combo.setFixedWidth(90)
        self.cam_combo.currentTextChanged.connect(self._on_camera_changed)
        row.addWidget(self.cam_combo)

        load_btn = QPushButton('Load')
        load_btn.setFixedWidth(60)
        load_btn.clicked.connect(self._load_all)
        row.addWidget(load_btn)

        row.addStretch()
        return box

    def _make_nav_bar(self) -> QWidget:
        bar = QWidget()
        row = QHBoxLayout(bar)
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(6)

        for label, mode in [('點雲', MODE_PCD),
                             ('同時',   MODE_BOTH),
                             ('圖片', MODE_IMG)]:
            btn = QPushButton(label)
            btn.setCheckable(True)
            btn.setFixedWidth(100)
            btn.setChecked(mode == MODE_BOTH)
            btn.clicked.connect(lambda _, m=mode: self._set_mode(m))
            setattr(self, f'mode_btn_{mode}', btn)
            row.addWidget(btn)

        row.addSpacing(10)

        self.btn_prev = QPushButton('←')
        self.btn_prev.setFixedWidth(80)
        self.btn_prev.setAutoRepeat(True)
        self.btn_prev.setAutoRepeatDelay(400)
        self.btn_prev.setAutoRepeatInterval(120)
        self.btn_prev.clicked.connect(self._prev)
        row.addWidget(self.btn_prev)

        self.btn_next = QPushButton('→')
        self.btn_next.setFixedWidth(80)
        self.btn_next.setAutoRepeat(True)
        self.btn_next.setAutoRepeatDelay(400)
        self.btn_next.setAutoRepeatInterval(120)
        self.btn_next.clicked.connect(self._next)
        row.addWidget(self.btn_next)

        btn_reset = QPushButton('重置')
        btn_reset.setFixedWidth(75)
        btn_reset.setToolTip('圖片與點雲都回到第 0 張')
        btn_reset.clicked.connect(self._reset)
        row.addWidget(btn_reset)

        btn_revert = QPushButton('還原參數')
        btn_revert.setFixedWidth(85)
        btn_revert.setToolTip('把 K / D / T 全部從 config 重新載入 — 撤銷所有 slider 微調')
        btn_revert.clicked.connect(self._revert_params)
        row.addWidget(btn_revert)

        self.frame_lbl = QLabel()
        self.frame_lbl.setAlignment(Qt.AlignCenter)
        f = QFont(); f.setPointSize(10)
        self.frame_lbl.setFont(f)
        row.addWidget(self.frame_lbl, stretch=1)

        reproj_btn = QPushButton('⟳ Reproject')
        reproj_btn.setFixedWidth(100)
        reproj_btn.clicked.connect(self._reproject)
        row.addWidget(reproj_btn)

        self.show_proj_chk = QCheckBox('顯示投影點雲')
        self.show_proj_chk.setChecked(True)
        self.show_proj_chk.stateChanged.connect(self._show_projection)
        row.addWidget(self.show_proj_chk)

        return bar

    def _make_viewer(self) -> QSplitter:
        spl = QSplitter(Qt.Horizontal)

        # Left: projection image
        proj_box  = QGroupBox('Projection — LiDAR on main image')
        proj_vbox = QVBoxLayout(proj_box)
        proj_vbox.setContentsMargins(2, 2, 2, 2)
        self.proj_lbl = ZoomableImageLabel()
        self.proj_lbl.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        proj_vbox.addWidget(self.proj_lbl)
        spl.addWidget(proj_box)

        # Right: parameter sliders
        spl.addWidget(self._make_slider_panel())

        spl.setSizes([1080, 300])
        spl.setCollapsible(1, True)
        return spl

    def _make_slider_panel(self) -> QScrollArea:
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setMinimumWidth(260)

        inner = QWidget()
        vbox  = QVBoxLayout(inner)
        vbox.setSpacing(6)
        vbox.setContentsMargins(4, 4, 4, 4)

        # ── Display ────────────────────────────────────────────────────
        disp_box  = QGroupBox('Display')
        disp_vbox = QVBoxLayout(disp_box)
        disp_vbox.setSpacing(2)
        self.s_pt_size = SliderParam('大小',   1.,  10.,  2., decimals=0, label_w=44)
        self.s_opacity = SliderParam('透明度', 0.,   1.,  1., decimals=2, label_w=44)
        for s in (self.s_pt_size, self.s_opacity):
            disp_vbox.addWidget(s)
            s.released.connect(self._on_slider_released)
        vbox.addWidget(disp_box)

        # ── Extrinsic ──────────────────────────────────────────────────
        extr_box  = QGroupBox('Extrinsic  (t: metres, rot: degrees)')
        extr_vbox = QVBoxLayout(extr_box)
        extr_vbox.setSpacing(2)
        self.s_tx    = SliderParam('tx',    -5.,  5.,   0.,    decimals=4)
        self.s_ty    = SliderParam('ty',    -5.,  5.,   0.,    decimals=4)
        self.s_tz    = SliderParam('tz',    -5.,  5.,   0.,    decimals=4)
        self.s_roll  = SliderParam('roll',  -180., 180., 0.,   decimals=3)
        self.s_pitch = SliderParam('pitch', -180., 180., 0.,   decimals=3)
        self.s_yaw   = SliderParam('yaw',   -180., 180., 0.,   decimals=3)
        for s in (self.s_tx, self.s_ty, self.s_tz,
                  self.s_roll, self.s_pitch, self.s_yaw):
            extr_vbox.addWidget(s)
            s.released.connect(self._on_slider_released)
        vbox.addWidget(extr_box)

        # ── Intrinsic ──────────────────────────────────────────────────
        intr_box  = QGroupBox('Intrinsic  (full-res pixels)')
        intr_vbox = QVBoxLayout(intr_box)
        intr_vbox.setSpacing(2)
        self.s_fx    = SliderParam('fx',    100.,  5000., 1000., decimals=1)
        self.s_fy    = SliderParam('fy',    100.,  5000., 1000., decimals=1)
        self.s_cx    = SliderParam('cx',    0.,    4000., 1000., decimals=1)
        self.s_cy    = SliderParam('cy',    0.,    2500., 500.,  decimals=1)
        self.s_scale = SliderParam('scale', 0.1,   2.0,   1.0,  decimals=6)
        for s in (self.s_fx, self.s_fy, self.s_cx, self.s_cy, self.s_scale):
            intr_vbox.addWidget(s)
            s.released.connect(self._on_slider_released)
        vbox.addWidget(intr_box)

        # ── Distortion ─────────────────────────────────────────────────
        dist_box  = QGroupBox('Distortion  D = [k1, k2, p1, p2, k3]')
        dist_vbox = QVBoxLayout(dist_box)
        dist_vbox.setSpacing(2)
        self.s_k1 = SliderParam('k1', -2.,  2.,  0., decimals=5)
        self.s_k2 = SliderParam('k2', -2.,  2.,  0., decimals=5)
        self.s_p1 = SliderParam('p1', -0.5, 0.5, 0., decimals=5)
        self.s_p2 = SliderParam('p2', -0.5, 0.5, 0., decimals=5)
        self.s_k3 = SliderParam('k3', -2.,  2.,  0., decimals=5)
        for s in (self.s_k1, self.s_k2, self.s_p1, self.s_p2, self.s_k3):
            dist_vbox.addWidget(s)
            s.released.connect(self._on_slider_released)
        vbox.addWidget(dist_box)

        vbox.addStretch()
        scroll.setWidget(inner)
        return scroll

    def _make_param_panel(self) -> QGroupBox:
        box = QGroupBox("Camera Parameters — main  (config: 'main')")
        self.param_box = box
        row = QHBoxLayout(box)
        row.setSpacing(8)

        # K matrix (3×3, full resolution)
        self.K_edit = MatrixEdit('K matrix  (full res)', 3, 3, cell_w=92)
        row.addWidget(self.K_edit)

        # D distortion  [k1, k2, p1, p2, k3]
        self.D_edit = VectorEdit('Distortion  D', ['k1', 'k2', 'p1', 'p2', 'k3'], cell_w=85)
        row.addWidget(self.D_edit)

        # T matrix (4×4)
        self.T_edit = MatrixEdit('T matrix  4×4  (LiDAR → camera, metres)', 4, 4, cell_w=100)
        row.addWidget(self.T_edit)

        # Scale + Save
        right_col = QVBoxLayout()
        scale_box  = QGroupBox('Scale')
        scale_vbox = QVBoxLayout(scale_box)
        scale_row  = QHBoxLayout()
        scale_row.addWidget(QLabel('scale:'))
        self.scale_edit = QLineEdit()
        self.scale_edit.setFixedWidth(90)
        scale_row.addWidget(self.scale_edit)
        scale_vbox.addLayout(scale_row)
        right_col.addWidget(scale_box)

        save_btn = QPushButton('Save\nConfig')
        save_btn.setFixedWidth(80)
        save_btn.setFixedHeight(48)
        save_btn.clicked.connect(self._save_config)
        right_col.addWidget(save_btn)
        right_col.addStretch()
        row.addLayout(right_col)

        # Depth filter
        depth_box  = QGroupBox('Depth (m)')
        depth_grid = QGridLayout(depth_box)
        depth_grid.setSpacing(3)
        depth_grid.addWidget(QLabel('Min:'), 0, 0)
        self.min_depth = QDoubleSpinBox()
        self.min_depth.setRange(0.0, 10.0); self.min_depth.setValue(0.5)
        self.min_depth.setSingleStep(0.1);  self.min_depth.setDecimals(1)
        depth_grid.addWidget(self.min_depth, 0, 1)
        depth_grid.addWidget(QLabel('Max:'), 1, 0)
        self.max_depth = QDoubleSpinBox()
        self.max_depth.setRange(1.0, 300.0); self.max_depth.setValue(80.0)
        self.max_depth.setSingleStep(5.0);   self.max_depth.setDecimals(0)
        depth_grid.addWidget(self.max_depth, 1, 1)
        row.addWidget(depth_box)

        # FOV half-angle filter (deg) — 0 = 不過濾
        fov_box  = QGroupBox('FOV (°)')
        fov_grid = QGridLayout(fov_box)
        fov_grid.setSpacing(3)
        fov_grid.addWidget(QLabel('half:'), 0, 0)
        self.fov_half = QDoubleSpinBox()
        self.fov_half.setRange(0.0, 120.0); self.fov_half.setValue(0.0)
        self.fov_half.setSingleStep(1.0);   self.fov_half.setDecimals(0)
        self.fov_half.setToolTip('光軸夾角過濾,0=不過濾。主魚眼 80–95、側 BSD 35–40')
        self.fov_half.valueChanged.connect(lambda _: self._refresh())
        fov_grid.addWidget(self.fov_half, 0, 1)
        row.addWidget(fov_box)

        return box

    # ── Mode ──────────────────────────────────────────────────────────────

    def _set_mode(self, mode: str):
        self.nav_mode = mode
        for m in (MODE_PCD, MODE_BOTH, MODE_IMG):
            getattr(self, f'mode_btn_{m}').setChecked(m == mode)
        self._update_frame_label()

    # ── Keyboard ──────────────────────────────────────────────────────────

    def keyPressEvent(self, event):
        if event.key() in (Qt.Key_Left, Qt.Key_Up):
            self._prev()
        elif event.key() in (Qt.Key_Right, Qt.Key_Down):
            self._next()
        else:
            super().keyPressEvent(event)

    # ── Paths ─────────────────────────────────────────────────────────────

    def _prefill_session(self):
        root = self.DEFAULT_DATA_ROOT
        # fallback: if data/ doesn't exist, try the parent directory
        if not os.path.isdir(root):
            root = os.path.dirname(root)
        self.root_edit.setText(root)
        self._refresh_sessions()

    def _browse_root(self):
        start = self.root_edit.text() or self.DEFAULT_DATA_ROOT
        path = QFileDialog.getExistingDirectory(self, 'Select Data Root Folder', start)
        if path:
            self.root_edit.setText(path)
            self._refresh_sessions()

    def _refresh_sessions(self):
        root = self.root_edit.text().strip()
        if not os.path.isdir(root):
            return
        try:
            sessions = sorted(
                (d for d in os.listdir(root)
                 if os.path.isdir(os.path.join(root, d))),
                key=lambda d: os.path.getmtime(os.path.join(root, d)),
                reverse=True,
            )
        except OSError:
            sessions = []

        self.session_combo.blockSignals(True)
        self.session_combo.clear()
        self.session_combo.addItems(sessions)
        self.session_combo.blockSignals(False)
        if sessions:
            self.session_combo.setCurrentIndex(0)

    def _on_session_changed(self, _: str):
        if not hasattr(self, 'param_box'):
            return
        self._load_all()

    def _current_session(self) -> str:
        root = self.root_edit.text().strip()
        name = self.session_combo.currentText()
        if root and name:
            return os.path.join(root, name)
        return root  # fallback if root itself is a session

    # ── Load ──────────────────────────────────────────────────────────────

    def _load_all(self):
        session = self._current_session()
        msgs = [f'Session: {session}']

        if not os.path.isdir(session):
            self.statusBar().showMessage(f'[ERROR] Session folder not found: {session}')
            return

        # PCD
        pcd_dir = os.path.join(session, 'VLS128_pcd')
        if os.path.isdir(pcd_dir):
            self.pcd_files = sorted(
                f for f in os.listdir(pcd_dir) if f.lower().endswith('.pcd'))
            self._pcd_dir = pcd_dir
            msgs.append(f'PCD: {len(self.pcd_files)} files')
        else:
            self.pcd_files = []
            self._pcd_dir = ''
            msgs.append('PCD: folder not found')

        # Images for the selected camera
        cam = self.cam_combo.currentText()
        self._load_camera_images(session, cam)
        if self.img_files:
            msgs.append(f'IMG[{cam}]: {len(self.img_files)} files')
        else:
            msgs.append(f'IMG[{cam}]: folder not found')

        # Config — search session dir then parent dir
        cfg_path = self._find_config(session)
        if cfg_path:
            try:
                self.config    = load_config(cfg_path)
                self._cfg_path = cfg_path
                self._populate_params(cam)
                msgs.append(f'Config: {os.path.basename(cfg_path)}')
            except Exception as e:
                msgs.append(f'Config ERROR: {e}')
        else:
            msgs.append('Config: not found')

        self.pcd_idx = 0
        self.img_idx = 0
        self._pts_cache = None
        self._refresh()
        self.statusBar().showMessage('  |  '.join(msgs))

    # 標準 config 檔名(優先載入);備份/副本字樣一律跳過
    PREFERRED_CONFIG = 'config_g6_6view.json'
    _BACKUP_MARKERS = ('複製', '副本', 'copy', 'backup', '.bak', '~', ' - ')

    def _find_config(self, session: str) -> str:
        """Walk up from session folder (up to 4 levels) for a config file.

        優先序:① 同層若有標準檔名 PREFERRED_CONFIG 直接用;
        ② 否則取第一個 config/cfg 檔,但跳過含備份字樣(複製/副本/copy/bak…)者,
        避免載到沒有 sideL/sideR 的舊備份。
        """
        def _is_backup(name: str) -> bool:
            nl = name.lower()
            return any(m.lower() in nl for m in self._BACKUP_MARKERS)

        d = session
        for _ in range(4):
            if os.path.isdir(d):
                names = sorted(os.listdir(d))
                # ① 標準檔名最優先
                for fname in names:
                    if fname.lower() == self.PREFERRED_CONFIG.lower():
                        return os.path.join(d, fname)
                # ② 其餘 config 檔,跳過備份名
                for fname in names:
                    nl = fname.lower()
                    if (('config' in nl or 'cfg' in nl)
                            and nl.endswith(('.json', '.py'))
                            and not _is_backup(fname)):
                        return os.path.join(d, fname)
            parent = os.path.dirname(d)
            if parent == d:
                break
            d = parent
        return ''

    def _load_camera_images(self, session: str, cam: str):
        # try session/images/<cam>/ first, then session/<cam>/ as fallback
        for candidate in (
            os.path.join(session, 'images', cam),
            os.path.join(session, cam),
        ):
            if os.path.isdir(candidate):
                self.img_files = sorted(
                    f for f in os.listdir(candidate)
                    if f.lower().endswith(self.IMG_EXTS))
                self._img_dir = candidate
                return
        self.img_files = []
        self._img_dir = ''

    def _on_camera_changed(self, cam: str):
        if not hasattr(self, 'param_box'):
            return  # called during widget construction — not ready yet
        session = self._current_session()
        if not os.path.isdir(session):
            return
        self._load_camera_images(session, cam)
        self.img_idx = min(self.img_idx, max(len(self.img_files) - 1, 0))
        if self.config:
            self._populate_params(cam)
        self.param_box.setTitle(f"Camera Parameters — {cam}  (config: '{cam}')")
        self._refresh()
        self.statusBar().showMessage(
            f'Camera → {cam}  ({len(self.img_files)} images)')

    def _populate_params(self, cam_name: str = 'main'):
        # use exact camera key; fall back to 'main' only if the key is missing
        cam = self.config.get(cam_name) or self.config.get('main', {})
        self._cam_type = cam.get('type', 'standard')
        # FOV 半角過濾(僅顯示用):config 有就用,否則用 DEFAULT_FOV 兜底
        # (pinhole 側相機幾何上必須過濾;main fisheye 預設 85 去雜訊)
        self._fov_half_deg = float(
            cam.get('fov_half_deg', self.DEFAULT_FOV.get(cam_name, 0.0)))
        if hasattr(self, 'fov_half'):
            self.fov_half.blockSignals(True)
            self.fov_half.setValue(self._fov_half_deg)
            self.fov_half.blockSignals(False)
        self.param_box.setTitle(
            f"Camera Parameters — {cam_name}  [{self._cam_type}]"
            + (f"  fov<{int(self._fov_half_deg)}deg" if self._fov_half_deg > 0 else '')
            + ('' if cam_name in self.config else "  (fallback: 'main')")
        )
        # 支援三種內參表示: K, K_base [fx fy cx cy], K_native(BSD pinhole 1920×1536)
        # K_native 之 sideL/sideR 須以實際影像解析度推導 scale_x/scale_y,故記住其原生尺寸
        self._k_native_size = None
        if 'K' in cam:
            K = np.array(cam['K'])
        elif 'K_native' in cam:
            K = np.array(cam['K_native'])
            self._k_native_size = (1920, 1536)   # BSD 針孔之原生標稱解析度
        elif 'K_base' in cam:
            fx, fy, cx, cy = cam['K_base']
            K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float32)
        else:
            K = np.eye(3, dtype=np.float32)
        D_raw = np.array(cam.get('D', np.zeros(5))).flatten()
        # fisheye D has 4 coefficients; pad to 5 for display
        D = np.zeros(5, dtype=np.float32)
        D[:len(D_raw)] = D_raw[:5]
        T   = np.array(cam.get('T',  np.eye(4)))
        s   = float(cam.get('scale', 1.0))

        # matrix display fields
        self.K_edit.set(K)
        self.D_edit.set(D)
        self.T_edit.set(T)
        self.scale_edit.setText(f'{s:.8g}')

        # sliders — intrinsic
        self.s_fx.set(K[0, 0]);  self.s_fy.set(K[1, 1])
        self.s_cx.set(K[0, 2]);  self.s_cy.set(K[1, 2])
        self.s_scale.set(s)

        # sliders — distortion
        self.s_k1.set(D[0]); self.s_k2.set(D[1])
        self.s_p1.set(D[2]); self.s_p2.set(D[3]); self.s_k3.set(D[4])

        # sliders — extrinsic (decompose T into RPY + translation)
        roll, pitch, yaw = _rot_to_rpy(T[:3, :3])
        tx, ty, tz = T[0, 3], T[1, 3], T[2, 3]
        self.s_tx.set(tx);   self.s_ty.set(ty);   self.s_tz.set(tz)
        self.s_roll.set(roll); self.s_pitch.set(pitch); self.s_yaw.set(yaw)

    # ── Navigation ────────────────────────────────────────────────────────

    def _reset(self):
        self.pcd_idx = 0
        self.img_idx = 0
        self._pts_cache = None
        self._refresh()

    def _revert_params(self):
        """把當前相機的 K/D/T 從 config 重新載入,撤銷所有 slider 微調。"""
        if not self.config:
            self.statusBar().showMessage('未載入 config — 無法還原')
            return
        cam = self.cam_combo.currentText()
        self._populate_params(cam)
        self._refresh()
        self.statusBar().showMessage(f'已還原 {cam} 之 K / D / T 至 config 原值', 4000)

    def _prev(self):
        changed = False
        if self.nav_mode in (MODE_PCD, MODE_BOTH) and self.pcd_idx > 0:
            self.pcd_idx -= 1
            self._pts_cache = None
            changed = True
        if self.nav_mode in (MODE_IMG, MODE_BOTH) and self.img_idx > 0:
            self.img_idx -= 1
            changed = True
        if changed:
            self._refresh()

    def _next(self):
        changed = False
        if self.nav_mode in (MODE_PCD, MODE_BOTH) and self.pcd_idx < len(self.pcd_files) - 1:
            self.pcd_idx += 1
            self._pts_cache = None
            changed = True
        if self.nav_mode in (MODE_IMG, MODE_BOTH) and self.img_idx < len(self.img_files) - 1:
            self.img_idx += 1
            changed = True
        if changed:
            self._refresh()

    def _on_slider_released(self):
        """Rebuild K, D, T from slider values → sync matrix fields → reproject."""
        K = np.array([
            [self.s_fx.get(), 0.,             self.s_cx.get()],
            [0.,              self.s_fy.get(), self.s_cy.get()],
            [0.,              0.,              1.             ],
        ], dtype=np.float32)
        D = np.array([self.s_k1.get(), self.s_k2.get(),
                      self.s_p1.get(), self.s_p2.get(),
                      self.s_k3.get()], dtype=np.float32)
        R = _rpy_to_rot(self.s_roll.get(), self.s_pitch.get(), self.s_yaw.get())
        T = np.eye(4, dtype=np.float32)
        T[:3, :3] = R
        T[:3, 3]  = [self.s_tx.get(), self.s_ty.get(), self.s_tz.get()]

        self.K_edit.set(K)
        self.D_edit.set(D)
        self.T_edit.set(T)
        self.scale_edit.setText(f'{self.s_scale.get():.8g}')

        self._show_projection()

    def _reproject(self):
        self._show_projection()

    # ── Refresh ───────────────────────────────────────────────────────────

    def _refresh(self):
        if not self.pcd_files and not self.img_files:
            self._update_frame_label()
            return

        if self._pts_cache is None and self.pcd_idx < len(self.pcd_files):
            pcd_path = os.path.join(self._pcd_dir, self.pcd_files[self.pcd_idx])
            try:
                self._pts_cache = read_pcd(pcd_path)
            except Exception as e:
                self.statusBar().showMessage(f'PCD error: {e}')
                self._pts_cache = None

        if self.img_idx < len(self.img_files) and self._img_dir:
            self._img_path = os.path.join(self._img_dir, self.img_files[self.img_idx])

        self._update_frame_label()
        self._show_projection()

    def _update_frame_label(self):
        n_pcd = len(self.pcd_files)
        n_img = len(self.img_files)
        if self.nav_mode == MODE_PCD:
            txt = f'PCD: {self.pcd_idx} / {max(n_pcd-1,0)}  │  IMG: {self.img_idx} (固定)'
        elif self.nav_mode == MODE_IMG:
            txt = f'PCD: {self.pcd_idx} (固定)  │  IMG: {self.img_idx} / {max(n_img-1,0)}'
        else:
            txt = f'PCD: {self.pcd_idx} / {max(n_pcd-1,0)}  │  IMG: {self.img_idx} / {max(n_img-1,0)}'
        self.frame_lbl.setText(txt)

        can_prev = (
            (self.nav_mode in (MODE_PCD, MODE_BOTH) and self.pcd_idx > 0) or
            (self.nav_mode in (MODE_IMG, MODE_BOTH) and self.img_idx > 0)
        )
        can_next = (
            (self.nav_mode in (MODE_PCD, MODE_BOTH) and self.pcd_idx < n_pcd - 1) or
            (self.nav_mode in (MODE_IMG, MODE_BOTH) and self.img_idx < n_img - 1)
        )
        self.btn_prev.setEnabled(can_prev)
        self.btn_next.setEnabled(can_next)

    def _show_projection(self):
        if not hasattr(self, '_img_path'):
            return

        if not self.show_proj_chk.isChecked():
            self._update_label(QPixmap(self._img_path))
            return

        if self._pts_cache is None:
            self._update_label(QPixmap(self._img_path))
            return

        K = self.K_edit.get()
        D = self.D_edit.get()
        T = self.T_edit.get()
        try:
            scale = float(self.scale_edit.text())
        except ValueError:
            scale = 1.0

        try:
            pix = project_and_draw(
                self._pts_cache, self._img_path,
                K, D, T, scale,
                min_depth = self.min_depth.value(),
                max_depth = self.max_depth.value(),
                point_r   = max(1, int(round(self.s_pt_size.get()))),
                cam_type  = getattr(self, '_cam_type', 'standard'),
                opacity   = self.s_opacity.get(),
                fov_half_deg = self.fov_half.value() if hasattr(self, 'fov_half') else getattr(self, '_fov_half_deg', 0.0),
                k_native_size = getattr(self, '_k_native_size', None),
            )
            self._update_label(pix)
        except Exception as e:
            self.statusBar().showMessage(f'Projection error: {e}')
            self._update_label(QPixmap(self._img_path))

    def _update_label(self, pix: QPixmap):
        if pix.isNull():
            return
        # ZoomableImageLabel 自行管理 zoom/pan;切換 frame 時保持當前 zoom 與 pan
        self.proj_lbl.set_pixmap(pix)

    # ── Save config ───────────────────────────────────────────────────────

    def _save_config(self):
        if not self.config or not hasattr(self, '_cfg_path'):
            self.statusBar().showMessage('No config loaded.')
            return

        cam = self.cam_combo.currentText()
        if cam not in self.config:
            self.config[cam] = {}
        entry = self.config[cam]
        K = self.K_edit.get()
        # 依該相機原本的內參表示形式存回:
        #   pinhole(_k_native_size 有值)→ K_edit 是原生 K,存回 K_native,不寫 K/scale
        #   fisheye/standard → 存 K + scale
        if getattr(self, '_k_native_size', None) is not None:
            entry['K_native'] = K
            entry.pop('K', None)        # 清掉可能殘留的 K,避免 load 時誤用未縮放值
        else:
            entry['K'] = K
            try:
                entry['scale'] = float(self.scale_edit.text())
            except ValueError:
                pass
        entry['D'] = self.D_edit.get()
        entry['T'] = self.T_edit.get()
        # fov_half_deg 為顯示用過濾,不屬標定參數 → 不寫回 config(由 DEFAULT_FOV 兜底)

        try:
            save_config(self._cfg_path, self.config)
            self.statusBar().showMessage(f'Saved → {self._cfg_path}', 4000)
        except Exception as e:
            self.statusBar().showMessage(f'Save error: {e}')


# ──────────────────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    app = QApplication(sys.argv)
    app.setStyle('Fusion')

    pal = QPalette()
    pal.setColor(QPalette.Window,          QColor(28, 28, 42))
    pal.setColor(QPalette.WindowText,      QColor(220, 220, 240))
    pal.setColor(QPalette.Base,            QColor(18, 18, 32))
    pal.setColor(QPalette.AlternateBase,   QColor(38, 38, 56))
    pal.setColor(QPalette.Text,            QColor(220, 220, 240))
    pal.setColor(QPalette.Button,          QColor(48, 48, 70))
    pal.setColor(QPalette.ButtonText,      QColor(220, 220, 240))
    pal.setColor(QPalette.Highlight,       QColor(90, 90, 200))
    pal.setColor(QPalette.HighlightedText, QColor(255, 255, 255))
    app.setPalette(pal)

    win = AlignmentTool()
    win.show()
    sys.exit(app.exec_())
