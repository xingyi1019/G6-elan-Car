"""
Target-less LiDAR-Camera 外參精煉 (Levinson & Thrun 風格 edge correlation)

原理:
  1. 影像端:Canny 邊緣 → 距離變換 IDT(離邊緣越近越亮)
  2. LiDAR 端:每點計算「邊緣權重」=深度不連續(neighbor depth diff)
  3. 投影所有 LiDAR 點,在 IDT 上查值,加權求和
  4. 多 frame 平均,scipy.optimize 找最低 cost 的 T

用法:
  python targetless_refine.py \
      --scenarios <scene1> <scene2> ... \
      --cam <sideL|sideR|left|right|rear> \
      --init <calib_tuned_xxx.json>  # 或從 config_g6_6view.json 自動讀
      --out refined_<cam>.json
"""
import os, sys, glob, json, argparse, time
import numpy as np
import cv2
from scipy.optimize import minimize

sys.stdout.reconfigure(encoding='utf-8')

CONFIG_PATH = r'C:\Users\xingy\OneDrive\Desktop\pcd_aligment\config_g6_6view.json'

# BSD pinhole (for sideL / sideR)
K_BSD_NATIVE = np.array([
    [2023.0, 0.0,    960.0],
    [0.0,    2030.0, 768.0],
    [0.0,    0.0,    1.0]
], dtype=np.float64)
D_BSD = np.array([-0.48825903, 0.35046806, 0.00301038, 0.00000390, -0.22704884],
                 dtype=np.float64)


# ════════════════════════════════════════════════════════════════════════════
def read_pcd(path):
    with open(path, 'rb') as f:
        header = {}
        while True:
            raw = f.readline()
            if not raw: break
            line = raw.decode('utf-8', errors='ignore').strip()
            if line.startswith('#') or not line: continue
            parts = line.split()
            header[parts[0].upper()] = parts[1:]
            if parts[0].upper() == 'DATA':
                offset = f.tell(); break
        dt_str = header.get('DATA', ['ascii'])[0].lower()
        fields = header.get('FIELDS', ['x', 'y', 'z'])
        sizes = [int(s) for s in header.get('SIZE', ['4'] * len(fields))]
        types = header.get('TYPE', ['F'] * len(fields))
        num = int((header.get('POINTS') or header.get('WIDTH') or ['0'])[0])
        if dt_str == 'ascii':
            f.seek(offset)
            d = np.loadtxt(f, max_rows=num or None)
            if d.ndim == 1: d = d.reshape(1, -1)
            xi, yi, zi = fields.index('x'), fields.index('y'), fields.index('z')
            return d[:, [xi, yi, zi]].astype(np.float32)
        tmap = {'F': 'f', 'I': 'i', 'U': 'u'}
        dt = np.dtype([(fn, tmap.get(t, 'f') + str(sz))
                       for fn, sz, t in zip(fields, sizes, types)])
        f.seek(offset)
        d = np.frombuffer(f.read(num * dt.itemsize), dtype=dt)
        return np.column_stack([d['x'].astype(np.float32),
                                 d['y'].astype(np.float32),
                                 d['z'].astype(np.float32)])


# ════════════════════════════════════════════════════════════════════════════
def load_init(cam, init_path=None):
    """從 JSON 或 config 拿初始 T、K、D、模型。"""
    if init_path and os.path.exists(init_path):
        with open(init_path, 'r', encoding='utf-8') as f:
            data = json.load(f)
        if 'T_lidar_to_cam_4x4' in data:
            T = np.asarray(data['T_lidar_to_cam_4x4'])
        elif 'T' in data:
            T = np.asarray(data['T'])
        else:
            raise ValueError(f"找不到 T in {init_path}")
        model = data.get('model', 'pinhole')
        print(f"📋 從 {init_path} 載入初始 T")
        return T, model

    # 從 config_g6_6view.json 讀
    if cam in ('left', 'right', 'rear'):
        ns = {'np': np}
        with open(CONFIG_PATH, 'r', encoding='utf-8') as f:
            exec(f.read(), ns)
        cfg = ns['CAM_CONFIGS'][cam]
        T = np.asarray(cfg['T'], dtype=np.float64)
        print(f"📋 從 config_g6_6view.json 載入 {cam} 初始 T (fisheye)")
        return T, 'fisheye'

    # sideL / sideR 沒有 config — 用 heuristic
    print(f"⚠️  {cam} 無初始 T,使用 heuristic 猜測")
    if cam == 'sideL':
        yaw = np.radians(135); pos = [-0.5, 0.7, 0.0]
    elif cam == 'sideR':
        yaw = np.radians(-135); pos = [-0.5, -0.7, 0.0]
    else:
        yaw = 0; pos = [0, 0, 0]
    R_align = np.array([[0, -1, 0], [0, 0, -1], [1, 0, 0]], dtype=np.float64)
    R_yaw = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                       [np.sin(yaw),  np.cos(yaw), 0],
                       [0, 0, 1]], dtype=np.float64)
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R_align @ R_yaw
    T[:3, 3] = pos
    return T, 'pinhole'


def get_KD(cam, img_shape):
    """回傳對應 cam 的 K, D, 與 (cv2 native 模型字串)"""
    h, w = img_shape[:2]
    if cam in ('left', 'right', 'rear'):
        # fisheye, K_base = [395, 395, 960, 768] @ 1920×1536
        s = w / 1920.0
        K = np.array([[395.06367868 * s, 0, 960.0 * s],
                      [0, 395.73048124 * s, 768.0 * s],
                      [0, 0, 1]], dtype=np.float64)
        D = np.array([0.42750887, -0.14506998, 0.02445243, -0.00135651],
                     dtype=np.float64).reshape(4, 1)
        return K, D, 'fisheye'
    # sideL / sideR: BSD pinhole @1920×1536
    sx, sy = w / 1920.0, h / 1536.0
    K = K_BSD_NATIVE.copy()
    K[0, 0] *= sx; K[0, 2] *= sx
    K[1, 1] *= sy; K[1, 2] *= sy
    return K, D_BSD, 'pinhole'


# ════════════════════════════════════════════════════════════════════════════
def compute_image_idt(img):
    """影像端:Canny + 距離變換 → IDT 圖,離邊緣越近數值越高 (~255)"""
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (3, 3), 0)
    edges = cv2.Canny(gray, 50, 150)
    # 距離變換:離邊緣的距離(像素)
    dt = cv2.distanceTransform(255 - edges, cv2.DIST_L2, 3)
    # 反向:離邊緣近 → 高分(用 exp 衰減)
    sigma = 5.0  # 像素
    idt = np.exp(-dt / sigma).astype(np.float32)  # [0, 1]
    return idt


def compute_lidar_edge_weight(pts, sample_step=1):
    """LiDAR 端:近似深度不連續權重。
    用每個點的最近鄰深度差判斷邊緣強度。簡化做法:K-NN 找 3 個最近,看深度 std。
    """
    pts = pts[::sample_step].astype(np.float32)
    if len(pts) < 10: return pts, np.ones(len(pts), dtype=np.float32)
    # 用 KDTree 找最近鄰
    from scipy.spatial import cKDTree
    tree = cKDTree(pts)
    _, idx = tree.query(pts, k=4)  # 自己 + 3 個鄰居
    neighbors = pts[idx[:, 1:]]   # (N, 3, 3)
    depths_self = np.linalg.norm(pts, axis=1, keepdims=True)
    depths_neigh = np.linalg.norm(neighbors, axis=2)
    # 鄰居深度的 max - min(越大代表越靠近邊緣)
    edge_w = (depths_neigh.max(axis=1) - depths_neigh.min(axis=1)).astype(np.float32)
    # clip 避免極端值
    edge_w = np.clip(edge_w, 0, 2.0) / 2.0
    return pts, edge_w


# ════════════════════════════════════════════════════════════════════════════
def project(pts_cam, K, D, kind):
    """投影 (3D in camera frame) → (u, v)"""
    if kind == 'fisheye':
        ipts, _ = cv2.fisheye.projectPoints(
            pts_cam.reshape(-1, 1, 3).astype(np.float64),
            np.zeros((3, 1)), np.zeros((3, 1)), K, D)
        return ipts.reshape(-1, 2)
    ipts, _ = cv2.projectPoints(
        pts_cam.astype(np.float64),
        np.zeros(3), np.zeros(3), K, D)
    return ipts.reshape(-1, 2)


def cost_for_frame(T_4x4, pts, edge_w, idt, K, D, kind, n_pts_total):
    """單一 frame cost,加上 coverage penalty 避免點雲跑出畫面"""
    R = T_4x4[:3, :3]
    t = T_4x4[:3, 3]
    p_cam = (R @ pts.T).T + t
    mask = (p_cam[:, 2] > 0.5) & (p_cam[:, 2] < 80.0)
    n_in_frustum = int(mask.sum())
    if n_in_frustum < 50:
        return 1.0    # 大懲罰:點雲幾乎沒進 camera frustum
    p_valid = p_cam[mask]
    w_valid = edge_w[mask]
    uv = project(p_valid, K, D, kind)
    u = uv[:, 0]; v = uv[:, 1]
    H, W = idt.shape
    inb = (u >= 0) & (u < W - 1) & (v >= 0) & (v < H - 1) & np.isfinite(u) & np.isfinite(v)
    n_inb = int(inb.sum())
    if n_inb < 30:
        return 1.0    # 大懲罰:沒幾個點打到影像
    ui = u[inb].astype(np.int32)
    vi = v[inb].astype(np.int32)
    # score: 平均 idt × edge_weight (每個 inb 點)
    score = (idt[vi, ui] * w_valid[inb]).sum() / n_inb
    # coverage ratio:相對 frustum 內點的比例(越大表示視野對得越好)
    coverage = n_inb / n_in_frustum
    # 最終 cost = -(score × coverage):兩者都要高
    return -(score * coverage)


# ════════════════════════════════════════════════════════════════════════════
def params_to_T(params, T0):
    """6 個 delta 參數 (drpy, dxyz) 套在 T0 上"""
    dr, dp, dy, dx, dyy, dz = params
    Rx = cv2.Rodrigues(np.array([np.deg2rad(dr), 0, 0]))[0]
    Ry = cv2.Rodrigues(np.array([0, np.deg2rad(dp), 0]))[0]
    Rz = cv2.Rodrigues(np.array([0, 0, np.deg2rad(dy)]))[0]
    dR = Rz @ Ry @ Rx
    T = T0.copy()
    T[:3, :3] = dR @ T0[:3, :3]
    T[:3, 3] = T0[:3, 3] + np.array([dx, dyy, dz])
    return T


# ════════════════════════════════════════════════════════════════════════════
def gather_frames(scenarios, cam, max_per_scene=15, step=4):
    """從多個場景抽 frame:每 step 個取一張,每場景最多 max_per_scene 張"""
    frames = []
    for sc in scenarios:
        img_dir = os.path.join(sc, 'images', cam)
        pcd_dir = os.path.join(sc, 'VLS128_pcd')
        if not os.path.isdir(img_dir) or not os.path.isdir(pcd_dir):
            continue
        files = sorted(glob.glob(os.path.join(img_dir, '*.png')))[::step][:max_per_scene]
        for ip in files:
            stem = os.path.splitext(os.path.basename(ip))[0]
            pp = os.path.join(pcd_dir, f'{stem}.pcd')
            if os.path.exists(pp):
                frames.append((ip, pp))
    return frames


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--scenarios', nargs='+', required=True)
    ap.add_argument('--cam', required=True,
                    choices=['sideL', 'sideR', 'left', 'right', 'rear'])
    ap.add_argument('--init', default=None, help='初始 T 的 JSON(預設從 config 讀)')
    ap.add_argument('--max-frames-per-scene', type=int, default=15)
    ap.add_argument('--sample-step', type=int, default=4)
    ap.add_argument('--lidar-step', type=int, default=2,
                    help='LiDAR 點抽樣步長(加速)')
    ap.add_argument('--out', default=None)
    args = ap.parse_args()

    T0, _ = load_init(args.cam, args.init)
    print(f"📌 初始 T translation: {T0[:3, 3].tolist()}")

    # 收集 frames
    frames = gather_frames(args.scenarios, args.cam,
                            max_per_scene=args.max_frames_per_scene,
                            step=args.sample_step)
    if not frames:
        print(f"❌ 沒有 frame 可用")
        sys.exit(1)
    print(f"📂 {len(frames)} frames 待用")

    # 預先計算每 frame 的 idt + lidar 邊緣權重
    cache = []
    print(f"🔄 預處理每 frame edge map ...")
    for i, (ip, pp) in enumerate(frames):
        img = cv2.imread(ip)
        if img is None: continue
        idt = compute_image_idt(img)
        pts_raw = read_pcd(pp)
        if len(pts_raw) < 1000: continue
        pts, ew = compute_lidar_edge_weight(pts_raw, sample_step=args.lidar_step)
        K, D, kind = get_KD(args.cam, img.shape)
        cache.append((pts, ew, idt, K, D, kind))
        if (i + 1) % 10 == 0:
            print(f"   {i+1}/{len(frames)}")
    print(f"✅ 預處理完成,可用 {len(cache)} frames")

    # 多 frame 平均 cost
    def total_cost(params):
        T = params_to_T(params, T0)
        total = 0.0
        for pts, ew, idt, K, D, kind in cache:
            total += cost_for_frame(T, pts, ew, idt, K, D, kind, len(pts))
        return total / len(cache)

    # 初始 cost
    c0 = total_cost(np.zeros(6))
    print(f"📊 初始 cost = {c0:.5f}  (越負越好)")

    # 有 bounds 的最佳化(關鍵!避免飛出去)
    # 旋轉:±10 度,平移:±0.3 m
    bounds = [(-10, 10), (-10, 10), (-10, 10),
              (-0.3, 0.3), (-0.3, 0.3), (-0.3, 0.3)]

    print(f"\n🔧 Stage 1: 粗調 (L-BFGS-B, bounded ±10° / ±0.3m)")
    res1 = minimize(total_cost, np.zeros(6), method='L-BFGS-B',
                     bounds=bounds,
                     options={'ftol': 1e-5, 'maxiter': 100, 'disp': False})
    print(f"   cost = {res1.fun:.5f}, delta = {[round(v, 3) for v in res1.x]}")

    print(f"\n🔧 Stage 2: 微調 (Powell, 窄範圍)")
    bounds_narrow = [(-3, 3), (-3, 3), (-3, 3),
                     (-0.1, 0.1), (-0.1, 0.1), (-0.1, 0.1)]
    # Powell 不支援 bounds,但用窄初始 + 嚴格 xtol 等效
    res2 = minimize(lambda p: total_cost(p), res1.x, method='Powell',
                     options={'xtol': 0.005, 'ftol': 1e-6,
                              'maxiter': 200, 'disp': False})
    # clamp 防爆
    clamped = np.clip(res2.x, [-15, -15, -15, -0.5, -0.5, -0.5],
                                [ 15,  15,  15,  0.5,  0.5,  0.5])
    res2_final_cost = total_cost(clamped)
    print(f"   cost = {res2_final_cost:.5f}, delta = {[round(v, 3) for v in clamped]}")
    # 用最終 clamped 值
    res2 = type('obj', (), {'x': clamped, 'fun': res2_final_cost})()

    T_final = params_to_T(res2.x, T0)
    print(f"\n📌 最終 T translation: {T_final[:3, 3].tolist()}")
    print(f"📌 改善 cost: {c0:.5f} → {res2.fun:.5f}  ({100*(res2.fun-c0)/abs(c0):.1f}%)")

    out_path = args.out or f"refined_{args.cam}.json"
    with open(out_path, 'w', encoding='utf-8') as f:
        json.dump({
            'cam': args.cam,
            'method': 'targetless_edge_correlation',
            'frames_used': len(cache),
            'cost_initial': float(c0),
            'cost_final': float(res2.fun),
            'delta_rpy_xyz': res2.x.tolist(),
            'T_initial': T0.tolist(),
            'T_refined': T_final.tolist(),
        }, f, indent=2)
    print(f"\n✅ 結果存:{out_path}")


if __name__ == '__main__':
    main()
