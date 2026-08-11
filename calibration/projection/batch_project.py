#!/usr/bin/env python3
"""
Headless batch projection — 從 alignment_tool.py 拆出純 OpenCV/Numpy 版本。
跑一個場景:讀每個 pcd + png,投影,輸出疊合圖,再串成 mp4。
不需要 PyQt5,不需要 ROS。
"""
import sys, os, glob, argparse, subprocess
import numpy as np
import cv2
from PIL import Image as PILImage
from matplotlib import cm as mpl_cm

sys.stdout.reconfigure(encoding='utf-8')


# ────────────────────────────── PCD reader (從 alignment_tool 拆出) ─────────
def read_pcd(filepath):
    with open(filepath, 'rb') as f:
        header = {}
        while True:
            raw = f.readline()
            if not raw: break
            line = raw.decode('utf-8', errors='ignore').strip()
            if line.startswith('#') or not line: continue
            parts = line.split()
            key = parts[0].upper()
            header[key] = parts[1:]
            if key == 'DATA':
                offset = f.tell()
                break

        dt_str = header.get('DATA', ['ascii'])[0].lower()
        fields = header.get('FIELDS', ['x', 'y', 'z'])
        sizes  = [int(s) for s in header.get('SIZE', ['4'] * len(fields))]
        types  = header.get('TYPE', ['F'] * len(fields))
        num_pts = int((header.get('POINTS') or header.get('WIDTH') or ['0'])[0])

        if dt_str == 'ascii':
            f.seek(offset)
            data = np.loadtxt(f, max_rows=num_pts or None)
            if data.ndim == 1: data = data.reshape(1, -1)
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


# ────────────────────────────── 載入 config ────────────────────────────────
def load_config(path):
    """exec() Python-dict 格式的 config(雖然副檔名是 .json)。"""
    ns = {'np': np}
    with open(path, 'r', encoding='utf-8') as f:
        exec(f.read(), ns)
    return ns['CAM_CONFIGS']


# ────────────────────────────── 投影函式(回傳 numpy 圖,不用 QPixmap) ─────
def project_to_image(pts, img_bgr, K, D, T, scale,
                     min_depth=0.5, max_depth=80.0, point_r=2,
                     cam_type='standard', opacity=1.0):
    R = T[:3, :3]
    t = T[:3, 3]
    pts_c = (R @ pts.T).T + t
    mask = (pts_c[:, 2] > min_depth) & (pts_c[:, 2] < max_depth)
    pts_c = pts_c[mask]
    if len(pts_c) == 0:
        return img_bgr.copy()

    depth = pts_c[:, 2]
    Ks = np.array([
        [K[0, 0] * scale, 0, K[0, 2] * scale],
        [0, K[1, 1] * scale, K[1, 2] * scale],
        [0, 0, 1],
    ], dtype=np.float64)

    if cam_type == 'fisheye':
        D_fish = np.asarray(D, dtype=np.float64).flatten()[:4].reshape(4, 1)
        pts_cam = pts_c.reshape(-1, 1, 3).astype(np.float64)
        img_pts, _ = cv2.fisheye.projectPoints(
            pts_cam, np.zeros((3, 1)), np.zeros((3, 1)), Ks, D_fish)
        u = img_pts[:, 0, 0]
        v = img_pts[:, 0, 1]
    else:
        xn = pts_c[:, 0] / depth
        yn = pts_c[:, 1] / depth
        Dl = np.asarray(D, dtype=np.float64).flatten()
        k1, k2, p1, p2, k3 = Dl[0], Dl[1], Dl[2], Dl[3], Dl[4] if len(Dl) > 4 else 0.0
        r2 = xn*xn + yn*yn
        rad = 1.0 + k1*r2 + k2*r2**2 + k3*r2**3
        xd = xn*rad + 2*p1*xn*yn + p2*(r2 + 2*xn*xn)
        yd = yn*rad + p1*(r2 + 2*yn*yn) + 2*p2*xn*yn
        u = Ks[0, 0] * xd + Ks[0, 2]
        v = Ks[1, 1] * yd + Ks[1, 2]

    arr = img_bgr.copy()  # BGR
    H, W = arr.shape[:2]
    valid = (u >= 0) & (u < W) & (v >= 0) & (v < H) & np.isfinite(u) & np.isfinite(v)
    u = u[valid].astype(np.int32)
    v = v[valid].astype(np.int32)
    d = depth[valid]
    if len(u) == 0:
        return arr

    d_n = np.clip((d - d.min()) / ((d.max() - d.min()) + 1e-6), 0.0, 1.0)
    colors_rgb = (mpl_cm.plasma(d_n)[:, :3] * 255).astype(np.float32)
    colors_bgr = colors_rgb[:, ::-1]  # BGR for OpenCV

    alpha = float(np.clip(opacity, 0.0, 1.0))
    for dy in range(-point_r, point_r + 1):
        for dx in range(-point_r, point_r + 1):
            if dx*dx + dy*dy <= point_r*point_r:
                pv = np.clip(v + dy, 0, H - 1)
                pu = np.clip(u + dx, 0, W - 1)
                bg = arr[pv, pu].astype(np.float32)
                arr[pv, pu] = (bg * (1.0 - alpha) + colors_bgr * alpha).astype(np.uint8)

    return arr


# ────────────────────────────── 批次處理 ────────────────────────────────────
def process_scenario(scenario_dir, cam_name, cfg, out_dir,
                     max_frames=None, point_r=2, opacity=0.9):
    """處理單一場景:依 frame index 對應 pcd + cam img,輸出投影圖。"""
    pcd_dir = os.path.join(scenario_dir, 'VLS128_pcd')
    img_dir = os.path.join(scenario_dir, 'images', cam_name)

    if not os.path.isdir(pcd_dir):
        print(f"  ❌ 缺 PCD 資料夾:{pcd_dir}")
        return 0
    if not os.path.isdir(img_dir):
        print(f"  ❌ 缺影像資料夾:{img_dir}")
        return 0

    pcd_files = sorted(glob.glob(os.path.join(pcd_dir, '*.pcd')))
    img_files_map = {os.path.splitext(os.path.basename(p))[0]: p
                     for p in glob.glob(os.path.join(img_dir, '*.png'))}

    cam = cfg[cam_name]
    K = cam.get('K')
    if K is None and 'K_base' in cam:
        kb = cam['K_base']
        K = np.array([[kb[0], 0, kb[2]], [0, kb[1], kb[3]], [0, 0, 1]], dtype=np.float32)
    D = np.asarray(cam['D'], dtype=np.float32)
    T = np.asarray(cam['T'], dtype=np.float32)
    scale = float(cam['scale'])
    cam_type = cam['type']

    os.makedirs(out_dir, exist_ok=True)
    count = 0
    for pcd_path in pcd_files:
        if max_frames is not None and count >= max_frames:
            break
        idx = os.path.splitext(os.path.basename(pcd_path))[0]
        if idx not in img_files_map:
            continue
        img_path = img_files_map[idx]
        img_bgr = cv2.imread(img_path)
        if img_bgr is None:
            continue
        pts = read_pcd(pcd_path)
        if len(pts) == 0:
            continue
        try:
            result = project_to_image(
                pts, img_bgr, K, D, T, scale,
                point_r=point_r, cam_type=cam_type, opacity=opacity)
        except Exception as e:
            print(f"  ⚠️ frame {idx} 投影失敗:{e}")
            continue
        out_path = os.path.join(out_dir, f'{idx}.png')
        cv2.imwrite(out_path, result)
        count += 1
        if count % 10 == 0:
            print(f"    [{cam_name}] {count} frames done", flush=True)
    return count


def make_video(image_dir, out_video, fps=10):
    """用 ffmpeg 把 image_dir 內的 *.png 串成 mp4。"""
    pattern = os.path.join(image_dir, '%06d.png')
    cmd = [
        'ffmpeg', '-y', '-framerate', str(fps),
        '-i', pattern,
        '-c:v', 'libx264', '-pix_fmt', 'yuv420p',
        '-vf', 'scale=trunc(iw/2)*2:trunc(ih/2)*2',
        out_video
    ]
    print(f"  → 產生影片:{out_video}")
    r = subprocess.run(cmd, capture_output=True)
    if r.returncode != 0:
        print(f"  ❌ ffmpeg 失敗:{r.stderr.decode('utf-8', errors='ignore')[-500:]}")
        return False
    return True


# ────────────────────────────── main ──────────────────────────────────────
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--data-root', default=r'C:\Users\xingy\OneDrive\Desktop\pcd_aligment\data')
    ap.add_argument('--config', default=r'C:\Users\xingy\OneDrive\Desktop\pcd_aligment\config_g6_6view.json')
    ap.add_argument('--scenario', required=True, help='場景資料夾名稱')
    ap.add_argument('--cams', default='main', help='相機,逗號分隔,例如 main,left,right')
    ap.add_argument('--max-frames', type=int, default=None)
    ap.add_argument('--out-root', default=r'C:\Users\xingy\OneDrive\Desktop\pcd_aligment\output')
    ap.add_argument('--fps', type=int, default=10)
    ap.add_argument('--point-r', type=int, default=2)
    ap.add_argument('--opacity', type=float, default=0.9)
    ap.add_argument('--no-video', action='store_true', help='只產 frame 不串影片')
    args = ap.parse_args()

    cfg = load_config(args.config)
    print(f"📋 載入 config:{len(cfg)} 個相機 → {list(cfg.keys())}")

    scenario_dir = os.path.join(args.data_root, args.scenario)
    print(f"📂 場景:{args.scenario}")

    for cam_name in args.cams.split(','):
        cam_name = cam_name.strip()
        if cam_name not in cfg:
            print(f"  ⚠️ 跳過:config 沒有 '{cam_name}'")
            continue
        out_dir = os.path.join(args.out_root, args.scenario, cam_name)
        print(f"\n▶ 相機:{cam_name} → {out_dir}")
        n = process_scenario(scenario_dir, cam_name, cfg, out_dir,
                              max_frames=args.max_frames,
                              point_r=args.point_r, opacity=args.opacity)
        print(f"  ✅ 完成 {n} 張 frame")

        if n > 0 and not args.no_video:
            video_out = os.path.join(args.out_root, args.scenario, f'{cam_name}_projected.mp4')
            make_video(out_dir, video_out, fps=args.fps)


if __name__ == '__main__':
    main()
