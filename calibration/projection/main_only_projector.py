"""
只跑主相機投影 — 用 config_g6_6view.json 的最新校正參數。
輸出主相機原解析度(2560×1440)直接覆蓋點雲。
"""
import cv2, numpy as np, os, glob, sys, argparse
sys.stdout.reconfigure(encoding='utf-8')

ap = argparse.ArgumentParser()
ap.add_argument('--scenario-dir', required=True)
ap.add_argument('--config', default=r'C:\Users\xingy\OneDrive\Desktop\pcd_aligment\config_g6_6view.json')
ap.add_argument('--out', required=True)
ap.add_argument('--fps', type=float, default=10.0)
ap.add_argument('--max-frames', type=int, default=None)
ap.add_argument('--pcd-offset', type=int, default=0)
ap.add_argument('--max-depth', type=float, default=40.0)
ap.add_argument('--point-r', type=int, default=2)
args = ap.parse_args()

# ─── 載入 config ──────────────────────────────────────────────────────────
ns = {'np': np}
with open(args.config, 'r', encoding='utf-8') as f:
    exec(f.read(), ns)
CFG = ns['CAM_CONFIGS']
cam = CFG['main']

scale = float(cam['scale'])
if 'K' in cam:
    K_full = np.asarray(cam['K'], dtype=np.float64)
    K = K_full.copy() * scale
    K[2, 2] = 1.0
else:
    kb = cam['K_base']
    K = np.array([[kb[0]*scale, 0, kb[2]*scale],
                  [0, kb[1]*scale, kb[3]*scale],
                  [0, 0, 1]], dtype=np.float64)
D = np.asarray(cam['D'], dtype=np.float64).flatten()[:4].reshape(4, 1)
T = np.asarray(cam['T'], dtype=np.float64)
print(f"📋 載入 main 相機參數:scale={scale:.4f}, K={K.flatten()[:6].tolist()}")
print(f"   D={D.flatten().tolist()}, T_trans={T[:3, 3].tolist()}")


# ─── PCD reader ───────────────────────────────────────────────────────────
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
                offset = f.tell(); break
        dt_str = header.get('DATA', ['ascii'])[0].lower()
        fields = header.get('FIELDS', ['x', 'y', 'z'])
        sizes  = [int(s) for s in header.get('SIZE', ['4']*len(fields))]
        types  = header.get('TYPE', ['F']*len(fields))
        num_pts = int((header.get('POINTS') or header.get('WIDTH') or ['0'])[0])
        if dt_str == 'ascii':
            f.seek(offset)
            data = np.loadtxt(f, max_rows=num_pts or None)
            if data.ndim == 1: data = data.reshape(1, -1)
            xi, yi, zi = fields.index('x'), fields.index('y'), fields.index('z')
            return data[:, [xi, yi, zi]].astype(np.float32)
        else:
            tmap = {'F': 'f', 'I': 'i', 'U': 'u'}
            dt = np.dtype([(fn, tmap.get(t, 'f')+str(sz))
                           for fn, sz, t in zip(fields, sizes, types)])
            f.seek(offset)
            data = np.frombuffer(f.read(num_pts*dt.itemsize), dtype=dt)
            return np.column_stack([data['x'].astype(np.float32),
                                     data['y'].astype(np.float32),
                                     data['z'].astype(np.float32)])
    return np.zeros((0, 3), np.float32)


# ─── 投影 ─────────────────────────────────────────────────────────────────
def project_main(img, pts):
    h, w = img.shape[:2]
    R, t = T[:3, :3], T[:3, 3]
    pts_cam = (R @ pts.T).T + t
    mask = (pts_cam[:, 2] > 0.5) & (pts_cam[:, 2] < 100.0)
    p = pts_cam[mask]
    if len(p) == 0: return img

    img_pts, _ = cv2.fisheye.projectPoints(
        p.reshape(-1, 1, 3).astype(np.float64),
        np.zeros((3, 1)), np.zeros((3, 1)), K, D)
    u = img_pts[:, 0, 0].astype(int)
    v = img_pts[:, 0, 1].astype(int)
    dist = np.linalg.norm(p, axis=1)
    vm = (u >= 0) & (u < w) & (v >= 0) & (v < h)
    u, v, dist = u[vm], v[vm], dist[vm]
    if len(u) == 0: return img
    colors = cv2.applyColorMap(
        (np.clip(dist / args.max_depth, 0, 1) * 255).astype(np.uint8),
        cv2.COLORMAP_JET)
    for i in range(len(u)):
        cv2.circle(img, (u[i], v[i]), args.point_r, colors[i][0].tolist(), -1)
    return img


# ─── 主迴圈 ───────────────────────────────────────────────────────────────
ROOT = args.scenario_dir
MAIN_DIR = os.path.join(ROOT, 'images', 'main')
PCD_DIR  = os.path.join(ROOT, 'VLS128_pcd')

main_files = sorted(glob.glob(os.path.join(MAIN_DIR, '*.png')))
if not main_files:
    print(f"❌ {MAIN_DIR} 沒有 png"); sys.exit(1)
indices = [int(os.path.basename(f).split('.')[0]) for f in main_files]
if args.max_frames:
    indices = indices[:args.max_frames]

first_img = cv2.imread(main_files[0])
H, W = first_img.shape[:2]
print(f"📐 主相機解析度:{W}×{H}")

os.makedirs(os.path.dirname(args.out), exist_ok=True)
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out = cv2.VideoWriter(args.out, fourcc, args.fps, (W, H))

print(f"📂 {os.path.basename(ROOT)}")
print(f"   {len(indices)} frames @ {args.fps} FPS  |  pcd_offset={args.pcd_offset}")

done = 0
for idx in indices:
    img_path = os.path.join(MAIN_DIR, f"{idx:06d}.png")
    if not os.path.exists(img_path): continue
    img = cv2.imread(img_path)
    pcd_path = os.path.join(PCD_DIR, f"{idx + args.pcd_offset:06d}.pcd")
    if os.path.exists(pcd_path):
        pts = read_pcd(pcd_path)
        if len(pts) > 0:
            img = project_main(img, pts)
    cv2.putText(img, f"frame {idx:06d}  |  {os.path.basename(ROOT)}",
                (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
    out.write(img)
    done += 1
    if done % 10 == 0:
        print(f"  [{done}/{len(indices)}]", flush=True)

out.release()
print(f"✅ 完成:{args.out}  ({os.path.getsize(args.out)/1024/1024:.1f} MB)")
