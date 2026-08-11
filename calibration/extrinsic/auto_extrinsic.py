"""
全自動側相機外參校正:
  1. 掃描指定相機資料夾,找出含棋盤格的 frame
  2. 自動偵測 9x6 棋盤(亞像素精煉)
  3. 在對應 PCD 中用 RANSAC 找棋盤平面
  4. 平面上重建 3D 角點 → solvePnP → 外參 T

  目前內參硬寫 BSD 針孔(fx=2023, fy=2030, cx=960, cy=768 @1920×1536)
  可從 config_g6_6view.json 自動 fallback fisheye 模型(left/right/rear)
"""
import os, sys, glob, json, argparse, time
import numpy as np
import cv2

sys.stdout.reconfigure(encoding='utf-8')

# ════════════════════════════════════════════════════════════════════════════
CHECKERBOARD = (9, 6)              # 內角點數量(9 寬 × 6 高)
SQUARE_SIZE_M = 0.108              # 每格 10.8 cm,你可改
# ════════════════════════════════════════════════════════════════════════════


# ── BSD 針孔內參 (1920×1536 native) ───────────────────────────────────────
K_NATIVE = np.array([
    [2023.0, 0.0,    960.0],
    [0.0,    2030.0, 768.0],
    [0.0,    0.0,    1.0  ]
], dtype=np.float64)
D_BROWN = np.array([-0.48825903, 0.35046806, 0.00301038, 0.00000390, -0.22704884],
                   dtype=np.float64)

# ── 魚眼 K_base + D(從 config_g6_6view.json 拷)───────────────────────────
K_FISH_BASE = [395.06367868, 395.73048124, 960.0, 768.0]
D_FISH = np.array([0.42750887, -0.14506998, 0.02445243, -0.00135651],
                  dtype=np.float64).reshape(4, 1)


# ════════════════════════════════════════════════════════════════════════════
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


# ════════════════════════════════════════════════════════════════════════════
def detect_chessboard(img_gray):
    """回傳 (corners_subpix, ok)"""
    flags = (cv2.CALIB_CB_ADAPTIVE_THRESH +
             cv2.CALIB_CB_NORMALIZE_IMAGE +
             cv2.CALIB_CB_FAST_CHECK)
    ok, corners = cv2.findChessboardCorners(img_gray, CHECKERBOARD, flags)
    if not ok:
        return None, False
    subpix_crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1)
    corners = cv2.cornerSubPix(img_gray, corners, (5, 5), (-1, -1), subpix_crit)
    return corners, True


def scan_camera_for_chessboard(scenario_dir, cam, max_frames=None, sample_step=1):
    """掃描單一相機,回傳含棋盤的 frame 清單 [(img_path, pcd_path, corners), ...]"""
    img_dir = os.path.join(scenario_dir, 'images', cam)
    pcd_dir = os.path.join(scenario_dir, 'VLS128_pcd')
    if not os.path.isdir(img_dir) or not os.path.isdir(pcd_dir):
        return []
    img_files = sorted(glob.glob(os.path.join(img_dir, '*.png')))[::sample_step]
    if max_frames: img_files = img_files[:max_frames]

    hits = []
    for ip in img_files:
        stem = os.path.splitext(os.path.basename(ip))[0]
        pp = os.path.join(pcd_dir, f'{stem}.pcd')
        if not os.path.exists(pp): continue
        img = cv2.imread(ip)
        if img is None: continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        corners, ok = detect_chessboard(gray)
        if ok:
            hits.append((ip, pp, corners))
    return hits


# ════════════════════════════════════════════════════════════════════════════
def ransac_plane(points, dist_thresh=0.03, max_iter=500):
    """RANSAC 找最大平面,回傳 (inlier_mask, normal, d)。平面方程式 n·p + d = 0"""
    N = len(points)
    if N < 100: return None, None, None
    best_inliers = 0
    best_normal = None
    best_d = None
    rng = np.random.default_rng(42)
    for _ in range(max_iter):
        idx = rng.choice(N, 3, replace=False)
        p1, p2, p3 = points[idx]
        v1 = p2 - p1
        v2 = p3 - p1
        n = np.cross(v1, v2)
        nm = np.linalg.norm(n)
        if nm < 1e-9: continue
        n = n / nm
        d = -n @ p1
        dist = np.abs(points @ n + d)
        inliers = dist < dist_thresh
        cnt = int(inliers.sum())
        if cnt > best_inliers:
            best_inliers = cnt
            best_normal = n
            best_d = d
    if best_normal is None: return None, None, None
    mask = np.abs(points @ best_normal + best_d) < dist_thresh
    # 用 inliers 精煉一次平面(SVD)
    pts_in = points[mask]
    centroid = pts_in.mean(axis=0)
    _, _, vh = np.linalg.svd(pts_in - centroid)
    n_refined = vh[-1]
    d_refined = -n_refined @ centroid
    return mask, n_refined, d_refined


def cluster_chessboard_region(points, n_normal, d, dist_thresh=0.03,
                              expected_size_m=1.0, lidar_origin=None):
    """從平面 inlier 中,找最像棋盤板的群集(去除地面、牆面)"""
    plane_dist = np.abs(points @ n_normal + d)
    mask = plane_dist < dist_thresh
    pts = points[mask]
    if len(pts) < 50: return None
    # 棋盤典型在距離車 1~5m,過遠或過近的點先過濾
    if lidar_origin is not None:
        dist_origin = np.linalg.norm(pts - lidar_origin, axis=1)
        rng_mask = (dist_origin > 0.8) & (dist_origin < 8.0)
        pts = pts[rng_mask]
        if len(pts) < 50: return None
    # 簡單 grid clustering:找最密集區域
    centroid = pts.mean(axis=0)
    diff = pts - centroid
    # 投影到平面內 2D
    u_axis = np.cross(n_normal, [0, 0, 1])
    if np.linalg.norm(u_axis) < 1e-6:
        u_axis = np.cross(n_normal, [1, 0, 0])
    u_axis /= np.linalg.norm(u_axis)
    v_axis = np.cross(n_normal, u_axis)
    u = diff @ u_axis
    v = diff @ v_axis
    # 只保留中央 1.5m × 1.5m
    keep = (np.abs(u) < 0.75) & (np.abs(v) < 0.75)
    if keep.sum() < 30: return None
    return pts[keep]


# ════════════════════════════════════════════════════════════════════════════
def build_obj_points():
    """棋盤格 3D 點(checkerboard 本身座標系)"""
    objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
    objp *= SQUARE_SIZE_M
    return objp


def get_K_for(cam, img_shape, cam_model='bsd_pinhole'):
    """取得縮放後 K"""
    h, w = img_shape[:2]
    if cam_model == 'bsd_pinhole':
        sx, sy = w / 1920.0, h / 1536.0
        K = K_NATIVE.copy()
        K[0, 0] *= sx; K[0, 2] *= sx
        K[1, 1] *= sy; K[1, 2] *= sy
        return K, D_BROWN, 'pinhole'
    elif cam_model == 'fisheye':
        sx = w / 1920.0
        K = np.array([
            [K_FISH_BASE[0] * sx, 0,                  K_FISH_BASE[2] * sx],
            [0,                  K_FISH_BASE[1] * sx, K_FISH_BASE[3] * sx],
            [0, 0, 1]
        ], dtype=np.float64)
        return K, D_FISH, 'fisheye'
    raise ValueError(f"unknown model {cam_model}")


def solve_extrinsic_from_pair(img_path, pcd_path, cam_model, verbose=True):
    """從單組 (image, pcd) 算外參 T。"""
    img = cv2.imread(img_path)
    if img is None: return None
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    corners, ok = detect_chessboard(gray)
    if not ok:
        if verbose: print(f"  ❌ 影像 {os.path.basename(img_path)} 找不到棋盤")
        return None

    pts = read_pcd(pcd_path)
    if len(pts) < 1000: return None

    # 找棋盤平面(LiDAR 原點 0,0,0)
    mask, n, d = ransac_plane(pts, dist_thresh=0.03)
    if mask is None:
        if verbose: print(f"  ❌ RANSAC 平面找不到")
        return None
    inlier_pts = cluster_chessboard_region(pts[mask], n, d,
                                            lidar_origin=np.zeros(3))
    if inlier_pts is None or len(inlier_pts) < 30:
        if verbose: print(f"  ⚠️  inlier 不足,RANSAC 平面 = {mask.sum()} 點")
        return None

    centroid_3d = inlier_pts.mean(axis=0)

    # K, D
    K, D, model_kind = get_K_for(None, img.shape, cam_model)

    # 棋盤格 3D 物件點(checkerboard 自己的座標系,Z=0)
    objp = build_obj_points()
    objp_centered = objp.copy()
    objp_centered[:, 0] -= objp[:, 0].mean()
    objp_centered[:, 1] -= objp[:, 1].mean()

    # 棋盤板實際在 LiDAR 座標系的姿態:用平面法向 n 跟 centroid 對齊
    # 把 board 的 (0,0,1) 對齊到 -n(板朝向相機),(0,0,0) 對齊到 centroid_3d
    z_board = -n / np.linalg.norm(n)  # 朝相機方向
    # 找正交的 x_board, y_board
    if abs(z_board[2]) < 0.99:
        x_board = np.cross(z_board, [0, 0, 1])
    else:
        x_board = np.cross(z_board, [1, 0, 0])
    x_board /= np.linalg.norm(x_board)
    y_board = np.cross(z_board, x_board)
    R_board_to_lidar = np.stack([x_board, y_board, z_board], axis=1)

    # 把 objp(在 board 座標系)變到 LiDAR 座標系
    objp_lidar = (R_board_to_lidar @ objp_centered.T).T + centroid_3d

    # 用 solvePnP:objp_lidar(3D in LiDAR) ↔ corners(2D image)
    if model_kind == 'fisheye':
        # 先把 2D corners 用 fisheye 去畸變到「歸一化 + 重投影為 pinhole」
        und = cv2.fisheye.undistortPoints(corners.reshape(-1, 1, 2).astype(np.float64),
                                            K, D)
        und_pix = und.reshape(-1, 2) * np.array([K[0, 0], K[1, 1]]) + np.array([K[0, 2], K[1, 2]])
        und_pix = und_pix.astype(np.float32).reshape(-1, 1, 2)
        ok2, rvec, tvec = cv2.solvePnP(
            objp_lidar.astype(np.float64),
            und_pix,
            K, np.zeros(5),
            flags=cv2.SOLVEPNP_ITERATIVE)
    else:
        ok2, rvec, tvec = cv2.solvePnP(
            objp_lidar.astype(np.float64),
            corners.reshape(-1, 1, 2).astype(np.float64),
            K, D,
            flags=cv2.SOLVEPNP_ITERATIVE)

    if not ok2: return None

    R, _ = cv2.Rodrigues(rvec)
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = tvec.reshape(3)

    # 算重投影誤差
    if model_kind == 'fisheye':
        proj, _ = cv2.fisheye.projectPoints(
            objp_lidar.reshape(-1, 1, 3).astype(np.float64),
            rvec, tvec, K, D)
    else:
        proj, _ = cv2.projectPoints(objp_lidar, rvec, tvec, K, D)
    err = np.linalg.norm(proj.reshape(-1, 2) - corners.reshape(-1, 2), axis=1)
    rms = float(np.sqrt((err ** 2).mean()))

    return {
        'T': T.tolist(),
        'rms_pixels': rms,
        'n_corners': int(corners.shape[0]),
        'n_plane_inliers': int(len(inlier_pts)),
        'img_path': img_path,
        'pcd_path': pcd_path,
    }


# ════════════════════════════════════════════════════════════════════════════
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--scenarios', nargs='+', required=True,
                    help='場景資料夾路徑(可多個)')
    ap.add_argument('--cams', nargs='+', default=['sideL', 'sideR', 'left', 'right'],
                    help='要校的相機')
    ap.add_argument('--max-frames-per-scene', type=int, default=80)
    ap.add_argument('--sample-step', type=int, default=1)
    ap.add_argument('--out-dir', default=r'C:\Users\xingy\OneDrive\Desktop\pcd_aligment\auto_extrinsic_results')
    args = ap.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)

    # 模型對應(根據你 config_g6_6view.json:left/right/rear 是 fisheye)
    cam_models = {
        'sideL': 'bsd_pinhole', 'sideR': 'bsd_pinhole',
        'left':  'fisheye',     'right': 'fisheye',     'rear': 'fisheye',
    }

    all_results = {cam: [] for cam in args.cams}

    for scene in args.scenarios:
        if not os.path.isdir(scene):
            print(f"⚠️ 場景不存在:{scene}")
            continue
        print(f"\n📂 場景 {os.path.basename(scene)}")
        for cam in args.cams:
            model = cam_models.get(cam, 'bsd_pinhole')
            t0 = time.time()
            hits = scan_camera_for_chessboard(
                scene, cam,
                max_frames=args.max_frames_per_scene,
                sample_step=args.sample_step)
            print(f"  [{cam:5s}] ({model}) 掃描 {time.time()-t0:.1f}s → 找到 {len(hits)} 張含棋盤")
            for ip, pp, _ in hits[:5]:    # 最多取 5 張
                result = solve_extrinsic_from_pair(ip, pp, model, verbose=False)
                if result is not None:
                    result['scene'] = os.path.basename(scene)
                    result['cam'] = cam
                    result['model'] = model
                    all_results[cam].append(result)
                    print(f"    ✓ {os.path.basename(ip)}  RMS={result['rms_pixels']:.2f}px  "
                          f"inliers={result['n_plane_inliers']}")

    # 統整輸出
    summary = {}
    for cam, results in all_results.items():
        if not results:
            summary[cam] = {'count': 0, 'msg': '找不到含棋盤的 frame 或都解算失敗'}
            continue
        # 按 RMS 排序
        results.sort(key=lambda r: r['rms_pixels'])
        best = results[0]
        # 多組結果取「中位數」當穩健估計(平移 + 旋轉)
        Ts = np.array([np.asarray(r['T']) for r in results])
        T_med = np.median(Ts, axis=0)
        summary[cam] = {
            'count': len(results),
            'best_rms': best['rms_pixels'],
            'best_T': best['T'],
            'median_T': T_med.tolist(),
            'all_rms': [r['rms_pixels'] for r in results],
            'model': best['model'],
        }
        print(f"\n📊 [{cam}] {len(results)} 組結果")
        print(f"    Best RMS: {best['rms_pixels']:.2f} px")
        print(f"    Median T translation: {T_med[:3, 3].tolist()}")

    # 存檔
    out_path = os.path.join(args.out_dir, 'extrinsic_summary.json')
    with open(out_path, 'w', encoding='utf-8') as f:
        json.dump({'summary': summary,
                    'all_results': all_results,
                    'checkerboard': list(CHECKERBOARD),
                    'square_size_m': SQUARE_SIZE_M},
                   f, indent=2, ensure_ascii=False)
    print(f"\n✅ 完整結果存:{out_path}")


if __name__ == '__main__':
    main()
