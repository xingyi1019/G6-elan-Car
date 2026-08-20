"""
魚眼內參校正 — 最終版:bootstrap 偵測 + 不用 CHECK_COND + 逐張剔高誤差
盡量保留最多張,只剔真正的壞圖。

用法(直接指向資料夾,不必改檔):
    python calib_fisheye_final.py --img-dir <棋盤照片資料夾>
    python calib_fisheye_final.py --img-dir <資料夾> --cb 9x6 --draw
    --draw 會把每張「偵測到的角點圖」存到 <out>/corners/(論文那種彩色點+連線)
不給參數時,沿用下方 IMG_DIR / CB 預設值。
"""
import cv2, glob, os, sys, argparse
import numpy as np
sys.stdout.reconfigure(encoding='utf-8')

# ── 預設值(不給 --img-dir 時沿用)──
IMG_DIR = r"E:\Car\calibration_data_rear\png"
CB = (9, 6)

ap = argparse.ArgumentParser(description="魚眼內參校正(KB 模型)")
ap.add_argument('--img-dir', default=None, help='棋盤照片資料夾(不給=懶人模式,自動找目前資料夾或 ./images)')
ap.add_argument('--cb', default=f"{CB[0]}x{CB[1]}", help='內角點,如 9x6')
ap.add_argument('--ext', default='', help='副檔名(png/jpg);留空=自動抓 png+jpg')
ap.add_argument('--draw', action='store_true', help='輸出每張角點偵測圖到 <out>/corners/')
ap.add_argument('--out', default='calib_out', help='輸出資料夾(--draw 用)')
ap.add_argument('--cam', default='cam', help='相機名稱(僅顯示用)')
args = ap.parse_args()

CB = tuple(int(x) for x in args.cb.lower().split('x'))
EXTS = (f"*.{args.ext.lstrip('.')}",) if args.ext else ("*.png","*.jpg","*.jpeg","*.bmp")
def grab(d):
    return sorted(sum((glob.glob(os.path.join(d, e)) for e in EXTS), []))

# 決定 IMG_DIR:--img-dir 優先;否則懶人模式(目前資料夾 → ./images → 檔案內預設)
if args.img_dir:
    IMG_DIR = args.img_dir; imgs = grab(IMG_DIR)
else:
    for cand in (".", "images", IMG_DIR):
        imgs = grab(cand)
        if imgs: IMG_DIR = cand; break
if not imgs:
    sys.exit(f"❌ 找不到影像。用法:python calib_fisheye_final.py --img-dir <資料夾>\n"
             f"   (或 cd 到有棋盤照片的資料夾直接跑)")
print(f"影像來源:{os.path.abspath(IMG_DIR)}")
if args.draw:
    os.makedirs(os.path.join(args.out, "corners"), exist_ok=True)

objp = np.zeros((1, CB[0]*CB[1], 3), np.float64)
objp[0, :, :2] = np.mgrid[0:CB[0], 0:CB[1]].T.reshape(-1, 2)
subpix = (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 50, 1e-4)

def detect(gray):
    try:
        ok, c = cv2.findChessboardCornersSB(gray, CB,
                    flags=cv2.CALIB_CB_EXHAUSTIVE+cv2.CALIB_CB_ACCURACY)
        if ok: return c
    except: pass
    ok, c = cv2.findChessboardCorners(gray, CB,
                cv2.CALIB_CB_ADAPTIVE_THRESH+cv2.CALIB_CB_NORMALIZE_IMAGE)
    if ok: return cv2.cornerSubPix(gray, c, (5,5), (-1,-1), subpix)
    return None

img_shape = None
for p in imgs:
    im = cv2.imread(p)
    if im is not None: img_shape = im.shape[:2][::-1]; break
W, H = img_shape
print(f"[{args.cam}] {len(imgs)} 張 @ {W}x{H},棋盤 {CB[0]}x{CB[1]}")

# bootstrap 起點 —— 依解析度自動設:光心=影像中心、焦距猜值≈0.21*W(涵蓋一般魚眼)
K0 = np.array([[0.21*W,0,W/2.0],[0,0.21*W,H/2.0],[0,0,1]], np.float64)
D0 = np.array([[0.357],[-0.0526],[-0.1046],[0.0583]], np.float64)
new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
    K0, D0, img_shape, np.eye(3), balance=1.0, new_size=(W,H))
map1, map2 = cv2.fisheye.initUndistortRectifyMap(
    K0, D0, np.eye(3), new_K, img_shape, cv2.CV_16SC2)

op, ip, names = [], [], []
for p in imgs:
    img = cv2.imread(p); name = os.path.basename(p)
    g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    c = detect(g)
    if c is not None:
        op.append(objp.copy()); ip.append(c.reshape(1,-1,2).astype(np.float64))
        names.append(name)
        if args.draw:
            vis = img.copy()
            cv2.drawChessboardCorners(vis, CB, c.reshape(-1,1,2).astype(np.float32), True)
            cv2.imwrite(os.path.join(args.out, "corners", os.path.splitext(name)[0]+"_corners.jpg"), vis)
        continue
    und = cv2.remap(img, map1, map2, cv2.INTER_LINEAR)
    cu = detect(cv2.cvtColor(und, cv2.COLOR_BGR2GRAY))
    if cu is not None:
        pts = cu.reshape(-1,2).astype(np.float64)
        norm = np.column_stack([(pts[:,0]-new_K[0,2])/new_K[0,0],
                                 (pts[:,1]-new_K[1,2])/new_K[1,1]])
        dist = cv2.fisheye.distortPoints(norm.reshape(1,-1,2), K0, D0).reshape(-1,2)
        op.append(objp.copy()); ip.append(dist.reshape(1,-1,2).astype(np.float64))
        names.append(name+"*")
print(f"偵測到 {len(op)} 張\n")

# 不用 CHECK_COND,改逐張剔高誤差
flags = (cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC + cv2.fisheye.CALIB_FIX_SKEW
         + cv2.fisheye.CALIB_USE_INTRINSIC_GUESS)

def calib(op, ip):
    n = len(op)
    K = K0.copy(); D = D0.copy()
    rv = [np.zeros((1,1,3)) for _ in range(n)]
    tv = [np.zeros((1,1,3)) for _ in range(n)]
    rms, K, D, rv, tv = cv2.fisheye.calibrate(op, ip, img_shape, K, D, rv, tv,
        flags, (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 200, 1e-8))
    # per-image error
    errs = []
    for i in range(n):
        pr, _ = cv2.fisheye.projectPoints(op[i], rv[i], tv[i], K, D)
        errs.append(np.linalg.norm(ip[i]-pr, axis=2).mean())
    return rms, K, D, np.array(errs)

op2, ip2, nm2 = list(op), list(ip), list(names)
while len(op2) >= 4:
    try:
        rms, K, D, errs = calib(op2, ip2)
    except cv2.error as e:
        # 發散就剔掉誤差最不穩的(無法算誤差時剔第一張)
        print(f"  發散,剔 {nm2[0]}"); op2.pop(0); ip2.pop(0); nm2.pop(0); continue
    worst = errs.argmax()
    if errs[worst] > 1.5 and len(op2) > 5:   # 只剔真的爛 (>1.5px) 且還夠多張
        print(f"  剔除 {nm2[worst]} (err={errs[worst]:.2f}px)")
        op2.pop(worst); ip2.pop(worst); nm2.pop(worst)
    else:
        break

fx, fy, cx, cy = K[0,0], K[1,1], K[0,2], K[1,2]
print(f"\n{'='*60}")
print(f"  {args.cam} 魚眼內參 (用 {len(op2)} 張, RMS={rms:.4f}px)")
print(f"{'='*60}")
print(f"\n# K_base @ {W}x{H}")
print(f"K_base = [{fx:.6f}, {fy:.6f}, {cx:.6f}, {cy:.6f}]")
print(f"\n# D [k1,k2,k3,k4]")
print(f"D = [{D[0,0]:.8f}, {D[1,0]:.8f}, {D[2,0]:.8f}, {D[3,0]:.8f}]")
print(f"\n# RMS = {rms:.4f} px,使用 {len(op2)}/{len(imgs)} 張")
if args.draw:
    print(f"# 角點偵測圖已存到: {os.path.join(args.out,'corners')}")
print(f"{'='*60}")
