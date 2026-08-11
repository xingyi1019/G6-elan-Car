"""
魚眼內參校正 — 最終版:bootstrap 偵測 + 不用 CHECK_COND + 逐張剔高誤差
盡量保留最多張,只剔真正的壞圖。
"""
import cv2, glob, os, sys, re
import numpy as np
sys.stdout.reconfigure(encoding='utf-8')

IMG_DIR = r"E:\Car\calibration_data_rear\png"
CB = (9, 6)
imgs = sorted(glob.glob(os.path.join(IMG_DIR, "*.png")))
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

# bootstrap 起點
K0 = np.array([[282.0,0,644.9],[0,281.2,508.5],[0,0,1]], np.float64)
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
        names.append(name); continue
    und = cv2.remap(img, map1, map2, cv2.INTER_LINEAR)
    cu = detect(cv2.cvtColor(und, cv2.COLOR_BGR2GRAY))
    if cu is not None:
        pts = cu.reshape(-1,2).astype(np.float64)
        norm = np.column_stack([(pts[:,0]-new_K[0,2])/new_K[0,0],
                                 (pts[:,1]-new_K[1,2])/new_K[1,1]])
        dist = cv2.fisheye.distortPoints(norm.reshape(1,-1,2), K0, D0).reshape(-1,2)
        op.append(objp.copy()); ip.append(dist.reshape(1,-1,2).astype(np.float64))
        names.append(name+"*")
print(f"偵測到 {len(op)} 張: {names}\n")

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
sc = W/1920.0
print(f"\n{'='*60}")
print(f"  rear 魚眼內參 (用 {len(op2)} 張, RMS={rms:.4f}px)")
print(f"{'='*60}")
print(f"\n# K_base @ {W}x{H}")
print(f"K_base = [{fx:.6f}, {fy:.6f}, {cx:.6f}, {cy:.6f}]")
print(f"\n# K_base @ 1920x1536 native")
print(f"K_base_native = [{fx/sc:.4f}, {fy/(H/1536.0):.4f}, {cx/sc:.4f}, {cy/(H/1536.0):.4f}]")
print(f"\n# D [k1,k2,k3,k4]")
print(f"D = [{D[0,0]:.8f}, {D[1,0]:.8f}, {D[2,0]:.8f}, {D[3,0]:.8f}]")
print(f"\n# RMS = {rms:.4f} px,使用圖: {nm2}")
print(f"{'='*60}")
