"""
平面對平面 LiDAR-主相機外參校正 (Zhang-Pless 風格)
  棋盤 9x6 內角點, 每格 0.10 m
  相機端: solvePnP -> 板平面 (法向 n_c, 中心點 p_c)
  光達端: 以初始外參定位板子 -> RANSAC 平面 (n_l, c_l)
  解 R: Kabsch (對齊法向量)；解 t: 點到平面最小平方
"""
import cv2, numpy as np, glob, os, sys, argparse
sys.stdout.reconfigure(encoding='utf-8')

ap = argparse.ArgumentParser(description="平面對平面 LiDAR-主相機外參校正")
ap.add_argument('--img-dir', required=True, help='棋盤影像資料夾')
ap.add_argument('--pcd-dir', required=True, help='對應點雲資料夾(檔名與影像相同)')
ap.add_argument('--ext', default='png', help='影像副檔名(png / jpg)')
ap.add_argument('--square', type=float, default=0.10, help='棋盤每格邊長(公尺)')
ap.add_argument('--max-frames', type=int, default=80)
ap.add_argument('--stride', type=int, default=1, help='抽樣間隔(資料多時可調大)')
ap.add_argument('--out', default='.', help='結果輸出資料夾')
args = ap.parse_args()

IMG_DIR, PCD_DIR = args.img_dir, args.pcd_dir
IMG_EXT = args.ext.lstrip('.')
CB = (9, 6)
SQUARE = args.square                # 每格邊長(公尺)
MAX_FRAMES = args.max_frames        # 最多用幾組
STRIDE = args.stride                # 跨整個 session 抽樣 (避免姿態單一)
OUT = args.out
os.makedirs(OUT, exist_ok=True)

# 主相機新內參 @ 2560x1440
K_2k5 = np.array([[1603.82, 0, 1288.68], [0, 1606.68, 737.29], [0, 0, 1]], np.float64)
D = np.array([-0.33734, 0.14133, -0.04107, 0.00647], np.float64).reshape(4, 1)

# 初始外參 (config) LiDAR->Camera
T_init = np.array([[-0.00363134, -0.99994677, 0.00965866, 0.05],
                   [-0.12839811, -0.00911254, -0.99168092, -0.15],
                   [ 0.99171609, -0.00484128, -0.12835820, -0.0245],
                   [0, 0, 0, 1]], np.float64)

# 棋盤 3D 物件點 (公制)
objp = np.zeros((CB[0]*CB[1], 3), np.float64)
objp[:, :2] = np.mgrid[0:CB[0], 0:CB[1]].T.reshape(-1, 2) * SQUARE
subpix = (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 50, 1e-4)


def read_pcd(path):
    with open(path, 'rb') as f:
        header = {}
        while True:
            raw = f.readline()
            if not raw: break
            line = raw.decode('utf-8', errors='ignore').strip()
            if line.startswith('#') or not line: continue
            p = line.split(); header[p[0].upper()] = p[1:]
            if p[0].upper() == 'DATA':
                off = f.tell(); break
        dt = header.get('DATA', ['ascii'])[0].lower()
        fields = header.get('FIELDS', ['x','y','z'])
        sizes = [int(s) for s in header.get('SIZE', ['4']*len(fields))]
        types = header.get('TYPE', ['F']*len(fields))
        num = int((header.get('POINTS') or header.get('WIDTH') or ['0'])[0])
        if dt == 'ascii':
            f.seek(off); d = np.loadtxt(f, max_rows=num or None)
            if d.ndim==1: d=d.reshape(1,-1)
            xi,yi,zi=fields.index('x'),fields.index('y'),fields.index('z')
            return d[:,[xi,yi,zi]].astype(np.float64)
        tmap={'F':'f','I':'i','U':'u'}
        ddt=np.dtype([(fn,tmap.get(t,'f')+str(sz)) for fn,sz,t in zip(fields,sizes,types)])
        f.seek(off); d=np.frombuffer(f.read(num*ddt.itemsize),dtype=ddt)
        return np.column_stack([d['x'],d['y'],d['z']]).astype(np.float64)


def ransac_plane(P, thr=0.02, iters=400):
    best=0; bn=None; bd=None
    rng=np.random.default_rng(0); N=len(P)
    for _ in range(iters):
        idx=rng.choice(N,3,replace=False)
        p1,p2,p3=P[idx]
        n=np.cross(p2-p1,p3-p1); nn=np.linalg.norm(n)
        if nn<1e-9: continue
        n/=nn; d=-n@p1
        inl=np.abs(P@n+d)<thr; c=int(inl.sum())
        if c>best: best=c; bn=n; bd=d
    mask=np.abs(P@bn+bd)<thr
    Pin=P[mask]; cen=Pin.mean(0)
    _,_,vh=np.linalg.svd(Pin-cen); n=vh[-1]; d=-n@cen
    return n, cen, mask.sum()


# ── 收集每個 frame 的平面對應 ──
pcds = sorted(glob.glob(os.path.join(PCD_DIR, '*.pcd')))[::STRIDE]   # 跨 session 抽樣
n_c_list, p_c_list, n_l_list, c_l_list, used = [], [], [], [], []
Ri, ti = T_init[:3,:3], T_init[:3,3]

print(f"掃描 frames (stride={STRIDE}, 候選 {len(pcds)}) ...")
for pp in pcds:
    if len(used) >= MAX_FRAMES: break
    stem = os.path.splitext(os.path.basename(pp))[0]
    ip = os.path.join(IMG_DIR, f'{stem}.{IMG_EXT}')
    if not os.path.exists(ip): continue
    img = cv2.imread(ip)
    if img is None: continue
    h, w = img.shape[:2]
    g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ok, corners = cv2.findChessboardCorners(g, CB,
        cv2.CALIB_CB_ADAPTIVE_THRESH+cv2.CALIB_CB_NORMALIZE_IMAGE+cv2.CALIB_CB_FAST_CHECK)
    if not ok: continue
    corners = cv2.cornerSubPix(g, corners, (7,7), (-1,-1), subpix)

    # K 縮到實際解析度
    sx, sy = w/2560.0, h/1440.0
    K = K_2k5.copy(); K[0,0]*=sx; K[0,2]*=sx; K[1,1]*=sy; K[1,2]*=sy

    # 魚眼去畸變角點 -> solvePnP (公制板)
    und = cv2.fisheye.undistortPoints(corners.reshape(-1,1,2), K, D, P=K)
    ok2, rvec, tvec = cv2.solvePnP(objp, und.reshape(-1,1,2), K, None,
                                    flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok2: continue
    Rb,_ = cv2.Rodrigues(rvec)
    n_c = Rb[:,2]                              # 板法向 (相機座標)
    p_c = (Rb @ objp.T).T.mean(0) + tvec.flatten()  # 板中心 (相機座標)
    if n_c[2] > 0: n_c = -n_c                  # 法向朝相機 (z<0 方向統一)

    # 光達端:用初始外參定位板子
    pts = read_pcd(pp)
    pc = (Ri @ pts.T).T + ti                   # lidar -> camera (初始)
    # 投影板中心到影像,取板子 2D bbox 內、深度接近板子的點
    cs = corners.reshape(-1,2)
    bx0,by0 = cs[:,0].min()-40, cs[:,1].min()-40
    bx1,by1 = cs[:,0].max()+40, cs[:,1].max()+40
    board_z = p_c[2]
    msk = (pc[:,2]>board_z-0.5)&(pc[:,2]<board_z+0.5)
    cand = pc[msk]
    if len(cand) < 50: continue
    und2,_ = cv2.fisheye.projectPoints(cand.reshape(-1,1,3),
        np.zeros((3,1)),np.zeros((3,1)),K,D)
    uu=und2[:,0,0]; vv=und2[:,0,1]
    inb=(uu>=bx0)&(uu<bx1)&(vv>=by0)&(vv<by1)
    board_cam = cand[inb]
    if len(board_cam) < 40: continue
    # 轉回 lidar 座標做 RANSAC
    board_lidar = (Ri.T @ (board_cam - ti).T).T
    n_l, c_l, ninl = ransac_plane(board_lidar, thr=0.03)
    if ninl < 30: continue

    n_c_list.append(n_c); p_c_list.append(p_c)
    n_l_list.append(n_l); c_l_list.append(c_l)
    used.append(stem)
    print(f"  {stem}: board@{board_z:.1f}m, lidar板點={ninl}")

print(f"\n收集 {len(used)} 組平面對應")
if len(used) < 5:
    print("❌ 有效組數太少"); sys.exit(1)

# ── 板子姿態多樣性檢查 (相機端法向量的分散程度) ──
Nc_chk = np.array(n_c_list)
spread = Nc_chk.std(axis=0)
print(f"板法向量分散度 (std xyz): {spread.round(3).tolist()}")
if spread.max() < 0.08:
    print("⚠️  警告:板子姿態過於單一 (法向量分散度低),外參解可能不可靠!")

# ── 解 R (Kabsch: 對齊法向量) ──
Nc = np.array(n_c_list); Nl = np.array(n_l_list)
# 統一光達法向方向 (與初始 R 投影後一致)
for i in range(len(Nl)):
    if (Ri @ Nl[i]) @ Nc[i] < 0: Nl[i] = -Nl[i]
H = Nl.T @ Nc
U,S,Vt = np.linalg.svd(H)
d = np.sign(np.linalg.det(Vt.T @ U.T))
R_new = Vt.T @ np.diag([1,1,d]) @ U.T

# ── 解 t (點到平面最小平方) ──
# 相機板平面: n_c·X + dc = 0, dc = -n_c·p_c
# 約束: n_c·(R c_l + t) + dc = 0 -> n_c·t = -dc - n_c·(R c_l)
A=[]; b=[]
for i in range(len(used)):
    nc=Nc[i]; pc=p_c_list[i]; cl=c_l_list[i]
    dc = -nc@pc
    A.append(nc); b.append(-dc - nc@(R_new@cl))
A=np.array(A); b=np.array(b)
t_new,_,_,_ = np.linalg.lstsq(A, b, rcond=None)

T_new = np.eye(4); T_new[:3,:3]=R_new; T_new[:3,3]=t_new

# ── 評估:平面對齊殘差 ──
def plane_residual(R,t):
    res=[]
    for i in range(len(used)):
        nc=Nc[i]; pc=p_c_list[i]; cl=c_l_list[i]; dc=-nc@pc
        res.append(abs(nc@(R@cl+t)+dc))
    return np.mean(res)*100  # cm

r_init = plane_residual(Ri, ti)
r_new = plane_residual(R_new, t_new)

print("\n"+"="*60)
print("  外參校正結果 (平面對平面法)")
print("="*60)
print(f"\n# 初始外參 (config) 平面殘差: {r_init:.2f} cm")
print(f"# 新校正外參   平面殘差: {r_new:.2f} cm")
print(f"\n# 初始 t: {ti.round(4).tolist()}")
print(f"# 新   t: {t_new.round(4).tolist()}")
# 物理合理性把關
dt = np.linalg.norm(t_new - ti)
print(f"\n# 新 t 與初始 t 距離: {dt:.3f} m")
if np.linalg.norm(t_new) > 1.0:
    print("⚠️  警告:平移量 > 1m,物理上不合理 (車頂感測器間距應 < 0.5m),解不可信")
elif dt > 0.5:
    print("⚠️  注意:新平移與初始差距 > 0.5m,請以投影圖確認")
else:
    print("✅ 平移量物理合理")
print(f"\nT_new = np.array([")
for row in T_new:
    print("    ["+", ".join(f"{v:12.8f}" for v in row)+"],")
print("], dtype=np.float64)")
print("="*60)
np.save(os.path.join(OUT,"T_new_extrinsic.npy"), T_new)
