"""用新主相機內參 + config 外參,投影 EXTRINSIC 校正板資料,看對齊效果。"""
import cv2, numpy as np, glob, os, sys
sys.stdout.reconfigure(encoding='utf-8')

IMG_DIR = r"F:\dumped_data_merged\EXTRINSIC\image"
PCD_DIR = r"F:\dumped_data_merged\EXTRINSIC\lidar"
OUT = r"C:\Users\xingy\OneDrive\Desktop\pcd_aligment\output"

# 新校正的主相機內參 @ 2560x1440
K_2k5 = np.array([[1603.82, 0, 1288.68], [0, 1606.68, 737.29], [0, 0, 1]], np.float64)
D = np.array([-0.33734, 0.14133, -0.04107, 0.00647], np.float64).reshape(4, 1)

# config 主相機外參 T (LiDAR -> Camera)
T = np.array([[-0.00363134, -0.99994677, 0.00965866, 0.05],
              [-0.12839811, -0.00911254, -0.99168092, -0.15],
              [ 0.99171609, -0.00484128, -0.12835820, -0.0245],
              [0, 0, 0, 1]], np.float64)

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
            return d[:,[xi,yi,zi]].astype(np.float32)
        tmap={'F':'f','I':'i','U':'u'}
        ddt=np.dtype([(fn,tmap.get(t,'f')+str(sz)) for fn,sz,t in zip(fields,sizes,types)])
        f.seek(off); d=np.frombuffer(f.read(num*ddt.itemsize),dtype=ddt)
        return np.column_stack([d['x'],d['y'],d['z']]).astype(np.float32)

def project(img, pts, K, D, T):
    h,w = img.shape[:2]
    R,t = T[:3,:3], T[:3,3]
    pc = (R@pts.T).T + t
    m = (pc[:,2]>0.5)&(pc[:,2]<60); pc=pc[m]
    if len(pc)==0: return img
    ip,_ = cv2.fisheye.projectPoints(pc.reshape(-1,1,3).astype(np.float64),
        np.zeros((3,1)),np.zeros((3,1)),K,D)
    u,v = ip[:,0,0],ip[:,0,1]
    dist=np.linalg.norm(pc,axis=1)
    vm=(u>=0)&(u<w)&(v>=0)&(v<h)&np.isfinite(u)&np.isfinite(v)
    u,v,dist=u[vm].astype(int),v[vm].astype(int),dist[vm]
    out=img.copy()
    col=cv2.applyColorMap((np.clip(dist/40,0,1)*255).astype(np.uint8),cv2.COLORMAP_JET)
    for i in range(len(u)):
        cv2.circle(out,(u[i],v[i]),3,col[i][0].tolist(),-1)
    return out

# 找前 3 個有配對的 frame
pcds = sorted(glob.glob(os.path.join(PCD_DIR,'*.pcd')))[:60]
done=0
for pp in pcds:
    stem = os.path.splitext(os.path.basename(pp))[0]
    ip = os.path.join(IMG_DIR, f'{stem}.jpg')
    if not os.path.exists(ip): continue
    img = cv2.imread(ip); h,w = img.shape[:2]
    # K 縮放到實際解析度 (校正在 2560x1440, 影像是 wxh)
    sx, sy = w/2560.0, h/1440.0
    K = K_2k5.copy(); K[0,0]*=sx; K[0,2]*=sx; K[1,1]*=sy; K[1,2]*=sy
    pts = read_pcd(pp)
    res = project(img, pts, K, D, T)
    res_small = cv2.resize(res, (1280, 720))
    outp = os.path.join(OUT, f'extrinsic_check_{stem}.png')
    cv2.imwrite(outp, res_small)
    print(f"✅ {stem}: {len(pts)} pts -> {outp}")
    done += 1
    if done >= 3: break
print(f"完成 {done} 張")
