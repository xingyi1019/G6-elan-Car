"""偵測棋盤格內角點尺寸 — 試多種尺寸,回報每種抓到幾張。"""
import cv2, glob, os, sys
sys.stdout.reconfigure(encoding='utf-8')

IMG_DIR = r"E:\Car\calibration_data_rear\png"
imgs = sorted(glob.glob(os.path.join(IMG_DIR, "*.png")))
print(f"共 {len(imgs)} 張影像\n")

# 常見內角點尺寸 (cols, rows)
candidates = [(9, 6), (8, 6), (7, 6), (8, 5), (7, 5), (6, 5),
              (10, 7), (9, 7), (11, 8), (6, 9), (5, 8), (6, 8)]

flags = (cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
         + cv2.CALIB_CB_FAST_CHECK)

results = {}
for size in candidates:
    hit = 0
    for ip in imgs:
        img = cv2.imread(ip)
        if img is None:
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ok, _ = cv2.findChessboardCorners(gray, size, flags)
        if ok:
            hit += 1
    results[size] = hit
    print(f"  {size}: {hit}/{len(imgs)} 張抓到")

best = max(results, key=results.get)
print(f"\n✅ 最佳尺寸: {best}  ({results[best]}/{len(imgs)} 張)")
