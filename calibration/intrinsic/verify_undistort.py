"""用新內參對棋盤格去畸變,看直線是否拉直 (視覺驗證)。"""
import cv2, numpy as np, sys
sys.stdout.reconfigure(encoding='utf-8')

# 新校正參數 @ 1280x1024
K = np.array([[282.007287, 0, 644.901365],
              [0, 281.199695, 508.545263],
              [0, 0, 1]], dtype=np.float64)
D = np.array([0.35706549, -0.05262878, -0.10464131, 0.05832310], dtype=np.float64)

img = cv2.imread(r"E:\Car\calibration_data_rear\png\001.png")
h, w = img.shape[:2]

# 去畸變 (魚眼)
new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
    K, D, (w, h), np.eye(3), balance=0.0)
map1, map2 = cv2.fisheye.initUndistortRectifyMap(
    K, D, np.eye(3), new_K, (w, h), cv2.CV_16SC2)
undist = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR)

# 並排原圖 + 去畸變
combo = np.hstack([cv2.resize(img, (640, 512)), cv2.resize(undist, (640, 512))])
cv2.putText(combo, "ORIGINAL (fisheye)", (20, 40), 1, 1.5, (0, 255, 0), 2)
cv2.putText(combo, "UNDISTORTED", (660, 40), 1, 1.5, (0, 255, 255), 2)
out = r"C:\Users\xingy\OneDrive\Desktop\pcd_aligment\output\undistort_check.png"
cv2.imwrite(out, combo)
print(f"✅ 存: {out}")
