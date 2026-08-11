# G6 光達–相機 校正(內參 / 外參 / 投影驗證)

這個資料夾是 G6 實驗車「光達 ↔ 相機」時空校正的核心程式。
輸入是**已配對好的影像 + 點雲**,輸出是**每台相機的內參(K, D)與外參(T)**,
最後可以把光達點雲**投影疊回影像**用眼睛驗證對得準不準。

> 只放校正核心,不含論文畫圖 / 統計實驗腳本。

---

## 0. 資料格式(重要)

所有腳本都假設「一個場景資料夾」長這樣(paired 格式,檔名對檔名):

```
<場景資料夾>/
├── images/
│   ├── main/     000000.png 000001.png ...   # 主相機(廣角)
│   ├── left/     000000.png ...              # 環景魚眼 左
│   ├── right/    ...                          # 環景魚眼 右
│   ├── rear/     ...                          # 環景魚眼 後
│   ├── sideL/    ...                          # 側方針孔 左
│   └── sideR/    ...                          # 側方針孔 右
└── VLS128_pcd/   000000.pcd 000001.pcd ...    # VLS-128 光達
```

**規則:同一個 frame 的影像跟點雲用同一個編號**(`images/main/000012.png` ↔ `VLS128_pcd/000012.pcd`)。
`read_pcd()` 同時支援 ascii 與 binary 的 .pcd。

---

## 1. 環境

```bash
pip install opencv-contrib-python numpy scipy matplotlib pillow
# 投影影片需要 ffmpeg(batch_project.py 用)
```

⚠️ 一定要 **opencv-contrib-python**(不是純 opencv-python),因為魚眼校正用到 `cv2.fisheye.*`。

---

## 2. 三階段流程

```
(1) 內參 intrinsic/   →  拍棋盤格 → 算每台相機的 K, D
(2) 外參 extrinsic/   →  光達+相機同時看棋盤/場景 → 算 T(光達→相機)
(3) 投影 projection/  →  把點雲投影疊回影像 → 眼睛驗證
```

把算出來的 K / D / T 填進 `config/`,投影腳本就會讀來用。

---

## 3. 內參 `intrinsic/`

| 腳本 | 做什麼 |
|---|---|
| `detect_grid.py` | 前置診斷:試各種棋盤內角點尺寸,回報每種抓到幾張。**先跑這個確認棋盤尺寸。** |
| `calib_fisheye_final.py` | 魚眼/廣角內參校正(最終版):bootstrap 偵測 + 逐張剔高誤差,盡量保留最多張。輸出 K_base、D、RMS。 |
| `verify_undistort.py` | 拿新內參對一張圖去畸變,並排「原圖 vs 去畸變」看直線有沒有拉直。 |

**用法**(這三支的資料夾路徑是**寫死在檔案開頭**,要自己改):
```python
# 打開檔案,改最上面這行成你的棋盤照片資料夾
IMG_DIR = r"E:\Car\calibration_data_rear\png"
```
```bash
python intrinsic/detect_grid.py            # 先確認棋盤尺寸 (預設 9x6)
python intrinsic/calib_fisheye_final.py    # 算內參,終端會印出 K_base / D / RMS
python intrinsic/verify_undistort.py       # 視覺驗證
```
> 針孔相機(sideL/sideR)的內參用 OpenCV 標準 `cv2.calibrateCamera` 流程即可;
> 這裡附的是比較麻煩的魚眼/廣角版本(`cv2.fisheye`)。

---

## 4. 外參 `extrinsic/`

| 腳本 | 方法 | 何時用 |
|---|---|---|
| `auto_extrinsic.py` | **有棋盤板**:自動掃含棋盤的 frame → 影像端偵測角點、點雲端 RANSAC 找棋盤平面 → `solvePnP` 解 T | 有拿棋盤板對著車拍的資料 |
| `targetless_refine.py` | **無棋盤板**:影像 Canny 邊緣做距離變換,點雲取深度不連續點,投影後做邊緣相關,`scipy.optimize` 微調 T | 已有初始 T,想用自然場景邊緣再精修 |
| `extrinsic_check.py` | 拿內參 + 外參把校正板資料投影疊圖,肉眼看對齊 | 驗證外參 |

**用法**:
```bash
# 有棋盤板 → 自動解外參(可多場景一起)
python extrinsic/auto_extrinsic.py \
    --scenarios <場景1> <場景2> \
    --cams sideL sideR left right \
    --out-dir ./ext_results
# 產出 extrinsic_summary.json,內含每台相機的 best_T / median_T / RMS

# 無棋盤板 → 從 config 初始 T 邊緣相關精修
python extrinsic/targetless_refine.py \
    --scenarios <場景1> <場景2> \
    --cam sideL \
    --out refined_sideL.json
```
> `auto_extrinsic.py` 內參是**寫死**的(BSD 針孔 fx≈2023、魚眼 K_base),要改就改檔案上方常數。
> `extrinsic_check.py` 的 `IMG_DIR/PCD_DIR/K/T` 也寫死在開頭,自己改。
> 棋盤格參數在 `auto_extrinsic.py` 開頭:`CHECKERBOARD=(9,6)`、`SQUARE_SIZE_M=0.108`(每格 10.8cm),依你的板子改。

---

## 5. 投影驗證 `projection/`

把點雲投影回影像、依深度上色、串成影片,用來**眼見為憑**確認校正對不對。

| 腳本 | 做什麼 |
|---|---|
| `batch_project.py` | 通用六路投影:讀 `config/config_g6_6view.json` → 逐 frame 疊圖 → ffmpeg 串 mp4。**主力工具。** |
| `main_only_projector.py` | 只跑主相機(原解析度),直接輸出 mp4。 |

**用法**(這兩支路徑是 **argparse 參數**,不用改檔案):
```bash
# 六路(或指定某幾路)投影 + 出影片
python projection/batch_project.py \
    --data-root <data 根目錄> \
    --config    config/config_g6_6view.json \
    --scenario  <場景資料夾名稱> \
    --cams      main,left,right \
    --fps 10

# 只跑主相機
python projection/main_only_projector.py \
    --scenario-dir <場景資料夾> \
    --config config/config_g6_6view.json \
    --out    ./main_projected.mp4
```

---

## 6. `config/` 校正參數

同一組 K / D / T,兩種格式並存:

| 檔案 | 格式 | 誰在用 |
|---|---|---|
| `config_g6_6view.json` | **Python dict**(用 `exec` 讀,裡面是 `np.array(...)`)| `batch_project.py`、`main_only_projector.py`、`targetless_refine.py` |
| `g6_calibration.json` | **標準 JSON**(純數字陣列)| 一般程式 / 其他工具讀取用 |

每台相機欄位:
- `type` / `model`:`fisheye`(main/left/right/rear)或 `pinhole`(sideL/sideR)
- `K`:內參矩陣(對應該解析度);`scale`:投影時的縮放
- `D`:畸變係數(魚眼 4 個 k1~k4;針孔 5 個 k1,k2,p1,p2,k3)
- `T`:4×4 外參,**光達座標 → 相機座標**

> ⚠️ 主相機雖然是**廣角鏡頭**,但數學上用 `cv2.fisheye`(Kannala-Brandt)模型投影 —— 這是投影模型的選擇,不代表硬體是魚眼。

---

## 7. 一次跑完的順序(新相機從零校正)

```
1. detect_grid.py           確認棋盤尺寸
2. calib_fisheye_final.py   算內參 K, D           → 填進 config
3. verify_undistort.py      確認去畸變正常
4. auto_extrinsic.py        算外參 T              → 填進 config
5. batch_project.py         投影疊圖,肉眼驗證對齊
6.（可選）targetless_refine.py  用場景邊緣再精修 T
```
