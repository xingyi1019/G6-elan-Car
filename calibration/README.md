# G6 光達–相機 校正教學

這份教學帶你把 G6 實驗車的**相機內參**校好,再用**互動工具手動對齊外參**,
最後把光達點雲投影疊回影像、用眼睛確認對得準不準。

> 從零開始寫,不預設你看過這些程式。照著章節順序做就好。

---

## 先搞懂:校正到底在做什麼(白話)

把光達的點「畫到」相機照片上,需要兩組參數:

| 名稱 | 白話 | 內容 |
|---|---|---|
| **內參**(K, D) | 相機**鏡頭自己**的特性 | 焦距、光心、畸變(把魚眼那種彎的還原成直的) |
| **外參**(T) | 光達 ↔ 相機的**相對位置姿態** | 一個 4×4 矩陣:光達座標系 → 相機座標系 |

流程:
```
光達 3D 點 --(外參 T)--> 相機座標 --(內參 K,D)--> 影像上的像素 (u,v)
```
把每個點都這樣算出 (u,v) 畫上去,**疊得準 = 校正好**。
所以要先有**內參**,再調**外參**,最後**投影**檢查。

---

## 0. 資料格式(所有腳本都吃這個格式)

一個「場景資料夾」長這樣,**影像跟點雲用同一個編號對檔**:

```
<場景資料夾>/
├── images/
│   ├── main/     000000.png 000001.png ...   # 主相機(廣角)
│   ├── left/  right/  rear/                   # 環景魚眼三路
│   └── sideL/ sideR/                          # 側方針孔兩路
└── VLS128_pcd/   000000.pcd 000001.pcd ...    # VLS-128 光達
```
`images/main/000012.png` ↔ `VLS128_pcd/000012.pcd` 是同一個 frame。

---

## 1. 環境(先裝好)

```bash
pip install opencv-contrib-python numpy scipy matplotlib pillow PyQt5
```
- ⚠️ 一定要 **opencv-contrib-python**(不是 opencv-python),魚眼校正要用 `cv2.fisheye.*`。
- `PyQt5` 只有外參微調 GUI 要用。
- 投影出影片要另外裝 **ffmpeg**。

---

# 內參校正教學(魚眼 / 廣角相機)

主相機(廣角)、left/right/rear(魚眼)這四路,用下面的流程。
針孔的 sideL/sideR 見 [§2.5](#25-針孔相機-sidelsider)。

## 事前:拍棋盤格

拿一張**棋盤格標定板**,用要校正的那台相機,**從不同角度、距離、畫面不同位置**各拍幾十張,
存成一個資料夾(png)。棋盤要拍**完整、清楚、不晃**,盡量佈滿畫面各處(中間、四角都要有)。

> 「內角點」= 棋盤上黑白格**交界的內部交叉點**,不是格子數。
> 例如 10×7 格的板子,內角點是 **9×6**。

---

## 2.1　Step 1:確認棋盤尺寸 — `intrinsic/detect_grid.py`

先確認你的板子在程式眼裡是幾乘幾。

1. 打開 `intrinsic/detect_grid.py`,改**最上面這行**成你的照片資料夾:
   ```python
   IMG_DIR = r"E:\Car\calibration_data_rear\png"   # ← 改成你的棋盤照片資料夾
   ```
2. 跑:
   ```bash
   python intrinsic/detect_grid.py
   ```
3. 它會試各種尺寸,印出每種抓到幾張,例如:
   ```
     (9, 6): 42/50 張抓到
     ...
   ✅ 最佳尺寸: (9, 6)  (42/50 張)
   ```
   **記住這個最佳尺寸**(通常就是 `(9, 6)`),下一步要用。

---

## 2.2　Step 2:算內參 — `intrinsic/calib_fisheye_final.py`

1. 打開 `intrinsic/calib_fisheye_final.py`,改開頭兩個地方:
   ```python
   IMG_DIR = r"E:\Car\calibration_data_rear\png"   # ← 同一個照片資料夾
   CB = (9, 6)                                      # ← 改成 Step 1 的最佳尺寸
   ```
2. 跑:
   ```bash
   python intrinsic/calib_fisheye_final.py
   ```
3. 它會自動偵測棋盤、逐張剔掉誤差太大的爛圖,最後印出結果:
   ```
   ============================================================
     rear 魚眼內參 (用 38 張, RMS=0.42px)
   ============================================================
   # K_base @ 1280x1024
   K_base = [263.375786, 263.820321, 640.000000, 512.000000]

   # D [k1,k2,k3,k4]
   D = [0.42750887, -0.14506998, 0.02445243, -0.00135651]
   ```

**怎麼看結果:**
| 項目 | 意思 | 好壞判斷 |
|---|---|---|
| **RMS** | 重投影誤差(像素) | **越小越好**,`< 1px` 算很好,`> 2px` 要檢查照片品質 |
| **K_base** | `[fx, fy, cx, cy]` = 焦距 x/y、光心 x/y | — |
| **D** | 魚眼畸變係數 `[k1,k2,k3,k4]` | — |

> ⚠️ 檔案裡的 `K0`/`D0`(bootstrap 起點)是針對某台相機調的初始猜值。
> 換相機如果一直發散,把 `K0` 的 `cx,cy` 改成你影像的**一半**(影像寬/2、高/2)再試。

---

## 2.3　Step 3:驗證有沒有校對 — `intrinsic/verify_undistort.py`

用剛算的內參把一張棋盤圖**去畸變**,看原本彎的線有沒有被拉直。

1. 打開 `intrinsic/verify_undistort.py`,把 `K`、`D` 換成 Step 2 算出來的值,
   `img` 改成一張你的棋盤照片路徑。
2. 跑:
   ```bash
   python intrinsic/verify_undistort.py
   ```
3. 會存一張「**原圖 vs 去畸變**」並排圖。**去畸變那邊的直線(棋盤邊、牆角)有變直 = 內參 OK。**

---

## 2.4　Step 4:把內參填回 config

打開 `config/config_g6_6view.json`,找到對應相機(例 `'rear'`),把值填進去:

```python
'rear': {
    'type': 'fisheye',
    'scale': 1.0,
    'K': np.array([                       # ← 用 K_base 組成 3×3
        [263.375786,   0.0,        640.0],   #   [fx, 0,  cx]
        [  0.0,      263.820321,   512.0],   #   [0,  fy, cy]
        [  0.0,        0.0,          1.0],
    ], dtype=np.float32),
    'D': np.array([0.42750887, -0.14506998, 0.02445243, -0.00135651, 0.0], dtype=np.float32),
    'T': np.array([ ... ]),               # ← 外參,下一階段調
}
```
內參到這就好了。**T(外參)先不用管,下一階段用工具調。**

---

## 2.5　針孔相機 sideL / sideR

側方兩路是**針孔**,不用魚眼流程。用 OpenCV 標準的 `cv2.calibrateCamera`
(棋盤偵測 → calibrateCamera → 得到 `K` 3×3 與 `D` 5 個係數 `[k1,k2,p1,p2,k3]`)。
這裡沒附針孔腳本,網路上 OpenCV 官方 camera calibration 教學即是標準做法;
算完一樣把 `K`、`D` 填回 config 的 `'sideL'`/`'sideR'`(`'type': 'pinhole'`)。

---

# 外參校正

外參有兩條路,建議搭配用:

| 方式 | 工具 | 何時用 |
|---|---|---|
| **§3 正規標定** | MATLAB **Lidar Camera Calibrator** | 有棋盤板,第一次 / 重新標定,要一個準確起點 |
| **§4 手動微調** | `tuner/alignment_tool.py` | 出車後感測器架重裝、外參跑掉,肉眼對齊修正(日常) |

實務上:**先用 MATLAB 算一組準的外參 → 填進 config → 之後每次出車用 alignment_tool 微調。**

## 3. MATLAB Lidar Camera Calibrator(正規求外參)

MATLAB 有官方 App「**Lidar Camera Calibrator**」,用棋盤板自動算光達↔相機外參。

### 需要
- MATLAB + **Lidar Toolbox** + **Computer Vision Toolbox**
- 棋盤板(知道**格子邊長**,通常填 mm)
- paired 資料:一疊「同時拍到棋盤」的影像 + 對應點雲(`.pcd` / `.ply`)
- **相機內參**(前面內參階段算好的 K, D → 做成 MATLAB 的 `cameraIntrinsics` 物件)

### 步驟
1. **開 App** — MATLAB 指令列輸入:
   ```matlab
   lidarCameraCalibrator
   ```
   (或上方 **Apps** 頁籤找 Lidar Camera Calibrator)
2. **匯入資料** — 工具列 Import:
   - 選**影像資料夾** + **點雲資料夾**
   - 填**棋盤格邊長**(square size)
   - 載入**相機內參**(`cameraIntrinsics` 物件,從 workspace 選)
3. **偵測** — App 自動在影像偵測棋盤角點、在點雲偵測棋盤平面。
   點雲那邊若抓不到,調 **ROI / Cluster Threshold / 板子尺寸**,把棋盤那塊框出來。
4. **Calibrate** — 按下去,算出每個 pair 的外參並統整。
5. **看誤差** — App 顯示 reprojection / translation / rotation error 長條圖。
   **把誤差特別大的 pair 取消勾選 → 重按 Calibrate**,直到誤差穩定。
6. **匯出** — Export:匯出到 workspace(得到 `tform`,光達→相機的剛體變換),
   或 Export → Generate MATLAB Script。

### 把 MATLAB 的 `tform` 轉成我們 config 的 `T`
- **新版 `rigidtform3d`**:直接用 `tform.A` —— 那就是 4×4 的 `[R t; 0 0 0 1]`(光達→相機),
  貼進 config 對應相機的 `'T'`。
- ⚠️ **舊版 `rigid3d`**:`tform.T` 是**列向量(post-multiply)慣例**,要**轉置**才對:
  ```matlab
  R = tform.T(1:3,1:3)';   t = tform.T(4,1:3)';
  T = [R t; 0 0 0 1];      % 這才是我們要的 光達→相機
  ```
- 填完**一定要用「投影驗證」看對不對**;沒對齊最常見就是**轉置**或**相機軸慣例**差異。

### ⚠️ 魚眼 / 廣角相機的注意
這個 App 的棋盤偵測/重投影是走**針孔內參**(`cameraIntrinsics`)。
主相機(廣角)、環景魚眼**不能直接**丟原始魚眼影像:
- 做法:先用前面內參把影像**去畸變成針孔影像**,再用**去畸變後的針孔內參**餵給 App;
  算出來的外參 T 照樣能用(**外參跟畸變無關**)。
- **sideL / sideR 本來就是針孔**,可以直接用。

> 各版本 MATLAB 介面細節略有不同,以 App 內提示為準。
> 不想開 App、想寫腳本批次算,對應函式是 `estimateLidarCameraTransform`。

## 4. `tuner/alignment_tool.py` 手動微調(日常 / 沒板子時)

出車後感測器架每次重裝,外參都會跑掉。**開這支 GUI,一邊看投影一邊拉 slider 對齊**,對好按 Save 存回 config。

**開啟:**
```bash
python tuner/alignment_tool.py
```

**畫面:**
- **左邊**:光達點雲即時投影疊在相機影像上(滾輪縮放、左鍵拖移、雙擊還原)
- **右邊 slider**:調**外參** tx/ty/tz(公尺)、roll/pitch/yaw(度);也能微調內參/畸變
- **上方**:選 Data Root、切場景 session、切相機、前後翻 frame

**操作:**
1. 上方 **Data Root** 選你的 data 根目錄(預設找 `alignment_tool.py` 同層的 `data/`)。
2. 選 **Camera**,它會自動載入 config 裡該相機的 K/D/T(建議先用 §3 MATLAB 算好的當起點)。
3. 拉 **Extrinsic** slider(先調 yaw/pitch/roll 對角度,再調 tx/ty/tz 對位置),
   讓點雲輪廓**貼齊**影像上物體邊緣(電線桿、車、號誌)。
4. 對好按 **Save Config** → 寫回 `config_g6_6view.json`;按錯按 **還原參數** 撤銷。

## 5. Python 自動外參腳本(用棋盤板算)

`extrinsic/` 內有兩支,都是 **target-based**(要有棋盤板 + 對應點雲):

| 腳本 | 方法 | 說明 |
|---|---|---|
| **`extrinsic_calibrate.py`** ★ | **平面對平面**(Zhang–Pless 風格) | 相機端 `solvePnP` 得板平面、光達端 RANSAC 得同一塊板平面 → **Kabsch 解 R、點到平面最小平方解 t**。會同時印出「初始外參 vs 新算外參」的平面對齊殘差(cm),可直接判斷有沒有變好。 |
| `auto_extrinsic.py` | 板平面 + PnP | 點雲 RANSAC 找棋盤平面 → 在平面上重建 3D 角點 → `solvePnP`;多張取中位數。內參為硬寫,需自行確認。 |

**用法(`extrinsic_calibrate.py`):**
```bash
python extrinsic/extrinsic_calibrate.py \
    --img-dir <棋盤影像資料夾> \
    --pcd-dir <對應點雲資料夾> \
    --ext png --square 0.10 \
    --out ./ext_result
```
影像與點雲**檔名要相同**(如 `001.png` ↔ `001.pcd`)。輸出 `T_new_extrinsic.npy`,
並印出 4×4 的 `T_new`,確認殘差有下降後再貼進 `config_g6_6view.json` 的 `'T'`。

> 實測參考:以 23 組棋盤配對執行,平面對齊殘差由 **30.25 cm → 3.21 cm**。
> 板子姿態要夠多樣(程式會檢查法向量分散度),否則解不可靠。

---

# 投影驗證(確認校正對不對)

內參 + 外參都填進 config 後,把整段場景投影出來、串成影片,肉眼確認。

## 5.1　六路(或指定幾路)批次投影 — `projection/batch_project.py`
```bash
python projection/batch_project.py \
    --data-root <data 根目錄> \
    --config    config/config_g6_6view.json \
    --scenario  <場景資料夾名稱> \
    --cams      main,left,right \
    --fps 10
```
每台相機會輸出疊合圖 + 一支 mp4 影片。

## 5.2　只跑主相機 — `projection/main_only_projector.py`
```bash
python projection/main_only_projector.py \
    --scenario-dir <場景資料夾> \
    --config config/config_g6_6view.json \
    --out    ./main_projected.mp4
```

**看影片:光達點雲的輪廓貼齊影像上的物體 = 校正成功。**

---

## 附錄:`config/` 說明

同一組 K/D/T,兩種格式並存:

| 檔案 | 格式 | 誰在用 |
|---|---|---|
| `config_g6_6view.json` | **Python dict**(用 `exec` 讀,值是 `np.array`)| `alignment_tool`、`batch_project`、`main_only_projector` |
| `g6_calibration.json` | **標準 JSON**(純數字)| 一般程式讀取用 |

每台相機欄位:
- `type`:`fisheye`(main/left/right/rear)或 `pinhole`(sideL/sideR)
- `K`:內參矩陣;`scale`:投影時縮放(通常 1.0)
- `D`:畸變(魚眼 4 個 k1~k4;針孔 5 個 k1,k2,p1,p2,k3)
- `T`:4×4 外參,**光達 → 相機**

> ⚠️ 主相機雖是**廣角鏡頭**,數學上仍用 `cv2.fisheye`(Kannala-Brandt)模型投影——這是投影模型的選擇,不代表硬體是魚眼。

---

## 一頁流程總覽

```
【內參】每台相機一次就好
  1. detect_grid.py         確認棋盤尺寸
  2. calib_fisheye_final.py 算 K, D(看 RMS)
  3. verify_undistort.py    去畸變驗證
  4. 填回 config

【外參】
  5a.(正規)MATLAB Lidar Camera Calibrator  用棋盤板算外參 → 填 config
  5b.(日常)tuner/alignment_tool.py  每次出車開 GUI 手動微調 → Save
     (進階:extrinsic/ Python 自動腳本  🚧 待補)

【驗證】
  6. batch_project.py        投影疊圖串影片,肉眼確認
```
