# G6 實驗車資料蒐集與交接手冊

## 目錄

1. [硬體配置](#一硬體配置)
2. [開發環境與需安裝套件](#二開發環境與需安裝套件)
3. [目錄結構](#三目錄結構)
4. [出車 SOP](#四出車-sop每次照順序做)
5. [各腳本做了什麼](#五各腳本做了什麼)
6. [Topic 對照表](#六topic-對照表)
7. [時間同步機制](#七時間同步機制重要)
8. [資料解析（rosbag → paired）](#八資料解析rosbag--paired)
9. [疑難排解](#九疑難排解)
10. [交接注意事項](#十交接注意事項)

---

## 一、硬體配置

| 感測器 | 型號 / 規格 | 網段 / 介面 |
|---|---|---|
| 光達 | Velodyne **VLS-128**（128 線、約 10 Hz） | `192.168.1.201`（乙太網） |
| 主相機 | 4K 前視（約 120° 廣角，運行降配 2560×1440） | RTSP `192.168.100.26` |
| 側方五路相機 | 三路環景魚眼 ＋ 兩路後側方針孔（1920×1536） | RTSP `192.168.100.2 ~ .6` |
| 雷達 | Continental **ARS408**（僅錄原始 CAN） | Kvaser CAN `can0` |
| IMU | 9-DoF（razor 系列，約 50 Hz） | `/dev/ttyACM0` |
| GPS | NMEA USB（約 10 Hz） | `/dev/ttyUSB0` |
| CAN 匯流排 | Kvaser 4×HS，bitrate 500000 | `can0`/`can1`/`can2` |

> 感測器架每次出車重裝，**外參需逐日校正**（以基準外參為起點，再依當日投影對齊人工微調）。

---

## 二、開發環境與需安裝套件

### 系統

| 項目 | 版本 | 下載 / 安裝說明 |
|---|---|---|
| 作業系統 | Ubuntu 20.04 (Focal) | https://releases.ubuntu.com/focal/ |
| ROS | Noetic | https://wiki.ros.org/noetic/Installation/Ubuntu |

### 兩個 catkin workspace

`~/catkin_ws`（光達 / IMU / CAN / 相機）、`~/gps_ws`（GPS）

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
source ~/gps_ws/devel/setup.bash
```

### 需安裝的 ROS 套件（新機器接手第一步）

| 用途 | 套件 | 安裝 / 下載位置 |
|---|---|---|
| 光達 VLS-128 | `velodyne`（velodyne_pointcloud） | `sudo apt install ros-noetic-velodyne`　或原始碼：https://github.com/ros-drivers/velodyne |
| IMU | `razor_imu_9dof` | clone 進 `~/catkin_ws/src` → `catkin_make`：https://github.com/ENSTABretagneRobotics/razor_imu_9dof （indigo-devel 分支，本車用 `razor-pub.launch`） |
| GPS | `nmea_navsat_driver` | `sudo apt install ros-noetic-nmea-navsat-driver`　或原始碼：https://github.com/ros-drivers/nmea_navsat_driver |
| CAN → topic | `socketcan_bridge` | `sudo apt install ros-noetic-socketcan-bridge`（`ros_launch/can2topic.launch` 用此節點；launch 檔在本 repo） |
| 影像壓縮 | `cv_bridge`、`image_transport` | `sudo apt install ros-noetic-cv-bridge ros-noetic-image-transport` |

其他相依：Python3 `opencv-python numpy rosbag`、系統 `ffmpeg`（含 NVDEC 硬解 `h264_cuvid`）、`chrony`、`sshpass`、`can-utils`（`cansend`／`candump`，驗證 CAN 用）。

---

## 三、目錄結構

```
evil_elan_ccar/
├── README.md              ← 本手冊
├── shell/                 ← 出車用啟動與錄製腳本
│   ├── ros_start.sh       ← 啟動 CAN/IMU/光達/相機/GPS 所有節點
│   ├── setup_radar.sh     ← 單獨啟動 Kvaser CAN（雷達）
│   ├── sync_camera_time.sh← 相機/光達時間同步（chrony + ntpd + NTP CGI）
│   └── record_time.sh     ← 設情境並錄製 rosbag
├── camera/                ← 相機蒐集節點
│   ├── cme_cv2_rtsp6cam.py← 六路相機整合蒐集（主要流程，含 OSD topic）
│   ├── cme_cv2_rtsp1cam.py← 單路相機蒐集（主要流程）
│   ├── cme_cv2_maincam.py ← 主相機（測試版本）
│   └── cme_cv2_sidecam.py ← 側方相機（測試版本）
├── parser/                ← 離線解析（rosbag → paired 資料）
│   └── g6sensorparserV9.py   ← OSD 純時戳配對版（現行）
├── ros_launch/can2topic.launch  ← CAN 轉 ROS topic
└── RViz/VLS128.rviz       ← 光達顯示設定
```

> **⚠️ 部署位置很重要（腳本用寫死的絕對路徑）**
>
> 本 repo 只是整齊的備份；`ros_start.sh` 內部固定呼叫 `~/Desktop/...` 路徑，與本 repo 的資料夾分類**不一樣**。實際跑車時，請把檔案放到下表位置，腳本才找得到（`~` = 家目錄）：
>
> | repo 內位置 | 車上要放到 |
> |---|---|
> | `shell/*.sh`、`camera/cme_cv2_*.py` | **`~/Desktop/Car/`**（在此執行 `./ros_start.sh`） |
> | `ros_launch/can2topic.launch` | **`~/Desktop/`**（注意：**不是** Car/） |
> | `RViz/VLS128.rviz` | **`~/Desktop/`**（注意：**不是** Car/） |
> | `parser/g6sensorparserV9.py` | 放與 `.bag` 同一資料夾即可 |
>
> 若不想遷就這些固定路徑，可把 `ros_start.sh` 內 4 處 `~/Desktop/...`（第 16、21、22、24 行）改成你的實際位置。

### 部署步驟（第一次在筆電/車機設定，仿學長流程）

```bash
# 1. 建立工作區資料夾
mkdir -p ~/Desktop/Car

# 2. clone 本 repo
git clone https://github.com/xingyi1019/G6-elan-Car.git

# 3. 安裝 ROS 套件（見第二節），放進對應 workspace 後 catkin_make
#    velodyne / razor_imu_9dof / socketcan_bridge → ~/catkin_ws/src
#    nmea_navsat_driver                            → ~/gps_ws/src

# 4. 依上表放檔案：
#    ~/Desktop/       ← can2topic.launch、VLS128.rviz
#    ~/Desktop/Car/   ← shell/*.sh、camera/cme_cv2_*.py
```

**完整檔案位置一覽**

```
~/catkin_ws/src/     ← velodyne、razor_imu_9dof、socketcan_bridge（第二節裝的 ROS 套件）
~/gps_ws/src/        ← nmea_navsat_driver
~/Desktop/Car/       ← 本 repo 的 shell/*.sh、camera/cme_cv2_*.py（在此執行 ./ros_start.sh）
~/Desktop/           ← can2topic.launch、VLS128.rviz
```

> ROS 套件（velodyne、razor、socketcan_bridge、nmea）本身**不放進 repo**，依第二節安裝即可——repo 只存自製腳本（與學長交接方式一致）。

---

## 四、出車 SOP（每次照順序做）

> ⚠️ 順序不能跳。時間同步沒做好，光達與相機會對不齊。

1. **接線開電**：VLS-128 網路、六路相機網段（`192.168.100.x`）、Kvaser、IMU（`ttyACM0`）、GPS（`ttyUSB0`）都接上。
2. **啟動雷達 CAN**：
   ```bash
   ./shell/setup_radar.sh
   ```
3. **啟動所有 ROS 節點**：
   ```bash
   ./shell/ros_start.sh
   ```
   啟動後**逐一檢查每個 gnome-terminal tab 沒有紅字**（roscore / can / imu / main_cam / side_cams / VLS128 / gps）。
4. **時間同步**（關鍵）：
   ```bash
   ./shell/sync_camera_time.sh
   ```
   確認：Chrony Stratum 正常、5 路側相機「✅ 已同步」、主相機「✅ ntpd 已啟動」。
5. **錄製**：
   ```bash
   ./shell/record_time.sh
   ```
   先設情境，再按鍵：`r [秒]` 錄製（預設 10s）、`t [秒]` 隧道場景、`c` 改情境、`s` 查時間同步/OSD latency、`q` 離開。輸出至 `bag_record/<日期>/<情境>/`。

---

## 五、各腳本做了什麼

### `shell/setup_radar.sh`
重置 `can0` → 設 bitrate 500000 → up → `cansend can0 200#1800000010040000` 送出 ARS408 啟動封包。之後可用 `candump can0` 驗證有無封包。

### `shell/ros_start.sh`
一次拉起整台車：開放 IMU/GPS 序列埠權限 → `pkexec` 設 `can0/1/2` → `roscore` → `can2topic.launch` → `razor_imu_9dof` → `cme_cv2_maincam.py`、`cme_cv2_sidecam.py` → `velodyne_pointcloud VLS128_points.launch` ＋ `rviz` → `nmea_navsat_driver`（GPS）。

### `shell/sync_camera_time.sh`
三層對時：① 本機 Chrony；② VLS-128 透過 CGI 設 NTP server；③ 5 路側相機 SSH `date -s`；④ 主相機 telnet 偵測並啟動 `ntpd -p <本機IP>` 常駐；⑤ 驗證 ntpd。相機網段 `192.168.100.x`、光達網段 `192.168.1.x`。

### `shell/record_time.sh`
互動式錄製選單，情境命名 `road_type_weather_time_period`（如 `citystreet_sunny_day`），`rosbag record --duration=N --buffsize=2048` 訂閱全部 topic。

---

## 六、Topic 對照表

| 感測器 | Driver | Topic |
|---|---|---|
| 光達 | velodyne_pointcloud | `/velodyne_points` |
| IMU | razor_imu_9dof | `/imu` |
| GPS | nmea_navsat_driver | `/fix` |
| CAN／雷達 | socketcan_bridge（can2topic） | `/can0/received_msg`、`/can1/received_msg`、`/can2/received_msg` |
| 主相機 | cme_cv2_maincam.py | `/cme_cam/main/compressed`、**`/cme_cam/main/osd`** |
| 側方五路 | cme_cv2_sidecam.py | `/cme_cam/{left,right,rear,sideL,sideR}/compressed` |

> `/cme_cam/main/osd`（OSD 燒錄時戳）是時間對齊的關鍵，只有主相機有。

---

## 七、時間同步機制（重要）

RTSP 網路相機影像在被標記時戳前已延遲數百毫秒，故需三層同步：

1. **Chrony**：主機對時，同時作為內網主時鐘。`chrony.conf` 須有 `allow 192.168.100.0/24`、`allow 192.168.1.0/24`，並設 `local stratum` 作斷網備援。查：`chronyc tracking`。
2. **相機/光達對時**：光達走 CGI 設 NTP、側相機走 SSH、主相機走 telnet `ntpd` 常駐（`sync_camera_time.sh` 全包）。
3. **主相機 OSD 時戳**：畫面右上角燒錄系統時戳，解碼回讀量測 RTSP 後段延遲（約 300 ms），據以做常數幀位移補償。可用 `record_time.sh` 的 `s` 選項即時查看 OSD latency。

> 解析時以光達為主時鐘，**光達與各相機時差須 ≤ 0.033 s（1 幀）才允許配對**。

---

## 八、資料解析（rosbag → paired）

```bash
# 放在與 .bag 同一資料夾內執行
python3 parser/g6sensorparserV9.py
```

依 bag 內各 topic 時戳對齊後輸出 **paired 格式**（每個光達幀配好對應影像）：

```
<session>/
├── images/{main,left,right,rear,sideL,sideR}/{i:06d}.png
├── VLS128_pcd/{i:06d}.pcd
├── imu/  gps/  can/
└── timestamps/
```

> G6 解析採「OSD 純時戳配對」：**移除人工平移湊數邏輯**，改用帶精準 OSD 曝光時間的影像時間軸，嚴格 ≤ 33 ms 物理時差限制。

---

## 九、疑難排解

| 症狀 | 檢查 |
|---|---|
| 光達與相機明顯對不齊 | 八成是**時間同步沒做好**（chrony offset、相機 ntpd） |
| latency 一直往上飄 | FFmpeg 緩衝累積；確認相機節點每次只取最新幀（drain-to-latest） |
| `chronyc tracking` Stratum=16 | 沒對到時間，檢查網路與 `chrony.conf` 的 allow 網段 |
| 側相機沒同步 / SSH 失敗 | 確認相機網段連得到、`sync_camera_time.sh` 內設定已填妥 |
| 某相機 tab 報錯 | 檢查該相機網段連線、RTSP URL、NVDEC 硬解路數上限 |
| CAN 收不到雷達 | 重跑 `setup_radar.sh`；`candump can0` 確認有封包 |
| IMU/GPS 權限錯誤 | 確認 `ttyACM0`／`ttyUSB0` 已開放讀寫權限 |
| 主相機 ntpd 啟動失敗 | 相機防火牆擋 UDP 123，或 Ubuntu chrony 未 `allow 192.168.100.0/24` |

---

## 十、交接注意事項

- **外參逐日校正**：每次出車感測器架重裝，需以棋盤格取基準外參、再依當日投影對齊人工微調。
- **rosbag 很大**：單段原始約 6 GB，解析標準化後約 500 MB；錄前確認硬碟空間。
- **相機成像模型**：main／left／right／rear 用 Kannala–Brandt（廣角／魚眼）、sideL／sideR 用 Brown–Conrady（針孔）。
- **雷達目前只錄不解**：ARS408 raw CAN 已保留，結構化解碼與融合列為未來工作。
- **密碼**：`shell/ros_start.sh`、`shell/sync_camera_time.sh` 內標示「填上你的密碼」處，請填入實際密碼（另行取得）。
