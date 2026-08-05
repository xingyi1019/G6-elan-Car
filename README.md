# evil_elan_ccar — G6 實驗車資料蒐集與交接手冊

> 交接自 **陳新義**（NTUT 資工碩，指導教授陳彥霖）。本 repo 為 **G6 多模態實驗車車上端（car-side）資料蒐集流程**，涵蓋感測器啟動、時間同步、rosbag 錄製與離線解析。
>
> **目前交接狀態**：與前一代 G5 相同，核心產出為**感測器 rosbag**（光達 + IMU + 六路相機 + CAN + GPS）。舊有已解析資料未一併移交，請依本流程重新蒐集。完整方法論見碩論《異質感測器資料蒐集與時空校正框架：以台灣複雜交通場景為例》第三章。

---

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

> 感測器架每次出車重裝，**外參需逐日校正**（基準外參＋當日投影對齊人工微調，見碩論 3.5 節）。

---

## 二、開發環境與需安裝套件

- **作業系統**：Ubuntu 20.04　**ROS**：Noetic
- **兩個 catkin workspace**：`~/catkin_ws`（光達/IMU/CAN/相機）、`~/gps_ws`（GPS）

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
source ~/gps_ws/devel/setup.bash
```

### 需安裝的 ROS 套件（新機器接手第一步）

| 用途 | 套件 | 安裝 |
|---|---|---|
| 光達 VLS-128 | `velodyne`（velodyne_pointcloud） | `sudo apt install ros-noetic-velodyne` |
| IMU | `razor_imu_9dof` | 由原始碼 clone 進 `~/catkin_ws/src` → `catkin_make`（apt 無 noetic 版） |
| GPS | `nmea_navsat_driver` | `sudo apt install ros-noetic-nmea-navsat-driver` |
| CAN → topic | SocketCAN + `ros_launch/can2topic.launch` | `sudo apt install can-utils`；launch 檔在本 repo |
| 影像壓縮 | `cv_bridge`、`image_transport` | `sudo apt install ros-noetic-cv-bridge ros-noetic-image-transport` |

其他相依：Python3 `opencv-python numpy rosbag`、系統 `ffmpeg`（含 NVDEC 硬解 `h264_cuvid`）、`chrony`、`sshpass`、`can-utils`（`cansend`/`candump`）。

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

> **路徑提醒**：`ros_start.sh` 目前引用固定路徑 `~/Desktop/can2topic.launch`、`~/Desktop/VLS128.rviz`、`~/Desktop/Car/cme_cv2_*.py`。若你的資料夾位置不同，請把腳本內這幾行路徑改成實際位置（或把這些檔案放到 `~/Desktop`）。

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
| CAN／雷達 | SocketCAN（can2topic） | `/can0/received_msg`、`/can1/received_msg`、`/can2/received_msg` |
| 主相機 | cme_cv2_maincam.py | `/cme_cam/main/compressed`、**`/cme_cam/main/osd`** |
| 側方五路 | cme_cv2_sidecam.py | `/cme_cam/{left,right,rear,sideL,sideR}/compressed` |

> `/cme_cam/main/osd`（OSD 燒錄時戳）是時間對齊的關鍵，只有主相機有。

---

## 七、時間同步機制（重要）

RTSP 網路相機影像在被標記時戳前已延遲數百毫秒，故需三層同步（碩論 3.4 節）：

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

- **外參逐日校正**：每次出車感測器架重裝，需以棋盤格取基準外參、再依當日投影對齊人工微調（碩論 3.5 節）。
- **rosbag 很大**：單段原始約 6 GB，解析標準化後約 500 MB；錄前確認硬碟空間。
- **相機成像模型**：main／left／right／rear 用 Kannala–Brandt（廣角／魚眼）、sideL／sideR 用 Brown–Conrady（針孔）。
- **雷達目前只錄不解**：ARS408 raw CAN 已保留，結構化解碼與融合列為未來工作。
- **相依論文章節**：時間同步＝3.4；空間校正＝3.5；資料格式與品質篩選＝3.7。

---

*本手冊由陳新義整理交接。細節請對照碩論第三章。*
