#!/bin/bash
DEFAULT_TIME=10
storage_base_path="./bag_record/$(date +"%Y-%m-%d")"
situation=""

if [ ! -d "$storage_base_path" ]; then mkdir -p "$storage_base_path"; fi

function record_data() {
    local duration=$1 record_situation=$2
    local storage_path="${storage_base_path}/${record_situation}"
    echo -e "\n[錄製] 場景=$record_situation  時長=${duration}s  $(date '+%H:%M:%S')"
    gnome-terminal --tab -t "record" -- bash -c "
        rosbag record \
            --duration=$duration -o $storage_path --buffsize=2048 \
            /can0/received_msg /can1/received_msg /can2/received_msg \
            /cme_cam/main/compressed /cme_cam/main/osd \
            /cme_cam/left/compressed /cme_cam/right/compressed \
            /cme_cam/rear/compressed /cme_cam/sideL/compressed /cme_cam/sideR/compressed \
            /imu /fix /velodyne_points ;
        echo 'rosbag 完成'"
    echo -e "Record 指令已送出！\n"
}

function change_situation() {
    local road_type weather time_period
    echo -e "+--------------------------------------------------------+"
    echo -e "|   situation|       0|          1|        2|           3|"
    echo -e "+--------------------------------------------------------+"
    echo -e "|   road_type| highway| citystreet| seashore| countryside|"
    echo -e "|     weather|   sunny|      rainy|         |            |"
    echo -e "| time_period|     day|      night|         |            |"
    echo -e "+--------------------------------------------------------+"
    while true; do
        read -rp "Enter road_type: " road_type
        case $road_type in
            0) road_type="highway";     break ;;
            1) road_type="citystreet";  break ;;
            2) road_type="seashore";    break ;;
            3) road_type="countryside"; break ;;
            *) echo "invalid" ;;
        esac
    done
    while true; do
        read -rp "Enter weather: " weather
        case $weather in
            0) weather="sunny"; break ;; 1) weather="rainy"; break ;; *) echo "invalid" ;;
        esac
    done
    while true; do
        read -rp "Enter time_period: " time_period
        case $time_period in
            0) time_period="day"; break ;; 1) time_period="night"; break ;; *) echo "invalid" ;;
        esac
    done
    situation="${road_type}_${weather}_${time_period}"
    echo -e "Setting situation as: $situation\n"
}

function check_time_sync() {
    echo -e "\n=== 時間同步狀態 ==="
    echo "系統時間 : $(date '+%Y-%m-%d %H:%M:%S')"
    if command -v chronyc &>/dev/null; then
        chronyc tracking | grep -E "System time|Stratum|Last offset"
    else
        echo "[警告] chrony 未安裝"
    fi

    echo ""
    echo "--- OSD latency ---"
    # 用 Python 直接訂閱一次，比 rostopic echo 更可靠
    timeout 5 python3 - << 'PYEOF'
import rospy, sys
from std_msgs.msg import Float64MultiArray

rospy.init_node("osd_check", anonymous=True, disable_signals=True)
got = [None]

def cb(msg):
    got[0] = msg.data

sub = rospy.Subscriber("/cme_cam/main/osd", Float64MultiArray, cb)
import time; t0 = time.time()
while got[0] is None and time.time()-t0 < 4.0:
    time.sleep(0.05)

if got[0] and len(got[0]) >= 7:
    d = got[0]
    print(f"  latency={d[3]:+.1f} ms  pitch={d[4]:+.2f}  roll={d[5]:+.2f}  yaw={d[6]:+.2f}")
else:
    print("  (OSD 節點未啟動或尚未收到資料)")
PYEOF
    echo ""
}

echo -e "╔══════════════════════════════════════════╗"
echo -e "║   G6 資料蒐集錄影腳本                    ║"
echo -e "║   r [秒] : 開始錄影 (預設 ${DEFAULT_TIME}s)          ║"
echo -e "║   t [秒] : 隧道場景                      ║"
echo -e "║   c      : 更改場景                      ║"
echo -e "║   s      : 時間同步狀態 / OSD latency    ║"
echo -e "║   q      : 離開                          ║"
echo -e "╚══════════════════════════════════════════╝"
echo ""
change_situation

while true; do
    read -rp "Enter option: " key
    option="${key:0:1}"; setTime="${key:2}"
    case $option in
        [rR]) record_data "${setTime:-$DEFAULT_TIME}" "$situation" ;;
        [tT]) record_data "${setTime:-$DEFAULT_TIME}" "tunnel" ;;
        [cC]) change_situation ;;
        [sS]) check_time_sync ;;
        [qQ]) echo -e "Exiting.\n"; exit 0 ;;
        *) echo "'$key' invalid. 使用 r/t/c/s/q" ;;
    esac
done
