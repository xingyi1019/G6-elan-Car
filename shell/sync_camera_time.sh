#!/bin/bash
# sync_camera_time.sh
# 一勞永逸版：先偵測相機有什麼時間工具，再用合適的方式啟動「常駐同步」

# ── 自動偵測本機 IP（不用手寫死）────────────────────────────────────────
HOST_CAM_IP=$(ip -4 -o addr show | awk '/192\.168\.100\./ {print $4; exit}' | cut -d/ -f1)
HOST_LIDAR_IP=$(ip -4 -o addr show | awk '/192\.168\.1\./ {print $4; exit}' | cut -d/ -f1)
LIDAR_IP="192.168.1.201"
MAIN_CAM_IP="192.168.100.26"
SSH_CAMS=("192.168.100.2" "192.168.100.3" "192.168.100.4" "192.168.100.5" "192.168.100.6")

if [ -z "$HOST_CAM_IP" ]; then
    echo "❌ 偵測不到本機 192.168.100.x 的 IP，請先確認網路接好"; exit 1
fi

echo "=================================================="
echo " 🚀 G6 感測器時間同步"
echo "    本機相機網段 IP : $HOST_CAM_IP"
echo "    本機光達網段 IP : ${HOST_LIDAR_IP:-未偵測到}"
echo "=================================================="

# ── Step 1：本機 Chrony ──────────────────────────────────────────────────
echo ""
echo "▶ Step 1：Ubuntu Chrony 狀態"
if ! systemctl is-active --quiet chrony; then
    sudo systemctl start chrony && sudo systemctl enable chrony
    sleep 2
fi
sudo chronyc makestep 2>/dev/null
chronyc tracking | grep -E "Reference ID|Stratum|System time"
echo "  系統時間 UTC: $(date -u '+%Y-%m-%d %H:%M:%S')"

# ── Step 2：VLS-128 NTP（一次設定永久生效）─────────────────────────────────
if [ -n "$HOST_LIDAR_IP" ]; then
    echo ""
    echo "▶ Step 2：VLS-128 NTP → $HOST_LIDAR_IP"
    if ping -c 1 -W 2 $LIDAR_IP > /dev/null 2>&1; then
        curl -s --max-time 5 "http://$LIDAR_IP/cgi/setting" \
            -d "ntp_ipaddr_1=$HOST_LIDAR_IP" >/dev/null
        echo "  ✅ 已送出"
    else
        echo "  ⚠️  連不到 LiDAR，跳過"
    fi
fi

# ── Step 3：側相機 5 顆 ───────────────────────────────────────────────────
echo ""
echo "▶ Step 3：側相機 SSH 時間同步"
UTC=$(date -u +"%Y-%m-%d %H:%M:%S")
for IP in "${SSH_CAMS[@]}"; do
    (sshpass -p '填上你的密碼' ssh -o StrictHostKeyChecking=no -o ConnectTimeout=3 \
        root@$IP "date -s '$UTC'" >/dev/null 2>&1) &
done
wait
echo "  ✅ 5 路側相機已同步"

# ── Step 4：主相機 - 偵測再啟動 ntpd ──────────────────────────────────────
echo ""
echo "▶ Step 4：主相機 - 偵測時間同步工具"
echo "--------------------------------------------------"

# 先 telnet 進去一次，看裡面有哪些可用工具
PROBE=$(
    {
        sleep 1; echo "root"; sleep 1
        echo "echo PROBE_START"
        echo "which ntpd"
        echo "which busybox"
        echo "ls /usr/sbin/ntpd /sbin/ntpd /bin/ntpd /usr/bin/ntpd 2>/dev/null"
        echo "busybox 2>&1 | head -2"
        echo "echo PROBE_END"
        sleep 1; echo "exit"
    } | timeout 8 telnet $MAIN_CAM_IP 2>/dev/null
)

# 解析探測結果
HAS_NTPD_BIN=$(echo "$PROBE" | grep -E "^/.*ntpd$" | head -1)
HAS_BUSYBOX=$(echo "$PROBE"  | grep -i "BusyBox v" | head -1)

echo "  探測結果："
echo "    ntpd 執行檔 : ${HAS_NTPD_BIN:-（沒有）}"
echo "    busybox    : ${HAS_BUSYBOX:-（沒有）}"
echo ""

# 根據偵測結果決定怎麼做
UTC=$(date -u +"%Y-%m-%d %H:%M:%S")

if [ -n "$HAS_NTPD_BIN" ]; then
    echo "  → 使用獨立 ntpd"
    NTPD_CMD="$HAS_NTPD_BIN -p $HOST_CAM_IP"
elif [ -n "$HAS_BUSYBOX" ]; then
    # busybox 有內建 ntpd
    echo "  → 使用 busybox ntpd"
    NTPD_CMD="busybox ntpd -p $HOST_CAM_IP"
else
    echo "  ⚠️  相機沒有 ntpd 也沒有 busybox"
    echo "      只能先 date -s 對一次時間（會飄）"
    NTPD_CMD=""
fi

# 執行
{
    sleep 1; echo "root"; sleep 1
    echo "date -s '$UTC'"
    if [ -n "$NTPD_CMD" ]; then
        sleep 0.5
        echo "killall ntpd 2>/dev/null"
        sleep 0.5
        echo "$NTPD_CMD &"
        sleep 0.5
        echo "ps | grep ntpd | grep -v grep"
    fi
    sleep 1; echo "exit"
} | timeout 10 telnet $MAIN_CAM_IP 2>/dev/null > /tmp/maincam_setup.log

# ── Step 5：驗證 ntpd 是否真的在跑 ────────────────────────────────────────
echo ""
echo "▶ Step 5：驗證主相機 ntpd 狀態"
sleep 2
VERIFY=$(
    {
        sleep 1; echo "root"; sleep 1
        echo "echo VERIFY"; echo "ps | grep ntpd | grep -v grep"
        sleep 1; echo "exit"
    } | timeout 8 telnet $MAIN_CAM_IP 2>/dev/null
)

if echo "$VERIFY" | grep -q "ntpd"; then
    echo "  ✅ ntpd 已啟動，相機會持續同步 $HOST_CAM_IP，無需定期重設"
    echo "  程序資訊："
    echo "$VERIFY" | grep ntpd | grep -v grep | head -2 | sed 's/^/      /'
else
    if [ -n "$NTPD_CMD" ]; then
        echo "  ❌ ntpd 啟動失敗，可能原因："
        echo "      - 相機防火牆擋了 UDP 123"
        echo "      - Ubuntu 上的 chrony 未開放 192.168.100.0/24 同步"
        echo "        → 檢查 /etc/chrony/chrony.conf 有沒有 'allow 192.168.100.0/24'"
    else
        echo "  ⚠️  此相機型號沒有 ntpd，需要其他方案："
        echo "      A. 改用 RTP 時間戳（修改 cme_cv2_maincam.py 取 RTCP SR）"
        echo "      B. 跑常駐重設背景程序（python script 每 N 秒 telnet 對時）"
        echo "      C. 改買有 ntpd 的相機 / 升級韌體"
    fi
fi

echo ""
echo "=================================================="
echo " 完成"
echo "=================================================="
