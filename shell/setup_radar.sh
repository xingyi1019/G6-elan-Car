#!/bin/bash

echo "🚀 正在啟動 Kvaser CAN ..."

# 1. 重置並設定 Baud Rate
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000

# 2. 啟動介面
sudo ip link set can0 up

cansend can0 200#1800000010040000
echo "✅ 成功！現在請直接執行 candump can0 或是跑你的 Python 程式。"
