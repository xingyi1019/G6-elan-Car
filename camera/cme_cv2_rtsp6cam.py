#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
import threading
import time
import sys
import os
import subprocess
from sensor_msgs.msg import CompressedImage

try:
    from turbojpeg import TurboJPEG
    jpeg = TurboJPEG()
    print("[系統] TurboJPEG 載入成功")
except ImportError:
    print("[錯誤] 找不到 PyTurboJPEG，請安裝: pip3 install PyTurboJPEG")
    sys.exit(1)

class CameraWorker:
    def __init__(self, config):
        self.id = config["id"]
        self.ip = config["ip"]
        self.topic = config["topic"]
        self.raw_w = config["w"]
        self.raw_h = config["h"]
        self.stream = config.get("stream", "av4")
        self.codec = config.get("codec", "h264")
        self.use_gpu = config.get("use_gpu", False) 
        self.do_resize = config.get("resize", True)

        # --- 解析度設定邏輯 ---
        if self.id == "Cam_Main":
            # 主相機縮圖至 2.5K
            self.target_w = 2560 if self.do_resize else self.raw_w
            self.target_h = 1440 if self.do_resize else self.raw_h
        else:
            # 側邊/後方相機：從 960x768 提升至 1280x1024
            # 這是畫質與效能的下一個平衡點
            self.target_w = 1280 if self.do_resize else self.raw_w
            self.target_h = 1024 if self.do_resize else self.raw_h

        self.pub = rospy.Publisher(self.topic + "/compressed", CompressedImage, queue_size=1)
        self.running = True
        self.fps = 0.0

        self.lock = threading.Lock()
        self.latest_frame = None

        self.ffmpeg_proc = None
        # 已移除 log 檔案變數

        self.capture_thread = threading.Thread(target=self.capture_loop, daemon=True)
        self.process_thread = threading.Thread(target=self.process_loop, daemon=True) 

        self.capture_thread.start()
        self.process_thread.start()

    def build_ffmpeg_cmd(self):
        rtsp_url = f"rtsp://{self.ip}/liveRTSP/{self.stream}"
        
        if self.use_gpu:
            hw_codec = "hevc_cuvid" if self.codec == "hevc" else "h264_cuvid"
            cmd = [
                "ffmpeg", "-hide_banner", "-loglevel", "error", "-user_agent", "VLC/3.0.16",
                "-rtsp_transport", "tcp", "-hwaccel", "cuda", "-hwaccel_output_format", "cuda",
                "-c:v", hw_codec
            ]
            if self.do_resize:
                # GPU 縮放參數放在 -i 前面
                cmd.extend(["-resize", f"{self.target_w}x{self.target_h}"])
                
            cmd.extend([
                "-i", rtsp_url,
                "-an", "-sn", "-vf", "hwdownload,format=nv12", "-pix_fmt", "nv12",
                "-f", "rawvideo", "pipe:1"
            ])
        else:
            cmd = [
                "ffmpeg", "-hide_banner", "-loglevel", "error", "-user_agent", "VLC/3.0.16",
                "-rtsp_transport", "tcp"
            ]
            
            # 明確指定軟體解碼器
            if self.codec == "hevc":
                cmd.extend(["-c:v", "hevc"])
            elif self.codec == "h264":
                cmd.extend(["-c:v", "h264"])
                
            cmd.extend(["-i", rtsp_url]) # 必須先讀入輸入檔
            
            if self.do_resize:
                # CPU 軟體縮放參數 -s 必須放在 -i 後面
                cmd.extend(["-s", f"{self.target_w}x{self.target_h}"])
                
            cmd.extend([
                "-an", "-sn", "-pix_fmt", "nv12",
                "-f", "rawvideo", "pipe:1"
            ])
        return cmd

    def start_ffmpeg(self):
        cmd = self.build_ffmpeg_cmd()
        # 直接將 stderr 導向 DEVNULL，不再產生 log 檔
        self.ffmpeg_proc = subprocess.Popen(
            cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, bufsize=10**8
        )

    def stop(self):
        self.running = False
        if self.ffmpeg_proc:
            try:
                self.ffmpeg_proc.terminate() # 溫柔關閉
                try:
                    self.ffmpeg_proc.wait(timeout=1.0) # 等一秒鐘
                except subprocess.TimeoutExpired:
                    # 如果一秒內沒關掉，強制擊殺防殭屍
                    self.ffmpeg_proc.kill() 
            except: pass
            self.ffmpeg_proc = None

    def read_exact(self, pipe, size):
        data = b""
        while len(data) < size and not rospy.is_shutdown() and self.running:
            chunk = pipe.read(size - len(data))
            if not chunk: return None
            data += chunk
        return data

    def capture_loop(self):
        frame_bytes = int(self.target_w * self.target_h * 1.5)
        
        while not rospy.is_shutdown() and self.running:
            try:
                self.start_ffmpeg()
                while not rospy.is_shutdown() and self.running:
                    raw = self.read_exact(self.ffmpeg_proc.stdout, frame_bytes)
                    if raw is None: break
                    
                    yuv_data = np.frombuffer(raw, dtype=np.uint8).reshape((int(self.target_h * 1.5), self.target_w))
                    frame = cv2.cvtColor(yuv_data, cv2.COLOR_YUV2BGR_NV12)
                    
                    with self.lock:
                        self.latest_frame = frame
            except Exception:
                self.stop()
                time.sleep(2.0)
        self.stop()

    def process_loop(self):
        prev_tick = cv2.getTickCount()
        frame_count = 0
        start_time = time.time()
        
        while not rospy.is_shutdown() and self.running:
            loop_start = time.time()
            frame_to_process = None
            
            with self.lock:
                if self.latest_frame is not None:
                    frame_to_process = self.latest_frame
                    self.latest_frame = None
                    
            if frame_to_process is None:
                time.sleep(0.005)
                continue

            try:
                # 進行 JPEG 壓縮
                encoded_image = jpeg.encode(frame_to_process, quality=40)
                msg = CompressedImage()
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = self.id
                msg.format = "jpeg"
                msg.data = encoded_image
                self.pub.publish(msg)
                
                frame_count += 1
                if time.time() - start_time >= 2.0:
                    curr_tick = cv2.getTickCount()
                    time_diff = (curr_tick - prev_tick) / cv2.getTickFrequency()
                    self.fps = frame_count / time_diff
                    start_time = time.time()
                    frame_count = 0
                    prev_tick = curr_tick
            except: pass
            
            elapsed = time.time() - loop_start
            sleep_time = (1.0 / 30.0) - elapsed
            if sleep_time > 0: time.sleep(sleep_time)

def main():
    rospy.init_node('cme_6cam_balanced', anonymous=True)
    print("=== 6鏡頭 [完美平衡：3路硬解+3路軟解+解析度減半] ===")

    # Main, Left, Right 使用 GPU 硬解
    # Rear, SideL, SideR 使用 CPU 軟解
    cameras = [
        {"id": "Cam_Main",  "ip": "192.168.100.26", "topic": "/cme_cam/main",  "w": 3840, "h": 2160, "codec": "hevc", "use_gpu": True,  "resize": True},
        {"id": "Cam_Left",  "ip": "192.168.100.2",  "topic": "/cme_cam/left",  "w": 1920, "h": 1536, "codec": "h264", "use_gpu": False,  "resize": True},
        {"id": "Cam_Right", "ip": "192.168.100.5",  "topic": "/cme_cam/right", "w": 1920, "h": 1536, "codec": "h264", "use_gpu": False,  "resize": True},
        {"id": "Cam_SideL", "ip": "192.168.100.4",  "topic": "/cme_cam/sideL", "w": 1920, "h": 1536, "codec": "h264", "use_gpu": False, "resize": True},
        {"id": "Cam_SideR", "ip": "192.168.100.3",  "topic": "/cme_cam/sideR", "w": 1920, "h": 1536, "codec": "h264", "use_gpu": False, "resize": True},
        {"id": "Cam_Rear",  "ip": "192.168.100.6",  "topic": "/cme_cam/rear",  "w": 1920, "h": 1536, "codec": "h264", "use_gpu": True, "resize": True},
    ]

    workers = [CameraWorker(c) for c in cameras]
    try:
        while not rospy.is_shutdown():
            status = " | ".join([f"[{w.id}]: {w.fps:.1f} FPS" for w in workers])
            sys.stdout.write("\r" + status + "   ")
            sys.stdout.flush()
            time.sleep(1)
    except KeyboardInterrupt: pass
    finally:
        for w in workers: w.stop()
        os.system('stty sane')

if __name__ == '__main__': main()
