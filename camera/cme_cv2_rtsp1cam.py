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
        self.do_resize = config.get("resize", False)
        self.stream = config.get("stream", "av4")
        self.codec = config.get("codec", "h264")
        self.use_gpu = config.get("use_gpu", True) 
        self.target_fps = config.get("target_fps", 30)

        # 主鏡頭縮圖目標設為 2K (2560x1440)
        self.target_w = 2560 if self.do_resize else self.raw_w
        self.target_h = 1440 if self.do_resize else self.raw_h

        self.pub = rospy.Publisher(self.topic + "/compressed", CompressedImage, queue_size=1)
        self.running = True
        self.fps = 0.0

        self.lock = threading.Lock()
        self.latest_frame = None

        self.ffmpeg_proc = None
        self.err_file = None 
        self.err_log_path = f"ffmpeg_err_{self.id}.log" 

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
            if self.do_resize:
                cmd.extend(["-s", f"{self.target_w}x{self.target_h}"])
                
            cmd.extend([
                "-i", rtsp_url,
                "-an", "-sn", "-pix_fmt", "nv12",
                "-f", "rawvideo", "pipe:1"
            ])
        return cmd

    def start_ffmpeg(self):
        cmd = self.build_ffmpeg_cmd()
        self.err_file = open(self.err_log_path, "w")
        self.ffmpeg_proc = subprocess.Popen(
            cmd, stdout=subprocess.PIPE, stderr=self.err_file, bufsize=10**8
        )

    def stop(self):
        self.running = False
        if self.ffmpeg_proc:
            try:
                self.ffmpeg_proc.terminate() 
                try:
                    # 給它1秒鐘關閉，否則強制擊殺
                    self.ffmpeg_proc.wait(timeout=1.0) 
                except subprocess.TimeoutExpired:
                    self.ffmpeg_proc.kill() 
            except: pass
            self.ffmpeg_proc = None
        if self.err_file:
            self.err_file.close()

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
        target_loop_time = 1.0 / self.target_fps
        
        while not rospy.is_shutdown() and self.running:
            loop_start = time.time()
            frame_to_process = None
            
            with self.lock:
                if self.latest_frame is not None:
                    frame_to_process = self.latest_frame
                    self.latest_frame = None
                    
            if frame_to_process is not None:
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
            sleep_time = target_loop_time - elapsed
            if sleep_time > 0: 
                time.sleep(sleep_time)
            else:
                time.sleep(0.001)

def main():
    rospy.init_node('cme_1cam_main', anonymous=True)
    print("=== 單鏡頭主相機 [GPU 硬解版] ===")

    # 只保留主相機
    cameras = [
        {"id": "Cam_Main",  "ip": "192.168.100.26", "topic": "/cme_cam/main",  "w": 3840, "h": 2160, "codec": "hevc", "use_gpu": True,  "resize": True, "target_fps": 30},
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
