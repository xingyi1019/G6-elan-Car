#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
6 鏡頭 RTSP → ROS CompressedImage  (v5)

v5 重點:
主相機的 HEVC 碼流 VUI 標記錯誤 —— ffprobe 顯示 yuvj420p(pc, gbr/...),
格式是 YUV 但矩陣係數寫成 gbr(RGB)。swscale 遇到這種矛盾標記會拒絕轉換,
所以 v2/v3/v4 只要輸出非 nv12 的格式,Cam_Main 就一幀都出不來。

兩種解法,用 out_pix_fmt 切換:
  "nv12"    → ffmpeg 直通輸出 nv12(不經 swscale),NV12→I420 在 numpy 做【推薦】
  "yuv420p" → 用 setparams 覆寫錯誤的色彩標記,讓 swscale 願意轉

其他:
- ffmpeg stderr 保留在 ring buffer,異常時印出來
- Main 用多條編碼執行緒(單執行緒壓不動 4K@30)
- 用 seq 保證發佈順序,多執行緒編碼不會發出過期畫面

需求:PyTurboJPEG >= 1.6.0
用法:
    python3 cme_cv2_rtsp6cam_v5.py
    CAM_DEBUG=1 python3 cme_cv2_rtsp6cam_v5.py
"""

import rospy
import numpy as np
import threading
import time
import sys
import os
import subprocess
from collections import deque
from sensor_msgs.msg import CompressedImage

try:
    from turbojpeg import TurboJPEG
    jpeg = TurboJPEG()
    if not hasattr(jpeg, "encode_from_yuv"):
        print("[錯誤] PyTurboJPEG 太舊,請執行: pip3 install -U PyTurboJPEG")
        sys.exit(1)
except ImportError:
    print("[錯誤] 找不到 PyTurboJPEG,請安裝: pip3 install PyTurboJPEG")
    sys.exit(1)

DEBUG = os.environ.get("CAM_DEBUG") == "1"
_print_lock = threading.Lock()


def log(msg):
    with _print_lock:
        sys.stdout.write("\r\033[K" + msg + "\n")
        sys.stdout.flush()


def nv12_to_i420(raw, w, h, out_buf):
    """NV12(Y + 交錯 UV)→ I420(Y + U + V 平面),TurboJPEG 需要的是後者。
    只是把 UV 解交錯,沒有任何色彩運算,比走一趟 swscale 便宜。"""
    arr = np.frombuffer(raw, dtype=np.uint8)
    ysize = w * h
    q = ysize // 4
    out_buf[:ysize] = arr[:ysize]
    uv = arr[ysize:].reshape(-1, 2)
    out_buf[ysize:ysize + q] = uv[:, 0]
    out_buf[ysize + q:] = uv[:, 1]
    return out_buf


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
        self.quality = config.get("quality", 40)
        self.enc_workers = config.get("enc_workers", 1)
        self.out_pix_fmt = config.get("out_pix_fmt", "yuv420p")  # nv12 | yuv420p

        if self.do_resize:
            self.target_w = config.get("out_w", 1280)
            self.target_h = config.get("out_h", 1024)
        else:
            self.target_w = self.raw_w
            self.target_h = self.raw_h

        self.frame_bytes = self.target_w * self.target_h * 3 // 2

        self.pub = rospy.Publisher(self.topic + "/compressed",
                                   CompressedImage, queue_size=1)
        self.running = True
        self.fps = 0.0
        self.drops = 0
        self.restarts = 0

        self.queue = deque(maxlen=max(2, self.enc_workers + 1))
        self.qlock = threading.Lock()
        self.seq = 0
        self.published_seq = -1

        self.fps_lock = threading.Lock()
        self.frame_count = 0
        self.fps_t0 = time.time()

        self.ffmpeg_proc = None
        self.err_tail = deque(maxlen=15)

        threading.Thread(target=self.capture_loop, daemon=True).start()
        for _ in range(self.enc_workers):
            threading.Thread(target=self.encode_loop, daemon=True).start()

    # ------------------------------------------------------------------ #

    def build_ffmpeg_cmd(self):
        rtsp_url = f"rtsp://{self.ip}/liveRTSP/{self.stream}"
        cmd = ["ffmpeg", "-hide_banner", "-loglevel", "warning",
               "-user_agent", "VLC/3.0.16", "-rtsp_transport", "tcp"]

        if self.use_gpu:
            hw = "hevc_cuvid" if self.codec == "hevc" else "h264_cuvid"
            cmd += ["-hwaccel", "cuda", "-hwaccel_output_format", "cuda",
                    "-c:v", hw]
            if self.do_resize:
                cmd += ["-resize", f"{self.target_w}x{self.target_h}"]
            cmd += ["-i", rtsp_url, "-an", "-sn"]

            vf = ["hwdownload", "format=nv12"]
            if self.out_pix_fmt == "yuv420p":
                # 覆寫攝影機填錯的 VUI,否則 swscale 會拒絕轉換
                vf += ["setparams=colorspace=bt709:color_primaries=bt709"
                       ":color_trc=bt709:range=pc",
                       "format=yuv420p"]
            cmd += ["-vf", ",".join(vf)]
        else:
            cmd += ["-c:v", self.codec, "-i", rtsp_url, "-an", "-sn"]
            if self.do_resize:
                cmd += ["-s", f"{self.target_w}x{self.target_h}",
                        "-sws_flags", "fast_bilinear"]

        cmd += ["-pix_fmt", self.out_pix_fmt, "-f", "rawvideo", "pipe:1"]
        return cmd

    def drain_stderr(self, proc):
        try:
            for line in iter(proc.stderr.readline, b""):
                text = line.decode("utf-8", "ignore").rstrip()
                if not text:
                    continue
                # 這台攝影機的 HEVC 會讓新版 ffmpeg 一直噴這行,是雜訊不是錯誤
                if "Multi-layer HEVC" in text:
                    continue
                self.err_tail.append(text)
                if DEBUG:
                    log(f"[{self.id}] {text}")
        except Exception:
            pass

    def start_ffmpeg(self):
        cmd = self.build_ffmpeg_cmd()
        if DEBUG:
            log(f"[{self.id}] {' '.join(cmd)}")
        self.err_tail.clear()
        self.ffmpeg_proc = subprocess.Popen(
            cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            bufsize=self.frame_bytes)
        threading.Thread(target=self.drain_stderr,
                         args=(self.ffmpeg_proc,), daemon=True).start()

    def kill_ffmpeg(self):
        proc, self.ffmpeg_proc = self.ffmpeg_proc, None
        if proc is None:
            return
        try:
            proc.terminate()
            try:
                proc.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait(timeout=1.0)
        except Exception:
            pass

    def stop(self):
        self.running = False
        self.kill_ffmpeg()

    # ------------------------------------------------------------------ #

    def read_exact(self, pipe, size):
        chunks = []
        remaining = size
        while remaining > 0 and not rospy.is_shutdown() and self.running:
            chunk = pipe.read(remaining)
            if not chunk:
                return None
            chunks.append(chunk)
            remaining -= len(chunk)
        if remaining > 0:
            return None
        return chunks[0] if len(chunks) == 1 else b"".join(chunks)

    def capture_loop(self):
        while not rospy.is_shutdown() and self.running:
            got_any = False
            try:
                self.start_ffmpeg()
                pipe = self.ffmpeg_proc.stdout
                while not rospy.is_shutdown() and self.running:
                    raw = self.read_exact(pipe, self.frame_bytes)
                    if raw is None:
                        break
                    got_any = True
                    with self.qlock:
                        if len(self.queue) == self.queue.maxlen:
                            self.drops += 1
                        self.seq += 1
                        self.queue.append((self.seq, raw))
            except Exception as ex:
                self.err_tail.append(f"python: {ex}")

            self.kill_ffmpeg()
            self.fps = 0.0
            if not (self.running and not rospy.is_shutdown()):
                break

            self.restarts += 1
            reason = " / ".join(list(self.err_tail)[-3:]) or "(無錯誤訊息)"
            log(f"[{self.id}] ffmpeg 結束,"
                f"{'讀到過畫面' if got_any else '完全沒讀到畫面'} "
                f"| 第 {self.restarts} 次重啟 | {reason}")
            time.sleep(2.0)

        self.kill_ffmpeg()

    def encode_loop(self):
        scratch = np.empty(self.frame_bytes, dtype=np.uint8) \
            if self.out_pix_fmt == "nv12" else None

        while not rospy.is_shutdown() and self.running:
            item = None
            with self.qlock:
                if self.queue:
                    item = self.queue.popleft()

            if item is None:
                time.sleep(0.002)
                continue

            seq, raw = item
            try:
                if scratch is not None:
                    yuv = nv12_to_i420(raw, self.target_w, self.target_h, scratch)
                else:
                    yuv = np.frombuffer(raw, dtype=np.uint8)

                encoded = jpeg.encode_from_yuv(
                    yuv, self.target_h, self.target_w, quality=self.quality)

                with self.fps_lock:
                    if seq <= self.published_seq:
                        self.drops += 1
                        continue
                    self.published_seq = seq

                msg = CompressedImage()
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = self.id
                msg.format = "jpeg"
                msg.data = encoded
                self.pub.publish(msg)

                with self.fps_lock:
                    self.frame_count += 1
                    now = time.time()
                    if now - self.fps_t0 >= 2.0:
                        self.fps = self.frame_count / (now - self.fps_t0)
                        self.frame_count = 0
                        self.fps_t0 = now
            except Exception as ex:
                self.err_tail.append(f"encode: {ex}")
                time.sleep(0.05)


# ---------------------------------------------------------------------- #

def main():
    rospy.init_node('cme_6cam_gpu', anonymous=True)
    print("=== 6 鏡頭 v5 [Main 走 nv12 直通,繞過壞掉的色彩標記] ===")

    cameras = [
        # Main:4K 不縮。out_pix_fmt="nv12" 完全不經 swscale,
        # 若想試 setparams 那條路,改成 "yuv420p"
        {"id": "Cam_Main",  "ip": "192.168.100.26", "topic": "/cme_cam/main",
         "w": 3840, "h": 2160, "codec": "hevc", "use_gpu": True,
         "resize": False, "quality": 75, "enc_workers": 3,
         "out_pix_fmt": "nv12"},

        {"id": "Cam_Left",  "ip": "192.168.100.2",  "topic": "/cme_cam/left",
         "w": 1920, "h": 1536, "codec": "h264", "use_gpu": False,
         "resize": True, "out_w": 1280, "out_h": 1024, "quality": 40},

        {"id": "Cam_Right", "ip": "192.168.100.5",  "topic": "/cme_cam/right",
         "w": 1920, "h": 1536, "codec": "h264", "use_gpu": False,
         "resize": True, "out_w": 1280, "out_h": 1024, "quality": 40},

        {"id": "Cam_SideL", "ip": "192.168.100.4",  "topic": "/cme_cam/sideL",
         "w": 1920, "h": 1536, "codec": "h264", "use_gpu": False,
         "resize": True, "out_w": 1280, "out_h": 1024, "quality": 40},

        {"id": "Cam_SideR", "ip": "192.168.100.3",  "topic": "/cme_cam/sideR",
         "w": 1920, "h": 1536, "codec": "h264", "use_gpu": False,
         "resize": True, "out_w": 1280, "out_h": 1024, "quality": 40},

        {"id": "Cam_Rear",  "ip": "192.168.100.6",  "topic": "/cme_cam/rear",
         "w": 1920, "h": 1536, "codec": "h264", "use_gpu": False,
         "resize": True, "out_w": 1280, "out_h": 1024, "quality": 40},
    ]

    workers = [CameraWorker(c) for c in cameras]
    try:
        while not rospy.is_shutdown():
            status = " | ".join([f"{w.id[4:]}:{w.fps:4.1f}" for w in workers])
            drops = sum(w.drops for w in workers)
            with _print_lock:
                sys.stdout.write(f"\r{status} | drop:{drops}   ")
                sys.stdout.flush()
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        for w in workers:
            w.stop()
        print()
        os.system('stty sane')


if __name__ == '__main__':
    main()
