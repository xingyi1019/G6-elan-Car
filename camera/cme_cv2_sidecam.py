#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
側相機節點 - G6 5 路側視相機
Left / Right / Rear：GPU 硬解 (h264_cuvid)
SideL / SideR     ：CPU 軟解 (h264)
"""

import rospy, cv2, numpy as np, threading, time, sys, os, subprocess
from sensor_msgs.msg import CompressedImage

try:
    from turbojpeg import TurboJPEG
    _jpeg = TurboJPEG()
    rospy.loginfo("[系統] TurboJPEG 載入成功")
except ImportError:
    rospy.logerr("[錯誤] 找不到 PyTurboJPEG，請執行: pip3 install PyTurboJPEG")
    _jpeg = None

SIDE_CAM_CONFIGS = [
    {"id": "Cam_Left",  "ip": "192.168.100.2",  "topic": "/cme_cam/left",
     "raw_w": 1920, "raw_h": 1536, "out_w": 1280, "out_h": 1024,
     "codec": "h264", "use_gpu": True},
    {"id": "Cam_Right", "ip": "192.168.100.5",  "topic": "/cme_cam/right",
     "raw_w": 1920, "raw_h": 1536, "out_w": 1280, "out_h": 1024,
     "codec": "h264", "use_gpu": True},
    {"id": "Cam_SideL", "ip": "192.168.100.4",  "topic": "/cme_cam/sideL",
     "raw_w": 1920, "raw_h": 1536, "out_w": 1280, "out_h": 1024,
     "codec": "h264", "use_gpu": False},
    {"id": "Cam_SideR", "ip": "192.168.100.3",  "topic": "/cme_cam/sideR",
     "raw_w": 1920, "raw_h": 1536, "out_w": 1280, "out_h": 1024,
     "codec": "h264", "use_gpu": False},
    {"id": "Cam_Rear",  "ip": "192.168.100.6",  "topic": "/cme_cam/rear",
     "raw_w": 1920, "raw_h": 1536, "out_w": 1280, "out_h": 1024,
     "codec": "h264", "use_gpu": True},
]


class SideCamWorker:
    def __init__(self, cfg):
        self.id      = cfg["id"]
        self.ip      = cfg["ip"]
        self.topic   = cfg["topic"]
        self.raw_w   = cfg["raw_w"]
        self.raw_h   = cfg["raw_h"]
        self.out_w   = cfg["out_w"]
        self.out_h   = cfg["out_h"]
        self.codec   = cfg["codec"]
        self.use_gpu = cfg.get("use_gpu", False)
        self.running = True
        self.fps     = 0.0

        self.pub = rospy.Publisher(
            self.topic + "/compressed", CompressedImage, queue_size=2)

        self.lock         = threading.Lock()
        self.latest_frame = None
        self.ffmpeg_proc  = None

        self._cap_thread  = threading.Thread(target=self._capture_loop, daemon=True)
        self._proc_thread = threading.Thread(target=self._process_loop, daemon=True)
        self._cap_thread.start()
        self._proc_thread.start()

    def _build_cmd(self):
        rtsp_url = f"rtsp://{self.ip}/liveRTSP/av4"
        if self.use_gpu:
            hw_codec = "h264_cuvid" if self.codec == "h264" else "hevc_cuvid"
            return [
                "ffmpeg", "-hide_banner", "-loglevel", "error",
                "-user_agent", "VLC/3.0.16",
                "-rtsp_transport", "tcp",
                "-hwaccel", "cuda", "-hwaccel_output_format", "cuda",
                "-c:v", hw_codec,
                "-resize", f"{self.out_w}x{self.out_h}",
                "-i", rtsp_url,
                "-an", "-sn",
                "-vf", "hwdownload,format=nv12",
                "-pix_fmt", "nv12",
                "-f", "rawvideo", "pipe:1",
            ]
        else:
            codec_flag = "h264" if self.codec == "h264" else "hevc"
            return [
                "ffmpeg", "-hide_banner", "-loglevel", "error",
                "-user_agent", "VLC/3.0.16",
                "-rtsp_transport", "tcp",
                "-c:v", codec_flag,
                "-i", rtsp_url,
                "-s", f"{self.out_w}x{self.out_h}",
                "-an", "-sn",
                "-pix_fmt", "nv12",
                "-f", "rawvideo", "pipe:1",
            ]

    def _start_ffmpeg(self):
        self._stop_ffmpeg()
        self.ffmpeg_proc = subprocess.Popen(
            self._build_cmd(),
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            bufsize=10**8,
        )

    def _stop_ffmpeg(self):
        if self.ffmpeg_proc:
            try:
                self.ffmpeg_proc.terminate()
                try:    self.ffmpeg_proc.wait(timeout=2.0)
                except subprocess.TimeoutExpired: self.ffmpeg_proc.kill()
            except Exception: pass
            self.ffmpeg_proc = None

    def _read_exact(self, pipe, size):
        data = b""
        while len(data) < size and self.running and not rospy.is_shutdown():
            chunk = pipe.read(size - len(data))
            if not chunk: return None
            data += chunk
        return data

    def _capture_loop(self):
        frame_bytes = int(self.out_w * self.out_h * 1.5)
        while not rospy.is_shutdown() and self.running:
            try:
                self._start_ffmpeg()
                while not rospy.is_shutdown() and self.running:
                    raw = self._read_exact(self.ffmpeg_proc.stdout, frame_bytes)
                    if raw is None:
                        rospy.logwarn(f"[{self.id}] stream 中斷，重連中...")
                        break
                    yuv = np.frombuffer(raw, dtype=np.uint8).reshape(
                              (int(self.out_h * 1.5), self.out_w))
                    bgr = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_NV12)
                    with self.lock:
                        self.latest_frame = bgr
            except Exception as e:
                rospy.logwarn(f"[{self.id}] capture 例外: {e}")
            finally:
                self._stop_ffmpeg()
                if self.running: time.sleep(2.0)

    def _process_loop(self):
        if _jpeg is None:
            rospy.logerr(f"[{self.id}] TurboJPEG 未載入，停止發布。")
            return

        prev_tick   = cv2.getTickCount()
        frame_count = 0
        start_time  = time.time()
        target_dt   = 1.0 / 30.0

        while not rospy.is_shutdown() and self.running:
            t0 = time.time()
            with self.lock:
                frame = self.latest_frame
                self.latest_frame = None
            if frame is None:
                time.sleep(0.005); continue
            try:
                msg = CompressedImage()
                msg.header.stamp    = rospy.Time.now()
                msg.header.frame_id = self.id
                msg.format = "jpeg"
                msg.data   = _jpeg.encode(frame, quality=40)
                self.pub.publish(msg)
            except Exception: pass

            frame_count += 1
            if time.time() - start_time >= 2.0:
                tick = cv2.getTickCount()
                dt   = (tick - prev_tick) / cv2.getTickFrequency()
                self.fps   = frame_count / dt
                start_time = time.time(); frame_count = 0; prev_tick = tick

            elapsed = time.time() - t0
            wait = target_dt - elapsed
            if wait > 0: time.sleep(wait)

    def stop(self):
        self.running = False
        self._stop_ffmpeg()


def main():
    rospy.init_node("cme_side_cams", anonymous=False)
    rospy.loginfo("=== 側相機節點啟動 (5路) ===")
    for cfg in SIDE_CAM_CONFIGS:
        mode = "GPU" if cfg["use_gpu"] else "CPU"
        rospy.loginfo(f"  {cfg['id']:12s}  {cfg['ip']}  "
                      f"{cfg['raw_w']}x{cfg['raw_h']} → {cfg['out_w']}x{cfg['out_h']}  [{mode}]")

    workers = [SideCamWorker(cfg) for cfg in SIDE_CAM_CONFIGS]
    try:
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            status = " | ".join(f"[{w.id}] {w.fps:.1f}" for w in workers)
            sys.stdout.write(f"\r{status}   ")
            sys.stdout.flush()
            rate.sleep()
    except KeyboardInterrupt: pass
    finally:
        for w in workers: w.stop()
        os.system("stty sane")
        rospy.loginfo("[Side] 節點關閉。")


if __name__ == "__main__": main()
