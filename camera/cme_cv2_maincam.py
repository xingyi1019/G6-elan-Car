#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
主相機節點 - G6 Cam_Main (4K / hevc / GPU)

時空倒流優化版：
  - OSD 直接從 NV12 的 Y-plane（全解析度灰階）解碼
  - 🔧 核心修改：將 Image 與 CompressedImage 的 header.stamp 從「電腦接收時間」修改為「OSD 曝光時間」
  - 預設只發 2.5K JPEG，4K raw 預設關閉

Topic:
  /cme_cam/main/compressed   2.5K JPEG (header.stamp 已同步為 OSD 時間)
  /cme_cam/main              4K BGR8 raw (header.stamp 已同步為 OSD 時間)
  /cme_cam/main/osd          Float64MultiArray [fc, osd_ns, ros_ns, latency_ms, p, r, y]
"""

import rospy, cv2, numpy as np, struct, threading, time, sys, os, subprocess
from collections import deque
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Float64MultiArray

try:
    from turbojpeg import TurboJPEG
    _jpeg = TurboJPEG()
except ImportError:
    _jpeg = None

# ─── OSD 解碼（吃灰階，不用 cvtColor）────────────────────────────────────────
_OSD_COLS=32; _OSD_ROWS=6; _OSD_BIT_SIZE=2; _OSD_POS_X=6; _OSD_POS_Y=6; _OSD_THRESHOLD=180

def decode_osd_gray(gray):
    """gray = 全解析度灰階影像（NV12 的 Y-plane 直接就是）"""
    H, W = gray.shape[:2]
    scale  = W / 3840.0
    bit_sz = max(1, round(_OSD_BIT_SIZE * scale))
    bx0    = W - _OSD_COLS*bit_sz - max(1, round(_OSD_POS_X*scale))
    by0    = max(1, round(_OSD_POS_Y*scale))
    roi = gray[by0:by0+_OSD_ROWS*bit_sz, bx0:bx0+_OSD_COLS*bit_sz]
    if roi.size == 0:
        return None
    def dr(r):
        val=0; ry=r*bit_sz
        for c in range(_OSD_COLS):
            cx=c*bit_sz; b=roi[ry:ry+bit_sz, cx:cx+bit_sz]
            val=(val<<1)|(1 if b.size>0 and int(b.mean())>_OSD_THRESHOLD else 0)
        return val
    try:
        rows=[dr(r) for r in range(_OSD_ROWS)]
    except Exception:
        return None
    ts_ns=(rows[1]<<32)|rows[2]
    if ts_ns==0:
        return None
    pitch=struct.unpack('f',struct.pack('I',rows[3]))[0]
    roll =struct.unpack('f',struct.pack('I',rows[4]))[0]
    yaw  =struct.unpack('f',struct.pack('I',rows[5]))[0]
    return rows[0], ts_ns, pitch, roll, yaw


class MainCamWorker:
    RTSP_URL="rtsp://192.168.100.26/liveRTSP/av1"
    RAW_W, RAW_H = 3840, 2160
    COMP_W, COMP_H = 2560, 1440

    def __init__(self, publish_compressed=True, publish_raw=False):
        self.publish_compressed = publish_compressed
        self.publish_raw        = publish_raw
        self.running = True
        self.fps = 0.0

        self.pub_osd = rospy.Publisher("/cme_cam/main/osd", Float64MultiArray, queue_size=5)
        if publish_compressed:
            self.pub_comp = rospy.Publisher("/cme_cam/main/compressed", CompressedImage, queue_size=1)
        if publish_raw:
            self.pub_raw = rospy.Publisher("/cme_cam/main", Image, queue_size=1)

        # 擷取執行緒把 raw NV12 bytes 和校正後的時間戳丟給發布執行緒
        self._q = deque(maxlen=1)
        self.ffmpeg_proc = None

        threading.Thread(target=self._capture_loop, daemon=True).start()
        threading.Thread(target=self._publish_loop, daemon=True).start()

    def _build_cmd(self):
        return ["ffmpeg","-hide_banner","-loglevel","error",
                "-user_agent","VLC/3.0.16","-rtsp_transport","tcp",
                "-fflags","nobuffer","-flags","low_delay","-max_delay","100000",
                "-hwaccel","cuda","-hwaccel_output_format","cuda","-c:v","hevc_cuvid",
                "-i",self.RTSP_URL,"-an","-sn",
                "-vf","hwdownload,format=nv12","-pix_fmt","nv12","-f","rawvideo","pipe:1"]

    def _start_ffmpeg(self):
        self._stop_ffmpeg()
        self.ffmpeg_proc=subprocess.Popen(self._build_cmd(),
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, bufsize=10**8)

    def _stop_ffmpeg(self):
        if self.ffmpeg_proc:
            try:
                self.ffmpeg_proc.terminate()
                try: self.ffmpeg_proc.wait(timeout=2.0)
                except subprocess.TimeoutExpired: self.ffmpeg_proc.kill()
            except: pass
            self.ffmpeg_proc=None

    def _read_exact(self, pipe, size):
        data=b""
        while len(data)<size and self.running and not rospy.is_shutdown():
            c=pipe.read(size-len(data))
            if not c: return None
            data+=c
        return data

    # ── 擷取執行緒：讀 NV12 → 解 OSD → 時間戳修正歸位 → 丟給發布執行緒 ───────
    def _capture_loop(self):
        fb = int(self.RAW_W * self.RAW_H * 1.5)   # NV12 總長
        y_len = self.RAW_W * self.RAW_H            # Y-plane 長度
        prev_tick=cv2.getTickCount(); fc=0; t0=time.time(); tgt=1.0/30.0

        while not rospy.is_shutdown() and self.running:
            try:
                rospy.loginfo("[Main] 連接 RTSP...")
                self._start_ffmpeg()
                while not rospy.is_shutdown() and self.running:
                    ls=time.time()
                    raw=self._read_exact(self.ffmpeg_proc.stdout, fb)
                    if raw is None:
                        rospy.logwarn("[Main] 重連..."); break
                    ts=rospy.Time.now()

                    # 預設使用電腦接收時間作為保底，避免解碼失敗時 ROS 時間戳出錯
                    img_stamp = ts 

                    # OSD：Y-plane 就是全解析度灰階
                    y = np.frombuffer(raw, dtype=np.uint8, count=y_len).reshape(self.RAW_H, self.RAW_W)
                    osd = decode_osd_gray(y)
                    if osd is not None:
                        f, osd_ns, p, r, yw = osd
                        ros_ns = ts.to_nsec()
                        lat = (ros_ns - osd_ns)/1e6
                        
                        # 🔧 核心修改點：將 OSD 的奈秒時間戳轉成 rospy.Time 格式
                        osd_ros_time = rospy.Time.from_sec(osd_ns / 1e9)
                        
                        # 防禦性檢查：確認解析出來的時間落再合理區間（大於2026年且不超越電腦時間太誇張）
                        if 1767225600 < (osd_ns / 1e9) < (ts.to_sec() + 5.0):
                            img_stamp = osd_ros_time  # 🚀 成功時空倒流！用相機曝光時間覆蓋接收時間
                        
                        m=Float64MultiArray()
                        m.data=[float(f),float(osd_ns),float(ros_ns),lat,p,r,yw]
                        self.pub_osd.publish(m)
                        
                        if f % 30 == 0:
                            if f == 0 or lat > 5000.0 or lat < -5000.0:
                                rospy.loginfo(f"[OSD] frame={f:6d}  latency=  INVALID (Syncing/Corrupted)")
                            else:
                                rospy.loginfo(f"[OSD] frame={f:6d}  latency={lat:+8.1f} ms")

                    # 🔧 將 raw 影像與決定好的「精準曝光時間戳 (img_stamp)」一起塞進佇列
                    self._q.append((raw, img_stamp))

                    fc+=1
                    if time.time()-t0>=2.0:
                        tick=cv2.getTickCount()
                        self.fps=fc/((tick-prev_tick)/cv2.getTickFrequency())
                        prev_tick=tick; fc=0; t0=time.time()

                    w=tgt-(time.time()-ls)
                    if w>0: time.sleep(w)
            except Exception as e:
                rospy.logwarn(f"[Main] capture 例外: {e}")
            finally:
                self._stop_ffmpeg()
                if self.running: time.sleep(2.0)

    # ── 發布執行緒：接收已校正的時間戳並發布（會自動丟幀）────────────────────
    def _publish_loop(self):
        while not rospy.is_shutdown() and self.running:
            if not self._q:
                time.sleep(0.003); continue
            
            # 🔧 這裡拿到的 ts 已經是被我們替換成精準 OSD 曝光時間的 img_stamp 了！
            raw, ts = self._q.pop()
            try:
                yuv = np.frombuffer(raw, dtype=np.uint8).reshape((int(self.RAW_H*1.5), self.RAW_W))
                bgr = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_NV12)
            except Exception:
                continue

            # 4K raw（預設關閉）
            if self.publish_raw:
                try:
                    im=Image(); im.header.stamp=ts; im.header.frame_id="cam_main"
                    im.height=self.RAW_H; im.width=self.RAW_W; im.encoding="bgr8"
                    im.is_bigendian=0; im.step=self.RAW_W*3; im.data=bgr.tobytes()
                    self.pub_raw.publish(im)
                except: pass

            # 2.5K JPEG（預設開）
            if self.publish_compressed and _jpeg is not None:
                try:
                    small=cv2.resize(bgr,(self.COMP_W,self.COMP_H),interpolation=cv2.INTER_LINEAR)
                    cm=CompressedImage(); cm.header.stamp=ts; cm.header.frame_id="cam_main"
                    cm.format="jpeg"; cm.data=_jpeg.encode(small,quality=40)
                    self.pub_comp.publish(cm)
                except: pass

    def stop(self):
        self.running=False; self._stop_ffmpeg()


def main():
    rospy.init_node("cme_main_cam", anonymous=False)
    pub_comp = rospy.get_param("~publish_compressed", True)
    pub_raw  = rospy.get_param("~publish_raw", False)
    rospy.loginfo(f"=== 主相機 | 2.5K compressed={'ON' if pub_comp else 'OFF'} | "
                  f"4K raw={'ON' if pub_raw else 'OFF'} | OSD ON ===")
    w=MainCamWorker(publish_compressed=pub_comp, publish_raw=pub_raw)
    try:
        r=rospy.Rate(1)
        while not rospy.is_shutdown():
            sys.stdout.write(f"\r[Main] {w.fps:.1f} FPS   "); sys.stdout.flush(); r.sleep()
    except KeyboardInterrupt: pass
    finally: w.stop(); os.system("stty sane")

if __name__=="__main__": main()
