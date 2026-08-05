#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
G6 Sensor Parser V9 - OSD 時間軸純時間配對版 (Bug Fix)

更新說明：
  - 徹底移除 SHIFT_OFFSETS 兩階段人工平移湊數邏輯
  - 🚀 改為純時間戳配對：利用改版後帶有精準 OSD 曝光時間的影像時間軸進行對齊
  - 🎯 嚴格物理時差限制：光達與每顆相機的時差必須小於等於 0.033 秒（1幀上限）才允許配對
  - 🔧 修正：將 generate_combined_gps_map 中錯誤的 .groups() 修改為 .values()
"""

import os
import rosbag
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridge
import cv2
import glob
from tqdm import tqdm
import bisect
import argparse
import folium
from datetime import datetime
import re
import math

# === 設定區 ===
CAM_TOPICS = {
    'main':  '/cme_cam/main/compressed',
    'left':  '/cme_cam/left/compressed',
    'right': '/cme_cam/right/compressed',
    'rear':  '/cme_cam/rear/compressed',
    'sideL': '/cme_cam/sideL/compressed',
    'sideR': '/cme_cam/sideR/compressed'
}

class RosbagProcessor:
    def __init__(self, bag_path, all_gps_data=None):
        self.bag_path = bag_path
        self.bag_name = os.path.splitext(os.path.basename(bag_path))[0]
        self.date_str, self.scenario_type = self.extract_date_and_scenario()
        self.output_dir = os.path.join(self.date_str, self.scenario_type, self.bag_name)
        self.bridge = CvBridge()
        
        self.data_dict = {
            'Cameras': {name: [] for name in CAM_TOPICS.keys()},
            'VLS128': [], 'IMU': [], 'CAN': []
        }
        self.gps_data = []
        self.all_gps_data = all_gps_data if all_gps_data is not None else []

    def extract_date_and_scenario(self):
        filename = self.bag_name
        date_pattern = r'(\d{4}-\d{2}-\d{2})'
        date_match = re.search(date_pattern, filename)
        date_dir = date_match.group(1) if date_match else datetime.now().strftime("%Y-%m-%d")
        
        filename_lower = filename.lower()
        scenario_type = 'general'
        if 'city' in filename_lower: scenario_type = 'citystreet'
        elif 'highway' in filename_lower: scenario_type = 'highway'
        elif 'tunnel' in filename_lower: scenario_type = 'tunnel'
        elif 'seashore' in filename_lower: scenario_type = 'seashore'
        elif 'countryside' in filename_lower: scenario_type = 'countryside'
        
        weather = 'sunny' if 'sunny' in filename_lower else 'general'
        if 'rain' in filename_lower: weather = 'rainy'
        if scenario_type != 'general': scenario_type = f"{scenario_type}_{weather}"
        return date_dir, scenario_type
    
    def create_output_dirs(self):
        os.makedirs(self.output_dir, exist_ok=True)
        for cam_name in CAM_TOPICS.keys():
            os.makedirs(os.path.join(self.output_dir, 'images', cam_name), exist_ok=True)
        os.makedirs(os.path.join(self.output_dir, 'VLS128_pcd'), exist_ok=True)
        os.makedirs(os.path.join(self.output_dir, 'gps'), exist_ok=True)
        os.makedirs(os.path.join(self.output_dir, 'imu'), exist_ok=True)
        os.makedirs(os.path.join(self.output_dir, 'can'), exist_ok=True)
        os.makedirs(os.path.join(self.output_dir, 'timestamps'), exist_ok=True)
        
        for cam_name in CAM_TOPICS.keys():
            os.makedirs(os.path.join(self.output_dir, 'paired', 'images', cam_name), exist_ok=True)
        os.makedirs(os.path.join(self.output_dir, 'paired', 'imu'), exist_ok=True)

    def get_timestamp(self, msg, t):
        if hasattr(msg, 'header') and msg.header.stamp.to_sec() > 0:
            return msg.header.stamp.to_sec(), msg.header.stamp
        return t.to_sec(), t

    def quaternion_to_euler_degrees(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)
        return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)

    def extract_gps_data(self):
        try:
            with rosbag.Bag(self.bag_path) as bag:
                idx = 0
                for topic, msg, t in bag.read_messages(topics=['/fix']):
                    if hasattr(msg, 'latitude') and hasattr(msg, 'longitude'):
                        time_sec, ros_time = self.get_timestamp(msg, t)
                        gps_point = { 
                            'lat': msg.latitude, 'lon': msg.longitude, 
                            'alt': getattr(msg, 'altitude', 0), 
                            'time': time_sec, 'ros_time': ros_time, 
                            'bag_name': self.bag_name, 'scenario_type': self.scenario_type, 
                            'date': self.date_str 
                        }
                        self.gps_data.append(gps_point)
                        self.all_gps_data.append(gps_point)
                        self.save_gps_data(idx, msg, ros_time)
                        idx += 1
        except Exception as e: print(f"提取GPS資料時出錯: {e}")
    
    def process_bag(self):
        print(f"處理 {self.bag_name} -> 日期: {self.date_str}, 情景: {self.scenario_type}")
        self.extract_gps_data()
        
        try:
            with rosbag.Bag(self.bag_path) as bag:
                print("正在讀取所有話題...")
                topics_to_read = ['/velodyne_points', '/can0/received_msg', '/imu']
                topics_to_read.extend(CAM_TOPICS.values())
                
                for topic, msg, t in bag.read_messages(topics=topics_to_read): 
                    time_sec, ros_time = self.get_timestamp(msg, t)
                    
                    is_camera = False
                    for cam_key, cam_topic in CAM_TOPICS.items():
                        if topic == cam_topic:
                            self.data_dict['Cameras'][cam_key].append((time_sec, msg, ros_time))
                            is_camera = True
                            break
                    if is_camera: continue

                    if topic == '/velodyne_points': 
                        self.data_dict['VLS128'].append((time_sec, msg, ros_time))
                    elif topic == '/imu': 
                        self.data_dict['IMU'].append((time_sec, msg, ros_time))
                    elif topic == '/can0/received_msg':
                        self.data_dict['CAN'].append((time_sec, msg, ros_time))
            
            self.data_dict['VLS128'].sort(key=lambda x: x[0])
            self.data_dict['IMU'].sort(key=lambda x: x[0])
            self.data_dict['CAN'].sort(key=lambda x: x[0])
            for cam_key in self.data_dict['Cameras']:
                self.data_dict['Cameras'][cam_key].sort(key=lambda x: x[0])

            self.process_individual_data() 
            self.process_paired_data()
                
        except Exception as e: print(f"處理 {self.bag_name} 時出錯: {str(e)}")

    def process_individual_data(self):
        print("輸出獨立數據 (原始格式)...")
        for cam_key, data_list in self.data_dict['Cameras'].items():
            if not data_list: continue
            for idx, (img_time, msg, ros_time) in enumerate(tqdm(data_list, desc=f"輸出 {cam_key}")):
                self.save_image_data(idx, msg, ros_time, cam_key, prefix="")
            
        for idx, (vls_time, msg, ros_time) in enumerate(tqdm(self.data_dict['VLS128'], desc="輸出 PCD")):
            self.save_pcd_data(idx, msg, ros_time)

        if self.data_dict['IMU']: self.save_imu_file('imu_data.txt', self.data_dict['IMU'])
        if self.data_dict['CAN']: self.save_can_file()

    def process_paired_data(self):
        print("處理配對資料 (V9 新邏輯：基於 OSD 曝光時間戳進行純時間配對)...")
        if not self.data_dict['VLS128']: return
        
        cam_times = {k: [x[0] for x in v] for k, v in self.data_dict['Cameras'].items()}
        imu_times = [x[0] for x in self.data_dict['IMU']]
        
        ts_file_path = os.path.join(self.output_dir, 'timestamps', 'pair_timestamps.txt')
        with open(ts_file_path, 'w') as f:
            header = "PCD_Index,LiDAR_Time"
            for k in CAM_TOPICS.keys(): header += f",{k}_Time,{k}_RawIndex"
            header += ",IMU_Time\n"
            f.write(header)

        print(">> 執行時空硬核對齊：最大允許時差 <= 0.033秒 (33ms)")
        for pcd_idx, (vls_time, vls_msg, vls_t) in enumerate(tqdm(self.data_dict['VLS128'], desc="配對輸出中")):
            
            matched_cams = {}
            ts_line_parts = []
            is_valid_frame = True
            
            for cam_key in CAM_TOPICS.keys():
                cam_idx = self.find_nearest_index(cam_times[cam_key], vls_time)
                
                if cam_idx is not None:
                    cam_data = self.data_dict['Cameras'][cam_key][cam_idx]
                    img_time = cam_data[0] 
                    
                    if abs(img_time - vls_time) <= 0.033:
                        matched_cams[cam_key] = cam_data
                        raw_idx = cam_idx
                        match_ros_t = cam_data[2]
                        ts_line_parts.append(f",{match_ros_t.to_nsec()},{raw_idx}")
                    else:
                        is_valid_frame = False
                        break
                else:
                    is_valid_frame = False
                    break

            if not is_valid_frame:
                continue 
            
            imu_str = ",0"
            imu_idx = self.find_nearest_index(imu_times, vls_time)
            if imu_idx is not None and abs(imu_times[imu_idx] - vls_time) <= 0.05:
                imu_data = self.data_dict['IMU'][imu_idx]
                self.save_single_imu_file(pcd_idx, imu_data)
                imu_str = f",{imu_data[2].to_nsec()}"

            ts_line = f"{pcd_idx:06d},{vls_t.to_nsec()}" + "".join(ts_line_parts) + imu_str
            with open(ts_file_path, 'a') as f:
                f.write(ts_line + "\n")
            
            for cam_key, cam_data in matched_cams.items():
                self.save_image_data(pcd_idx, cam_data[1], cam_data[2], cam_key, prefix="pair_")
                
        print(f"配對大功告成！完美執行「OSD 時間硬對齊 -> 同名存檔」全新 V9 策略。")

    def find_nearest_index(self, sorted_list, target):
        if not sorted_list: return None
        pos = bisect.bisect_left(sorted_list, target)
        if pos == 0: return 0
        if pos == len(sorted_list): return len(sorted_list) - 1
        before, after = sorted_list[pos - 1], sorted_list[pos]
        return pos if after - target < target - before else pos - 1
    
    def save_image_data(self, idx, msg, t, cam_name, prefix=""):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if cv_image is None: return

            folder_type = 'paired' if prefix == "pair_" else ''
            target_folder = os.path.join(self.output_dir, folder_type, 'images', cam_name)
            if prefix == "": target_folder = os.path.join(self.output_dir, 'images', cam_name)

            os.makedirs(target_folder, exist_ok=True)
            filename = os.path.join(target_folder, f'{idx:06d}.png')
            cv2.imwrite(filename, cv_image)
        except Exception: pass
    
    def save_imu_file(self, filename_only, data_list):
        filename = os.path.join(self.output_dir, 'imu', filename_only)
        with open(filename, 'w') as f:
            f.write("Timestamp,Roll,Pitch,Yaw,Accel_X,Accel_Y,Accel_Z,Gyro_X,Gyro_Y,Gyro_Z\n")
            for time_sec, msg, ros_time in data_list:
                r, p, y = self.quaternion_to_euler_degrees(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
                ax, ay, az = msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z
                gx, gy, gz = msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z
                f.write(f"{time_sec:.6f},{r:.6f},{p:.6f},{y:.6f},{ax:.6f},{ay:.6f},{az:.6f},{gx:.6f},{gy:.6f},{gz:.6f}\n")

    def save_single_imu_file(self, idx, imu_tuple):
        time_sec, msg, ros_time = imu_tuple
        filename = os.path.join(self.output_dir, 'paired', 'imu', f'{idx:06d}.txt')
        os.makedirs(os.path.dirname(filename), exist_ok=True)
        with open(filename, 'w') as f:
            r, p, y = self.quaternion_to_euler_degrees(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
            ax, ay, az = msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z
            gx, gy, gz = msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z
            f.write(f"{time_sec:.6f},{r:.6f},{p:.6f},{y:.6f},{ax:.6f},{ay:.6f},{az:.6f},{gx:.6f},{gy:.6f},{gz:.6f}")

    def save_can_file(self):
        filename = os.path.join(self.output_dir, 'can', 'can_raw.txt')
        with open(filename, 'w') as f:
            f.write("Timestamp,ID,Data_Hex\n")
            for time_sec, msg, ros_time in self.data_dict['CAN']:
                hex_d = msg.data.hex() if isinstance(msg.data, bytes) else ""
                f.write(f"{time_sec:.6f},{msg.id},{hex_d}\n")

    def save_pcd_data(self, idx, msg, t):
        points = list(pc2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True))
        if not points: return
        pts = np.array(points, dtype=np.float32)
        header = f"""# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z intensity
SIZE 4 4 4 4
TYPE F F F F
COUNT 1 1 1 1
WIDTH {pts.shape[0]}
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS {pts.shape[0]}
DATA binary
"""
        filename = os.path.join(self.output_dir, 'VLS128_pcd', f'{idx:06d}.pcd')
        os.makedirs(os.path.dirname(filename), exist_ok=True)
        with open(filename, 'wb') as f:
            f.write(header.encode('utf-8'))
            f.write(pts.tobytes())
        
        ts_file = os.path.join(self.output_dir, 'timestamps', 'pointcloud_timestamps.txt')
        with open(ts_file, 'a') as f: f.write(f"{t.to_nsec()}\n")
    
    def save_gps_data(self, idx, msg, t):
        filename = os.path.join(self.output_dir, 'gps', f'{idx:06d}.txt')
        os.makedirs(os.path.dirname(filename), exist_ok=True)
        with open(filename, 'w') as f:
            f.write("timestamp,latitude,longitude,altitude\n")
            alt = getattr(msg, 'altitude', 0)
            f.write(f"{t.to_nsec()},{msg.latitude:.6f},{msg.longitude:.6f},{alt:.6f}\n")

def generate_combined_gps_map(all_gps_data, output_dir="."):
    if not all_gps_data: return
    date_groups = {} 
    for point in all_gps_data: date_groups.setdefault(point['date'], []).append(point)
    m = folium.Map(location=[23.5, 121.0], zoom_start=15)
    
    # 🔧 關鍵修正點：將原先錯誤的 .groups() 改為標準字典的 .values()
    if date_groups:
        for points in date_groups.values():
            if points: 
                m.location = [points[0]['lat'], points[0]['lon']]
                break
                
    colors = ['blue', 'red', 'green', 'purple', 'orange']
    for i, (date, points) in enumerate(date_groups.items()):
        color = colors[i % len(colors)]
        scenario_subgroups = {}
        for point in points: scenario_subgroups.setdefault(point['scenario_type'], []).append(point)
        for scenario, scenario_points in scenario_subgroups.items():
            valid_coordinates = [[p['lat'], p['lon']] for p in scenario_points if math.isfinite(p['lat'])]
            if valid_coordinates: folium.PolyLine(valid_coordinates, color=color, weight=4, opacity=0.7, popup=f"{date} - {scenario}").add_to(m)
    m.save(os.path.join(output_dir, "combined_gps_routes.html"))

def main():
    parser = argparse.ArgumentParser(description='G6 Parser V9 - OSD Pure Time Sync')
    parser.add_argument('input', help='ROS bag path or directory')
    args = parser.parse_args()
    all_gps_data = []
    
    if os.path.isfile(args.input):
        processor = RosbagProcessor(args.input, all_gps_data)
        processor.create_output_dirs()
        processor.process_bag()
        if all_gps_data: generate_combined_gps_map(all_gps_data, os.path.dirname(args.input))
            
    elif os.path.isdir(args.input):
        bag_files = glob.glob(os.path.join(args.input, '*.bag'))
        if not bag_files: print(f"No bag files found in {args.input}")
        else:
            for bag_file in sorted(bag_files):
                processor = RosbagProcessor(bag_file, all_gps_data)
                processor.create_output_dirs()
                processor.process_bag()
            if all_gps_data: generate_combined_gps_map(all_gps_data, args.input)

if __name__ == '__main__':
    main()
