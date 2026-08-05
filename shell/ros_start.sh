#!/bin/bash
source /opt/ros/noetic/setup.bash

echo -e "填上你的密碼\n" | sudo -S chmod a+rw /dev/ttyACM0  # IMU（把「填上你的密碼」換成你的 sudo 密碼）

pkexec bash -c "ip link set can0 type can bitrate 500000 && ip link set up can0"
pkexec bash -c "ip link set can1 type can bitrate 500000 && ip link set up can1"
pkexec bash -c "ip link set can2 type can bitrate 500000 && ip link set up can2"

gnome-terminal --tab -t "roscore" -- bash -c "roscore; exec bash"
sleep 6

### catkin workspace ###
source ~/catkin_ws/devel/setup.bash

gnome-terminal --tab -t "can" -- bash -c "roslaunch ~/Desktop/can2topic.launch; exec bash"
gnome-terminal --tab -t "imu" -- bash -c "roslaunch razor_imu_9dof razor-pub.launch; exec bash"
sleep 6

# gnome-terminal --tab -t "lidar" -- bash -c "roslaunch rslidar_sdk start.launch; exec bash"
gnome-terminal --tab -t "main_cam"  -- bash -c "python3 ~/Desktop/Car/cme_cv2_maincam.py;  exec bash"
gnome-terminal --tab -t "side_cams" -- bash -c "python3 ~/Desktop/Car/cme_cv2_sidecam.py; exec bash"
gnome-terminal --tab -t "VLS128"    -- bash -c "roslaunch velodyne_pointcloud VLS128_points.launch"
rviz -d ~/Desktop/VLS128.rviz &

### gps workspace ###
source ~/gps_ws/devel/setup.bash
echo -e "填上你的密碼\n" | sudo -S chmod a+rw /dev/ttyUSB0  # GPS（把「填上你的密碼」換成你的 sudo 密碼）

gnome-terminal --tab -t "gps" -- bash -c "roslaunch nmea_navsat_driver nmea_serial_driver.launch"
