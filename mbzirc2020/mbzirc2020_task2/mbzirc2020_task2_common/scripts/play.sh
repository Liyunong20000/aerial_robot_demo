#!/bin/bash

echo -e "\e[31m usage:
if hydrus ID is 1, execute
rosrun mbzirc2020_task2_common play.sh 1 hoge.bag

if hydrus doesn't have ID, execute
rosrun mbzirc2020_task2_common play.sh NONE hoge.bag \e[m"

if [ $1 = "NONE" ]; then
    echo "NO ID"
    ID=""
else
    ID=$1
fi

NS=/hydrus$ID

echo namespace=$NS

shift

rosbag play $@ --clock --pause $NS/rectangle_detection_color/target_object_color:=$NS/rectangle_detection_color/target_object_color_orig $NS/rectangle_detection_depth/target_object_depth:=$NS/rectangle_detection_depth/target_object_depth_orig $NS/channel_center/pos:=$NS/channel_center/pos_orig $NS/kf/alt1/data:=$NS/kf/alt1/data_orig $NS/kf/gps1/data:=$NS/kf/gps1/data_orig $NS/kf/imu1/data:=$NS/kf/imu1/data_orig $NS/kf/vo1/data:=$NS/kf/vo1/data_orig $NS/kf/plane_detection1/data:=$NS/kf/plane_detection1/data_orig
