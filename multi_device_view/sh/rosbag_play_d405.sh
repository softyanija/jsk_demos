#!/bin/bash

roslaunch multi_device_view rosbag_play_d405.launch rosbag:=$1 start:=$2 range:=$3
