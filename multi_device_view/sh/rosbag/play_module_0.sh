#!/bin/bash

roslaunch multi_device_view play_module_0.launch rosbag:=$1 start:=$2 range:=3
