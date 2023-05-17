#!/bin/bash

# 新しいセッションを作成
tmux new-session -d

# 2つのパネルを水平に分割
tmux split-window -h

# # 1番目のパネルに移動し、2つのパネルを垂直に分割
tmux select-pane -t 1
tmux split-window -v

# 3番目のパネルに移動し、2つのパネルを垂直に分割
tmux select-pane -t 0
tmux split-window -v

tmux send-keys -t 0 "echo 0" C-m
tmux send-keys -t 0 "rossetmaster pr1012" C-m
tmux send-keys -t 0 "roslaunch multi_device_view timercams.launch" C-m

tmux send-keys -t 1 "echo 1" C-m
tmux send-keys -t 1 "rossetmaster pr1012" C-m
tmux send-keys -t 1 "roslaunch multi_device_view pub_keypoints_tf.launch" C-m

tmux send-keys -t 2 "echo 2" C-m
tmux send-keys -t 2 "rossetmaster pr1012" C-m
tmux send-keys -t 2 "cd ~/robothand_dataset_ws/src/DREAM" C-m
tmux send-keys -t 2 "sh shell/ros_interface_timercam.sh train/kp4_0510/kp4_0510.pth" C-m

tmux send-keys -t 3 "echo 3" C-m
tmux send-keys -t 3 "rossetmaster pr1012" C-m
tmux send-keys -t 3 "roscd multi_device_view" C-m
tmux send-keys -t 3 "rviz -d config/multicam_tfframe_debug.rviz" C-m

# 新しいセッションをアタッチ
tmux attach-session
