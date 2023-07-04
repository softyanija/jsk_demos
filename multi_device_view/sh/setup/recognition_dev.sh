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
tmux send-keys -t 0 "rossetmaster 133.11.216.44" C-m
tmux send-keys -t 0 "rossetip" C-m
tmux send-keys -t 0 "roslaunch multi_device_view reconstruct_points.launch" C-m

tmux send-keys -t 1 "echo 1" C-m
tmux send-keys -t 1 "rossetmaster 133.11.216.44" C-m
tmux send-keys -t 1 "rossetip" C-m
tmux send-keys -t 1 "rviz -d $(rospack find multi_device_view)/config/d405_remote.rviz" C-m

tmux send-keys -t 2 "echo 2" C-m
tmux send-keys -t 2 "ssh heavens" C-m
tmux send-keys -t 2 "rossetip" C-m
tmux send-keys -t 2 "rossetmaster pr1012" C-m
tmux send-keys -t 2 "cd ~/robothand_dataset_ws/src/DREAM" C-m
tmux send-keys -t 2 "sh shell/ros_interface_timercam_1.sh trained_models/kp4_0510/kp4_0510.pth" C-m

tmux send-keys -t 3 "echo 3" C-m
tmux send-keys -t 3 "ssh heavens" C-m
tmux send-keys -t 3 "rossetip" C-m
tmux send-keys -t 3 "rossetmaster pr1012" C-m
tmux send-keys -t 3 "cd ~/robothand_dataset_ws/src/DREAM" C-m
tmux send-keys -t 3 "sh shell/ros_interface_timercam_2.sh trained_models/kp4_0510/kp4_0510.pth" C-m

tmux new-window


# 新しいセッションをアタッチ
tmux attach-session

tmux split-window -h

tmux send-keys -t 0 "echo 0" C-m
tmux send-keys -t 0 "rossetmaster pr1012" C-m

tmux send-keys -t 1 "echo 1" C-m
tmux send-keys -t 1 "rossetmaster pr1012" C-m

