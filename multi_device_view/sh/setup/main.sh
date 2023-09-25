#!/bin/bash

# launch new session
tmux new-session -d -s experiment

# horizontal split of two panels
tmux split-window -h

# move to panel 1, and vertical split
tmux select-pane -t 1
tmux split-window -v

# move to panel 3, and vertical split
tmux select-pane -t 0
tmux split-window -v

tmux send-keys -t 0 "echo 0" C-m
tmux send-keys -t 0 "rossetmaster pr1012" C-m
tmux send-keys -t 0 "rossetip" C-m
tmux send-keys -t 0 "roscd multi_device_view" C-m
tmux send-keys -t 0 "roslaunch multi_device_view manage_tf.launch" C-m

tmux send-keys -t 1 "echo 1" C-m
tmux send-keys -t 1 "rossetmaster pr1012" C-m
tmux send-keys -t 1 "rossetip" C-m
tmux send-keys -t 1 "roscd multi_device_view" C-m
tmux send-keys -t 1 "rviz -d $(rospack find multi_device_view)/config/main.rviz" C-m

tmux send-keys -t 2 "echo 2" C-m
tmux send-keys -t 2 "rossetmaster pr1012" C-m
tmux send-keys -t 2 "rossetip" C-m
tmux send-keys -t 2 "roscd multi_device_view" C-m
tmux send-keys -t 2 "roslaunch multi_device_view lsd_3d.launch" C-m

tmux send-keys -t 3 "echo 3" C-m
tmux send-keys -t 3 "rossetmaster pr1012" C-m
tmux send-keys -t 3 "rossetip" C-m
tmux send-keys -t 0 "roscd multi_device_view" C-m
tmux send-keys -t 3 "roslaunch realsense2_camera rs_rgbd.launch" C-m

tmux new-window


# attatch new session
tmux select-window -t experiment:1

tmux split-window -h

tmux send-keys -t 0 "echo 0" C-m
tmux send-keys -t 0 "rossetmaster pr1012" C-m
tmux send-keys -t 0 "rossetip" C-m
tmux send-keys -t 0 "roscd multi_device_view" C-m
tmux send-keys -t 0 "ipython3 -i -- scripts/screw/set_d405_tf.py" C-m

tmux send-keys -t 1 "echo 1" C-m
tmux send-keys -t 1 "rossetmaster pr1012" C-m
tmux send-keys -t 1 "rossetip" C-m
tmux send-keys -t 1 "roscd multi_device_view" C-m
