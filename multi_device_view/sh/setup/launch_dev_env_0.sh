#!/bin/bash

DEVICE="module_0"

# launch new session
tmux new-session -d -s dev_env

# horizontal split of two panels
tmux split-window -v

# move to panel 1, and vertical split
# tmux select-pane -t 1
# tmux split-window -v

tmux send-keys -t 0 "echo 0" C-m
tmux send-keys -t 0 "source ~/stereo_camera_ws/devel/setup.bash bash" C-m
tmux send-keys -t 0 "roscd multi_device_view/scripts/recognition" C-m
# tmux send-keys -t 0 "emacs -nw " C-m
tmux send-keys -t 0 "rossetip" C-m

tmux send-keys -t 1 "echo 1" C-m
tmux send-keys -t 1 "source ~/robothand_dataset_ws/devel/setup.bash bash" C-m
tmux send-keys -t 1 "rossetip" C-m
tmux send-keys -t 1 "roscd multi_device_view/scripts/recognition" C-m
# tmux send-keys -t 1 "roslaunch multi_device_view manage_tf.launch" C-m

tmux new-window


# attatch new session
tmux select-window -t dev_env:1
tmux split-window -v

tmux send-keys -t 0 "echo 0" C-m
tmux send-keys -t 0 "source ~/stereo_camera_ws/devel/setup.bash bash" C-m
tmux send-keys -t 0 "roscd multi_device_view/scripts/recognition" C-m
tmux send-keys -t 0 "rossetip" C-m
tmux send-keys -t 0 "roscd multi_device_view" C-m
tmux send-keys -t 0 "rosservice call /$DEVICE/set_background"

tmux send-keys -t 1 "echo 1" C-m
tmux send-keys -t 1 "source ~/robothand_dataset_ws/devel/setup.bash bash" C-m
tmux send-keys -t 1 "roscd multi_device_view/scripts/recognition" C-m
tmux send-keys -t 1 "rossetip" C-m
tmux send-keys -t 1 "roscd multi_device_view" C-m


tmux new-window
tmux select-window -t dev_env:2
tmux split-window -v

tmux send-keys -t 0 "echo 0" C-m
tmux send-keys -t 0 "source ~/robothand_dataset_ws/devel/setup.bash bash" C-m
tmux send-keys -t 0 "roscd multi_device_view/launch/recognition"
# tmux send-keys -t 0 "emacs -nw " C-m
tmux send-keys -t 0 "rossetip" C-m
tmux send-keys -t 0 "roslaunch multi_device_view launch_realsense.launch"

tmux send-keys -t 1 "echo 1" C-m
tmux send-keys -t 1 "source ~/robothand_dataset_ws/devel/setup.bash bash" C-m
tmux send-keys -t 1 "rossetip" C-m
tmux send-keys -t 1 "roscd multi_device_view/launch/recognition" C-m
tmux send-keys -t 1 "roslaunch multi_device_view upper_arm_hole.launch DEVICE:=$DEVICE"
# tmux send-keys -t 1 "roslaunch multi_device_view manage_tf.launch" Cm-

tmux a
