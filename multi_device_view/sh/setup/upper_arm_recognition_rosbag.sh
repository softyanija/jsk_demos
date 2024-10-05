#!/bin/bash

# launch new session
tmux new-session -d -s upper_arm_recognition_rosbag

# horizontal split of two panels
tmux split-window -v

# move to panel 1, and vertical split
# tmux select-pane -t 1
# tmux split-window -v

tmux send-keys -t 0 "echo 0" C-m
tmux send-keys -t 0 "source ~/robothand_dataset_ws/devel/setup.bash bash" C-m
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
tmux select-window -t upper_arm_recognition_rosbag:1
tmux split-window -v

tmux send-keys -t 0 "echo 0" C-m
tmux send-keys -t 0 "source ~/robothand_dataset_ws/devel/setup.bash bash" C-m
tmux send-keys -t 0 "roscd multi_device_view/scripts/recognition" C-m
tmux send-keys -t 0 "rossetip" C-m
tmux send-keys -t 0 "roscd multi_device_view" C-m
tmux send-keys -t 0 "ipython3 -i -- upper_arm_hole.py module_0"

tmux send-keys -t 1 "echo 1" C-m
tmux send-keys -t 1 "source ~/robothand_dataset_ws/devel/setup.bash bash" C-m
tmux send-keys -t 1 "roscd multi_device_view/scripts/recognition" C-m
tmux send-keys -t 1 "rossetip" C-m
tmux send-keys -t 1 "roscd multi_device_view" C-m
tmux send-keys -t 1 "ipython3 -i -- servo_gear.py module_0"


tmux new-window

# attatch new session
tmux select-window -t upper_arm_recognition_rosbag:2
tmux split-window -v

tmux send-keys -t 0 "echo 0" C-m
tmux send-keys -t 0 "source ~/robothand_dataset_ws/devel/setup.bash bash" C-m
tmux send-keys -t 0 "roscd multi_device_view/scripts/recognition" C-m
tmux send-keys -t 0 "rossetip" C-m
tmux send-keys -t 0 "roscd multi_device_view" C-m
tmux send-keys -t 0 "ipython3 -i -- upper_arm_hole.py module_1"

tmux send-keys -t 1 "echo 1" C-m
tmux send-keys -t 1 "source ~/robothand_dataset_ws/devel/setup.bash bash" C-m
tmux send-keys -t 1 "roscd multi_device_view/scripts/recognition" C-m
tmux send-keys -t 1 "rossetip" C-m
tmux send-keys -t 1 "roscd multi_device_view/scripts/recognition" C-m
tmux send-keys -t 1 "ipython3 -i -- servo_gear.py module_1"


tmux new-window

# attatch new session
tmux select-window -t upper_arm_recognition_rosbag:3
tmux split-window -v

tmux send-keys -t 0 "echo 0" C-m
tmux send-keys -t 0 "source ~/robothand_dataset_ws/devel/setup.bash bash" C-m
tmux send-keys -t 0 "roscd multi_device_view/scripts/recognition" C-m
tmux send-keys -t 0 "rossetip" C-m
tmux send-keys -t 0 "roscd multi_device_view" C-m
tmux send-keys -t 0 "rosservice call /module_0/set_background"

tmux send-keys -t 1 "echo 1" C-m
tmux send-keys -t 1 "source ~/robothand_dataset_ws/devel/setup.bash bash" C-m
tmux send-keys -t 1 "roscd multi_device_view/scripts/recognition" C-m
tmux send-keys -t 1 "rossetip" C-m
tmux send-keys -t 1 "roscd multi_device_view/scripts/recognition" C-m
tmux send-keys -t 1 "rosservice call /module_1/set_background"


tmux new-window
tmux select-window -t upper_arm_recognition_rosbag:4
tmux split-window -v

tmux send-keys -t 0 "echo 0" C-m
tmux send-keys -t 0 "source ~/robothand_dataset_ws/devel/setup.bash bash" C-m
tmux send-keys -t 0 "roscd multi_device_view/launch/recognition"
# tmux send-keys -t 0 "emacs -nw " C-m
tmux send-keys -t 0 "rossetip" C-m
tmux send-keys -t 0 "roslaunch multi_device_view servo_gear.launch DEVICE:=module_0"

tmux send-keys -t 1 "echo 1" C-m
tmux send-keys -t 1 "source ~/robothand_dataset_ws/devel/setup.bash bash" C-m
tmux send-keys -t 1 "rossetip" C-m
tmux send-keys -t 1 "roscd multi_device_view/launch/recognition" C-m
tmux send-keys -t 1 "roslaunch multi_device_view upper_arm_hole.launch DEVICE:=module_0"
# tmux send-keys -t 1 "roslaunch multi_device_view manage_tf.launch" Cm-

tmux new-window
tmux select-window -t upper_arm_recognition_rosbag:5
tmux split-window -v

tmux send-keys -t 0 "echo 0" C-m
tmux send-keys -t 0 "source ~/robothand_dataset_ws/devel/setup.bash bash" C-m
tmux send-keys -t 0 "roscd multi_device_view/launch/recognition"
# tmux send-keys -t 0 "emacs -nw " C-m
tmux send-keys -t 0 "rossetip" C-m
tmux send-keys -t 0 "roslaunch multi_device_view servo_gear.launch DEVICE:=module_1"

tmux send-keys -t 1 "echo 1" C-m
tmux send-keys -t 1 "source ~/robothand_dataset_ws/devel/setup.bash bash" C-m
tmux send-keys -t 1 "rossetip" C-m
tmux send-keys -t 1 "roscd multi_device_view/launch/recognition" C-m
tmux send-keys -t 1 "roslaunch multi_device_view upper_arm_hole.launch DEVICE:=module_1"
# tmux send-keys -t 1 "roslaunch multi_device_view manage_tf.launch" Cm-

tmux new-window
tmux select-window -t upper_arm_recognition_rosbag:6
tmux split-window -v

tmux send-keys -t 0 "echo 0" C-m
tmux send-keys -t 0 "source ~/robothand_dataset_ws/devel/setup.bash bash" C-m
tmux send-keys -t 0 "roscd multi_device_view/sh/rosbag"
# tmux send-keys -t 0 "emacs -nw " C-m
tmux send-keys -t 0 "rossetip" C-m
tmux send-keys -t 0 "sh play_module_images.sh ~/rosbag/distributed/"

tmux send-keys -t 1 "echo 1" C-m
tmux send-keys -t 1 "source ~/robothand_dataset_ws/devel/setup.bash bash" C-m
tmux send-keys -t 1 "rossetip" C-m
tmux send-keys -t 1 "roscd multi_device_view/launch/recognition" C-m
tmux send-keys -t 1 ""
# tmux send-keys -t 1 "roslaunch multi_device_view manage_tf.launch" Cm-

tmux a
