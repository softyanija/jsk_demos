#!/bin/bash

# launch new session
tmux new-session -d -s laptop_dev

# horizontal split of two panels
tmux split-window -v

# move to panel 1, and vertical split
# tmux select-pane -t 1
# tmux split-window -v

# move to panel 3, and vertical split
# tmux select-pane -t 0
# tmux split-window -v

tmux send-keys -t 0 "echo 0" C-m
tmux send-keys -t 0 "source ~/robothand_dataset_ws/devel/setup.bash " C-m
tmux send-keys -t 0 "rossetmaster pr1012" C-m
tmux send-keys -t 0 "rossetip" C-m
tmux send-keys -t 0 "roscd multi_device_view" C-m

tmux send-keys -t 1 "echo 1" C-m
tmux send-keys -t 1 "source ~/robothand_dataset_ws/devel/setup.bash " C-m
tmux send-keys -t 1 "rossetmaster pr1012" C-m
tmux send-keys -t 1 "rossetip" C-m
tmux send-keys -t 1 "roscd multi_device_view" C-m


tmux new-window

# attatch new session
tmux select-window -t laptop_dev:1

tmux split-window -v

tmux send-keys -t 0 "echo 0" C-m
tmux send-keys -t 0 "source ~/robothand_dataset_ws/devel/setup.bash " C-m
tmux send-keys -t 0 "rossetmaster pr1012" C-m
tmux send-keys -t 0 "rossetip" C-m
tmux send-keys -t 0 "roscd multi_device_view" C-m

tmux send-keys -t 1 "echo 1" C-m
tmux send-keys -t 1 "source ~/robothand_dataset_ws/devel/setup.bash " C-m
tmux send-keys -t 1 "rossetmaster pr1012" C-m
tmux send-keys -t 1 "rossetip" C-m
tmux send-keys -t 1 "roscd multi_device_view" C-m

tmux a
