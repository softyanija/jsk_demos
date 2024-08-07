#!/bin/bash

SESSION_NAME="memory_insertion"

# Check if session already exists
tmux has-session -t $SESSION_NAME 2>/dev/null

if [ $? != 0 ]; then
    # Create new session
    tmux new-session -d -s $SESSION_NAME

    # Split windows
    tmux split-window -h -t $SESSION_NAME:0
    tmux split-window -v -t $SESSION_NAME:0.0
    tmux split-window -v -t $SESSION_NAME:0.2

    # Send commands to each pane
    tmux send-keys -t $SESSION_NAME:0.0 "echo 0" C-m
    tmux send-keys -t $SESSION_NAME:0.0 "rossetmaster pr1012" C-m
    tmux send-keys -t $SESSION_NAME:0.0 "rossetip" C-m
    tmux send-keys -t $SESSION_NAME:0.0 "source ~/pr2_ws/devel/setup.bash" C-m
    tmux send-keys -t $SESSION_NAME:0.0 "roscd memory_insertion/script/setup" C-m
    tmux send-keys -t $SESSION_NAME:0.0 "bash bind_cameras.bash" C-m

    tmux send-keys -t $SESSION_NAME:0.1 "echo 1" C-m
    tmux send-keys -t $SESSION_NAME:0.1 "rossetmaster pr1012" C-m
    tmux send-keys -t $SESSION_NAME:0.1 "rossetip" C-m
    tmux send-keys -t $SESSION_NAME:0.1 "source ~/pr2_ws/devel/setup.bash" C-m
    tmux send-keys -t $SESSION_NAME:0.1 "roscd memory_insertion/launch/recognition" C-m

    tmux send-keys -t $SESSION_NAME:0.2 "echo 2" C-m
    tmux send-keys -t $SESSION_NAME:0.2 "rossetmaster pr1012" C-m
    tmux send-keys -t $SESSION_NAME:0.2 "rossetip" C-m
    tmux send-keys -t $SESSION_NAME:0.2 "source ~/pr2_ws/devel/setup.bash" C-m
    tmux send-keys -t $SESSION_NAME:0.2 "roscd memory_insertion/launch/recognition" C-m
    tmux send-keys -t $SESSION_NAME:0.2 "roslaunch memory_insertion recog_cam1.launch" C-m

    tmux send-keys -t $SESSION_NAME:0.3 "echo 3" C-m
    tmux send-keys -t $SESSION_NAME:0.3 "rossetmaster pr1012" C-m
    tmux send-keys -t $SESSION_NAME:0.3 "rossetip" C-m
    tmux send-keys -t $SESSION_NAME:0.3 "source ~/pr2_ws/devel/setup.bash" C-m
    tmux send-keys -t $SESSION_NAME:0.3 "roscd memory_insertion/launch/recognition" C-m
    tmux send-keys -t $SESSION_NAME:0.3 "roslaunch memory_insertion recog_cam2.launch" C-m

    tmux new-window

    # attatch new session
    tmux select-window -t $SESSION_NAME:1

    # Split windows
    tmux split-window -h -t $SESSION_NAME:1
    tmux split-window -v -t $SESSION_NAME:1.0
    tmux split-window -v -t $SESSION_NAME:1.2

    tmux send-keys -t $SESSION_NAME:1.0 "echo 0" C-m
    tmux send-keys -t $SESSION_NAME:1.0 "rossetmaster pr1012" C-m
    tmux send-keys -t $SESSION_NAME:1.0 "rossetip" C-m
    tmux send-keys -t $SESSION_NAME:1.0 "source ~/pr2_ws/devel/setup.bash" C-m
    tmux send-keys -t $SESSION_NAME:1.0 "roscd memory_insertion/script/recognitoin" C-m
    # tmux send-keys -t $SESSION_NAME:1.0 "ipython3 -i -- set_camera_tf.py" C-m

    tmux send-keys -t $SESSION_NAME:1.1 "echo 1" C-m
    tmux send-keys -t $SESSION_NAME:1.1 "rossetmaster pr1012" C-m
    tmux send-keys -t $SESSION_NAME:1.1 "rossetip" C-m
    tmux send-keys -t $SESSION_NAME:1.1 "source ~/pr2_ws/devel/setup.bash" C-m
    tmux send-keys -t $SESSION_NAME:1.1 "roscd memory_insertion/script/demo" C-m
    tmux send-keys -t $SESSION_NAME:1.1 "ipython3 -i -- main.py"

    tmux send-keys -t $SESSION_NAME:1.2 "echo 2" C-m
    tmux send-keys -t $SESSION_NAME:1.2 "rossetmaster pr1012" C-m
    tmux send-keys -t $SESSION_NAME:1.2 "rossetip" C-m
    tmux send-keys -t $SESSION_NAME:1.2 "source ~/pr2_ws/devel/setup.bash" C-m
    tmux send-keys -t $SESSION_NAME:1.2 "roscd memory_insertion" C-m
#    tmux send-keys -t $SESSION_NAME:1.2 "roslaunch multi_device_view pointcloud_processing.launch" C-m

    tmux send-keys -t $SESSION_NAME:1.3 "echo 3" C-m
    tmux send-keys -t $SESSION_NAME:1.3 "rossetmaster pr1012" C-m
    tmux send-keys -t $SESSION_NAME:1.3 "rossetip" C-m
    tmux send-keys -t $SESSION_NAME:1.3 "source ~/pr2_ws/devel/setup.bash" C-m
#    tmux send-keys -t $SESSION_NAME:1.3 "roscd memory_insertion" C-m
    
fi

# Attach to the session
tmux attach -t $SESSION_NAME
