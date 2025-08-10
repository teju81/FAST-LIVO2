#!/bin/bash

# Build and source workspace
source /opt/ros/noetic/setup.bash
#catkin_make
source devel/setup.bash
rm -rf /root/code/datasets/fastlivo2/output/Log

# --- Config ---
SESSION="fastlivo2_ros_session"
ROS_BAG=${1:-"/root/code/datasets/fastlivo2/Retail_Street.bag"}
#ROS_BAG=${1:-"/root/code/datasets/hilti/2022/exp01_construction_ground_level.bag"}

# Command to run fastlivo2 mapping node + RViz + republish
LAUNCH_FASTLIVO2="roslaunch fast_livo mapping_avia.launch"
#LAUNCH_FASTLIVO2="roslaunch fast_livo mapping_hesaixt32_hilti22.launch"
# Command to play rosbag
PLAY_ROSBAG="rosbag play $ROS_BAG"

# Create tmux session if it doesn't exist
tmux has-session -t $SESSION 2>/dev/null
if [ $? != 0 ]; then
    # Step 1: Create a new tmux session and run rosbag in the first pane
    tmux new-session -d -s $SESSION -n "ROS"

    # Step 2: Split the window into 4 panes (2x2 grid)
    tmux split-window -h  # Split horizontally

    tmux send-keys -t 0 "$PLAY_ROSBAG" C-m # left pane
    tmux send-keys -t 1 "$LAUNCH_FASTLIVO2" C-m # right pane

    # Step 3: Attach to session
    tmux attach-session -t $SESSION
else
    echo "Session $SESSION already exists. Attaching to it."
    tmux attach-session -t $SESSION
fi
