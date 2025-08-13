#!/bin/bash

# Build and source workspace
source /opt/ros/noetic/setup.bash
#catkin_make
source devel/setup.bash
rm -rf /root/code/datasets/fastlivo2/output/Log

# --- Config ---
SESSION="fastlivo2_ros_session"

CONFIG_FILE="avia.yaml" # avia.yaml or HILTI22.yaml

CONFIG_DIR="/root/code/catkin_ws/src/FAST-LIVO2/config"
CONFIG_FILE="${CONFIG_DIR}/${CONFIG_FILE}"

ROS_BAG=$(yq '.dataset.ros_bag' $CONFIG_FILE)
LAUNCH_FILE=$(yq '.launch_file' $CONFIG_FILE)
OUTPUT_DIR=$(yq '.path.output_dir' $CONFIG_FILE)
COLMAP_DIR="${OUTPUT_DIR}Log/Colmap/"

TARGET_DIRS=(
    "${COLMAP_DIR}images"
    "${COLMAP_DIR}sparse/0"
)

# Command to run fastlivo2 mapping node + RViz + republish
LAUNCH_FASTLIVO2="roslaunch fast_livo $LAUNCH_FILE"

# Command to play rosbag
PLAY_ROSBAG="rosbag play $ROS_BAG"

ECHO_COMMANDS="echo 'CONFIG_DIR: $CONFIG_DIR'; \
echo 'CONFIG_FILE: $CONFIG_FILE'; \
echo 'ROS_BAG: $ROS_BAG'; \
echo 'OUTPUT_DIR: $OUTPUT_DIR'; \
echo 'Play ROSBAG Command: $PLAY_ROSBAG'; \
echo 'LAUNCH_FASTLIVO2 Command: $LAUNCH_FASTLIVO2'; \
bash"

for dir in "${TARGET_DIRS[@]}"; do
    if [ -d "$dir" ]; then
        rm -rf "$dir"
        echo "Removed: $dir"
    else
        echo "Not found: $dir"
    fi
done

for dir in "${TARGET_DIRS[@]}"; do
    if [ ! -d "$dir" ]; then
        mkdir -p "$dir"
        echo "Created: $dir"
    else
        echo "Exists: $dir"
    fi
done

# Create tmux session if it doesn't exist
tmux has-session -t $SESSION 2>/dev/null
if [ $? != 0 ]; then
    # Step 1: Create a new tmux session and run rosbag in the first pane
    tmux new-session -d -s $SESSION -n "ROS"

    # Step 2: Split the window into 4 panes (2x2 grid)
    tmux split-window -h  # Split horizontally
    tmux split-window -v  # Split the left pane vertically
    tmux select-pane -t 0  # Move focus to the first pane (top-left)
    tmux split-window -v  # Split the right pane vertically

    tmux send-keys -t 0 "$PLAY_ROSBAG" C-m # Top-left pane
    tmux send-keys -t 1 "$LAUNCH_FASTLIVO2" C-m # Top-right pane
    tmux send-keys -t 2 "$ECHO_COMMANDS" C-m # Bottom-left pane

    # Step 3: Attach to session
    tmux attach-session -t $SESSION
else
    echo "Session $SESSION already exists. Attaching to it."
    tmux attach-session -t $SESSION
fi
