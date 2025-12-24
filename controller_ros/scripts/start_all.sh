#!/bin/bash
# TurtleBot + ViNT 一键启动脚本
# 使用 tmux 分屏启动所有节点

SESSION_NAME="turtlebot"

# 如果 session 已存在，先关闭
tmux kill-session -t $SESSION_NAME 2>/dev/null

# 创建新 session，第一个窗格运行 ViNT
tmux new-session -d -s $SESSION_NAME -n main

# 窗格 0: ViNT (cd + modprobe + roslaunch)
tmux send-keys -t $SESSION_NAME "cd ~/visualnav-transformer/deployment/src && sudo modprobe gspca_kinect && roslaunch vint_locobot.launch" C-m

# 水平分割，窗格 1: explore_new.py
tmux split-window -h -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME "sleep 5 && cd ~/visualnav-transformer/deployment/src && python explore_new.py" C-m

# 垂直分割窗格 0，窗格 2: controller_ros
tmux select-pane -t $SESSION_NAME:0.0
tmux split-window -v -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME "sleep 3 && roslaunch controller_ros turtlebot1.launch" C-m

# 垂直分割窗格 1，窗格 3: trajectory_publisher
tmux select-pane -t $SESSION_NAME:0.1
tmux split-window -v -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME "sleep 8 && rosrun controller_ros trajectory_publisher.py" C-m

# 设置窗格布局为均匀分布
tmux select-layout -t $SESSION_NAME tiled

# 附加到 session
tmux attach-session -t $SESSION_NAME
