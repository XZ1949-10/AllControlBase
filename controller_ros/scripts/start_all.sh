#!/bin/bash
# TurtleBot + ViNT 一键启动脚本
# 使用 tmux 分屏，手动控制启动顺序

SESSION_NAME="turtlebot"

# 如果 session 已存在，先关闭
tmux kill-session -t $SESSION_NAME 2>/dev/null

# 创建新 session
tmux new-session -d -s $SESSION_NAME -n main

# 窗格 0 (左上): ViNT
tmux send-keys -t $SESSION_NAME:0.0 "cd ~/visualnav-transformer/deployment/src && sudo modprobe gspca_kinect && roslaunch vint_locobot.launch"

# 水平分割，窗格 1 (右上): explore_new.py
tmux split-window -h -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME:0.1 "cd ~/visualnav-transformer/deployment/src && python explore_new.py"

# 垂直分割窗格 0，窗格 2 (左下): controller_ros
tmux select-pane -t $SESSION_NAME:0.0
tmux split-window -v -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME:0.2 "roslaunch controller_ros turtlebot1.launch"

# 垂直分割窗格 1，窗格 3 (右下): trajectory_publisher
tmux select-pane -t $SESSION_NAME:0.1
tmux split-window -v -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME:0.3 "rosrun controller_ros trajectory_publisher.py"

# 设置窗格布局为均匀分布
tmux select-layout -t $SESSION_NAME tiled

# 回到第一个窗格
tmux select-pane -t $SESSION_NAME:0.0

echo "=========================================="
echo "  命令已准备好，按回车执行"
echo "  窗格切换: Ctrl+B 然后方向键"
echo "=========================================="
echo "  1. 左上: ViNT (先执行)"
echo "  2. 左下: controller_ros"
echo "  3. 右上: explore_new.py"  
echo "  4. 右下: trajectory_publisher"
echo "=========================================="

# 附加到 session
tmux attach-session -t $SESSION_NAME
