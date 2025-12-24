#!/bin/bash
# TurtleBot + ViNT 一键启动脚本

SESSION="turtlebot"

tmux kill-session -t $SESSION 2>/dev/null

tmux new-session -d -s $SESSION

# 创建 4 个窗格
tmux split-window -t $SESSION
tmux split-window -t $SESSION
tmux split-window -t $SESSION

# 强制 tiled 布局 (四方格)
tmux select-layout -t $SESSION tiled

# 发送命令到各窗格 (不执行，等用户按回车)
tmux send-keys -t ${SESSION}:0.0 'cd ~/visualnav-transformer/deployment/src && sudo modprobe gspca_kinect && roslaunch vint_locobot.launch'
tmux send-keys -t ${SESSION}:0.1 'cd ~/visualnav-transformer/deployment/src && python explore_new.py'
tmux send-keys -t ${SESSION}:0.2 'roslaunch controller_ros turtlebot1.launch'
tmux send-keys -t ${SESSION}:0.3 'rosrun controller_ros trajectory_publisher.py'

tmux select-pane -t ${SESSION}:0.0

echo "四方格布局，Ctrl+B+方向键切换，按回车执行"
tmux attach -t $SESSION
