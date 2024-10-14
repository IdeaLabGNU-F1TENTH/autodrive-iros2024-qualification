#!/bin/bash

# tmux 세션 이름 설정
SESSION_NAME="ros_session"

# tmux 세션이 이미 존재하는지 확인하고 없으면 생성
tmux has-session -t $SESSION_NAME 2>/dev/null

if [ $? != 0 ]; then
    # 새로운 세션 생성 (-d 옵션은 백그라운드에서 실행)
    tmux new-session -d -s $SESSION_NAME -n "simulator"
    
    # 첫 번째 창에서 ROS 환경 설정 및 launch 명령어 실행
    tmux send-keys -t $SESSION_NAME "source /opt/ros/foxy/setup.bash" C-m
    tmux send-keys -t $SESSION_NAME "source \$HOME/auto_api_ws/install/setup.bash" C-m
    tmux send-keys -t $SESSION_NAME "ros2 launch autodrive_f1tenth simulator_bringup_headless.launch.py" C-m
    
    # 두 번째 창 생성 및 ROS 환경 설정 및 run 명령어 실행
    tmux new-window -t $SESSION_NAME:1 -n "reactive_node"
    tmux send-keys -t $SESSION_NAME:1 "source /opt/ros/foxy/setup.bash" C-m
    tmux send-keys -t $SESSION_NAME:1 "source \$HOME/auto_api_ws/install/setup.bash" C-m
    tmux send-keys -t $SESSION_NAME:1 "ros2 run gap_follow reactive_node.py" C-m
    
    # 세션을 시작하고 첫 번째 창으로 이동
    tmux select-window -t $SESSION_NAME:0
    tmux attach-session -t $SESSION_NAME
else
    echo "tmux 세션 '$SESSION_NAME'이 이미 존재합니다."
    tmux attach-session -t $SESSION_NAME
fi

