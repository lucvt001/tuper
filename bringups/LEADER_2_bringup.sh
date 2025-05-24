#!/bin/bash

SESSION="tuper"

# Start a new tmux session without running anything yet
tmux -2 new-session -d -s $SESSION

# === Window 0: MAVLink Docker Compose ===
tmux rename-window -t $SESSION:0 'mav'
tmux send-keys -t $SESSION:0 "cd ~/repo/waraps-arduagent" C-m
tmux send-keys -t $SESSION:0 "docker compose up" C-m

# === Window 1: TUPER Quadrant Launch ===
tmux new-window -t $SESSION:1 -n 'ros2'

# Split the window into 4 quadrants
tmux select-window -t $SESSION:1
tmux split-window -h
tmux split-window -v
tmux select-pane -t 0
tmux split-window -v

# Send commands to each pane
tmux select-pane -t 0
tmux send-keys "cd ~ && . start_tuper_container.sh" C-m
tmux send-keys "ros2 launch arduagent rover_bringup.launch.py ns:=leader2 is_leader:=False mqtt_params_file:=src/tuper/arduagent/config/mqtt_params_leader2.yaml" C-m

tmux select-pane -t 1
tmux send-keys "cd ~ && . start_tuper_container.sh" C-m
tmux send-keys "" C-m

tmux select-pane -t 2
tmux send-keys "cd ~ && . start_tuper_container.sh" C-m
tmux send-keys "" C-m

tmux select-pane -t 3
tmux send-keys "cd ~ && . start_tuper_container.sh" C-m
tmux send-keys "" C-m

# === Window 2: Backup A ===
tmux new-window -t $SESSION:2 -n "topic monitor"
# Split the window into 4 quadrants
tmux select-window -t $SESSION:2
tmux split-window -h
tmux split-window -v
tmux select-pane -t 0
tmux split-window -v

# Send commands to each pane
tmux select-pane -t 0
tmux send-keys "cd ~ && . start_tuper_container.sh" C-m
tmux send-keys "" C-m

tmux select-pane -t 1
tmux send-keys "cd ~ && . start_tuper_container.sh" C-m
tmux send-keys "" C-m

tmux select-pane -t 2
tmux send-keys "cd ~ && . start_tuper_container.sh" C-m
tmux send-keys "" C-m

tmux select-pane -t 3
tmux send-keys "cd ~ && . start_tuper_container.sh" C-m
tmux send-keys "" C-m

# === Window 3: Backup B ===
tmux new-window -t $SESSION:3 -n 'RaPi'

# === Start in the Quadrants window ===
tmux select-window -t $SESSION:1
tmux select-pane -t 0

# === Attach to the session ===
tmux -2 attach-session -t $SESSION
