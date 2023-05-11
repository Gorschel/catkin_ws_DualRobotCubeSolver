#!/usr/bin/env bash

# Define command list
command_list=(
  "roscore"
  "roslaunch control_dual_robot robot_0.launch"
  "roslaunch control_dual_robot robot_1.launch"
  "conda activate masterprojekt && roslaunch twophase_solver_ros solver.launch"
  "roslaunch control_dual_robot control.launch"
)

# Open new terminal window with first command
gnome-terminal -- bash -c "${command_list[0]}; exec bash"

# Loop through remaining commands and execute each in a new tab of the same window
for ((i=1; i<${#command_list[@]}; i++)); do
  gnome-terminal --tab -- bash -c "${command_list[i]}; exec bash"
done

