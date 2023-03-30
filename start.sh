#!/bin/bash

gnome-terminal --title=Rubics Cube Robot Solver \\
               -- bash -c "roscore; bash"

# Define the commands to run in an array
commands=("roslaunch control_dual_robot robots.launch" \
          "conda activate masterprojekt; roslaunch control_dual_robot solver.launch" \
          "roslaunch control_dual_robot control.launch")

# Loop through the array and run each command in a new tab
for cmd in "${commands[@]}"
do
gnome-terminal --tab --title=XXX -e "$cmd; bash"
sleep 1
done
