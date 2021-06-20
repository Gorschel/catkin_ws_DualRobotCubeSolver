#!/usr/bin/env bash

### fkie_multimaster
# master 1
export ROS_MASTER_URI=http://localhost:11311
roscore --port 11311 >/dev/null 2>&1 &
# was auf erstem master laufen soll

# master 2
export ROS_MASTER_URI=http://localhost:11312
roscore --port 11312 >/dev/null 2>&1 &
# was auf zweitem master laufen soll

### fkie_sync
# master 1
export ROS_MASTER_URI=http://localhost:11311
rosrun fkie_master_discovery master_discovery >/dev/null 2>&1 &
rosrun fkie_master_sync master_sync >/dev/null 2>&1 &
# master 2
export ROS_MASTER_URI=http://localhost:11312
rosrun fkie_master_discovery master_discovery >/dev/null 2>&1 &
rosrun fkie_master_sync master_sync >/dev/null 2>&1 &

