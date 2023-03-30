# Requirements

- v4l2 (v4l-utils)
- ROS melodic
- python 2.7
  - cv2
  - numpy

# How to run
1. Preparations
   * use terminal commands from the repository's base directory `catkin_ws`
   * build catkin workspace if needed 
   * use `$ conda activate masterprojekt` for the Terminal / Tabs if needed
2. `$ roscore`
3. `$ roslaunch control_dual_robot run.launch `
   * or manually launch each package:
     + robot_0.launch
     + robot_1.launch
     + (solver.launch)
     + control.launch
4. python run `src/control_dual_robot/scripts/run.py` (either in your IDE or from conda activated Terminal)
   + or call a specific ROS action by replacing `GOAL_CODE` with any of the definitions in 

     `src/control_dual_robot/scripts/control.py` inside `Control.execute`
     ``` python
     import rospy
     rospy.init_node('control_client')
     client = actionlib.SimpleActionClient('control', ControlAction)
     client.wait_for_server()
     goal = ControlGoal('GOAL_CODE')
     client.send_goal(goal)
     ```
