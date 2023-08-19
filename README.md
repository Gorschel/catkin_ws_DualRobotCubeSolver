# Requirements
- v4l2 (v4l-utils)
- ROS melodic
- ROS packages:
  * http://wiki.ros.org/arbotix
  * http://wiki.ros.org/dynamixel_motor
  * http://wiki.ros.org/phantomx_reactor_arm
  * 
  
- python 2.7
  * cv2
  * numpy

# Preparations / Building
   * use terminal commands from the repository's base directory `catkin_ws`
   * build catkin workspace if needed

# Execution
1. open Terminal CTRL + ALT + T
2. (recommended) open a new Tab for each step using CTRL + SHIFT + T or the "right-click-context-menu"
3. (recommended) execute `$ roscore`. This ensures we don't accidentally kill it when it's attached to a launch.
4. launch modules:
    - (not recommended) launch everything together: `$ roslaunch control_dual_robot all_at_once.launch`
    - (recommended) launch each module separately (again in new tab each): 
         + `$ roslaunch control_dual_robot robot_0.launch`
         + `$ roslaunch control_dual_robot robot_1.launch`
         + `$ conda activate masterprojekt && roslaunch twophase_solver_ros solver.launch`
         + `$ conda activate masterprojekt && roslaunch control_dual_robot control.launch`

5. select action from the dropdown and click on send goal