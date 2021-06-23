#!/usr/bin/env python
# license removed for brevity

import rospy
import math as m

from coord import Coord
#from joints import Joint_publisher
#from IK import inv_kinematics
from robot_phantomx_reactor import Robot

def BspProgROS():
    rp0 = Joint_publisher(0)
    rp1 = Joint_publisher(1)

    TCP = Coord(r = 150, z=350, th=0)

    while not rospy.is_shutdown():
        r0 = inv_kinematics('hor', TCP)
        r1 = inv_kinematics('hor', TCP)
        rp0.publish(r0)
        rp1.publish(r1)

def ImprovedIK():
    import time
    TCP = Coord(r=150, z=350, th=0, ort='hor')
    r0 = Robot(0)
    r1 = Robot(1)
    r0.TCP = TCP
    r1.TCP = TCP

    while not rospy.is_shutdown():
        r0.ik()
        r1.ik()

        r0.gripper.open()
        r1.gripper.close()
        time.sleep(2)

        r0.gripper.close()
        r1.gripper.open()
        time.sleep(2)

def debug():
    """ module testing etc"""
    #c1 = coord(x = 200, z = 100)
    #c1.cnv_cylindrical()
    #ist = joints()
    #soll = inv_kinematics('hor', ist, c1)

def rosnode():
    rospy.init_node('control_dual_robots_debug', anonymous=True)
    #BspProgROS()
    ImprovedIK()
    rospy.spin()

if __name__ == '__main__':
    rosnode()
    #debug()