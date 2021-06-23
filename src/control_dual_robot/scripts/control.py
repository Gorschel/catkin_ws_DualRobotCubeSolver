#!/usr/bin/env python
# license removed for brevity

import rospy
import math as m

from coord import coord
from joints import joint_publisher
from IK import inv_kinematics

def BspProgROS():
    rp0 = joint_publisher(0)
    rp1 = joint_publisher(1)
    TCP = coord(r = 150, z=350, th=0)

    while not rospy.is_shutdown():
        r0 = inv_kinematics('hor', TCP)
        r1 = inv_kinematics('hor', TCP)
        rp0.publish(r0)
        rp1.publish(r1)

def debug():
    """ module testing etc"""
    #c1 = coord(x = 200, z = 100)
    #c1.cnv_cylindrical()
    #ist = joints()
    #soll = inv_kinematics('hor', ist, c1)

def rosnode():
    rospy.init_node('control_dual_robots_debug', anonymous=True)
    BspProgROS()
    rospy.spin()

if __name__ == '__main__':
    rosnode()
    #debug()