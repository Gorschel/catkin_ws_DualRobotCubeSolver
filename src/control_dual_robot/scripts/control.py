#!/usr/bin/env python
# -*- coding: utf-8 -*-
# license removed for brevity

import rospy
import time

from consts import upw, hor, dwd
from coord import Coord
from robot_phantomx_reactor import Robot


def close(robot):
    robot.gripper.close()
    robot.publish()
    wait()

def open(robot):
    robot.gripper.open()
    robot.publish()
    wait()

def wait():
    time.sleep(3)


def IKdemo():
    import time
    TCP0 = Coord(r=150, z=350, th=0, ort=hor)
    TCP1 = Coord(x=150, y=-150, z=100, ort=dwd)
    r0 = Robot(0)
    r1 = Robot(1)
    r0.TCP = TCP0
    r1.TCP = TCP1
    r1.state.q4 = r1.state.q0
    r1.publish()

    while not rospy.is_shutdown():
        open(r0)
        





def rosnode():
    rospy.init_node('control_dual_robots_manual', anonymous=True)
    IKdemo()
    rospy.spin()


def debug():
    """ module testing etc"""
    pass


if __name__ == '__main__':
    rosnode()
    #debug()