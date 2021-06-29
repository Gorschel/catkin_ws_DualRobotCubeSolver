#!/usr/bin/env python
# -*- coding: utf-8 -*-
# license removed for brevity

import rospy

from consts import upw, hor, dwd, homepos, cubepos
from coord import Coord
from robot_phantomx_reactor import Robot
from robot_maneuvers import *

def pickup(robot):
    moveP2P(robot, cubepos)


def IKdemo():
    # get robot obj
    r0 = Robot(0)
    r1 = Robot(1)
    # create/set points
    P0 = Coord(r=150, z=350, ort=hor)
    P1 = Coord(x=130, y=-130, z=100, ort=dwd)
    r0.TCP = P0
    r1.TCP = P1
    # calc joint states
    r0.ik()
    r1.ik()
    r1.state.q4 = r1.state.q0 # parallel zu y achse greifen

    #moveP2P(r1, cubepos + Coord(z=100))

    while not rospy.is_shutdown():
        p3 = cubepos + Coord(z=75)
        moveP2P(r1, p3)
        pickup(r1)
        grip(r1)
        moveP2P(r1, p3)
        drop(r1)
        moveP2P(r1, homepos)


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