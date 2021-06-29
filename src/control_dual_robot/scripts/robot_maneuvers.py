#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import time

from consts import upw, hor, dwd
from coord import Coord
from robot_phantomx_reactor import Robot


def moveP2P(robot, P1):
    #robot = Robot() # 4debug
    if not isinstance(P1, Coord): raise Exception("no coordinate given")
    old = robot.state
    robot.TCP = P1
    robot.ik()
    robot.publish()

    # wait time depends on distance
    t = 3 #debug
    wait(t)


def grip(robot):
    robot.gripper.close()
    robot.publish()
    wait()


def drop(robot):
    robot.gripper.open()
    robot.publish()
    wait()


def wait(t = 3):
    time.sleep(t)

