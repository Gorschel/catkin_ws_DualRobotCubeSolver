#!/usr/bin/env python
# -*- coding: utf-8 -*-
# license removed for brevity

import rospy

from consts import *
from coord import Coord
from robot import Robot


def ik_demo():
    # get robot objs
    r0 = Robot(0)
    r1 = Robot(1)
    """
    # debug points
    p0 = Coord(r=150, z=450, ort=upw)
    p1 = Coord(x=130, y=-130, z=100, ort=dwd)
    r0.p2p(p0)
    """
    # goto home
    #r0.p2p(r0.pos.home)
    #r1.p2p(r1.pos.home)

    """
    # wrist rotate an basis anpassen bzw. parallel zu y achse greifen
    #r1.state.q4 = r1.state.q0
    """

    while not rospy.is_shutdown():

        r0.p2p(r0.pos.home)
        r1.p2p(r1.pos.home)

        # transfer coords working?
        #r1.pickup()
        #r1.handover(r0)
        #r0.putdown()

def rosnode():
    rospy.init_node('control_dual_robots_manual', anonymous=True)
    ik_demo()
    rospy.spin()


def debug():
    """ module testing etc"""
    pass


if __name__ == '__main__':
    rosnode()
    #debug()