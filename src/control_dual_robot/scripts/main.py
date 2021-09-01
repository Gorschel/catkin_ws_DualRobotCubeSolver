#!/usr/bin/env python
# -*- coding: utf-8 -*-
# license removed for brevity

import rospy

from robot import Robot
from misc import wait



def demo():
    print 'init demo'
    r0 = Robot(0)
    r1 = Robot(1)

    while not rospy.is_shutdown():
        print 'start cycle'
        r0.home() #p2p(r0.pos.home)
        r1.home() #p2p(r1.pos.home)
        wait(1.0)

        #"""
        r0.pickup()
        r0.handover(r1)
        r0.turn(r1,'U',1)
        r1.putdown()
        #"""

        print 'end cycle'
        wait(2.0)

def control():
    rospy.init_node('control_dual_robots_manual')
    print 'init control'
    r0 = Robot(0)
    r1 = Robot(1)

    # () wait for start event

    # get cube
    r0.pickup()

    # make images


    # start detection


    # get solStr


    #parse_solStr()


    #plan_movements()


    #apply_solution()


    # put down cube
    if r0.hascube:  r0.putdown()
    else:           r1.putdown()

    rospy.spin()


def debug():
    rospy.init_node('debug')
    demo()
    rospy.spin()


if __name__ == '__main__':
    #control()
    debug()