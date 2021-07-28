#!/usr/bin/env python
# -*- coding: utf-8 -*-
# license removed for brevity

import rospy

from robot import Robot
from misc import wait


def demo():
    print 'init'
    r0 = Robot(0)
    r1 = Robot(1)

    while not rospy.is_shutdown():
        print 'start cycle'
        r0.p2p(r0.pos.home)
        r1.p2p(r1.pos.home)

        # transfer cube multiple times
        r0.pickup()
        r0.handover(r1)
        #r1.handover(r0)
        #r0.handover(r1)
        r1.putdown()

        r0.p2p(r0.pos.home)
        r1.p2p(r1.pos.home)
        wait(5.0)
        print 'end cycle'

def rosnode():
    rospy.init_node('control_dual_robots_manual', anonymous=True)
    demo()
    rospy.spin()


def debug():
    """ module testing etc"""
    pass


if __name__ == '__main__':
    rosnode()
    #debug()