#!/usr/bin/env python
# -*- coding: utf-8 -*-
# license removed for brevity

import rospy

from robot import Robot
from misc import wait
from joints import Joints
from math import pi
import cv2
import os


class Main(object):
    def __init__(self):
        rospy.init_node('pickup and scan')
        self.r0 = Robot(0)
        self.r1 = Robot(1)
        self.r0.home()
        self.r0.home()
        self.r0.home()
        self.r0.home()
        self.r1.home()
        self.r1.home()
        self.r1.home()
        self.r1.home()
        self.main()
        #self.cam = cv2.VideoCapture(0)
    
    def main(self):
        #while not rospy.is_shutdown():
            # self.demo()
            # self.control()
        self.pick_scan_cube(self.r1, self.r0)
    
    def demo(self):
        rospy.init_node('demo')
        #while not rospy.is_shutdown():
        self.r0.home() #p2p(self.r0.pos.home)
        self.r1.home() #p2p(self.r1.pos.home)
        print "3"
        wait(2.0)
        self.r0.home() #p2p(self.r0.pos.home)
        self.r1.home() #p2p(self.r1.pos.home)
        print "2"
        wait(2.0)
        self.r0.home() #p2p(self.r0.pos.home)
        self.r1.home() #p2p(self.r1.pos.home)
        print "1"
        wait(2.0)
        print 'start cycle'
        ## init vorbei

        self.r0.pickup()
        self.r0.handover(self.r1)
        self.r1.turn(self.r0,'F',1)
        self.r1.handover(self.r0)
        self.r0.turn(self.r1, 'U', 2)
        self.r0.handover(self.r1)
        self.r1.putdown()          

        print 'end cycle'
        wait(2.0)

    def control(self):
        rospy.init_node('control_dual_robots_manual')
        print 'init control'
        self.r0 = Robot(0)
        self.r1 = Robot(1)

        # () wait for start event

        self.r0.pickup()


        # put down cube
        if self.r0.hascube:  self.r0.putdown()
        else:           self.r1.putdown()

        rospy.spin()

    def pick_scan_cube(self, ra, rb):
        #rospy.init_node('pickup and scan')
        # pickup cube
        ra.pickup()
        rb.home()
        # scan 3 faces
        self.scan_half_cube(ra, rb)
        self.r0.p2p(ra.pos.home)
        ra.handover(rb)
        ra.home()
        self.scan_half_cube(rb, ra)
        # TODO ...
        
    def scan_half_cube(self, r, other):
        
        scan_a = Joints(0.0,
                        -35*pi/180, 
                        -5*pi/180, 
                        35*pi/180, 
                        0.0, 0.0)
        scan_b = Joints(0.0, 
                        10.0*pi/180, 
                        -5.0*pi/180, 
                        -98*pi/180, 
                        0.0, 0.0)
        
        a, b, c = None, None, None
        if r.id == 0:
            a = 'R'
            b = 'D'
            c = 'L'
        elif r.id == 1:
            a = 'F'
            b = 'U'
            c = 'B'
        
        r.angular(scan_a)
        self.take_image(r.id, face = a)
        
        r.angular(scan_b)
        wait(1)
        self.take_image(r.id, face = b)
        
        r.flip(other) #  flip cube 180° (to scan D side) # ! 2. iteration: timing fails on right side (r1)
        
        r.angular(scan_a)
        self.take_image(r.id, face = c)
        
    def take_image(self, rid, face):
        print "Taking image of {} Face".format(face)
        # opencv save img
        cam = cv2.VideoCapture(0)
        if cam.isOpened():
            print "    > cam open"
            ret, frame = cam.read()
            if ret:
                # correct img rotation
                if face == 'U':
                    frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
                elif face == 'R':
                    frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
                elif face == 'F':
                    frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
                elif face == 'D':
                    pass
                elif face == 'L':
                    frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
                elif face == 'B':
                    frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
                        
                path = "/home/student/catkin_ws/src/twophase_solver_ros/images/scan_" + 'Color.' + face + '.png'
                print "    > saving image to {}".format(path) 
                cv2.imwrite(path, frame)
            else: raise Exception("no image from camera received")
        else: raise Exception("could not open camera")
        
         


def debug():
    rospy.init_node('debug')
    #demo()
    rospy.spin()


if __name__ == '__main__':
    print "starting Main Program"
    m = Main()
    print "ending Main Program"