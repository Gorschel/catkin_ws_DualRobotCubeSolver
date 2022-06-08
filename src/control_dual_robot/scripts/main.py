#!/usr/bin/env python
# -*- coding: utf-8 -*-
# license removed for brevity
import random

import rospy
import rospkg

from robot import Robot
from misc import wait
from joints import Joints
import cv2
from math import pi
from misc import flip_dict_values


class Main(object):
    def __init__(self):
        rospy.init_node('control')
        self.r0 = Robot(0)
        self.r1 = Robot(1)

        self.r0.home()  # p2p(self.r0.pos.home)
        self.r1.home()  # p2p(self.r1.pos.home)
        print "3"
        wait(2.0)
        self.r0.home()  # p2p(self.r0.pos.home)
        self.r1.home()  # p2p(self.r1.pos.home)
        print "2"
        wait(2.0)
        self.r0.home()  # p2p(self.r0.pos.home)
        self.r1.home()  # p2p(self.r1.pos.home)
        print "1"
        wait(2.0)
        print 'start'

        self.main()

        print 'end cycle'
        self.r0.home()
        self.r1.home()

    def main(self):
        ## demo
        # self.demo_handover(self.r0, self.r1)
        self.demo_turning(self.r0, self.r1)
        # self.demo_speed(self.r0, self.r1)

        ## scan
        #self.pick_scan_cube(self.r0, self.r1)

        ## analyze & apply
        #solution = "U1 R2 F3 D3 L2 B1 (20 moves)"
        #self.analyze_solution(self.r0, self.r1, solution)

    def demo_handover(self, ra, rb):
        ra.pickup()
        for i in range(5):
            print "Durchlauf: {}".format(i)
            ra.handover(rb)
            rb.handover(ra)
        ra.putdown()

        # TODO: Lösung anwenden (iter jedes zeichen)
        #  - roboter hält würfel, weiß welche ("ursprungs")-flächen blockiert sind
        #  - ist nächste drehung machbar?  ja: dreh a,b,c  nein: übergabe

    def demo_turning(self, ra, rb):
        ra.pickup()
        # ra validation test loop:
        #ra.turn(rb, 'b', 2)
        for face in ['a', 'b', 'c']:
            for n in range(-1, 3):
                if n is 0:
                    continue
                ra.turn(rb, face, n)
        ra.putdown()

    def demo_speed(self, ra, rb):
        ra.angular(Joints(0, -pi/2, -pi / 2, 0, 0, 0))
        ra.angular(Joints(0, -pi/2, pi / 2, 0, 0, 0))

    def analyze_solution(self, ra, rb, solution):
        """
        ra(Robot):test
        """
        # blocking table
        ra_turns = {'R': 'a',
                    'D': 'b',
                    'L': 'c'}
        rb_turns = {'F': 'a',
                    'U': 'b',
                    'B': 'c'}
        solution = solution[:solution.find('(') - 1]  # cut postfix
        maneuvers = solution.split(' ')
        for maneuver in maneuvers:
            letter = maneuver[:1]
            n = int(maneuver[1:])

            # 3 clockwise turns are equal to 1 counter-clockwise turn
            if n == 3:
                n = -1

            # pick up if not already
            if not ra.hascube and not rb.hascube:
                x = random.randint(0, 1)
                if x == 0:
                    ra.pickup()
                    rb_turns = flip_dict_values(rb_turns)
                else:
                    rb.pickup()
                    ra_turns = flip_dict_values(ra_turns)

            # check if blocked & turn face
            if letter in ra_turns.keys():
                if rb.hascube:
                    rb.handover(ra)
                if ra.hascube:
                    face = ra_turns[letter]
                    ra.turn(rb, face, n)
            if letter in rb_turns:
                if ra.hascube:
                    ra.handover(rb)
                if rb.hascube:
                    face = rb_turns[letter]
                    rb.turn(ra, face, n)

        # put cube back down
        if ra.hascube:
            ra.putdown()
        elif rb.hascube:
            rb.putdown()

    def pick_scan_cube(self, ra, rb):
        # pickup cube
        ra.pickup()
        rb.home()

        # scan first 3 faces
        self.scan_half_cube(ra, rb)
        ra.p2p(ra.pos.home)

        # transfer
        ra.handover(rb)
        ra.home()

        # second half
        self.scan_half_cube(rb, ra)

        rb.putdown()
        rb.home()

    def scan_half_cube(self, r, other):
        # scanning poses (joint values)
        scan_a = Joints(0.0, -35 * pi / 180, -5 * pi / 180, 35 * pi / 180, 0.0, 0.0)
        scan_b = Joints(0.0, 10.0 * pi / 180, -5.0 * pi / 180, -98 * pi / 180, 0.0, 0.0)

        a, b, c = None, None, None
        if r.id == 0:
            a, b, c = 'R', 'D', 'L'
        elif r.id == 1:
            a, b, c = 'F', 'U', 'B'

        # begin scanning
        r.angular(scan_a)
        self.take_image(r.id, face=a)

        r.angular(scan_b)
        wait(1)
        self.take_image(r.id, face=b)

        r.flip(other)  # flip cube 180° (to scan D side) # ! 2. iteration: timing fails on right side (r1)

        r.angular(scan_a)
        self.take_image(r.id, face=c)

    def take_image(self, rid, face):
        print "Taking image of {} Face".format(face)
        rospack = rospkg.RosPack()
        cam = cv2.VideoCapture(0)

        # cam.set(cv2.CAP_PROP_EXPOSURE, 0.1)
        if cam.isOpened():
            print "    > cam open"
            ret, frame = cam.read()
            if ret:
                # cv2.imshow("img "+face, frame)
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

                fpath = rospack.get_path('twophase_solver_ros') + '/images/scan_' + 'Color.' + face + '.png'
                print "    > saving image to {}".format(fpath)
                cv2.imwrite(fpath, frame)
            else:
                raise Exception("no image from camera received")
            cam.release()
        else:
            print("! could not open camera")


if __name__ == '__main__':
    print "starting Main Program"
    m = Main()
    print "ending Main Program"
