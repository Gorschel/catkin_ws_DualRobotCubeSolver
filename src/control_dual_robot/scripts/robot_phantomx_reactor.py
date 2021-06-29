#!/usr/bin/env python
# -*- coding: utf-8 -*-

from math import pi, atan2, acos, sin, cos, sqrt

from joints import *
from coord import *
from consts import *
from misc import cossatz


class Robot(object):
    """class containing joint states for given robot id, inverse kinematics.

    r = Robot(id)

    methods & vars:
        r.ik()
        r.gripper.open()
        r.gripper.close()
        r.gripper.twist(n)
        r.publish()
        r.TCP
        r.state
    """

    def __init__(self, id = None):
        if id is None:
            raise Exception("no robot id given")
        else:
            self.TCP     = homepos
            self.state   = Joints()
            self.pub     = Joint_publisher(id)
            self.const   = Robot_structure()
            self.gripper = Gripper(self)

    def publish(self):
        self.pub.publish(self.state)

    def ik(self):
        """ inverse kinematics for joints 0..3. publishes to robot """

        # get orientation vector                        # TODO maybe rework for variable orientation angle
        d45 = self.const.d4 + self.const.d5
        if   self.TCP.ort == 'hor': P3TCP = Coord(r =  d45)
        elif self.TCP.ort == 'upw': P3TCP = Coord(z =  d45)
        elif self.TCP.ort == 'dwd': P3TCP = Coord(z = -d45)
        else: raise Exception("orientation not specified")

        # get points/vectors
        P1 = Coord(z = self.const.d0 + self.const.d1)
        P3 = self.TCP - P3TCP
        P1P3 = Coord(P1, P3)

        # check if point reachable
        if abs(P1P3) == abs(P1) + abs(P3):
            raise Exception("! Point not reachable with given orientation !")

        #get angles [RAD]
        alpha = cossatz(abs(P1P3), self.const.d2, self.const.d3)
        beta = cossatz(self.const.d3, self.const.d2, abs(P1P3))
        gamma = cossatz(abs(P1P3), self.const.d3, self.const.d2)
        phi = cossatz(abs(P1), abs(P1P3), abs(P3))
        if P3.r < 0.0: phi = 2*pi - phi # case for phi > pi (point close to base)

        # get joint values [RAD]
        self.state.q0 = self.TCP.th # atan2(TCP.y, TCP.x)
        self.state.q1 = pi - alpha - phi - self.const.psiz
        self.state.q2 = pi - beta - self.const.psir
        if   self.TCP.ort == 'hor': self.state.q3 = -pi/2 - gamma + phi
        elif self.TCP.ort == 'upw': self.state.q3 = -pi - gamma + phi
        elif self.TCP.ort == 'dwd': self.state.q3 = - gamma + phi

        #self.pub.publish(self.state)


class Gripper(object):
    """ subclass for gripper control and twist logic """
    def __init__(self, robot):
        self.turns = 0
        self.robot = robot

    def open(self):
        """ opens the gripper to 0 position. publishes to robot """
        self.robot.state.q5 = 0.0
        #self.robot.pub.j5.publish(self.robot.state.q5)

    def close(self):
        """ closes the gripper to const defined position. publishes to robot """
        self.robot.state.q5 = self.robot.const.closed
        #self.robot.pub.j5.publish(self.robot.state.q5)

    def twist(self, n):
        """
        n * 90° increments with servo cable protection. publishes to robot.
        gripper rotation is limited to ~140° in both directions -> only single and double turns possible -> need to check for desired cube turn before gripping (R <-> 3Ri not possible!)
        """
        if n in range(-2, 3):

            if self.turns + n not in range(-1, 2):
                n *= (-1) # change direction if cable twist limit reached [obsolete]

            twist = n*(pi/2)
            self.turns += n
            self.robot.state.q4 += twist
            #self.robot.pub.j4.publish(self.robot.state.q4)
        else: raise Exception("no valid twist parameter")

