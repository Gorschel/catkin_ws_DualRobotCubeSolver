#!/usr/bin/env python
# -*- coding: utf-8 -*-

from math import pi, atan2, acos, sin, cos, sqrt, ceil

from joints import *
from coord import *
from consts import *
from misc import *


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
            self.id      = id
            self.TCP     = Coord()
            self.state   = Joints()
            self.pub     = JointPublisher(id)
            self.const   = RobotStructure()
            self.pos     = Positions(self)
            self.gripper = Gripper(self)

    def publish(self):
        self.pub.publish(self.state)

    def ik(self):
        """ inverse kinematics for joints 0..3. publishes to robot """

        # TODO maybe rework for variable orientation angle
        # get orientation vector
        d45 = self.const.d4 + self.const.d5
        if   self.TCP.ort == 'hor': P3TCP = Coord(r =  d45)
        elif self.TCP.ort == 'upw': P3TCP = Coord(z =  d45)
        elif self.TCP.ort == 'dwd': P3TCP = Coord(z = -d45)
        else: raise Exception("orientation not specified")

        # get points/vectors
        OP1 = Coord(z = self.const.d0 + self.const.d1)
        OP3 = self.TCP - P3TCP
        P1P3 = Coord(a = OP1, b = OP3)

        # check if point reachable
        if abs(P1P3) == abs(OP1) + abs(OP3):
            raise Exception("! Point not reachable with given orientation !")

        #get angles [RAD]
        alpha = cossatz(abs(P1P3), self.const.d2, self.const.d3)
        beta = cossatz(self.const.d3, self.const.d2, abs(P1P3))
        gamma = cossatz(abs(P1P3), self.const.d3, self.const.d2)
        phi = cossatz(abs(OP1), abs(P1P3), abs(OP3))
        if OP3.r < 0.0: phi = 2*pi - phi # case for phi > pi (point close to base)

        # get joint values [RAD]
        self.state.q0 = self.TCP.th # atan2(TCP.y, TCP.x)
        self.state.q1 = pi - alpha - phi - self.const.psiz
        self.state.q2 = pi - beta - self.const.psir
        if   self.TCP.ort == 'hor': self.state.q3 = -pi/2 - gamma + phi
        elif self.TCP.ort == 'upw': self.state.q3 = -pi - gamma + phi
        elif self.TCP.ort == 'dwd': self.state.q3 = - gamma + phi

        self.state.q4 = self.state.q0 # align gripper rotation with base coordinate system

    def p2p(self, point):
        if not isinstance(point, Coord): raise Exception("no coordinate given")
        vect, dist = self.TCP.distto(point)
        self.TCP = point

        # move
        self.ik()
        self.publish()

        # wait time depends on distance
        t = dist / speed
        wait(t)

    def lin_p2p(self, point, steps = 10):
        """moves from current pos to given point. higher stepsize, the higher the precision (but may be slower)"""
        if not isinstance(point, Coord):        raise Exception("no coordinate given")
        if not isinstance(steps, (int, float)): raise Exception("invalid stepsize")

        # get vect & dist
        p0 = self.TCP
        v, dist = self.TCP.distto(point)
        vn = v / dist
        ds = dist / steps

        # get timing
        t = dist / speed
        dt = t / steps

        # step loop
        for n in range(1, steps + 1):
            # get new point
            step = p0 + (vn * (n * ds))
            # goto new point and wait
            self.TCP = step
            self.ik()
            self.publish()
            wait(dt)

    def pickup(self):
        """picks the cube from resting pos"""
        print "picking up .."
        self.p2p(self.pos.cube_retr)
        self.lin_p2p(self.pos.cube, 50)
        self.gripper.close()
        self.lin_p2p(self.pos.cube_retr, 50)
        print "picked up."

    def putdown(self):
        """puts the cube to resting pos"""
        print "putting down .."
        self.p2p(self.pos.cube_retr)
        self.lin_p2p(self.pos.cube, 50)
        self.gripper.open()
        self.lin_p2p(self.pos.cube_retr, 50)
        print "put down."

    def handover(ra, rb):
        print "handing over .."
        # retracted points
        if ra.id is 0 and rb.id is 1:
            ra_retr = ra.pos.center - Coord(y = 50, ort = hor)
            rb_retr = rb.pos.center + Coord(y = 50, ort = hor)

        elif ra.id is 1 and rb.id is 0:
            ra_retr = ra.pos.center + Coord(y = 50, ort = hor)
            rb_retr = rb.pos.center - Coord(y = 50, ort = hor)

        # compensated points r0
        pc0  = None
        pcc0 = None
        # compensated points r0
        pc1  = None
        pcc1 = None

        # bring cube to center pos
        ra.p2p(ra_retr)
        ra.lin_p2p(ra.pos.center)

        # prepare rb-pos and grip cube
        rb.p2p(rb_retr)
        rb.gripper.twist(-1)
        rb.lin_p2p(rb.pos.center)
        rb.gripper.close()

        ra.gripper.open()
        ra.lin_p2p(ra_retr)

        # vlt überflüssig
        ra.p2p(ra.pos.home)
        rb.gripper.twist(1)
        print "handed over."


class Gripper(object):
    """ subclass for gripper control and twist logic """
    def __init__(self, robot):
        self.turns = 0
        self.robot = robot

    def open(self):
        """ opens the gripper to 0 position. publishes to robot """
        self.robot.state.q5 = 0.0
        self.robot.publish()
        wait()

    def close(self):
        """ closes the gripper to const defined position. publishes to robot """
        self.robot.state.q5 = self.robot.const.closed
        self.robot.publish()
        wait()

    def twist(self, n):
        """
        n * 90° increments with servo cable protection. publishes to robot.

        gripper rotation is limited to ~140° in both directions -> only single and double turns possible -> need to check for desired cube turn before gripping 

        (R <-> 3Ri not possible!)
        """
        if n in range(-2, 3):

            if self.turns + n not in range(-1, 2):
                n *= (-1) # change direction if cable twist limit reached [obsolete]

            twist = n*(pi/2)
            self.turns += n
            self.robot.state.q4 += twist
            self.robot.publish()
            wait() # wait time needs to depend on angle
        else: raise Exception("no valid twist parameter")

