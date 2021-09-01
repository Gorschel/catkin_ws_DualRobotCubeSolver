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
            self.hascube = False
            self.TCP     = Coord()
            self.state   = Joints()
            self.pub     = JointPublisher(id)
            self.const   = RobotStructure()
            self.pos     = Positions(self)
            self.gripper = Gripper(self)

            """
            # base rotation offset
            if id is 0:
                self.q0_offset = 1.8 * pi/180
            elif id is 1:
                self.q0_offset = -0.7 * pi/180


            self.p2p(self.pos.home)

            # hotfix: q0 not actuated
            self.state.q0 = pi*5.0/180
            self.publish()
            self.state.q0 = 0.0
            self.publish()
            #"""

    def publish(self):
        self.pub.publish(self.state)

    def ik(self):
        """ inverse kinematics for joints 0..3. publishes to robot """

        # TODO maybe rework for variable orientation angle
        # get orientation vector
        wristTCP = self.const.d45 + self.const.dgrip
        if   self.TCP.ort == 'hor': P3TCP = Coord(r =  wristTCP)
        elif self.TCP.ort == 'upw': P3TCP = Coord(z =  wristTCP)
        elif self.TCP.ort == 'dwd': P3TCP = Coord(z = -wristTCP)
        else: raise Exception("orientation not specified")

        # get points/vectors
        OP1 = Coord(z = self.const.d01)
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
        self.state.q0 = self.TCP.th #+ self.q0_offset           # atan2(TCP.y, TCP.x)
        self.state.q1 = pi - alpha - phi - self.const.psiz
        self.state.q2 = pi - beta - self.const.psir
        if   self.TCP.ort == 'hor':
            self.state.q3 = -pi/2 - gamma + phi
        elif self.TCP.ort == 'upw':
            self.state.q3 = -pi - gamma + phi
        elif self.TCP.ort == 'dwd':
            self.state.q3 = - gamma + phi
            #self.state.q4 = self.state.q0 # align gripper rotation with base coordinate system

    def home(self):
        """moves to home position. drops gripped objects"""
        self.state.q0 = 0.0
        self.state.q1 = -pi/2
        self.state.q2 = 0.0
        self.state.q3 =  pi/2
        self.state.q4 = 0.0
        self.state.q5 = 0.0
        self.publish()

    def compensate(self, hascube, deg = None):
        """adds offset to wrist angle to compensate arm/cube weight. depends on hascube. optional: deg > 0

        only updating joint values. no publish"""
        #"""
        if deg is None:
            if self.TCP.ort is hor:
                if hascube: self.state.q3 -= pi*8.5/180
                else:       self.state.q3 -= pi*4.5/180
            elif self.TCP.ort is dwd:
                if hascube: self.state.q3 -= pi*4.5/180
                else:       self.state.q3 -= pi*4.5/180
        elif deg > 0:
            self.state.q3 -= deg*pi/180
        #"""

    def p2p(self, point, comp = None):
        if not isinstance(point, Coord): raise Exception("no coordinate given")
        vect, dist = self.TCP.distto(point)
        if dist > 0:
            self.TCP = point

            # move
            self.ik()
            self.compensate(self.hascube, comp)
            self.publish()

            # wait time depends on distance
            t = dist / speed
            wait(t)

    def lin_p2p(self, point, steps = 50, comp = None):
        """moves from current pos to given point. higher stepsize, the higher the precision (but may be slower)"""
        if not isinstance(point, Coord):        raise Exception("no position to go to")
        if not isinstance(steps, (int, float)): raise Exception("invalid step value")

        # get vect & dist
        v, dist = self.TCP.distto(point)
        if dist > 0:
            p0 = self.TCP
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
                self.compensate(self.hascube, comp)
                self.publish()
                wait(dt)

    def pickup(self):
        """picks the cube from resting pos"""
        #print "r{}: picking up ..".format(self.id)
        #wait(1.0)
        self.p2p(self.pos.cube_retr)
        self.lin_p2p(self.pos.cube, 50)
        wait(1.0)
        self.gripper.close()
        self.lin_p2p(self.pos.cube_retr, 50)
        print "r{}: picked up.".format(self.id)

    def putdown(self):
        """puts the cube to resting pos"""
        #print "r{}: putting down ..".format(self.id)
        #wait(1.0)
        self.p2p(self.pos.cube_retr)
        self.lin_p2p(self.pos.cube, 50)
        #wait(1.0)
        self.gripper.open()
        self.lin_p2p(self.pos.cube_retr, 50)
        print "r{}: put down.".format(self.id)

    def handover(ra, rb):
        print "handing over r{} >> r{}".format(ra.id, rb.id)

        comp = 8.5

        # retracted points
        if ra.id is 0:
            ra_center = ra.pos.center + Coord(z = comp)
            rb_center = rb.pos.center
        elif ra.id is 1:
            ra_center = ra.pos.center
            rb_center = rb.pos.center + Coord(z = comp)

        ra_center_retr = ra.pos.center - Coord(x = 40, ort = hor)
        rb_center_retr = rb.pos.center - Coord(x = 40, ort = hor)

        print "    > r{}: bring cube to center pos".format(ra.id)
        ra.p2p(ra_center_retr)
        ra.gripper.twist(1)
        ra.lin_p2p(ra_center, comp = 9.5)# + Coord(z = comp))

        print "    > r{}: grip cube".format(rb.id)
        rb.p2p(rb_center_retr)
        wait(1.5)
        rb.lin_p2p(rb_center + Coord(x=4, z=1))
        wait(1.0)
        rb.gripper.close()

        # switch compensation
        #rb.lin_p2p(rb.pos.center)# + Coord(z = comp))
        #ra.lin_p2p(ra.pos.center)

        print "    > r{}: let go and leave".format(ra.id)
        ra.gripper.open()
        ra.lin_p2p(ra_center_retr)
        ra.gripper.twist(0)

        print "    > done. r{} has cube. returning to home positions".format(rb.id)
        ra.home()
        rb.p2p(rb.pos.home)

    def turn(ra, rb, face, turns):
        """rb brings cube to one of 3 maneuver positions and ra turns the face

        Args:
            ra (Robot)(self): Robot(a) making the move
            rb (Robot): Robot(b) holding cube center
            ort (str): wrist orientation of robot b (upw, hor, dwd)
            turns (int): number of desired 90°-turns [0,2]
        """
        if turns > 2: raise Exception("invalid amount of turns")
        if rb.hascube:

            #TODO# Hier: Fallunterscheidungen. weniger doppelter code

            print "r{} turning local {}-face on r{}".format(ra.id, face, rb.id)
            if face is 'D':
                rb.p2p(rb.pos.D_hold)

                print "r{} approaching turning position".format(ra.id)
                wait(1.0)
                ra.p2p(ra.pos.home)
                if turns > 1: ra.gripper.twist(-1)  # prepare wrist for 180° turn # vlt für alle fälle lassen
                wait(1.0)
                ra.p2p(ra.pos.D_retr)
                ra.lin_p2p(ra.pos.D_turn)
                ra.gripper.close()

                print "turning cube {} deg".format(turns * 90)
                ra.gripper.twist(-1+turns)
                ra.gripper.open()
                ra.lin_p2p(ra.pos.D_retr)

            elif face is 'F':
                rb.p2p(rb.pos.F_hold, comp = 8.5)

                print "r{} approaching turning position".format(ra.id)
                ra.p2p(ra.pos.home)
                if turns > 1: ra.gripper.twist(-1)  # prepare wrist for 180° turn # vlt für alle fälle lassen
                ra.p2p(ra.pos.F_retr)
                ra.lin_p2p(ra.pos.F_turn, comp = 6)
                ra.gripper.close()

                print "turning cube {} deg".format(turns * 90)
                ra.gripper.twist(-1+turns)
                ra.gripper.open()
                ra.lin_p2p(ra.pos.F_retr)

            elif face is 'U':
                # bring cube to hold pos
                rb.p2p(rb.pos.U_hold + Coord(z=100))
                wait(0.2)
                rb.lin_p2p(rb.pos.U_hold, comp = 8.5)
                wait(1.0)

                print "r{} approaching turning position".format(ra.id)
                # ra vorbereiten
                ra.p2p(ra.pos.home)
                ra.gripper.twist(-1)  # prepare wrist for 180° turn # vlt für alle fälle lassen

                # zu würfel fahren und greifen
                ra.p2p(ra.pos.U_retr)
                wait(1.0)
                ra.lin_p2p(ra.pos.U_turn, comp = 6)
                wait(1.0)
                ra.gripper.close()
                wait(1.0)

                print "turning cube {} deg".format(turns * 90)
                # twist
                ra.gripper.twist(-1+turns)
                wait(3.0)
                # open ra and leave
                ra.gripper.open()
                ra.lin_p2p(ra.pos.U_retr)
                ra.home()
                # return midpos
                wait(1.0)
                rb.lin_p2p(rb.pos.U_hold + Coord(z=100))

            else: raise Exception("invalid faceturn")

            # return home
            rb.p2p(rb.pos.home)
        else:
            raise Exception("robot gripping the cube center cannot turn")

    def scan_cube(ra, rb):
        pass


class Gripper(object):
    """ subclass for gripper control and twist logic """
    def __init__(self, robot):
        self.turns = 0
        self.robot = robot

    def open(self):
        """ opens the gripper to 0 position. publishes to robot """
        self.robot.state.q5 = 0.0
        self.robot.publish()
        self.robot.hascube = False
        wait(1.0)

    def close(self):
        """ closes the gripper to const defined position. publishes to robot """
        self.robot.state.q5 = self.robot.const.closed
        self.robot.publish()
        self.robot.hascube = True
        wait(1.0)

    def twist(self, n):
        """
        roate wrist to n * 90° position.

        gripper rotation is limited to ~140° in both directions -> only single and double turns possible -> need to check for desired cube turn before gripping

        (R <=> 3Ri not possible!)
        """

        # move increment. indirect position n
        """
        if isinstance(n, int) and abs(n) <= 2:
            if not abs(self.turns + n) <= 1:
                if n is 2: raise Exception("out of bounds")
                n *= (-1) # change direction if cable twist limit reached
            twist = n*(pi/2)
            self.turns += n
            self.robot.state.q4 += twist
            self.robot.publish()
            wait(abs(n)*2.5) # wait time needs to depend on angle
        else: raise TypeError('n has to be int [-1,1]')
        """

        # direct position n
        bounds = range(-1,2)
        if isinstance(n, int):
            if n in bounds:
                self.turns = n
                self.robot.state.q4 = n*(pi/2) + n*pi/180   # n times 90° +- 1°
                self.robot.publish()
                wait(abs(n)*3.5)
            else: raise ValueError('n has to be in bounds [-1,1]')
        else: raise TypeError('n not a integer')

