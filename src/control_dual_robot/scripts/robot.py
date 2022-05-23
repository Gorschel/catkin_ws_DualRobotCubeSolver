#!/usr/bin/env python
# -*- coding: utf-8 -*-

from copy import copy
from joints import Joints, JointPublisher
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

    def __init__(self, id=None):

        if id is None:
            raise Exception("no robot id given")
        else:
            self.id = id
            self.hascube = False
            self.TCP = Coord()
            self.state = Joints()
            self.pub = JointPublisher(id)
            self.const = RobotStructure()
            self.pos = Positions(self)
            self.gripper = Gripper(self)

            # base rotation offset
            if id is 0:
                self.q0_offset = 1.8 * pi / 180
            elif id is 1:
                self.q0_offset = -0.7 * pi / 180

            """
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
        if self.TCP.ort == 'hor':
            P3TCP = Coord(r=wristTCP)
        elif self.TCP.ort == 'upw':
            P3TCP = Coord(z=wristTCP)
        elif self.TCP.ort == 'dwd':
            P3TCP = Coord(z=-wristTCP)
        else:
            raise Exception("orientation not specified")

        # get points/vectors
        OP1 = Coord(z=self.const.d01)
        OP3 = self.TCP - P3TCP
        P1P3 = Coord(a=OP1, b=OP3)

        # check if point reachable
        if abs(P1P3) == abs(OP1) + abs(OP3):
            raise Exception("! Point not reachable with given orientation !")

        # get angles [RAD]
        alpha = cossatz(abs(P1P3), self.const.d2, self.const.d3)
        beta = cossatz(self.const.d3, self.const.d2, abs(P1P3))
        gamma = cossatz(abs(P1P3), self.const.d3, self.const.d2)
        phi = cossatz(abs(OP1), abs(P1P3), abs(OP3))
        if OP3.r < 0.0: phi = 2 * pi - phi  # case for phi > pi (point close to base)

        # get joint values [RAD]
        self.state.q0 = self.TCP.th + self.q0_offset  # atan2(TCP.y, TCP.x)
        self.state.q1 = pi - alpha - phi - self.const.psiz
        self.state.q2 = pi - beta - self.const.psir
        if self.TCP.ort == 'hor':
            self.state.q3 = -pi / 2 - gamma + phi
        elif self.TCP.ort == 'upw':
            self.state.q3 = -pi - gamma + phi
        elif self.TCP.ort == 'dwd':
            self.state.q3 = - gamma + phi
            # self.state.q4 = self.state.q0 # align gripper rotation with base coordinate system

    def home(self):
        """moves to home position. drops gripped objects"""
        self.state.q0 = 0.0
        self.publish()
        self.state.q1 = -pi / 2
        self.publish()
        self.state.q2 = 0.0
        self.publish()
        self.state.q3 = pi / 2
        self.publish()
        self.state.q4 = 0.0
        self.publish()
        self.state.q5 = 0.0
        self.publish()

    def compensate(self, deg=None):
        """
            adds offset to wrist angle to compensate arm/cube weight. depends on hascube. optional: deg > 0
            deg (float): compenation angle
            joint (Joint.ax): specific axis to manipulate
            only updating joint values. no publish
        """
        # """
        if deg is None:
            if self.TCP.ort is hor:
                if self.hascube:
                    self.state.q3 -= pi * 8.5 / 180
                else:
                    self.state.q3 -= pi * 4.5 / 180
            elif self.TCP.ort is dwd:
                if self.hascube:
                    self.state.q3 -= pi * 4.5 / 180
                else:
                    self.state.q3 -= pi * 4.5 / 180
        elif deg > 0:
            self.state.q3 -= deg * pi / 180

    def angular(self, j):
        print "r{} moving to angle pos: {}".format(self.id, j)
        self.state.q0 = j.q0
        self.state.q1 = j.q1
        self.state.q2 = j.q2
        self.state.q3 = j.q3
        self.state.q4 = j.q4
        self.publish()
        wait(4)

    def p2p(self, point, comp=None):
        if not isinstance(point, Coord): raise Exception("no coordinate given")
        vect, dist = self.TCP.distto(point)
        if dist > 0:
            self.TCP = point
            self.ik()
            self.compensate(comp)
            self.publish()

            # wait time depends on distance
            t = dist / speed
            wait(t)

    def sync_p2p(self, point, steps=50, comp=None):
        """moves from current pos to given point. higher stepsize, the higher the precision (but may be slower)"""
        if not isinstance(point, Coord):        raise Exception("no position to go to")
        if not isinstance(steps, (int, float)): raise Exception("invalid step value")

        # get vect & dist
        v, dist = self.TCP.distto(point)
        if dist > 0:
            s0 = copy(self.state)
            self.TCP = point
            self.ik()
            angles = copy(self.state - s0)

            # get timing
            t = dist / speed
            dt = t / steps

            for n in range(1, steps + 1):
                # get new joint state in between 
                progress_norm = float(n) / float(steps)
                sn = angles * progress_norm
                # goto new point and wait
                self.state = copy(s0 + sn)
                if n > 1:
                    self.compensate(comp)
                self.publish()
                wait(dt)

    def pickup(self):
        """picks the cube from resting pos"""
        # print "r{}: picking up ..".format(self.id)

        self.p2p(self.pos.cube_retr)
        self.sync_p2p(self.pos.cube, 50)
        wait(0.5)
        self.gripper.close()
        self.sync_p2p(self.pos.cube_retr, 50)
        print "r{}: picked up.".format(self.id)

    def putdown(self):
        """puts the cube to resting pos"""
        # print "r{}: putting down ..".format(self.id)
        # wait(1.0)
        self.p2p(self.pos.cube_retr)
        self.sync_p2p(self.pos.cube, 50)
        # wait(1.0)
        self.gripper.open()
        self.sync_p2p(self.pos.cube_retr, 50)
        print "r{}: put down.".format(self.id)

    def handover(ra, rb):
        print "handing over r{} >> r{}".format(ra.id, rb.id)

        comp = 8.5

        ra_center = ra.pos.center
        rb_center = rb.pos.center

        # retracted points
        if ra.id is 0:
            ra_center += Coord(z=comp)
            # rb_center = rb.pos.center
        elif ra.id is 1:
            # ra_center = ra.pos.center
            rb_center += Coord(z=3)

        ra_center_retr = ra.pos.center - Coord(x=40, ort=hor)
        rb_center_retr = rb.pos.center - Coord(x=40, ort=hor)

        print "    > r{}: bring cube to center pos".format(ra.id)
        ra.p2p(ra_center_retr)
        ra.gripper.twist(1)
        ra.sync_p2p(ra_center, comp=9.5)

        print "    > r{}: grip cube".format(rb.id)
        rb.p2p(rb_center_retr)
        wait(0.5)
        rb.sync_p2p(rb_center + Coord(x=7, z=2))
        wait(2.0)
        rb.gripper.close()

        # TODO switch compensation
        # rb.lin_p2p(rb.pos.center)# + Coord(z = comp))
        # ra.lin_p2p(ra.pos.center)

        print "    > r{}: let go and leave".format(ra.id)
        ra.gripper.open()
        ra.sync_p2p(ra_center_retr)
        ra.gripper.twist(0)

        print "    > done. r{} has cube. returning to home positions".format(rb.id)
        ra.home()
        rb.p2p(rb.pos.home)

    def flip(ra, rb):
        """ derived from handover. flip U to D side
        """
        print "flipping cube on robot r{}".format(ra.id)

        comp = 8.0

        ra_center = ra.pos.center
        rb_center = rb.pos.center

        # retracted points
        if ra.id is 0:
            ra_center += Coord(z=comp)
            # rb_center = rb.pos.center
        elif ra.id is 1:
            # ra_center = ra.pos.center
            rb_center += Coord(z=3)

        ra_center_retr = ra.pos.center - Coord(x=50, ort=hor)
        rb_center_retr = rb.pos.center - Coord(x=40, ort=hor)

        print "    > r{}: bring cube to center pos".format(ra.id)
        ra.p2p(ra_center_retr)
        ra.gripper.twist(1)
        ra.sync_p2p(ra_center, comp=9.5)
        rb.home()

        print "    > r{}: grip cube".format(rb.id)
        rb.p2p(rb_center_retr)
        wait(0.5)
        rb.sync_p2p(rb_center + Coord(x=7, z=2))
        wait(2.0)
        rb.gripper.close()

        print "    > switching compensation".format(ra.id)
        rb.hascube = True
        # rb.compensate(9.0)
        ra.hascube = False
        # ra.compensate(-8.0)

        print "    > r{}: flipping gripper".format(ra.id)
        ra.gripper.open()
        ra.sync_p2p(ra_center_retr)

        ra.gripper.twist(0)
        rb.gripper.twist(-1)
        wait(0.5)
        ra.sync_p2p(ra_center - Coord(z=comp))
        wait(2.0)
        ra.gripper.close()

        print "    > r{} leaving".format(rb.id)
        rb.gripper.open()
        rb.sync_p2p(rb_center_retr)

        print "    > done. r{} has cube. returning to home positions".format(ra.id)
        rb.home()
        ra.p2p(ra.pos.home)

    def turn(ra, rb, face, turns):
        """ra brings cube to one of 3 maneuver positions and rb turns the face

        Args:
            rb (Robot)(self): Robot(a) making the move
            ra (Robot): Robot(b) holding cube center
            face (str): 'a' or 'b' or 'c'
            turns (int): number of desired 90°-turns -1..2

        Face names (horizontal grip):
            a: upper
            b: frontal
            c: lower
        """
        if turns > 2:
            raise Exception("invalid amount of turns")
        if ra.hascube:
            # TODO# Hier: Fallunterscheidungen. weniger doppelter code bzw keine roboter zuweisung im hauptprogramm nötig
            if face in ['a', 'b', 'c']:
                print "r{} turning {}-face on r{}".format(rb.id, face, ra.id)

                if face is 'a':
                    # bring cube to hold pos
                    ra.sync_p2p(ra.pos.a_hold + Coord(z=130)) # make sure we dont hit the ground
                    wait(1.2)
                    ra.sync_p2p(ra.pos.a_hold)
                    wait(1.0)

                    print "\tr{} approaching turning position".format(rb.id)
                    # rb vorbereiten
                    rb.p2p(rb.pos.home)
                    # pretwist = 1 if turns < 0 else -1
                    pretwist = -1 if abs(turns) > 1 else 0
                    rb.gripper.twist(pretwist)  # prepare wrist for 180° turn

                    """
                    # zu würfel fahren und greifen (seitlich reindrehen)
                    rb.p2p(rb.pos.a_retr_prepare)
                    rb.sync_p2p(rb.pos.a_turn_prepare)
                    rb.gripper.close(partial=0.7)
                    rb.sync_p2p(rb.pos.a_turn, comp=6)
                    wait(1.0)
                    rb.gripper.close()
                    wait(1.0)
                    """
                    # zu würfel fahren (von oben)
                    rb.p2p(rb.pos.a_retr)
                    rb.gripper.close(partial=0.7)  # es ist zu eng
                    rb.sync_p2p(rb.pos.a_turn)
                    wait(1.0)
                    rb.gripper.close()
                    wait(1.0)

                    print "\tturning cube {} deg".format(turns * 90)
                    rb.gripper.twist(pretwist + turns)
                    if pretwist != 0:
                        wait(3.0)
                    # open ra and leave
                    rb.gripper.close(partial=0.7)  # partially open. immer noch zu eng
                    rb.sync_p2p(rb.pos.a_retr)
                    rb.gripper.open()
                    ra.gripper.twist(0)
                    rb.home()
                    # return midpos
                    wait(1.0)
                    ra.sync_p2p(ra.pos.a_hold + Coord(z=100))

                elif face is 'b':
                    """ Methode normal:
                    ra.p2p(ra.pos.b_hold, comp=8.5)

                    print "\tr{} approaching turning position".format(rb.id)
                    rb.p2p(rb.pos.home)
                    pretwist = -1 if abs(turns) > 1 else 0

                    ra.gripper.twist(pretwist)  # prepare wrist for 180° turn # vlt für alle fälle lassen
                    rb.p2p(rb.pos.b_retr)
                    rb.sync_p2p(rb.pos.b_turn, comp=6)
                    rb.gripper.close()

                    print "\tturning cube {} deg".format(turns * 90)
                    ra.gripper.twist(0)
                    rb.gripper.twist(pretwist + turns)
                    if pretwist != 0:
                        wait(3.0) # wartezeit in twist buggy
                    rb.gripper.open()
                    rb.sync_p2p(rb.pos.b_retr)
                    """
                    # Methode "Boden":
                    pretwist = -1 if abs(turns) > 1 else 0
                    ra.p2p(ra.pos.cube_retr)
                    ra.gripper.twist(pretwist)
                    ra.sync_p2p(ra.pos.cube)
                    ra.gripper.twist(pretwist+turns)
                    if abs(turns) > 1:
                        wait(3.0)
                    ra.sync_p2p(ra.pos.cube_retr)
                    ra.gripper.twist(0)

                elif face is 'c':
                    # prepare robots
                    rb.home()
                    ra.p2p(ra.pos.home)
                    ra.p2p(ra.pos.c_hold)

                    # compensate weight
                    ra.state.q2 -= 5 * pi/180
                    ra.publish()

                    print "\tr{} approaching turning position".format(rb.id)
                    #wait(1.0)
                    pretwist = -1 if abs(turns) > 1 else 0  # prepare wrist for 180° turn # vlt für alle fälle lassen
                    wait(1.0)
                    rb.gripper.twist(pretwist)
                    rb.p2p(rb.pos.c_retr)
                    rb.sync_p2p(rb.pos.c_turn, steps=70)
                    rb.gripper.close()

                    print "\tturning cube {} deg".format(turns * 90)
                    rb.gripper.twist(pretwist + turns)
                    wait(3)
                    rb.gripper.open()
                    rb.sync_p2p(rb.pos.c_retr, comp=-8, steps=50)
                    ra.gripper.twist(0)
                    rb.home()
                    wait(1)
            else:
                raise Exception("invalid face-turn")

            # return home
            ra.p2p(ra.pos.home)
        else:
            raise Exception("robot gripping the cube center cannot turn")

    def scan_cube(ra, rb):
        pass
        # TODO: kann weg?


class Gripper(object):
    """ subclass for gripper control and twist logic """

    def __init__(self, robot):
        self.turns = 0  # obsolete
        self.robot = robot

    def open(self):
        """ opens the gripper to 0 position. publishes to robot """
        self.robot.state.q5 = 0.0
        self.robot.publish()
        self.robot.hascube = False
        wait(1.0)

    def close(self, partial=None):
        """
        closes the gripper to const defined position. publishes to robot
        partial (float): partially close gripper
        """
        if partial is None:
            self.robot.state.q5 = self.robot.const.closed
        else:
            self.robot.state.q5 = self.robot.const.closed * partial
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
        bounds = range(-1, 2)
        if isinstance(n, int):
            if n in bounds:
                self.turns = n  # obsolete
                self.robot.state.q4 = n * (pi / 2) + n * pi / 180  # n times 90° +- n°
                self.robot.publish()
                if abs(n) < 2:
                    wait(3)
                else:
                    wait(8)
            else:
                raise ValueError('n has to be in bounds [-1,1]')
        else:
            raise TypeError('n not a integer')
