#!/usr/bin/env python
# -*- coding: utf-8 -*-

from math import pi
from coord import Coord

# possible orientations
upw = 'upw'
hor = 'hor'
dwd = 'dwd'

speed = 45  # mm/s   # for timing purposes


# TODO: speed in rad per sec instead of lin speed


class Positions(object):
    """constant world coordinates translated to each robot KOS"""
    __init = False

    def __init__(self, robot):
        self.home = Coord(r=160, z=180, ort=hor)
        r0_r1 = Coord(isvect=True, x=465)  # vector from global-origin/r0-origin to r1-origin

        if robot.id is 0:
            self.center = (r0_r1 / 2) + Coord(z=180, x=-5, ort=hor)
            self.cube = (r0_r1 / 2) + Coord(z=43, x=-5, ort=dwd)
            self.cube_retr = self.cube + Coord(z=50)

        if robot.id is 1:
            self.center = (r0_r1 / 2) + Coord(z=180, x=-5, ort=hor)
            self.cube = (r0_r1 / 2) + Coord(z=55, x=-4, ort=dwd)
            self.cube_retr = self.cube + Coord(z=50)

        # fixed turning positions
        self.a_hold = Coord(x=300, z=70, ort=hor)
        self.a_turn = Coord(x=157, z=90, ort=dwd)
        self.a_turn_prepare = Coord(th=-(pi * 20 / 180), r=157, z=95, ort=dwd)
        self.a_retr = self.a_turn + Coord(z=40)
        self.a_retr_prepare = Coord(th=-(pi * 20 / 180), r=155, z=120, ort=dwd)

        self.b_hold = self.center
        self.b_turn = self.center - Coord(x=18.5, z=10)
        self.b_retr = self.b_turn - Coord(x=50)

        # comp: q3 -= 3deg
        self.c_hold = Coord(x=220, z=377, ort=upw)
        self.c_turn = Coord(x=195, z=363, ort=hor)
        self.c_retr = self.c_turn - Coord(x=50)

        # uncomment if robots not facing fronts and cubepos symmetrical:
        """
        # transform points for robot id
        if robot.id is 0:
            self.cube       = cube      + Coord(y = 1)
            self.cube_retr  = cube_retr + Coord(y = 1)
            self.center     = center    + Coord(y = 1)

        elif robot.id is 1:
            self.cube       = cube      + Coord(y = 1)
            self.cube_retr  = cube_retr + Coord(y = 1)
            self.center     = r0_center# - r0_r1
        #"""

        self.__init = True

    def __setattr__(self, attr, value):
        if self.__init:
            raise Exception('const value. may not be modified!')
        else:
            super(Positions, self).__setattr__(attr, value)


class HardwareLimits(object):
    """Approximated hardware limits. Exceeding them would result in structural collision"""
    __init = False

    def __init__(self):
        self.th0min = -pi
        self.th1min = -pi / 2
        self.th2min = -pi * 3 / 4
        self.th3min = -pi * 5 / 9  # -100*pi/180
        self.th4min = -pi
        self.th5min = -pi / 2

        self.th0max = pi
        self.th1max = pi / 2
        self.th2max = pi / 2
        self.th3max = pi * 5 / 9  # 100*pi/180
        self.th4max = pi
        self.th5max = pi / 2

        self.__init = True

    def __setattr__(self, attr, value):
        if self.__init:
            raise Exception('const value. may not be modified!')
        else:
            super(HardwareLimits, self).__setattr__(attr, value)


class RobotStructure(object):
    """structural parameters. maybe get values from stl files for better precision"""
    __init = False

    def __init__(self):
        """
        # base height
        self.d0 = 12.5 + 78
        self.d1 = 26
        # shoulder length
        self.d2 = 150
        #self.d2r = 5*m.sqrt(59) #38.4057 # ~ 22 + (32 / 2)
        self.d2z = 145
        self.psir = 1.3118747847887 #m.asin(self.d2z / self.d2) #~1.31
        self.psiz = 0.25892154200621 #m.acos(self.d2z / self.d2) #~0.26
        # elbow length
        self.d3 = 147
        # wrist length
        self.d4 = 70
        self.d5 = 93
        """

        self.d01 = 104.5 + 8.0
        self.d2 = 150
        self.d2z = 145.146
        self.d2r = 37.851
        self.psiz = 0.255092
        self.psir = 1.3157
        self.d3 = 144.471
        self.d45 = 115.5
        self.dgrip = 45.175

        self.closed = GripStructure().closed

        self.__init = True

    def __setattr__(self, attr, value):
        if self.__init:
            raise Exception('const value. may not be modified!')
        else:
            super(RobotStructure, self).__setattr__(attr, value)


class GripStructure(object):
    """structureal parameters of cube and gripper"""
    __init = False

    def __init__(self):
        # self.cubesize = 56.0
        # self.servo_horn_radius = 8.0
        # self.gripper_arm_length = 29.0
        # self.sponge_dist = 3.0
        # self.sponge_squish = 0.0
        self.closed = 54 * pi / 180  # für 19mm greifer-stangen # cossatz(  a = self.servo_horn_radius,
        # self.closed = 30 * pi / 180  # std greifer-stangen
        # b = 32,#(self.cubesize/2)+(self.sponge_dist - self.sponge_squish),
        # c = self.gripper_arm_length )
        self.__init = True

    def __setattr__(self, attr, value):
        if self.__init:
            raise Exception('const value. may not be modified!')
        else:
            super(GripStructure, self).__setattr__(attr, value)
