#!/usr/bin/env python
# -*- coding: utf-8 -*-

from math import pi
from misc import cossatz
from coord import Coord

# possible orientations
upw = 'upw'
hor = 'hor'
dwd = 'dwd'

speed = 40  # mm/s   # for timing purposes


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

        # holding pos
        self.D_hold = Coord(x=180, y=0, z=420, ort=hor)
        self.F_hold = self.center
        self.U_hold = Coord(x=300, y=0, z=70, ort=hor)

        # turning pos
        self.D_turn = Coord(x=250, y=0, z=400, ort=dwd)
        self.F_turn = self.center - Coord(x=18.5)
        self.U_turn = Coord(x=158, y=0, z=90, ort=dwd)

        # retracted turning pos
        self.D_retr = self.D_turn - Coord(x=50)
        self.F_retr = self.F_turn - Coord(x=50)
        self.U_retr = self.U_turn + Coord(z=25)

        # scanning poses (joint values)

        # self.scan_c = Joints(0.0, -pi/4, pi/4, 0.0, 0.0, 0.0)  # ! TODO: 2x übergabe (gebender muss vertikal halten) , danach

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
        #self.closed = 30 * pi / 180  # std greifer-stangen
        # b = 32,#(self.cubesize/2)+(self.sponge_dist - self.sponge_squish),
        # c = self.gripper_arm_length )
        self.__init = True

    def __setattr__(self, attr, value):
        if self.__init:
            raise Exception('const value. may not be modified!')
        else:
            super(GripStructure, self).__setattr__(attr, value)
