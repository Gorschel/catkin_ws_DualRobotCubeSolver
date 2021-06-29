#!/usr/bin/env python
# -*- coding: utf-8 -*-

from math import pi
from misc import cossatz
from coord import Coord

upw = 'upw'
hor = 'hor'
dwd = 'dwd'
home = Coord(r=150, z=350, ort=hor)

class Hw_limits(object):
    """ Approximated hardware limits. Exceeding them will result in structural collision """
    __init = False

    def __init__(self):
        self.th0min = -pi
        self.th1min = -pi/2
        self.th2min = -pi*3/4
        self.th3min = -pi*5/9 #-100*pi/180
        self.th4min = -pi
        self.th5min = -pi/2

        self.th0max = pi
        self.th1max = pi/2
        self.th2max = pi/2
        self.th3max = pi*5/9  #100*pi/180
        self.th4max = pi
        self.th5max = pi/2

        self.__init = True

    def __setattr__(self, attr, value):
        if self.__init: raise Exception('value may not be modified!')
        else: super(Hw_limits, self).__setattr__(attr, value)


class Robot_structure(object):
    """ structural parameters. maybe get values from stl files for better precision """
    __init = False

    def __init__(self):
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

        self.closed = Grip_structure().closed

        self.__init = True

    def __setattr__(self, attr, value):
        if self.__init: raise Exception('value may not be modified!')
        else: super(Robot_structure, self).__setattr__(attr, value)

class Grip_structure(object):
    """ structureal parameters of cube and gripper"""
    __init = False

    def __init__(self):
        self.cubesize = 56.0
        self.servo_horn = 8.0
        self.open_gripper = 64.0
        self.sponge_dist = 3.0
        self.sponge_squish = 0.0
        self.closed = cossatz(  a = self.servo_horn,
                                b = ((self.cubesize + 2 * self.sponge_dist) / 2 ) - self.sponge_squish,
                                c = self.open_gripper / 2 - self.servo_horn
        )
        self.__init = True

    def __setattr__(self, attr, value):
        if self.__init: raise Exception('value may not be modified!')
        else: super(Grip_structure, self).__setattr__(attr, value)

