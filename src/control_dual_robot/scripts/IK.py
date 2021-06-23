#!/usr/bin/env python
# -*- coding: utf-8 -*-

from math import pi, acos

from consts import *
from joints import *
from coord import *

def cossatz(a, b, c):
    try:
        angle = acos((pow(a,2)+pow(b,2)-pow(c,2))/(2*a*b))
    except ValueError:
        raise Exception("! desired point out of reach !")
    return angle

def inv_kinematics(orientation, TCP = Coord()):
    """ <<--joints(). get joint values for given tool center coordinate (make sure coord values have been converted). orientation values: 'upw', 'hor', 'dwd' """
    state = Joints()

    # get consts
    const = Robot_structure()

    # get orientation vector
    d45 = const.d4 + const.d5
    if   orientation == 'hor': v3tcp = Vector2D(a = Point2D(r =  d45))
    elif orientation == 'upw': v3tcp = Vector2D(a = Point2D(z =  d45))
    elif orientation == 'dwd': v3tcp = Vector2D(a = Point2D(z = -d45))
    else: raise Exception("orientation not specified")

    # get points
    p1 = Point2D(z = const.d0 + const.d1)
    p3 = TCP - v3tcp

    # get vectors
    v01 = Vector2D(b = p1)
    v03 = Vector2D(b = p3)
    v13 = Vector2D(p1, p3)

    # maybe check if point reachable
    if abs(v13) == abs(v01) + abs(v03):
        raise Exception("! Point not reachable with given orientation !")

    #get angles [RAD]
    alpha = cossatz(abs(v13), const.d2, const.d3)
    beta = cossatz(const.d3, const.d2, abs(v13))
    gamma = cossatz(abs(v13), const.d3, const.d2)
    phi = cossatz(abs(v01), abs(v13), abs(v03))
    if v03.r < 0.0: phi = 2*pi - phi # case for phi > pi (point close to base)

    # get joint values [RAD]
    state.q0 = TCP.th # atan2(TCP.y, TCP.x)
    state.q1 = pi - alpha - phi - const.psiz
    state.q2 = pi - beta - const.psir
    if   orientation == 'hor': state.q3 = -pi/2 - gamma + phi
    elif orientation == 'upw': state.q3 = -pi - gamma + phi
    elif orientation == 'dwd': state.q3 = - gamma + phi

    #TODO# maybe add gripper and rotation joints

    return state