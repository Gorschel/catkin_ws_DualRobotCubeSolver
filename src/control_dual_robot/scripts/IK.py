#!/usr/bin/env python

import math as m

from  consts import robot_structure
from joints import joints
from coord import *

def cossatz(a, b, c):
    try:
        angle = m.acos((pow(a,2)+pow(b,2)-pow(c,2))/(2*a*b))
    except ValueError:
        raise Exception("! desired point out of reach !")
    return angle

def inv_kinematics(orientation, TCP = coord):
    """ <<--joints(). get joint values for given tool center coordinate (make sure coord values have been converted). orientation values: 'upw', 'hor', 'dwd' """
    state = joints()

    # get consts
    const = robot_structure()

    # get orientation vector
    d45 = const.d4 + const.d5
    if   orientation == 'hor': v3tcp = vector2D(a = point2D(r =  d45))
    elif orientation == 'upw': v3tcp = vector2D(a = point2D(z =  d45))
    elif orientation == 'dwd': v3tcp = vector2D(a = point2D(z = -d45))
    else: raise Exception("orientation not specified")

    # get points
    p1 = point2D(z = const.d0 + const.d1)
    p3 = TCP - v3tcp

    # get vectors
    v01 = vector2D(b = p1)
    v03 = vector2D(b = p3)
    v13 = vector2D(p1, p3)

    # maybe check if point reachable
    if abs(v13) == abs(v01) + abs(v03):
        raise Exception("! Point not reachable with given orientation !")

    #get angles [RAD]
    alpha = cossatz(abs(v13), const.d2, const.d3)
    beta = cossatz(const.d3, const.d2, abs(v13))
    gamma = cossatz(abs(v13), const.d3, const.d2)
    phi = cossatz(abs(v01), abs(v13), abs(v03))
    if v03.r < 0.0: phi = 2*m.pi - phi # case for phi > pi (point close to base)

    # get joint values [RAD]
    state.j0 = TCP.th # m.atan2(TCP.y, TCP.x)
    state.j1 = m.pi - alpha - phi - const.psiz
    state.j2 = m.pi - beta - const.psir
    if   orientation == 'hor': state.j3 = -m.pi/2 - gamma + phi
    elif orientation == 'upw': state.j3 = -m.pi - gamma + phi
    elif orientation == 'dwd': state.j3 = - gamma + phi

    #TODO# maybe add gripper and rotation joints

    return state