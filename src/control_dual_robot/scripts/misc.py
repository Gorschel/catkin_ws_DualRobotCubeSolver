#!/usr/bin/env python
# -*- coding: utf-8 -*-

from math import acos, pi

def cossatz(a, b, c):
    """ returns angle between triangle sides a and b """
    try:
        angle = acos((pow(a,2)+pow(b,2)-pow(c,2))/(2*a*b))
    except ValueError:
        raise Exception("! desired point out of reach !")
    return angle

