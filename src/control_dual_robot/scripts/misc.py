#!/usr/bin/env python
# -*- coding: utf-8 -*-

from math import acos

def cossatz(a, b, c):
    try:
        angle = acos((pow(a,2)+pow(b,2)-pow(c,2))/(2*a*b))
    except ValueError:
        raise Exception("! desired point out of reach !")
    return angle

