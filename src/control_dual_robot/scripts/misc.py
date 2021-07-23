#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
from math import acos

# convenience functions

def cossatz(a, b, c):
    """ returns angle between triangle sides a and b """
    try:
        return acos((pow(a,2)+pow(b,2)-pow(c,2))/(2*a*b))
    except ValueError:
        raise Exception("! desired point out of reach !")


def wait(t = 2.5):
    """ wait time in secs """
    if isinstance(t, (float, int)):
        time.sleep(t)
    else: raise Exception("parameter not a number")

