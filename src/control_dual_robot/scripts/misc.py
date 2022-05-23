#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
from math import acos


def cossatz(a, b, c):
    """ returns angle between triangle sides a and b """
    try:
        return acos((c**2 - b**2 - a**2)/(-2.0 * a * b))
    except ValueError:
        raise Exception("! desired point out of reach !")


def wait(t=3):
    """ wait time in secs """
    if isinstance(t, (float, int)):
        time.sleep(t)
    else:
        raise Exception("parameter not a number")
