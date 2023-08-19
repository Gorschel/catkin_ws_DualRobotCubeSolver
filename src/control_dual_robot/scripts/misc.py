#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
from math import acos

COMMANDS = ['ping', 'home',
            'demo_handover', 'demo_turning', 'demo_speed', 'demo_apply', 'demo_apply_inverse',
            'scan', 'solve', 'apply', 'complete']


def law_of_cosine(a, b, c):
    """
    custom function. checks if
    :returns: angle gamma between triangle sides a and b
    """
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


def flip_dict_values(indict):
    """ flip dict values """
    outdict = {}
    values = indict.values()
    values.reverse()
    for i, (k, v) in enumerate(indict.items()):
        outdict[k] = values[i]
    return outdict
