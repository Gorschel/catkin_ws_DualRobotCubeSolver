#!/usr/bin/env python
# -*- coding: utf-8 -*-

#import math as m
from math import cos, sin, sqrt, atan2


class Coord(object):
    """ class containing robot coordinate structure and conversion methods. Objects own the full coordinate set and a orientation specifier. """
    def __init__(self, x = None, y = None, z = None, r = None, th = None, ort = None):
        """  """
        if x is None and y is None: autocvt = self.cnv_catesian
        if r is None and th is None: autocvt = self.cnv_cylindrical
        if x is None and y is None and z is None and r is None and th is None: autocvt = None
        self.x = x if x is not None else 0.0
        self.y = y if y is not None else 0.0
        self.z = z if z is not None else 0.0
        self.r = r if r is not None else 0.0
        self.th = th if th is not None else 0.0
        self.ort = ort
        if autocvt is not None: autocvt()

    def cnv_catesian(self):
        """ convert to cartesian coordinate form """
        self.x = self.r * cos(self.th)
        self.y = self.r * sin(self.th)

    def cnv_cylindrical(self):
        """ convert to cylindrical coordinate form """
        self.r = sqrt(pow(self.x, 2) + pow(self.y, 2))
        self.th = atan2(self.y, self.x)

    def __add__(self, other):
        if isinstance(other, Vector2D) or isinstance(other, Point2D):
            return Point2D(self.r + other.r, self.z + other.z)
        else: raise TypeError

    def __sub__(self, other):
        if isinstance(other, Vector2D) or isinstance(other, Point2D):
            return Point2D(self.r - other.r, self.z - other.z)
        else: raise TypeError

    def __repr__(self):
        return [self.x, self.y, self.z, self.r, self.th]

    def __str__(self):
        return "%s x:%s y:%s z:%s r:%s th:%s " % (self.__class__.__name__ , self.x, self.y, self.z, self.r, self.th)


class Point2D(object):
    """ point in r-z plane """
    def __init__(self, r = None, z = None):
        if r is None:
            self.r = 0.0
            self.z = z
        elif z is None:
            self.r = r
            self.z = 0.0
        elif r is None and z is None:
            self.r = 0.0
            self.z = 0.0
        else:
            self.r = r
            self.z = z

    def __add__(self, other):
        if isinstance(other, Vector2D) or isinstance(other, Point2D):
            return Point2D(self.r + other.r, self.z + other.z)
        else: raise TypeError

    def __sub__(self, other):
        if isinstance(other, Vector2D) or isinstance(other, Point2D):
            return Point2D(self.r - other.r, self.z - other.z)
        else: raise TypeError

    def __repr__(self):
        return [self.r, self.z]

    def __str__(self):
        return "%s r:%s z:%s" % (self.__class__.__name__ , self.r, self.z)


class Vector2D(object):
    """ vector in r-z plane """
    def __init__(self, a = None, b = None):
        if a is None and b is None:
            self.r = 0.0
            self.z = 0.0
        elif a is None:
            if isinstance(b, Point2D):
                self.r = b.r
                self.z = b.z
            else: raise TypeError
        elif b is None:
            if isinstance(a, Point2D):
                self.r = a.r
                self.z = a.z
            else: raise TypeError
        elif isinstance(a, Point2D) and isinstance(b, Point2D):
            self.r = b.r - a.r
            self.z = b.z - a.z
        elif isinstance(a, Point2D) and isinstance(b, Coord):
            self.r = b.r - a.r
            self.z = b.z - a.z
        else: raise TypeError

    def __abs__(self):
        return sqrt(pow(self.r, 2) + pow(self.z, 2))

    def __add__(self, other):
        if isinstance(other, Vector2D):
            return Vector2D(self.r + other.r, self.z + other.z)
        else: raise TypeError

    def __sub__(self, other):
        if isinstance(other, Vector2D):
            return Vector2D(self.r - other.r, self.z - other.z)
        else: raise TypeError

    def __radd__(self, other):
        if isinstance(other, Point2D) or isinstance(other, Coord):
            return Point2D(other.r + self.r, other.z + self.z)
        elif isinstance(other, Vector2D):
            return Vector2D(other.r + self.r, other.z + self.z)
        else: raise TypeError

    def __rsub__(self, other):
        if isinstance(other, Point2D) or isinstance(other, Coord):
            return Point2D(other.r - self.r, other.z - self.z)
        elif isinstance(other, Vector2D):
            return Vector2D(other.r - self.r, other.z - self.z)
        else: raise TypeError

    def __repr__(self):
        return [self.r, self.z]

    def __str__(self):
        return "%s r:%s z:%s" % (self.__class__.__name__ , self.r, self.z)

