#!/usr/bin/env python
# -*- coding: utf-8 -*-

from math import cos, sin, sqrt, atan2


class Coord(object):
    """Coordinate structure containing both cartesian and cylindrical coordinates

    Usage:
        Coord (x, y, z): creates Point. autoconverts missing r,z,th
        Coord (r, z, th): creates Point. autoconverts missing x,y,z
        Coord (P1, P2): creates Vector between two points
    """
    def __init__(self, a = None, b = None, x = None, y = None, z = None, r = None, th = None, ort = None):
        """ create vector or point """

        # vector from point a to b
        if isinstance(a, Coord) and isinstance(b, Coord) and None in (x, y, z, r, th):
            # orientation
            if ort is None:
                if a.ort is not b.ort:
                    raise Exception("orientations of points a & b do not match")
                if a.ort is b.ort:
                    self.ort = a.ort
            else: self.ort = ort

            # get vect coords
            self.__isvect = True
            self.x = b.x - a.x
            self.y = b.y - a.y
            self.z = b.z - a.z
            self.cnv_cylindrical()

        # point p in both coordinate forms
        else:
            if ort is None: raise Exception("no point orientation specified")
            if   None in (x, y, z, r, th):  autocvt = None
            elif None in (x, y):            autocvt = self.cnv_cartesian
            elif None in (r, th):           autocvt = self.cnv_cylindrical

            self.__isvect = False
            self.x = x if x is not None else 0.0
            self.y = y if y is not None else 0.0
            self.z = z if z is not None else 0.0
            self.r = r if r is not None else 0.0
            self.th = th if th is not None else 0.0
            self.ort = ort

            if autocvt is not None: autocvt()

    def cnv_cartesian(self):
        """ convert to cartesian coordinate form """
        self.x = self.r * cos(self.th)
        self.y = self.r * sin(self.th)

    def cnv_cylindrical(self):
        """ convert to cylindrical coordinate form """
        self.r = sqrt(pow(self.x, 2) + pow(self.y, 2))
        self.th = atan2(self.y, self.x)

    def __abs__(self):
        return sqrt(pow(self.r, 2) + pow(self.z, 2)) # valid for 2D and 3D

    def __add__(self, other):
        if isinstance(other, Point2D):
            return Point2D(self.r + other.r, self.z + other.z)
        elif isinstance(other, Coord):
            return Coord(r = self.r + other.r, z = self.z + other.z)
        else: raise TypeError

    def __sub__(self, other):
        if isinstance(other, Point2D):
            return Point2D(self.r - other.r, self.z - other.z)
        elif isinstance(other, Coord):
            return Coord(r = self.r - other.r, z = self.z - other.z)
        else: raise TypeError

    def __radd__(self, other):
        if isinstance(other, Point2D):
            return Point2D(other.r + self.r, other.z + self.z)
        elif isinstance(other, Coord):
            return Coord(r = other.r + self.r, z = other.z + self.z)
        else: raise TypeError


    def __rsub__(self, other):
        if isinstance(other, Point2D):
            return Point2D(other.r - self.r, other.z - self.z)
        elif isinstance(other, Coord):
            return Coord(r = other.r - self.r, z = other.z - self.z)
        else: raise TypeError

    def __repr__(self):
        return [self.x, self.y, self.z, self.r, self.th, self.ort]

    def __str__(self):
        if self.__isvect:
            return "Vector %s x:%s y:%s z:%s r:%s th:%s ort:%s" % (self.__class__.__name__ , self.x, self.y, self.z, self.r, self.th, self.ort)
        else:
            return "%s x:%s y:%s z:%s r:%s th:%s ort:%s" % (self.__class__.__name__ , self.x, self.y, self.z, self.r, self.th, self.ort)


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
        if isinstance(other, Coord) or isinstance(other, Point2D):
            return Point2D(self.r + other.r, self.z + other.z)
        else: raise TypeError

    def __sub__(self, other):
        if isinstance(other, Coord) or isinstance(other, Point2D):
            return Point2D(self.r - other.r, self.z - other.z)
        else: raise TypeError

    def __repr__(self):
        return [self.r, self.z]

    def __str__(self):
        return "%s r:%s z:%s" % (self.__class__.__name__ , self.r, self.z)

