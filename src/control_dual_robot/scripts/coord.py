#!/usr/bin/env python
# -*- coding: utf-8 -*-

from math import cos, sin, sqrt, atan2


class Coord(object):
    """
    Structure for cartesian and cylindrical coordinate form.

    Usage:
        P = Coord(x=, y=, z=, ort): creates Point with optional orientation. autoconverts missing r,z,th
        P = Coord(r=, z=, th=, ort): creates Point with optional orientation. autoconverts missing x,y,z
        V = Coord(P1): creates vector from origin to point
        V = Coord(P1, P2): creates Vector between two points
    """

    def __init__(self, isvect = False, a = None, b = None, x = None, y = None, z = None, r = None, th = None, ort = None):
        """ create vector or point """
        #__isvect = isvect

        # vector from point a to b
        if isinstance(a, Coord) and isinstance(b, Coord) and all(var is None for var in [x, y, z, r, th]):
            # orientation
            if ort is None:
                if a.ort is None and b.ort is None:     raise Exception("orientation missing")
                elif a.ort is None:                     self.ort = b.ort
                elif b.ort is None:                     self.ort = a.ort
                elif a.ort is b.ort:                    self.ort = a.ort
            else: self.ort = ort

            # get vect coords
            self.__isvect = True
            self.x = b.x - a.x
            self.y = b.y - a.y
            self.z = b.z - a.z
            self.cnv_cylindrical()

        # point p in both coordinate forms
        elif a is None and b is None:
            #if ort is None: raise Exception("no point orientation specified") # 2d points cant have this line
            if all(var is None for var in [x, y, r, th]):   autocvt = None
            elif x is None and y is None:                   autocvt = self.cnv_cartesian
            elif r is None and th is None:                  autocvt = self.cnv_cylindrical
            else:                                           autocvt = None

            self.__isvect = isvect
            self.x = x if x is not None else 0.0
            self.y = y if y is not None else 0.0
            self.z = z if z is not None else 0.0
            self.r = r if r is not None else 0.0
            self.th = th if th is not None else 0.0
            self.ort = ort

            if autocvt is not None: autocvt()

        elif all(var is None for var in [a, b, x, y, z, r, th, ort]):
            self.x = 0.0
            self.y = 0.0
            self.z = 0.0
            self.r = 0.0
            self.th = 0.0
            self.ort = None

        else: raise Exception("Usage not intended")

    def cnv_cartesian(self):
        """ convert to cartesian coordinate form """
        self.x = self.r * cos(self.th)
        self.y = self.r * sin(self.th)

    def cnv_cylindrical(self):
        """ convert to cylindrical coordinate form """
        self.r = sqrt(pow(self.x, 2) + pow(self.y, 2))
        self.th = atan2(self.y, self.x)

    def distto(self, other):
        """
        get distance between current pos and given point

        vect, dist <--
        """
        if other is not None and isinstance(other, Coord):
            dx = other.x - self.x
            dy = other.y - self.y
            dz = other.z - self.z
            v = Coord(isvect = True, x = dx, y = dy, z = dz)
            return v, abs(v)
        else: raise Exception("Parameter needs to be Coord object")

    def __abs__(self):
        return sqrt(pow(self.r, 2) + pow(self.z, 2)) # valid for 2D and 3D

    def __add__(self, other):
        if isinstance(other, Coord):
            if self.__isvect and other.__isvect:    isvector = True
            else:                                   isvector = False
            if self.ort is other.ort:                                   orient = self.ort
            elif self.ort is not other.ort:
                if self.ort is not None and other.ort is None:          orient = self.ort
                elif self.ort is None and other.ort is not None:        orient = other.ort
                elif self.ort is not None and other.ort is not None:    orient = self.ort   # if both not none and not equal, pick one
                else:                                                   orient = None
            return Coord(isvect = isvector, x = self.x + other.x, y = self.y + other.y, z = self.z + other.z, ort = orient)
        else: raise TypeError

    def __sub__(self, other):
        if isinstance(other, Coord):
            if self.__isvect and other.__isvect:    isvector = True
            else:                                   isvector = False
            if self.ort is other.ort:                                   orient = self.ort
            elif self.ort is not other.ort:
                if self.ort is not None and other.ort is None:          orient = self.ort
                elif self.ort is None and other.ort is not None:        orient = other.ort
                elif self.ort is not None and other.ort is not None:    orient = self.ort   # if both not none and not equal, pick one
                else:                                                   orient = None
            return Coord(isvect = isvector, x = self.x - other.x, y = self.y - other.y, z = self.z - other.z, ort = orient)
        else: raise TypeError

    def __div__(self, other):
        if isinstance(other, (int, float)):
            return Coord(isvect = self.__isvect, x = self.x / other, y = self.y / other, z = self.z / other, ort = self.ort)
        else: raise TypeError

    def __truediv__(self, other):
        if isinstance(other, (int, float)):
            return Coord(isvect = self.__isvect, x = self.x / other, y = self.y / other, z = self.z / other, ort = self.ort)
        else: raise TypeError

    def __mul__(self, other):
        if isinstance(other, (int, float)):
            return Coord(isvect = self.__isvect, x = self.x * other, y = self.y * other, z = self.z * other, ort = self.ort)
        else: raise TypeError

    def __radd__(self, other):
        if isinstance(other, Coord):
            if self.__isvect and other.__isvect:    isvector = True
            else:                                   isvector = False
            if self.ort is other.ort:                                   orient = self.ort
            elif self.ort is not other.ort:
                if self.ort is not None and other.ort is None:          orient = self.ort
                elif self.ort is None and other.ort is not None:        orient = other.ort
                elif self.ort is not None and other.ort is not None:    orient = self.ort   # if both not none and not equal, pick one
                else:                                                   orient = None
            return Coord(isvect = isvector, x = other.x + self.x, y = other.y + self.y, z = other.z + self.z, ort = orient)
        else: raise TypeError

    def __rsub__(self, other):
        if isinstance(other, Coord):
            if self.__isvect and other.__isvect:    isvector = True
            else:                                   isvector = False
            if self.ort is other.ort:                                   orient = self.ort
            elif self.ort is not other.ort:
                if self.ort is not None and other.ort is None:          orient = self.ort
                elif self.ort is None and other.ort is not None:        orient = other.ort
                elif self.ort is not None and other.ort is not None:    orient = self.ort   # if both not none and not equal, pick one
                else:                                                   orient = None
            return Coord(isvect = isvector, x = other.x - self.x, y = other.y - self.y, z = other.z - self.z, ort = orient)
        else: raise TypeError

    def __repr__(self):
        return [self.x, self.y, self.z, self.r, self.th, self.ort]

    def __str__(self):
        if self.__isvect:
            return "Vector %s x:%s y:%s z:%s r:%s th:%s ort:%s" % (self.__class__.__name__ , self.x, self.y, self.z, self.r, self.th, self.ort)
        else:
            return "%s x:%s y:%s z:%s r:%s th:%s ort:%s" % (self.__class__.__name__ , self.x, self.y, self.z, self.r, self.th, self.ort)

