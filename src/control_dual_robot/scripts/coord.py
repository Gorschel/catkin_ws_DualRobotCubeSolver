#!/usr/bin/env python

class coord:
    """ class containing robot coordinate structure and conversion methods """
    def __init__(self, x = None, y = None, z = None, r = None, th = None):
        """  """
        if x is None and y is None: autocvt = self.cnv_catesian
        if r is None and th is None: autocvt = self.cnv_cylindrical
        if x is None and y is None and z is None and r is None and th is None: autocvt = pass
        self.x = x if x is not None else 0.0
        self.y = y if y is not None else 0.0
        self.z = z if z is not None else 0.0
        self.r = r if r is not None else 0.0
        self.th = th if th is not None else 0.0
        autocvt()

    def cnv_catesian(self):
        """ convert to cartesian coordinate form """
        self.x = self.r * m.cos(self.th)
        self.y = self.r * m.sin(self.th)

    def cnv_cylindrical(self):
        """ convert to cylindrical coordinate form """
        self.r = m.sqrt(pow(self.x, 2) + pow(self.y, 2))
        self.th = m.atan2(self.y, self.x)

    def __add__(self, other):
        if isinstance(other, vector2D) or isinstance(other, point2D):
            return point2D(self.r + other.r, self.z + other.z)
        else: raise TypeError

    def __sub__(self, other):
        if isinstance(other, vector2D) or isinstance(other, point2D):
            return point2D(self.r - other.r, self.z - other.z)
        else: raise TypeError

    def __repr__(self):
        return [self.x, self.y, self.z, self.r, self.th]

    def __str__(self):
        return "%s x:%s y:%s z:%s r:%s th:%s " % (self.__class__.__name__ , self.x, self.y, self.z, self.r, self.th)

class point2D:
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
        if isinstance(other, vector2D) or isinstance(other, point2D):
            return point2D(self.r + other.r, self.z + other.z)
        else: raise TypeError

    def __sub__(self, other):
        if isinstance(other, vector2D) or isinstance(other, point2D):
            return point2D(self.r - other.r, self.z - other.z)
        else: raise TypeError

    def __repr__(self):
        return [self.r, self.z]

    def __str__(self):
        return "%s r:%s z:%s" % (self.__class__.__name__ , self.r, self.z)

class vector2D:
    """ vector in r-z plane """
    def __init__(self, a = None, b = None):
        if a is None and b is None:
            self.r = 0.0
            self.z = 0.0
        elif a is None:
            if isinstance(b, point2D):
                self.r = b.r
                self.z = b.z
            else: raise TypeError
        elif b is None:
            if isinstance(a, point2D):
                self.r = a.r
                self.z = a.z
            else: raise TypeError
        elif isinstance(a, point2D) and isinstance(b, point2D):
            self.r = b.r - a.r
            self.z = b.z - a.z
        elif isinstance(a, point2D) and isinstance(b, coord):
            self.r = b.r - a.r
            self.z = b.z - a.z
        else: raise TypeError

    def __abs__(self):
        return m.sqrt(pow(self.r, 2) + pow(self.z, 2))

    def __add__(self, other):
        if isinstance(other, vector2D):
            return vector2D(self.r + other.r, self.z + other.z)
        else: raise TypeError

    def __sub__(self, other):
        if isinstance(other, vector2D):
            return vector2D(self.r - other.r, self.z - other.z)
        else: raise TypeError

    def __radd__(self, other):
        if isinstance(other, point2D) or isinstance(other, coord):
            return point2D(other.r + self.r, other.z + self.z)
        elif isinstance(other, vector2D):
            return vector2D(other.r + self.r, other.z + self.z)
        else: raise TypeError

    def __rsub__(self, other):
        if isinstance(other, point2D) or isinstance(other, coord):
            return point2D(other.r - self.r, other.z - self.z)
        elif isinstance(other, vector2D):
            return vector2D(other.r - self.r, other.z - self.z)
        else: raise TypeError

    def __repr__(self):
        return [self.r, self.z]

    def __str__(self):
        return "%s r:%s z:%s" % (self.__class__.__name__ , self.r, self.z)

