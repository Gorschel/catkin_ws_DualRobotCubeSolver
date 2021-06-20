#!/usr/bin/env python
# license removed for brevity
import rospy
#import inv_kinematics
from std_msgs.msg import String, Float64
import math as m

class joints:
    """ todo. prepare to publish joint set"""
    def j_init(self, j, th):
        if th is None: self.j = 0.0
        else: self.j = th
        return self

    def __init__(self, th0 = None, th1 = None, th2 = None, th3 = None, th4 = None, th5 = None):
        if th0 is None: self.j0 = 0.0
        else: self.j0 = th0

        if th1 is None: self.j1 = 0.0
        else: self.j1 = th1

        if th2 is None: self.j2 = 0.0
        else: self.j2 = th2

        if th3 is None: self.j3 = 0.0
        else: self.j3 = th3

        if th4 is None: self.j4 = 0.0
        else: self.j4 = th4

        if th5 is None: self.j5 = 0.0
        else: self.j5 = th5

    def __repr__(self):
        return [self.j0, self.j1, self.j2, self.j3, self.j4, self.j5]

    def __str__(self):
        return "%s j0:%s j1:%s j2:%s j3:%s j4:%s j5:%s" % (self.__class__.__name__ , self.j0, self.j1, self.j2, self.j3, self.j4, self.j5)

class joint_publisher:
    """ creates set of controllable joint publishers """
    def __init__(self, id):
        self.j0 = rospy.Publisher('/phantomx_reactor_controller_' + str(id) + '/shoulder_yaw_joint/command', Float64, queue_size=1)
        self.j1 = rospy.Publisher('/phantomx_reactor_controller_' + str(id) + '/shoulder_pitch_joint/command', Float64, queue_size=1)
        self.j2 = rospy.Publisher('/phantomx_reactor_controller_' + str(id) + '/elbow_pitch_joint/command', Float64, queue_size=1)
        self.j3 = rospy.Publisher('/phantomx_reactor_controller_' + str(id) + '/wrist_pitch_joint/command', Float64, queue_size=1)
        self.j4 = rospy.Publisher('/phantomx_reactor_controller_' + str(id) + '/wrist_roll_joint/command', Float64, queue_size=1)
        self.j5 = rospy.Publisher('/phantomx_reactor_controller_' + str(id) + '/gripper_revolute_joint/command', Float64, queue_size=1)

    def publish(self, soll):
        """ publishes set of joint values"""
        # check for illegal joint values
        robot = check4limits(soll)
        # publish
        self.j0.publish(robot.j0)
        self.j1.publish(robot.j1)
        self.j2.publish(robot.j2)
        self.j3.publish(robot.j3)
        self.j4.publish(robot.j4)
        self.j5.publish(robot.j5)

class hw_limits(object):
    """ Approximated hardware limits. Exceeding them will result in structural collision"""
    __init = False
    def __init__(self):
        self.th0min = -m.pi
        self.th1min = -m.pi/2
        self.th2min = -m.pi*3/4
        self.th3min = -m.pi*5/9 #-100*m.pi/180
        self.th4min = -m.pi
        self.th5min = 0.0

        self.th0max = m.pi
        self.th1max = m.pi/2
        self.th2max = m.pi/2
        self.th3max = m.pi*5/9  #100*m.pi/180
        self.th4max = m.pi
        self.th5max = m.pi/2

        self.__init = True

    def __setattr__(self, attr, value):
        if self.__init: raise Exception('value may not be modified!')
        else: super(hw_limits, self).__setattr__(attr, value)

class robot_structure(object):
    """ structural parameters. maybe get values from stl files for better precision"""
    __init = False
    def __init__(self):
        # base height
        self.d0 = 12.5 + 78
        self.d1 = 26
        # shoulder length
        self.d2 = 147
        self.d2x = 38.41 # ~ 22 + (32 / 2)
        self.d2y = 145
        # elbow length
        self.d3 = 150
        # wrist length
        self.d4 = 70
        self.d5 = 93

        self.__init = True

    def __setattr__(self, attr, value):
        if self.__init: raise Exception('value may not be modified!')
        else: super(robot_structure, self).__setattr__(attr, value)

class coord:
    """ class containing robot coordinate structure and conversion methods"""
    def __init__(self, x = None, y = None, z = None, r = None, theta = None):
        """ """
        self.x = x
        self.y = y
        self.z = z
        self.r = r
        self.th = theta

        if x is None: self.x = 0.0
        if y is None: self.y = 0.0
        if z is None: self.z = 0.0
        if r is None: self.r = 0.0
        if theta is None: self.th = 0.0

    def cnv_catesian(self):
        """ convert to cartesian coordinate form"""
        self.x = r * m.cos(self.th)
        self.y = r * m.sin(self.th)

    def cnv_cylindrical(self):
        """ convert to cylindrical coordinate form"""
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
    """ point in r-z plane"""
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
    """ vector in r-z plane"""
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

def cossatz(a, b, c):
    return m.acos((pow(a,2)+pow(b,2)-pow(c,2))/(2*a*b))

def check4limits(joints = joints()):
    """check if desired joint state is reachable. No colission warning"""
    lim = hw_limits()
    if (
    lim.th0min <= joints.j0 <= lim.th0max and
    lim.th1min <= joints.j1 <= lim.th1max and
    lim.th2min <= joints.j2 <= lim.th2max and
    lim.th3min <= joints.j3 <= lim.th3max and
    lim.th4min <= joints.j4 <= lim.th4max and
    lim.th5min <= joints.j5 <= lim.th5max ) :
        return joints
    else:
        raise Exception('desired joint states off hardware limits!')

def inv_kinematics(orientation, TCP = coord):
    """ get joint values for given tool center coordinate (make sure coord values have been converted). orientation values: 'upw', 'hor', 'dwd' """
    state = joints()
    const = robot_structure()
    # create
    d45 = const.d4 + const.d5
    if   orientation == 'hor':
        v3tcp = vector2D(a = point2D(r =  d45))
    elif orientation == 'upw':
        v3tcp = vector2D(a = point2D(z =  d45))
    elif orientation == 'dwd':
        v3tcp = vector2D(a = point2D(z = -d45))
    else: raise Exception("orientation not specified")
    # get points
    p1 = point2D(z = const.d0 + const.d1)
    p3 = TCP - v3tcp
    # get vectors
    v01 = vector2D(b = p1)
    v03 = vector2D(b = p3)
    v13 = vector2D(p1, p3)
    #get angles [RAD]
    alpha = cossatz(abs(v13), const.d2, const.d3)
    beta = cossatz(const.d3, const.d2, abs(v13))
    gamma = cossatz(abs(v13), const.d3, const.d2)
    phi = cossatz(abs(v01), abs(v13), abs(v03))
    psi = m.atan(const.d2y / const.d2x)
    chi = phi - m.pi/2 #m.atan2(p3.z - p1.z, p3.r) # vorzeichen !
    # get joint values [RAD]
    state.j0 = m.atan2(TCP.y, TCP.x)
    state.j1 = m.pi - alpha - phi
    state.j2 = m.pi - beta - psi
    if   orientation == 'hor': state.j3 = gamma - chi
    elif orientation == 'upw': state.j3 = m.pi/2 + gamma + chi
    elif orientation == 'dwd': state.j3 = m.pi/2 - gamma - chi

    return state    # returns whole set of joints including zero joints

def BspProgROS():
    rp0 = joint_publisher(0)
    #rp1 = joint_publisher(1)
    #ist = joints()
    #r1 = joints()

    TCP = coord(z = 200.0, r = 150.0, theta = 0.0)

    while not rospy.is_shutdown():
        rate.sleep()

        #TCP.z += 0.01
        #TCP.cnv_cylindrical()
        #print TCP

        #soll = inv_kinematics('hor', TCP)
        soll = joints()
        rp0.publish(soll)

def debug():
    """ module testing etc"""
    #c1 = coord(x = 200, z = 100)
    #c1.cnv_cylindrical()
    #ist = joints()
    #soll = inv_kinematics('hor', ist, c1)

def rosnode():
    rospy.init_node('control_dual_robots_debug', anonymous=True)
    BspProgROS()
    rospy.spin()

if __name__ == '__main__':
    rosnode()
    #debug()