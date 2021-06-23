#!/usr/bin/env python

import rospy
import consts

class joints:
    """ todo. prepare to publish joint set """
    def __init__(self, th0 = None, th1 = None, th2 = None, th3 = None, th4 = None, th5 = None):
        self.j0 = th0 if th0 is not None else 0.0
        self.j1 = th1 if th1 is not None else 0.0
        self.j2 = th2 if th2 is not None else 0.0
        self.j3 = th3 if th3 is not None else 0.0
        self.j4 = th4 if th4 is not None else 0.0
        self.j5 = th5 if th5 is not None else 0.0

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

    def publish(self, joints):
        """ publishes set of joint values """
        # check for illegal joint values
        soll = check4limits(joints)
        # publish
        self.j0.publish(soll.j0)
        self.j1.publish(soll.j1)
        self.j2.publish(soll.j2)
        self.j3.publish(soll.j3)
        self.j4.publish(soll.j4)
        self.j5.publish(soll.j5)

def check4limits(joints = joints()):
    """ check if desired joint state is reachable. No colission warning """
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
        raise Exception('! desired joint states off hardware limits !')

