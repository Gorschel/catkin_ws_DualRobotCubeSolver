#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float64

from consts import HardwareLimits


class Joints(object):
    """ set of joints """

    def __init__(self, th0=None, th1=None, th2=None, th3=None, th4=None, th5=None):
        self.q0 = th0 if th0 is not None else 0.0
        self.q1 = th1 if th1 is not None else 0.0
        self.q2 = th2 if th2 is not None else 0.0
        self.q3 = th3 if th3 is not None else 0.0
        self.q4 = th4 if th4 is not None else 0.0
        self.q5 = th5 if th5 is not None else 0.0

    def max_angle(self):
        return max(list(map(abs, [self.q0, self.q1, self.q2, self.q3, self.q4, self.q5])))

    def __repr__(self):
        return self.q0, self.q1, self.q2, self.q3, self.q4, self.q5

    def __str__(self):
        return "%s q0:%s q1:%s q2:%s q3:%s q4:%s q5:%s" % (
            self.__class__.__name__, self.q0, self.q1, self.q2, self.q3, self.q4, self.q5)

    def __sub__(self, other):
        q0 = self.q0 - other.q0
        q1 = self.q1 - other.q1
        q2 = self.q2 - other.q2
        q3 = self.q3 - other.q3
        q4 = self.q4 - other.q4
        q5 = self.q5 - other.q5
        return Joints(q0, q1, q2, q3, q4, q5)

    def __add__(self, other):
        """overrides left!"""
        q0 = self.q0 + other.q0
        q1 = self.q1 + other.q1
        q2 = self.q2 + other.q2
        q3 = self.q3 + other.q3
        q4 = self.q4 + other.q4
        q5 = self.q5 + other.q5
        return Joints(q0, q1, q2, q3, q4, q5)

    def __div__(self, other):
        if other > 0:
            q0 = self.q0 / other
            q1 = self.q1 / other
            q2 = self.q2 / other
            q3 = self.q3 / other
            q4 = self.q4 / other
            q5 = self.q5 / other
            return Joints(q0, q1, q2, q3, q4, q5)
        else:
            raise Exception("division by 0")

    def __mul__(self, other):
        q0 = self.q0 * other
        q1 = self.q1 * other
        q2 = self.q2 * other
        q3 = self.q3 * other
        q4 = self.q4 * other
        q5 = self.q5 * other
        return Joints(q0, q1, q2, q3, q4, q5)


class JointPublisher(object):
    """ creates set of controllable joint publishers """

    def __init__(self, id):
        self.j0 = rospy.Publisher('/phantomx_reactor_controller_' + str(id) + '/shoulder_yaw_joint/command',
                                  Float64, queue_size=1)
        self.j1 = rospy.Publisher('/phantomx_reactor_controller_' + str(id) + '/shoulder_pitch_joint/command',
                                  Float64, queue_size=1)
        self.j2 = rospy.Publisher('/phantomx_reactor_controller_' + str(id) + '/elbow_pitch_joint/command',
                                  Float64, queue_size=1)
        self.j3 = rospy.Publisher('/phantomx_reactor_controller_' + str(id) + '/wrist_pitch_joint/command',
                                  Float64, queue_size=1)
        self.j4 = rospy.Publisher('/phantomx_reactor_controller_' + str(id) + '/wrist_roll_joint/command',
                                  Float64, queue_size=1)
        self.j5 = rospy.Publisher('/phantomx_reactor_controller_' + str(id) + '/gripper_revolute_joint/command',
                                  Float64, queue_size=1)

    def publish(self, joints):
        """ checks for hardware limits and publishes all joint values """
        # check for illegal joint values
        soll = self.check4limits(joints)
        # publish
        self.j0.publish(soll.q0)
        self.j1.publish(soll.q1)
        self.j2.publish(soll.q2)
        self.j3.publish(soll.q3)
        self.j4.publish(soll.q4)
        self.j5.publish(soll.q5)

    def check4limits(self, joints=Joints()):
        """ check if desired joint state is reachable. No colission warning """
        lim = HardwareLimits()
        if (
                lim.th0min <= joints.q0 <= lim.th0max and
                lim.th1min <= joints.q1 <= lim.th1max and
                lim.th2min <= joints.q2 <= lim.th2max and
                lim.th3min <= joints.q3 <= lim.th3max and
                lim.th4min <= joints.q4 <= lim.th4max and
                lim.th5min <= joints.q5 <= lim.th5max):
            return joints
        else:
            print(joints.q0, joints.q1, joints.q2, joints.q3, joints.q4, joints.q5)
            raise Exception('! desired joint states off hardware limits !')
