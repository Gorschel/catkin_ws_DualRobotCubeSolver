#!/usr/bin/env python
# license removed for brevity
import rospy
import math as m
from std_msgs.msg import String, Float64

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()



def BspProg():
    pub_base = rospy.Publisher('phantomx_reactor_controller/shoulder_yaw_joint/command', Float64, queue_size=1)
    pub_shoulder = rospy.Publisher('phantomx_reactor_controller/shoulder_pitch_joint/command', Float64, queue_size=1)
    pub_elbow = rospy.Publisher('phantomx_reactor_controller/elbow_pitch_joint/command', Float64, queue_size=1)
    pub_wrist = rospy.Publisher('phantomx_reactor_controller/wrist_pitch_joint/command', Float64, queue_size=1)
    pub_twist = rospy.Publisher('phantomx_reactor_controller/wrist_roll_joint/command', Float64, queue_size=1)
    pub_grip = rospy.Publisher('phantomx_reactor_controller/gripper_revolute_joint/command', Float64, queue_size=1)
    rospy.init_node('control', anonymous=True)
    rate = rospy.Rate(10)
    x = y = z = 0
    while not rospy.is_shutdown():
        j1 = 0
        j2 = 0
        j3 = 0
        j4 = 0
        j5 = 0
        j6 = 0

        pub_base.publish(j1)
        pub_shoulder.publish(j2)
        pub_elbow.publish(j3)
        pub_wrist.publish(j4)
        pub_twist.publish(j5)
        pub_grip.publish(j6)


        #UNFERTIG
        
        rate.sleep()



if __name__ == '__main__':
    try:
        BspProg()
    except rospy.ROSInterruptException:
        pass