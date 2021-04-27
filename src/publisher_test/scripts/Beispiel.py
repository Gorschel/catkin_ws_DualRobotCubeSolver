#!/usr/bin/env python
# license removed for brevity
import rospy
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
    pub_gripper = rospy.Publisher('phantomx_reactor_controller/gripper_revolute_joint/command', Float64, queue_size=1)
    rospy.init_node('control', anonymous=True)
    freq = 50
    rate = rospy.Rate(freq)
    t = 0
    tmax = freq
    tmax *= 100
    counter = 0
    while not rospy.is_shutdown():
        if t <= tmax:
            data = 0.5/tmax*t
        else:
            data = 0.5 - 0.5/tmax*(t-tmax)
        t+=1
        if t == 2*tmax: 
            t = 0
            counter +=1
        if counter > 1: 
            rospy.sleep(5) # wait 4 ende

        pub_base.publish(3*(data-.25))
        pub_shoulder.publish(data-1)
        pub_elbow.publish(2*data)
        pub_wrist.publish(-2*data)
        pub_twist.publish(4*(data-.25))
        pub_gripper.publish(4*(data-.25))

        #rate.sleep()
        


if __name__ == '__main__':
    try:
        BspProg()
    except rospy.ROSInterruptException:
        pass