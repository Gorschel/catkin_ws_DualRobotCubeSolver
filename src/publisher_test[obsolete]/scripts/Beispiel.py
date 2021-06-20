#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String, Float64

def BspProg():
    # init publisher for robot_0
    pub_base_0 = rospy.Publisher('/phantomx_reactor_controller_0/shoulder_yaw_joint/command', Float64, queue_size=1)
    pub_shoulder_0 = rospy.Publisher('/phantomx_reactor_controller_0/shoulder_pitch_joint/command', Float64, queue_size=1)
    pub_elbow_0 = rospy.Publisher('/phantomx_reactor_controller_0/elbow_pitch_joint/command', Float64, queue_size=1)
    pub_wrist_0 = rospy.Publisher('/phantomx_reactor_controller_0/wrist_pitch_joint/command', Float64, queue_size=1)
    pub_twist_0 = rospy.Publisher('/phantomx_reactor_controller_0/wrist_roll_joint/command', Float64, queue_size=1)
    pub_gripper_0 = rospy.Publisher('/phantomx_reactor_controller_0/gripper_revolute_joint/command', Float64, queue_size=1)
    
    # init publisher for robot_1
    pub_base_1 = rospy.Publisher('/phantomx_reactor_controller_1/shoulder_yaw_joint/command', Float64, queue_size=1)
    pub_shoulder_1 = rospy.Publisher('/phantomx_reactor_controller_1/shoulder_pitch_joint/command', Float64, queue_size=1)
    pub_elbow_1 = rospy.Publisher('/phantomx_reactor_controller_1/elbow_pitch_joint/command', Float64, queue_size=1)
    pub_wrist_1 = rospy.Publisher('/phantomx_reactor_controller_1/wrist_pitch_joint/command', Float64, queue_size=1)
    pub_twist_1 = rospy.Publisher('/phantomx_reactor_controller_1/wrist_roll_joint/command', Float64, queue_size=1)
    pub_gripper_1 = rospy.Publisher('/phantomx_reactor_controller_1/gripper_revolute_joint/command', Float64, queue_size=1)

    #>> ein roboter wird angesteuert

    rospy.init_node('control_bsp', anonymous=True)
    freq = 50
    tmax = freq
    tmax *= 100
    rate = rospy.Rate(freq)
    t = 0
    counter = 0
    while not rospy.is_shutdown():
        if t<= tmax:
            data = 0.5/tmax*t
        else:
            data = 0.5 - 0.5/tmax*(t-tmax)
        t+=1
        if t == 2*tmax:
            t = 0
            counter +=1
        if counter > 1:
            rospy.sleep(5) # wait 4 ende

        pub_base_0.publish(3*(data-.25))
        pub_shoulder_0.publish(data-1)
        pub_elbow_0.publish(2*data)
        pub_wrist_0.publish(-2*data)
        pub_twist_0.publish(4*(data-.25))
        pub_gripper_0.publish(4*(data-.25))

        pub_base_1.publish(3*(data-.25))
        pub_shoulder_1.publish(data-1)
        pub_elbow_1.publish(2*data)
        pub_wrist_1.publish(-2*data)
        pub_twist_1.publish(4*(data-.25))
        pub_gripper_1.publish(4*(data-.25))

        #rate.sleep()



if __name__ == '__main__':
    try:
        BspProg()
    except rospy.ROSInterruptException:
        pass