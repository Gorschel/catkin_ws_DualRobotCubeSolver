#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64



def joint_states_callback(message):
    # filter out joint0 position:
    try:
        for i,name in enumerate(message.name):
            if name == "shoulder_pitch_joint":
                pos = message.position[i]
                pub.publish(pos)
                print(pos)
    except:
        print(message)
    return

if __name__ == '__main__':
    rospy.init_node("example_repub")
    pub = rospy.Publisher("/phantomx_reactor_controller_1/shoulder_pitch_joint/command", Float64, queue_size=1)
    rospy.Subscriber("joint_states_1", JointState, joint_states_callback, queue_size=1)
    rospy.spin()