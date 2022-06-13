import rospy
import actionlib
from control import Control
from control_dual_robot.msg import ControlAction, ControlGoal


def control_client():
    rospy.init_node('control_client')
    client = actionlib.SimpleActionClient('control', ControlAction)
    client.wait_for_server()

    goal = ControlGoal('demo_apply')
    #goal = ControlGoal('solve')
    #goal = ControlGoal('apply')

    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(9000.0))


if __name__ == '__main__':
    control_client()
