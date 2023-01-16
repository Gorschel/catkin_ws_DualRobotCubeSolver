import rospy
import actionlib
from control_dual_robot.msg import ControlAction, ControlGoal

from control import COMMANDS
from qtpy.QtWidgets import QApplication, QLabel, QWidget, QHBoxLayout, QVBoxLayout, QGridLayout, QMainWindow, \
    QPushButton, QComboBox


class RobotGUI(QMainWindow):
    def __init__(self, parent):
        super(RobotGUI, self).__init__()
        self.parent = parent
        self.client = None
        self.goal = None
        self.goal_list = COMMANDS

        self.__init__ui()
        # self.__init__client()

    def __init__ui(self):
        widget = QWidget(self)
        layout = QVBoxLayout()
        lbl_info = QLabel('chose a action to execute')

        goal_picker = QComboBox()
        goal_picker.addItems(self.goal_list)

        self.btn_send = QPushButton('send goal')
        self.btn_send.clicked.connect(lambda test: self.parent.execute_command(goal_picker.currentText()))
        #self.btn_send.clicked.connect(lambda test: self.set_send_goal(goal_picker.currentText()))
        self.btn_abort = QPushButton('abort goal')
        self.btn_abort.clicked.connect(self.abort_goal)

        layout.addWidget(lbl_info)
        layout.addWidget(goal_picker)
        layout.addWidget(self.btn_send)
        layout.addWidget(self.btn_abort)

        widget.setLayout(layout)
        self.setCentralWidget(widget)

    def __init__client(self):
        try:
            rospy.init_node('control_client')
            self.client = actionlib.SimpleActionClient('control_server', ControlAction)
            if self.client.wait_for_server():
                return True
            else:
                print ('ros action server timeout')
                return False
        except Exception as e:
            print("Exception @ Node initialisation: ", e)
            return False


    def set_send_goal(self, goal_str="demo_apply", recursion=False):
        if self.client is not None:
            try:
                goal = ControlGoal(goal_str)
                self.client.send_goal(goal)
                self.client.wait_for_result(rospy.Duration.from_sec(9000.0))
            except Exception as e:
                print("Exception @ sending goal (\'" + str(self.goal) + "\') : ", e)
        else:
            if self.__init__client() and not recursion:
                self.set_send_goal(goal_str, recursion=True)
            else:
                print("Client not initialized. Goal could not be set")

    def abort_goal(self):
        try:
            self.client.cancel_goal()
        except Exception as e:
            print(e)


if __name__ == '__main__':
    app = QApplication([])
    g = RobotGUI()
    g.show()
    app.exec_()
