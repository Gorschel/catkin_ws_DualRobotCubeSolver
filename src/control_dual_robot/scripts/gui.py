import rospy
import actionlib
from control_dual_robot.msg import ControlAction, ControlGoal
from qtpy.QtWidgets import QLabel, QWidget, QVBoxLayout, QMainWindow, QPushButton, QComboBox
from misc import COMMANDS

class RobotGUI(QMainWindow):
    def __init__(self, parent, r0, r1):
        super(RobotGUI, self).__init__()
        self.parent = parent
        self.r0 = r0
        self.r1 = r1
        self.client = None
        self.goal = None
        self.goal_list = COMMANDS

        self.__init__ui()
        # self.__init__client()

    def __init__ui(self):
        widget = QWidget(self)
        layout = QVBoxLayout()
        lbl_info = QLabel('chose a action to execute')
        #self.windowIcon(QStyle.SP_DialogOpenButton)

        goal_picker = QComboBox()
        goal_picker.addItems(self.goal_list)

        self.btn_send = QPushButton('send goal')
        self.btn_send.clicked.connect(lambda test: self.parent.execute_command(goal_picker.currentText()))
        # self.btn_send.clicked.connect(lambda test: self.set_send_goal(goal_picker.currentText()))
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
            self.r0._enabled = False
            self.r1._enabled = False
        except Exception as e:
            print('abortion failed. ', e)