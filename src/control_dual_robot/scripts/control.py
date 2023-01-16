#!/usr/bin/env python
# -*- coding: utf-8 -*-
# license removed for brevity

# general imports
import random
from math import pi
import subprocess
import cv2
import re

# ros imports
import rospy
import rospkg
import actionlib
from control_dual_robot.msg import ControlAction
from twophase_solver_ros.srv import Solver, SolverResponse
from control_dual_robot.msg import ControlAction, ControlGoal

# local imports
from robot import Robot
from misc import wait, flip_dict_values
from joints import Joints
# from run import RobotGUI
from PyQt5.QtCore import pyqtSlot, pyqtSignal
from qtpy.QtWidgets import QApplication, QLabel, QWidget, QHBoxLayout, QVBoxLayout, QGridLayout, QMainWindow, \
    QPushButton, QComboBox

COMMANDS = ['ping', 'home',
            'demo_handover', 'demo_turning', 'demo_speed', 'demo_apply', 'demo_apply_inverse',
            'scan', 'solve', 'apply', 'complete']


def set_camera(position):
    cmd_list = [['v4l2-ctl', '-d', '/dev/video0', '--set-ctrl=power_line_frequency=1'],
                ['v4l2-ctl', '-d', '/dev/video0', '--set-ctrl=saturation=220'],
                ['v4l2-ctl', '-d', '/dev/video0', '--set-ctrl=sharpness=128'],
                ['v4l2-ctl', '-d', '/dev/video0', '--set-ctrl=exposure_auto=1'],
                ['v4l2-ctl', '-d', '/dev/video0', '--set-ctrl=exposure_absolute=50'],
                ['v4l2-ctl', '-d', '/dev/video0', '--set-ctrl=pan_absolute=-3600'],
                ['v4l2-ctl', '-d', '/dev/video0', '--set-ctrl=focus_auto=0']]

    if position in ['R', 'L', 'F', 'B']:
        cmd_list.append(['v4l2-ctl', '-d', '/dev/video0', '--set-ctrl=focus_absolute=10'])
    elif position in ['D', 'U']:
        cmd_list.append(['v4l2-ctl', '-d', '/dev/video0', '--set-ctrl=focus_absolute=15'])
    else:
        raise Exception("invalid face param")

    for cmd in cmd_list:
        subprocess.call(cmd)
    wait(1.0)


def process_images_and_get_solution():
    """
    calls ros service. -> SolverResponse
    """
    rospy.wait_for_service('cube_resolutor')
    try:
        cube_solver = rospy.ServiceProxy('cube_resolutor', Solver)
        resp = cube_solver('input')
        return resp
    except rospy.ServiceException as e:
        print "Service call failed: %s" % e


def demo_turning(ra, rb):
    ra.pickup()
    # ra validation test loop:
    # ra.turn(rb, 'b', 2)
    for face in ['a', 'b', 'c']:
        for n in range(-1, 3):
            if n is 0:
                continue
            ra.turn(rb, face, n)
    ra.putdown()


def demo_speed(ra, rb):
    ra.angular(Joints(0, -pi / 2, -pi / 2, 0, 0, 0))
    ra.angular(Joints(0, -pi / 2, pi / 2, 0, 0, 0))


def demo_handover(ra, rb):
    ra.pickup()
    for i in range(5):
        print "Durchlauf: {}".format(i)
        ra.handover(rb)
        rb.handover(ra)
    ra.putdown()


def home(ra, rb):
    for i in range(5):
        print('send home cmd to both robots')
        ra.home()
        rb.home()


def analyze_solution(ra, rb, solution):
    print('solution is {}'.format(solution))
    if solution is None or '(0f)' in solution:
        print('will not execute any maneuvers.')
        return False

    # turning tables
    ra_turns = {'R': 'a',
                'D': 'b',
                'L': 'c'}
    rb_turns = {'B': 'a',
                'U': 'b',
                'F': 'c'}

    picker_id = 0  # random.randint(0, 1)

    # swap turing tables for all possible cases
    # TODO (check): dict-flip still required for "swapped" cases?
    if ra.id == 0:
        if picker_id == 0:
            ra_turns = {'R': 'a',
                        'D': 'b',
                        'L': 'c'}
            rb_turns = {'B': 'a',
                        'U': 'b',
                        'F': 'c'}
        else:
            tmp = rb_turns
            rb_turns = ra_turns
            ra_turns = tmp
    else:
        if picker_id == 0:
            tmp = rb_turns
            rb_turns = ra_turns
            ra_turns = tmp
        else:
            pass

    # parse maneuver string
    pairs = re.findall(r'([A-Z]+)(\d+)', solution)

    for i, item in enumerate(pairs):
        letter, n = item
        n = -1 if int(n) > 2 else int(n)
        new = (letter, n)
        pairs[i] = new

    # randomly pick up cube if not already
    if not ra._hascube and not rb._hascube:
        if picker_id == ra.id:
            ra.pickup()
        elif picker_id == rb.id:
            rb.pickup()
    for i, (letter, n) in enumerate(pairs):
        print('##### turn {} / {} ##### turning face {} x {}*90° #####'.format(i + 1, len(pairs), letter, int(n)))
        # check if blocked & turn face
        if letter in ra_turns.keys():
            if rb._hascube:
                rb.handover(ra)
            if ra._hascube:
                face = ra_turns[letter]
                ra.turn(rb, face, n)
        if letter in rb_turns.keys():
            if ra._hascube:
                ra.handover(rb)
            if rb._hascube:
                face = rb_turns[letter]
                rb.turn(ra, face, n)

    # put cube back down
    if picker_id == ra.id:
        if ra._hascube:
            ra.putdown()
        elif rb._hascube:
            rb.handover(ra)
            ra.putdown()
    elif picker_id == rb.id:
        if rb._hascube:
            rb.putdown()
        elif ra._hascube:
            ra.handover(rb)
            rb.putdown()


def take_image(rid, face):
    print "Taking image of {} Face".format(face)
    cam = cv2.VideoCapture(0)
    wait(0.5)

    # cam.set(cv2.CAP_PROP_EXPOSURE, 0.1)
    if cam.isOpened():
        print "    > cam open"

        set_camera(face)

        ret, frame = cam.read()
        if ret:
            # cv2.imshow("img "+face, frame)
            # cv2.waitKey(0)
            # pre cut image & correct rotation
            if face in ['R', 'L', 'F', 'B']:
                frame = frame[130:380, 200:460, :]
                frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
            elif face in ['D', 'U']:
                frame = frame[120:430, 180:490, :]
                if face == 'U':
                    frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
            """
            if face == 'U':
                frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
            elif face == 'R':
                frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
            elif face == 'F':
                frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
            elif face == 'D':
                pass
            elif face == 'L':
                frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
            elif face == 'B':
                frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
            """
            rospack = rospkg.RosPack()
            fpath = rospack.get_path('twophase_solver_ros') + '/images/scan_' + 'Color.' + face + '.png'
            print "    > saving image to {}".format(fpath)
            cv2.imwrite(fpath, frame)
            wait(0.5)
        else:
            raise Exception("no image from camera received")
        cam.release()
    else:
        print("! could not open camera")


def scan_half_cube(r, other):
    # scanning poses (joint values)
    scan_a = Joints(0.0, -35 * pi / 180, -5 * pi / 180, 35 * pi / 180, 0.0, 0.0)
    scan_b = Joints(0.0, 10.0 * pi / 180, -5.0 * pi / 180, -98 * pi / 180, 0.0, 0.0)

    a, b, c = None, None, None
    if r.id == 0:
        a, b, c = 'R', 'D', 'L'
    elif r.id == 1:
        a, b, c = 'F', 'U', 'B'

    # begin scanning
    r.angular(scan_a)
    take_image(r.id, face=a)

    r.angular(scan_b)
    wait(1)
    take_image(r.id, face=b)

    r.flip(other)  # flip cube 180° (to scan D side) # ! 2. iteration: timing fails on right side (r1)

    r.angular(scan_a)
    take_image(r.id, face=c)


def pick_scan_cube(ra, rb):
    # pickup cube
    ra.pickup()
    rb.home()

    # scan first 3 faces
    scan_half_cube(ra, rb)
    ra.p2p(ra.pos.home)

    # transfer
    ra.handover(rb)
    ra.home()

    # second half
    scan_half_cube(rb, ra)

    # hand over again to make sure solution letters are in sync with cube orientation
    rb.handover(ra)
    rb.home()
    ra.putdown()
    ra.home()


class Control(object):
    def __init__(self):
        """
        starts the control node & moves both robots to home position
        """
        self.server = None
        rospy.init_node('control')

        self.r0 = Robot(0)
        self.r1 = Robot(1)

        print "ensuring home positions are reached.."
        self.r0.home()  # p2p(self.r0.pos.home)
        self.r1.home()  # p2p(self.r1.pos.home)
        wait(1.0)
        print "3"
        self.r0.home()  # p2p(self.r0.pos.home)
        self.r1.home()  # p2p(self.r1.pos.home)
        wait(1.0)
        print "2"
        self.r0.home()  # p2p(self.r0.pos.home)
        self.r1.home()  # p2p(self.r1.pos.home)
        wait(1.0)
        print "1"
        print "done. PowerOff the robots if any robot did not reach home position!"

        """
        self.server = actionlib.SimpleActionServer(name='control_server',
                                                   ActionSpec=ControlAction,
                                                   execute_cb=self.goal_received,
                                                   auto_start=False)
        self.server.register_preempt_callback(self.abort)
        self.server.start()
        """
        self.solver_resp = SolverResponse()
        self.solution = None
        app = QApplication([])
        gui = RobotGUI(self)
        gui.show()
        app.exec_()


    #@pyqtSlot(str)
    def execute_command(self, g):
        print 'starting. command: ' + g
        self.r0._enabled = True
        self.r1._enabled = True
        if g == 'demo_handover':
            demo_handover(self.r0, self.r1)

        elif g == 'home':
            home(self.r0, self.r1)

        elif g == 'demo_turning':
            demo_turning(self.r0, self.r1)

        elif g == 'demo_speed':
            print 'Warning, make way. Robots may move outside base plate!'
            demo_speed(self.r0, self.r1)

        elif g == 'scan':
            pick_scan_cube(self.r0, self.r1)

        elif g == 'solve':
            self.solver_resp = process_images_and_get_solution()
            try:
                self.solution = self.solver_resp.solution
            except AttributeError as e:
                print 'solver did not deliver a SolverResponse'

        elif g == 'demo_apply':
            solution = "F2 D1 L2 D1 U1 (5 move)"  # demo
            analyze_solution(self.r0, self.r1, solution)

        elif g == 'demo_apply_inverse':
            solution = "U3 D3 L2 D3 F2 (5 move)"  # demo
            analyze_solution(self.r0, self.r1, solution)

        elif g == 'apply':
            analyze_solution(self.r0, self.r1, self.solution)

        elif g == 'complete':
            # complete solvig cycle
            pick_scan_cube(self.r0, self.r1)
            self.solver_resp = process_images_and_get_solution()
            analyze_solution(self.r0, self.r1, self.solver_resp.solution)

    def goal_received(self, goal):
        if self.server.is_preempt_requested():
            self.server.set_preempted()
            return
        print(goal)
        g = str(goal.input)
        if g not in COMMANDS:
            print('invalid goal received.')
            self.server.set_aborted()
            return
        else:
            self.execute_command(g)

        self.server.set_succeeded()
        print 'done. returning home'
        self.r0.home()
        self.r1.home()

    def abort(self):
        self.r0._enabled = False
        self.r1._enabled = False


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
            self.client.cancel_goal()
        except Exception as e:
            print(e)


if __name__ == '__main__':
    print "manually starting Main Program"
    c = Control()
    rospy.spin()
    print "ending Main Program"
