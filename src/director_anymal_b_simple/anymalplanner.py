import os
import sys
from director import vtkAll as vtk
import math
import time
import types
import functools
import numpy as np
import string, random # locomotion planner goal

from director import transformUtils
from director.timercallback import TimerCallback
from director.asynctaskqueue import AsyncTaskQueue
from director import objectmodel as om
from director import visualization as vis
from director import applogic as app
from director.debugVis import DebugData
from director import ikplanner
from director.ikparameters import IkParameters
from director import ioUtils
from director.simpletimer import SimpleTimer
from director.utime import getUtime
from director import affordanceitems
from director import robotstate
from director import robotplanlistener
from director import segmentation
from director import planplayback
from director import affordanceupdater
from director import segmentationpanel
from director import vtkNumpy as vnp

from director.tasks.taskuserpanel import TaskUserPanel
from director.tasks.taskuserpanel import ImageBasedAffordanceFit

import director.tasks.robottasks as rt
import director.tasks.taskmanagerwidget as tmw

import director.rosutils as rosutils

import copy

from PythonQt import QtCore, QtGui

# ros focused
import rospy
import rospkg
rospack = rospkg.RosPack()

from std_msgs.msg import Int16
from std_msgs.msg import String
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Transform
from geometry_msgs.msg import PointStamped
from free_gait_msgs.msg import *


from rocoma_msgs.srv import SwitchController as rocomaSwitchController
from free_gait_msgs.srv import *


import signal
class Timeout():
    """Timeout class using ALARM signal."""
    class Timeout(Exception):
        pass

    def __init__(self, sec):
        self.sec = sec

    def __enter__(self):
        signal.signal(signal.SIGALRM, self.raise_timeout)
        signal.alarm(self.sec)

    def __exit__(self, *args):
        signal.alarm(0)    # disable alarm

    def raise_timeout(self, *args):
        raise Timeout.Timeout()


class AnymalPlanner(object):
    def __init__(self, robotSystem):
        self.robotSystem = robotSystem
        self.robotModel = robotSystem.robotStateModel

        self.active_controller = "unknown"

        # Send goal to lidar navigation or Send goal directly to position controller (bypass lidar navigation)
        self.fixedFrame = "map" # only set in the anymaldriverpanel

        try:
            app.addToolbarMacro('square up anymal', self.squareUp)
        except AttributeError:
            pass

        # This is the only place in Director where a ROS node is started
        # If a roscore is running, this will connect or else skip ROS connections
        try:
            with Timeout(3):
                print "Attempting to initialize rospy (for 3 seconds)"
                rospy.init_node('director_py', anonymous=True, disable_signals=True)
                print "Successfully connected to roscore"
        except Timeout.Timeout:
            print "Failed to connect to roscore! Starting Director without ROS"


    def switchController(self, controlType):
        print "Switch Controller to", controlType
        rospy.wait_for_service('/anymal_highlevel_controller/switch_controller',5)

        try:
            switch_control_srv = rospy.ServiceProxy('/anymal_highlevel_controller/switch_controller', rocomaSwitchController)
            resp1 = switch_control_srv(controlType)
            print "Switch Controller to", controlType, resp1
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def sendAction(self, action, preview=False):
        rospy.wait_for_service('/free_gait_action_loader/send_action',5)
        try:
            if preview:
                action_srv = rospy.ServiceProxy('/free_gait_action_loader/preview_action', SendAction)
            else:
                action_srv = rospy.ServiceProxy('/free_gait_action_loader/send_action', SendAction)
        
            sendAction = SendActionRequest()
            sendAction.goal.action_id = action

            resp1 = action_srv(sendAction)
            print "Free Gait Action:", action, resp1.result

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e


    def executeFreeGaitAction(self, actionName):
        print "switch to free_gait_impedance_ros controller"
        self.switchController('free_gait_impedance_ros')
        print "switch action to: " + actionName
        self.sendAction(actionName)

    # Square up the robot's stance
    # TODO: remove this function and just call executeFreeGaitAction
    def squareUp(self):
        self.executeFreeGaitAction("square_up")
