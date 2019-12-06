import PythonQt
from PythonQt import QtCore, QtGui, QtUiTools
from PyQt5.QtCore import QObject, pyqtSignal, pyqtSlot
from director import applogic as app
from director.utime import getUtime
from director import transformUtils
from director import objectmodel as om
from director import visualization as vis
from director.debugVis import DebugData
from director import ioUtils
from director.filterUtils import *
import director.tasks.robottasks as rt
import director.rosutils as rosutils

import os
import numpy as np

import rospy
from std_srvs.srv import *
from std_msgs.msg import Bool
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray

import dynamic_reconfigure.client

def addWidgetsToDict(widgets, d):

    for widget in widgets:
        if widget.objectName:
            d[str(widget.objectName)] = widget
        addWidgetsToDict(widget.children(), d)


class WidgetDict(object):

    def __init__(self, widgets):
        addWidgetsToDict(widgets, self.__dict__)



class AnymalDriverPanel(QObject):
    # a signal has to be declared outside the constructor
    # signal used to notify observers when the frame changed
    signal_frame = pyqtSignal(str)

    def __init__(self, planner):
        QObject.__init__(self)
        self.planner = planner

        loader = QtUiTools.QUiLoader()
        uifile = QtCore.QFile(os.path.join(os.path.dirname(__file__), 'ui/ddAnymalDriverPanel.ui'))
        assert uifile.open(uifile.ReadOnly)

        self.widget = loader.load(uifile)
        self.widget.setWindowTitle('Anymal Panel')
        self.ui = WidgetDict(self.widget.children())

        # Main Panel
        self.ui.sendPoseWalkingGoalButton.connect('pressed()', self.sendPoseWalkingGoalButton)
        self.ui.useElevationMapCheckBox.connect('pressed()', self.useElevationMapCheckBox)

        self.ui.turnLeft10Button.connect('pressed()', self.turnLeft10Button)
        self.ui.turnRight10Button.connect('pressed()', self.turnRight10Button)
        self.ui.turnLeft90Button.connect('pressed()', self.turnLeft90Button)
        self.ui.turnRight90Button.connect('pressed()', self.turnRight90Button)
        self.ui.turnInPlaceSpinBox.valueChanged.connect(self.turnInPlaceSpinBoxChange)

        self.ui.stopExecutionButton.connect('pressed()', self.stopExecutionButton)

        self.ui.squareUpButton.connect('pressed()', self.squareUpButton)
        self.ui.freezeRobotButton.connect('pressed()', self.freezeRobotButton)
        self.ui.standUpButton.connect('pressed()', self.standUpButton)
        self.ui.lieDownButton.connect('pressed()', self.lieDownButton)

        self.ui.lockLieStandCheck.connect('clicked()', self.onLockLieStand)
        self.lockLieStand()
        self.onLockLieStand()

        self.ui.pauseplayGazeboButton.connect('pressed()', self.pauseplayGazeboButton)
        self.ui.resetGazeboButton.connect('pressed()', self.resetGazeboButton)
        self.pauseGazeboState = False
        self.ui.clearEstopButton.connect('pressed()', self.clearEstopButton)

        self.ui.resetElevationMapButton.connect('pressed()', self.resetElevationMapButton)
        self.ui.resetEstimatorHereButton.connect('pressed()', self.resetEstimatorHereButton)

        self.ui.useMapFrameButton.connect('pressed()', self.useMapFrameButton)
        self.ui.useOdomFrameButton.connect('pressed()', self.useOdomFrameButton)

        # Print high level state of controller in status bar
        #statusBar = app.getMainWindow().statusBar()
        #self.label = QtGui.QLabel('')
        #statusBar.addPermanentWidget(self.label)
        rospy.Subscriber("/anymal_highlevel_controller/notify_active_controller", String, self.notifyActiveControllerCallback)

        self.ui.pollPositionControllerParamsButton.connect('pressed()', self.pollPositionControllerParamsButton)
        self.ui.sendPositionControllerParamsButton.connect('pressed()', self.sendPositionControllerParamsButton)

        self.addWalkingWidget()


    def notifyActiveControllerCallback(self, msg):
        self.planner.active_controller = msg.data
        #self.label.text = self.planner.active_controller
        app.getMainWindow().statusBar().showMessage(self.planner.active_controller, 0)


    def addWalkingWidget(self):

        w = QtGui.QWidget()
        l = QtGui.QHBoxLayout(w)

        label = QtGui.QLabel('Walking:')
        execButton = QtGui.QPushButton('')
        execButton.setIcon(QtGui.QApplication.style().standardIcon(QtGui.QStyle.SP_MediaPlay))
        stopButton = QtGui.QPushButton('')
        stopButton.setIcon(QtGui.QApplication.style().standardIcon(QtGui.QStyle.SP_MediaStop))
        clearButton = QtGui.QPushButton('')
        clearButton.setIcon(QtGui.QApplication.style().standardIcon(QtGui.QStyle.SP_TrashIcon))

        l.addWidget(label)
        l.addWidget(execButton)
        l.addWidget(stopButton)
        l.addWidget(clearButton)
        l.setContentsMargins(0, 0, 0, 0)

        execButton.setShortcut(QtGui.QKeySequence('Ctrl+Return'))
        execButton.connect('clicked()', self.sendPoseWalkingGoalButton)
        clearButton.connect('clicked()', self.clearWalkingPlanButton)
        stopButton.connect('clicked()', self.stopExecutionButton)
        self.toolbarWidget = app.getMainWindow().toolBar().addWidget(w)


    def pollPositionControllerParamsButton(self):
        # get params from position controller
        self.positionControllerReconfigure = dynamic_reconfigure.client.Client("position_controller", timeout=5)
        config = self.positionControllerReconfigure.get_configuration(timeout=5)
        self.ui.maxForwardLinearVelocitySpinBox.value = config.get('max_forward_linear_velocity')
        self.ui.maxLateralLinearVelocitySpinBox.value = config.get('max_lateral_linear_velocity')
        self.ui.maxAngularVelocitySpinBox.value = config.get('max_angular_velocity')
        self.ui.maxTurningLinearVelocitySpinBox.value = config.get('max_turning_linear_velocity')
        self.ui.trotModeComboBox.currentIndex = config.get('motion_mode')
        self.ui.goalBehindModeComboBox.currentIndex = config.get('goal_behind_mode')


    def sendPositionControllerParamsButton(self):
        # send params to position controller
        self.positionControllerReconfigure = dynamic_reconfigure.client.Client("position_controller", timeout=5)
        paramUpdate = dict()
        paramUpdate['max_forward_linear_velocity'] = self.ui.maxForwardLinearVelocitySpinBox.value
        paramUpdate['max_lateral_linear_velocity'] = self.ui.maxLateralLinearVelocitySpinBox.value
        paramUpdate['max_angular_velocity'] = self.ui.maxAngularVelocitySpinBox.value
        paramUpdate['max_turning_linear_velocity'] = self.ui.maxTurningLinearVelocitySpinBox.value
        paramUpdate['motion_mode'] = self.ui.trotModeComboBox.currentIndex
        paramUpdate['goal_behind_mode'] = self.ui.goalBehindModeComboBox.currentIndex
        #print "sending:",paramUpdate
        #print "sending param", paramName , "=", paramValue
        #self.positionControllerReconfigure.update_configuration({paramName:paramValue})
        self.positionControllerReconfigure.update_configuration(paramUpdate)


    def configureAnymal(self):
        vo = om.findObjectByName('view options')
        vo.setProperty('Gradient background',True)
        vo.setProperty('Background color',[0.0, 0.0, 0.0])
        vo.setProperty('Background color 2',[0.3, 0.3, 0.3])

        grid = om.findObjectByName('grid')
        grid.setProperty('Surface Mode','Wireframe')
        grid.setProperty('Alpha',0.05)
        grid.setProperty('Color',[1.0, 1.0, 1.0])
        grid.setProperty('Color By',0)

        ms = om.findObjectByName('Multisense')
        ms.setProperty('Min Range', 0.7)
        ms.setProperty('Max Range', 30.0)
        ms.setProperty('Max Height', 1.5)
        ms.setProperty('Edge Filter Angle', 0.0)
        ms.setProperty('Point Size', 2.0)
        ms.setProperty('Color By', 'Z Coordinate')

    def useElevationMapCheckBox(self):
        if (self.ui.useElevationMapCheckBox.checked):
            self.planner.useElevationMap = False
        else:
            self.planner.useElevationMap = True

    def turnLeft10Button(self):
        self.ui.turnInPlaceSpinBox.value = self.ui.turnInPlaceSpinBox.value + 10

    def turnRight10Button(self):
        self.ui.turnInPlaceSpinBox.value = self.ui.turnInPlaceSpinBox.value - 10

    def turnLeft90Button(self):
        self.ui.turnInPlaceSpinBox.value = self.ui.turnInPlaceSpinBox.value + 90

    def turnRight90Button(self):
        self.ui.turnInPlaceSpinBox.value = self.ui.turnInPlaceSpinBox.value - 90

    def turnInPlaceSpinBoxChange(self):

        turnWalkingTransform = self.planner.getStanceFrame()
        turnTransform = transformUtils.frameFromPositionAndRPY([0,0,0], [0,0,self.ui.turnInPlaceSpinBox.value])
        turnWalkingTransform.Concatenate(turnTransform)
        vis.updateFrame(turnWalkingTransform, 'walking goal')
      
    def stopExecutionButton(self):
        self.planner.stopExecutionButton()

    def clearWalkingPlanButton(self):
        om.removeFromObjectModel(om.findObjectByName('walking goal'))

    def sendPoseWalkingGoalButton(self):
        self.planner.changeWalkingMode(self.ui.walkingModeComboBox.currentText)
        goal = om.findObjectByName('walking goal')
        controllerMode = self.ui.walkingModeComboBox.currentText
        self.planner.sendLocomotionGoalMode(goal.transform, controllerMode)

        # Reset the turnInPlace spinner.
        self.ui.turnInPlaceSpinBox.value = 0

    def squareUpButton(self):
        self.planner.executeFreeGaitAction("square_up")

    def freezeRobotButton(self):
        self.planner.changeWalkingMode("Freeze")

    def standUpButton(self):
        self.planner.executeFreeGaitAction("stand_up")

    def lieDownButton(self):
        self.planner.executeFreeGaitAction("lie_down")

    def pauseplayGazeboButton(self):
        self.pauseGazeboState = not self.pauseGazeboState
        if (self.pauseGazeboState):
            self.pauseGazebo()
        else:
            self.playGazebo()

    def pauseGazebo(self):
        rospy.wait_for_service('/gazebo/pause_physics',5)
        try:
            pause_srv = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
            sendPause = EmptyRequest()
            resp1 = pause_srv(sendPause)

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def playGazebo(self):
        rospy.wait_for_service('/gazebo/unpause_physics',5)
        try:
            pause_srv = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
            sendPause = EmptyRequest()
            resp1 = pause_srv(sendPause)

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def resetGazeboButton(self):
        rospy.wait_for_service('/gazebo/reset_world',5)
        try:
            pause_srv = rospy.ServiceProxy('/gazebo/reset_world', Empty)
            sendPause = EmptyRequest()
            resp1 = pause_srv(sendPause)

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def clearEstopButton(self):
        rospy.wait_for_service('/anymal_highlevel_controller/clear_emergency_stop',5)
        try:
            thisSrv = rospy.ServiceProxy('/anymal_highlevel_controller/clear_emergency_stop', Trigger)
            thisRequest = TriggerRequest()
            resp1 = thisSrv(thisRequest)
            print resp1
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def resetElevationMapButton(self):
        print "wait for service: /elevation_mapping/clear_map"
        rospy.wait_for_service('/elevation_mapping/clear_map',5)
        print "done wait. will try"
        try:
            thisSrv = rospy.ServiceProxy('/elevation_mapping/clear_map', Empty)
            thisRequest = EmptyRequest()
            resp1 = thisSrv(thisRequest)
            print resp1
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def resetEstimatorHereButton(self):
        print "wait for service: /state_estimator/reset_here"
        rospy.wait_for_service('/state_estimator/reset_here',5)
        print "done wait. will try"
        try:
            thisSrv = rospy.ServiceProxy('/state_estimator/reset_here', Trigger)
            thisRequest = TriggerRequest()
            resp1 = thisSrv(thisRequest)
            print resp1
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def useMapFrameButton(self):
        self.setFixedFrame('map')
        self.signal_frame.emit('map')

    def useOdomFrameButton(self):
        self.setFixedFrame('odom')
        self.signal_frame.emit('odom')

    def setFixedFrame(self, fixedFrame):
        self.planner.robotSystem.robotStateJointController.fixedFrame = fixedFrame
        self.planner.robotSystem.pointCloudSource.reader.SetFixedFrame(fixedFrame)
        self.planner.robotSystem.gridMapSource.reader.SetFixedFrame(fixedFrame)        
        self.planner.robotSystem.headCameraPointCloudSource.reader.SetFixedFrame(fixedFrame)
        self.planner.robotSystem.groundCameraPointCloudSource.reader.SetFixedFrame(fixedFrame)
        self.planner.fixedFrame = fixedFrame

    def lockLieStand(self):
        self.ui.lieStandFrame.setEnabled(True)

    def unlockLieStand(self):
        self.ui.lieStandFrame.setEnabled(False)

    def onLockLieStand(self):
        if self.ui.lockLieStandCheck.checked:
            self.lockLieStand()
        else:
            self.unlockLieStand()


def _getAction():

    actionName = 'ActionAnymalDriverPanel'
    action = app.getToolBarActions().get(actionName)

    if action is None:

        icon = QtGui.QIcon(os.path.join(os.path.dirname(__file__), 'images/anymal.png'))
        assert os.path.isfile(os.path.join(os.path.dirname(__file__), 'images/anymal.png'))

        action = QtGui.QAction(icon, 'Ros Operator Panel', None)
        action.objectName = 'ActionAnymalDriverPanel'
        action.checkable = True

        mainWindow = app.getMainWindow()
        toolbar = mainWindow.panelToolBar()

        toolbar.insertAction(toolbar.actions()[0], action)

    return action


def init(planner):

    global panel
    global dock

    panel = AnymalDriverPanel(planner)
    dock = app.addWidgetToDock(panel.widget, action=_getAction())
    dock.hide()

    return panel
