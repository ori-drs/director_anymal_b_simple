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
from trajectory_msgs.msg import MultiDOFJointTrajectory
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint
#from high_level_actions import *
from actionlib_msgs.msg import GoalStatusArray
#from anymal_actions_drs.srv import ExecuteWalk
from locomotion_planner_msgs.msg import NavigateToGoalPoseActionGoal

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

        self.assignFrames()

        # Walking
        self.freeGaitGoalPub = rospy.Publisher('/free_gait_action_loader/drs_walk/goal_pose', PoseStamped, queue_size=10)

        # Locomotion Planner (uses height map). This topics is part of an action server
        self.locomotionPlannerGoalPub = rospy.Publisher('/locomotion_planner/goal_pose_action_server/goal', NavigateToGoalPoseActionGoal, queue_size=10)

        # Send goal to lidar navigation or Send goal directly to position controller (bypass lidar navigation)
        self.useElevationMap = False
        self.fixedFrame = "map" # only set in the anymaldriverpanel
        self.positionControllerGoalPubUsingHeightMap = rospy.Publisher('/lidar_exploration/new_ultimate_goal', PoseStamped, queue_size=10)       
        self.positionControllerGoalPubIgnoringHeightMap = rospy.Publisher('/position_controller/footstep_plan_request', PoseStamped, queue_size=10)

        self.positionControllerGoalPubUsingHeightMapList = rospy.Publisher('/lidar_exploration/new_ultimate_goal_list', PoseArray, queue_size=10)       
        self.positionControllerGoalPubIgnoringHeightMapList = rospy.Publisher('/position_controller/footstep_plan_request_list', PoseArray, queue_size=10)

        self.stopWalkingCommandPub = rospy.Publisher('/stop_walking_cmd', Int16, queue_size=10)

        # Body Motions
        self.bodyMotionCommandPub = rospy.Publisher('/free_gait_action_loader/walk/body_motion_command', Step, queue_size=10)
        self.bodyMotionCommandSequencePub = rospy.Publisher('/free_gait_action_loader/walk/body_motion_command_sequence', ExecuteStepsGoal, queue_size=10)

        # Mapping
        self.collectLidarSweepCommandPub = rospy.Publisher('/collect_lidar_sweep_command', Int16, queue_size=10)


        try:
            app.addToolbarMacro('square up anymal', self.squareUp)
        except AttributeError:
            pass

        rospy.Subscriber("/anymal_ctrl_free_gait/execute_steps/result", ExecuteStepsActionResult, self.goalStatusCallback)
        self.freeGaitContinuousComplete = False
        rospy.Subscriber("/position_controller/controller_status", Bool, self.positionControllerStatusCallback)
        self.positionControllerActive = False
        self.actionClient = None

        # This is the only place in Director where a ROS node is started
        # If a roscore is running, this will connect or else skip ROS connections
        try:
            with Timeout(3):
                print "Attempting to initialize rospy (for 3 seconds)"
                rospy.init_node('director_py', anonymous=True, disable_signals=True)
                print "Successfully connected to roscore"
        except Timeout.Timeout:
            print "Failed to connect to roscore! Starting Director without ROS"


    def goalStatusCallback(self, msg):
        #print "goal status received",  msg.status.status
        if (msg.status.status == 3): # complete
            print "goal finished: ",  msg.status.status
            self.freeGaitContinuousComplete = True

    def positionControllerStatusCallback(self,msg):
        self.positionControllerActive = msg.data

    def executeFreeGaitAction(self, actionName):
        print "switch to free_gait_impedance_ros controller"
        switch_controller('free_gait_impedance_ros')
        print "switch action to: " + actionName
        send_action(actionName)

    # Square up the robot's stance
    # TODO: remove this function and just call executeFreeGaitAction
    def squareUp(self):
        self.executeFreeGaitAction("square_up")


    ####################################################
    def getWalkingGoalTransformFromStairEdgeTransform(self, stairTransform):
        # Robot should walk/freegait to a fixed offset back from a staircase to climb it
        walkingGoalTransform = transformUtils.copyFrame(stairTransform)
        walkingGoalTransform.PreMultiply()
        walkingGoalTransform.Concatenate(self.stairEdgeToWalkingGoalFrame)
        return walkingGoalTransform


    def stopExecutionButton(self):
        if (self.active_controller=="trot_ros"):
            print "stop trotting"
            stop_msg = Int16()
            stop_msg.data = 2 # Terminate (TODO: use enum message)
            self.stopWalkingCommandPub.publish(stop_msg)
        elif (self.active_controller=="dynamic_gaits_ros"):
            print "stop dynamic_gaits_ros"
            stop_msg = Int16()
            stop_msg.data = 2 # Terminate (TODO: use enum message)
            self.stopWalkingCommandPub.publish(stop_msg)
        elif (self.active_controller=="free_gait_impedance_ros"):
            print "stop free gait"
            stop_free_gait_execution()
        else:
            print "I don't know what to stop"
            print "current controller is ", self.active_controller


    def changeWalkingMode(self, walkingMode):
        if (walkingMode == 'Free Gait'):
            print "switch to free_gait_impedance_ros controller (walk)"
            switch_controller("free_gait_impedance_ros")
        elif (walkingMode == 'Terrain Loco'):
            print "switch to free_gait_impedance_ros controller (loco)"
            switch_controller("free_gait_impedance_ros")
        elif (walkingMode == 'Dynamic Trot'):
            print "switch to dynamic_gaits_ros controller (trot)"
            switch_controller("dynamic_gaits_ros")
            switch_dynamic_gaits_gait("walk")
        elif (walkingMode == 'Trot'):
            print "switch to trot_ros::WalkingTrot controller"
            switch_controller("trot_ros")
            switch_trot_mode("walk")
        elif (walkingMode == 'Freeze'):
            print "switch to freeze controller"
            switch_controller("freeze")


    def trotToWalkingGoal(self):
        print "trotToWalkingGoal"
        self.changeWalkingMode('Dynamic Trot') # previously was 'Trot'
        walkingGoalTransform = om.findObjectByName('walking goal').transform
        self.sendLocomotionGoalMode(walkingGoalTransform, 'Dynamic Trot')


    def freeGaitToWalkingGoal(self):
        print "freeGaitToWalkingGoal"
        self.changeWalkingMode('Free Gait')
        walkingGoalTransform = om.findObjectByName('walking goal').transform
        self.sendLocomotionGoalMode(walkingGoalTransform, 'Free Gait')


    def sendLocomotionPlannerGoal(self, goal_pose):
        msg = NavigateToGoalPoseActionGoal()

        msg.header = goal_pose.header
        msg.goal_id.id = ''.join(random.choice(string.ascii_lowercase) for i in range(10))
        msg.goal.goal_pose = goal_pose
        self.locomotionPlannerGoalPub.publish(msg)


    def sendLocomotionGoalMode(self, goalTransform, controllerMode):
        goalTransformCopy = transformUtils.copyFrame(goalTransform)
        #vis.updateFrame(goalTransformCopy, 'locomotion goal', scale=0.2)
        pos,quat = transformUtils.poseFromTransform(goalTransformCopy)

        msg = PoseStamped()
        t = rospy.get_rostime()

        msg.header.stamp = t
        msg.header.frame_id = self.fixedFrame

        msg.pose = rosutils.rosPoseFromTransform( goalTransformCopy)

        if (controllerMode == 'Free Gait'):
            send_action('drs_walk')
            print "free gait goal pub"

            try:
                rospy.wait_for_service('/free_gait_action_loader/drs_walk/goal_pose')
                request_srv = rospy.ServiceProxy('/free_gait_action_loader/drs_walk/goal_pose', ExecuteWalk)
                resp = request_srv(msg)
                rospy.loginfo("Response: success=" + str(resp.success))
                print "Finished"
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e

        elif (controllerMode == 'Terrain Loco'):
            send_action('locomotion_planner')
            # triggering locomotion planner runs a different launch
            # this is not instant. this 2 sec sleep gives it time to finish
            # but it would be better to "wait until ready" before sending goal
            print "locomotion planner. sleep 2 seconds"
            time.sleep(2)
            print "locomotion planner goal pub"
            self.sendLocomotionPlannerGoal(msg)

        elif (controllerMode == 'Dynamic Trot'):
            if (self.useElevationMap):
                print "dynamic trot pub (using height map)"
                self.positionControllerGoalPubUsingHeightMap.publish(msg)
            else:
                print "dynamic trot pub (ignoring height map)"
                self.positionControllerGoalPubIgnoringHeightMap.publish(msg)

        elif (controllerMode == 'Trot'):
            if (self.useElevationMap):
                print "trot pub (using height map)"
                self.positionControllerGoalPubUsingHeightMap.publish(msg)
            else:
                print "trot pub (ignoring height map)"
                self.positionControllerGoalPubIgnoringHeightMap.publish(msg)

        else:
            print "cannot walk to goal with controller:", controllerMode
        return True


    def trotToWalkingGoalList(self):
        print "trotToWalkingGoalList"
        self.changeWalkingMode('Dynamic Trot')
        listChildren = om.findObjectByName('walking goal list').children()
        walkingGoalList = []
        for frame in listChildren:
            walkingGoalList.append(frame.transform)
        self.sendLocomotionGoalModeList(walkingGoalList, 'Dynamic Trot')


    def trotToWalkingGoalListRos(self, goalPoseArrayMsg):
        print "trotToWalkingGoalListRos"
        self.changeWalkingMode('Dynamic Trot')
        self.sendLocomotionGoalModeListRos(goalPoseArrayMsg, 'Dynamic Trot')


    def freeGaitToWalkingGoalList(self):
        print "freeGaitToWalkingGoalList - not implemented"


    def sendLocomotionGoalModeList(self, goalTransformList, controllerMode):
        msg = rosutils.rosPoseArrayFromTransformArray(goalTransformList, self.fixedFrame, rospy.get_rostime())
        self.sendLocomotionGoalModeListRos(msg, controllerMode)


    def sendLocomotionGoalModeListRos(self, goalPoseArrayMsg, controllerMode):

        if (controllerMode == 'Free Gait'):
            print "sendLocomotionGoalModeList with Free Gait - not implemented"
            return False
            #send_action('drs_walk')
            #print "free gait goal pub"
            #self.freeGaitGoalPub.publish(msg)
        elif (controllerMode == 'Trot'):
            if (self.useElevationMap):
                print "trot pub (using height map) - LIST"
                self.positionControllerGoalPubUsingHeightMapList.publish(goalPoseArrayMsg)
            else:
                print "trot pub (ignoring height map) - LIST"
                self.positionControllerGoalPubIgnoringHeightMapList.publish(goalPoseArrayMsg)
        elif (controllerMode == 'Dynamic Trot'):
            if (self.useElevationMap):
                print "dynamic trot pub (using height map) - LIST"
                self.positionControllerGoalPubUsingHeightMapList.publish(goalPoseArrayMsg)
            else:
                print "dynamic trot pub (ignoring height map) - LIST"
                self.positionControllerGoalPubIgnoringHeightMapList.publish(goalPoseArrayMsg)
        else:
            print "cannot walk to goal list with controller:", controllerMode
        return True


    def vtkTransformToRosTransform(self,transformIn):
        transformInCopy = transformUtils.copyFrame(transformIn)
        pos,quat = transformUtils.poseFromTransform(transformInCopy)

        transformOut = Transform()
        transformOut.translation.x = pos[0]
        transformOut.translation.y = pos[1]
        transformOut.translation.z = pos[2]
        transformOut.rotation.w = quat[0]
        transformOut.rotation.x = quat[1]
        transformOut.rotation.y = quat[2]
        transformOut.rotation.z = quat[3]
        return transformOut


    def sendBodyMotionCommandInFootprint(self, goalTransform, timeFromStart):
        # this is the same as sendBodyMotionCommand, but in the footprint frame
        # is the preferred command for basic actions because its effect is easy to see
        self.executeFreeGaitAction("body_motion_command")

        step = Step()
        base_trajectory = BaseTrajectory()
        trajectory = MultiDOFJointTrajectory()
        trajectory.header.frame_id = "footprint"
        trajectory.joint_names.append("base")

        point = MultiDOFJointTrajectoryPoint()
        transform = self.vtkTransformToRosTransform(goalTransform)
        point.transforms.append(transform)
        point.time_from_start = rospy.Duration.from_sec(timeFromStart)

        trajectory.points.append(point)
        base_trajectory.trajectory = trajectory
        step.base_trajectory.append(base_trajectory)

        print "free gait goal pub to body_motion_command"
        self.bodyMotionCommandPub.publish(step)

        return True


    def sendBodyMotionCommand(self, goalTransform, timeFromStart):
        self.executeFreeGaitAction("body_motion_command")

        step = Step()
        base_trajectory = BaseTrajectory()
        trajectory = MultiDOFJointTrajectory()
        trajectory.header.frame_id = "odom"
        trajectory.joint_names.append("base")

        point = MultiDOFJointTrajectoryPoint()
        transform = self.vtkTransformToRosTransform(goalTransform)
        point.transforms.append(transform)
        point.time_from_start = rospy.Duration.from_sec(timeFromStart)

        trajectory.points.append(point)
        base_trajectory.trajectory = trajectory
        step.base_trajectory.append(base_trajectory)

        print "free gait goal pub to body_motion_command"
        self.bodyMotionCommandPub.publish(step)

        return True


    def getFootMove(self, goalPosition, ignoreContact):
        # Get a basic motion of a Leg, with BaseAuto
        # this can be used to move the leg around in air
        # or to bring it into ground contact

        step = Step()
        base_auto = BaseAuto()
        step.base_auto.append(base_auto)

        footstep = Footstep()
        footstep.name = "RF_LEG"
        target = PointStamped()
        target.header.stamp.secs = 10
        target.header.frame_id = "odom"
        target.point.x = goalPosition[0]
        target.point.y = goalPosition[1]
        target.point.z = goalPosition[2]
        footstep.target = target

        footstep.profile_type = "straight"
        footstep.ignore_contact = ignoreContact
        footstep.average_velocity = 0.05

        step.footstep.append(footstep)
        return step


    def sendLegMotionCommand(self, goalPosition, ignoreContact=False):
        # this is simply move a foot to the goal position (in air)
        # ignoreContact = False will use the foot to stand on
        # ignoreContact = True will not use the foot to stand on
        self.executeFreeGaitAction("body_motion_command")
        step = self.getFootMove(goalPosition, ignoreContact)
        self.bodyMotionCommandPub.publish(step)


    def getStanceFrame(self):
        # the point on the ground between the feet
        return self.robotSystem.footstepsDriver.getFeetMidPoint(self.robotSystem.robotStateModel, useWorldZ=False)


    def assignFrames(self):
        # this is the distance from the bottom of any staircase, back to a safe pre-climb standing point
        self.stairEdgeToWalkingGoalFrame = transformUtils.frameFromPositionAndRPY([-0.5,0,0],[0,0,0])


    # Mapping functions below this
    def collectLidarSweep(self):
        self.executeFreeGaitAction('mapping_base_roll_40')

        print "collect lidar sweep"
        msg = Int16()
        msg.data = 13 # 12 second sweep. roll action is 13 plus 1 for margin
        self.collectLidarSweepCommandPub.publish(msg)
