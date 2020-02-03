import os
import math
from director.timercallback import TimerCallback
from director.simpletimer import SimpleTimer
from director import robotstate
from director import getDRCBaseDir
from director import transformUtils
from director import visualization as vis
#import bot_core
import numpy as np

import rospy
from anymal_msgs.msg import *
import PythonQt

class JointController(object):

    def __init__(self, models, poseCollection=None, jointNames=None, pushToModel=True):
        self.jointNames = jointNames or robotstate.getDrakePoseJointNames()
        self.numberOfJoints = len(self.jointNames)
        self.models = list(models)
        self.poses = {}
        self.poseCollection = poseCollection
        self.currentPoseName = None
        self.lastRobotStateMessage = None
        self.ignoreOldStateMessages = False
        self.setPose('q_zero', np.zeros(self.numberOfJoints), pushToModel)

    def setJointPosition(self, jointId, position):
        '''
        Set joint position in degrees.
        '''
        assert jointId >= 0 and jointId < len(self.q)
        self.q[jointId] = math.radians(position % 360.0)
        self.push()

    def push(self):
        for model in self.models:
            model.model.setJointPositions(self.q, self.jointNames)

    def setPose(self, poseName, poseData=None, pushToModel=True):
        if poseData is not None:
            self.addPose(poseName, poseData)
        if poseName not in self.poses:
            raise Exception('Pose %r has not been defined.' % poseName)
        self.q = self.poses[poseName]
        self.currentPoseName = poseName
        if pushToModel:
            self.push()

    def setZeroPose(self):
        self.setPose('q_zero')

    def getPose(self, poseName):
        return self.poses.get(poseName)

    def addPose(self, poseName, poseData):
        assert len(poseData) == self.numberOfJoints
        self.poses[poseName] = np.asarray(poseData)
        if self.poseCollection is not None:
            self.poseCollection.setItem(poseName, poseData)

    def resetTime(self):
        self.subscriberRos.resetTime()

    def loadPoseFromFile(self, filename):
        ext = os.path.splitext(filename)[1].lower()

        if ext == '.mat':
            import scipy.io
            matData = scipy.io.loadmat(filename)
            pose = np.array(matData['xstar'][:self.numberOfJoints].flatten(), dtype=float)
        elif ext == '.csv':
            pose = np.loadtxt(filename, delimiter=',', dtype=float).flatten()
        else:
            raise Exception('Unsupported pose file format: %s' % filename)

        assert pose.shape[0] == self.numberOfJoints
        return pose

    def addLCMUpdater(self, channelName):
        '''
        adds an lcm subscriber to update the joint positions from
        lcm robot_state_t messages
        '''

        def onRobotStateMessage(msg):
            if self.ignoreOldStateMessages and self.lastRobotStateMessage is not None and msg.utime < self.lastRobotStateMessage.utime:
                return
            poseName = channelName
            pose = robotstate.convertStateMessageToDrakePose(msg)
            self.lastRobotStateMessage = msg

            # use joint name/positions from robot_state_t and append base_{x,y,z,roll,pitch,yaw}
            jointPositions = np.hstack((msg.joint_position, pose[:6]))
            jointNames = msg.joint_name + robotstate.getDrakePoseJointNames()[:6]

            self.setPose(poseName, pose, pushToModel=False)
            for model in self.models:
                model.model.setJointPositions(jointPositions, jointNames)

        def convertRosTransformToVtk(rosTransform):
            # rosTransform is x,y,z,qx,qy,qz,qw
            quat_xyzw = rosTransform[3:7]
            quat_wxyz = [quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]]
            return transformUtils.transformFromPose(rosTransform[0:3],quat_wxyz)


        def onRobotStateMessageRos(msg):
            channelName = "quadruped_state"
            # this is base in odom
            robotTransform = convertRosTransformToVtk(self.subscriberRos.getRobotPose())

            if (self.fixedFrame is "map"):
                odomInMapTransform = convertRosTransformToVtk(self.subscriberRos.getOdomInMap())
                robotInMap = transformUtils.copyFrame(robotTransform)
                robotInMap.PostMultiply()
                robotInMap.Concatenate( odomInMapTransform )
                robotPosition, robotOrientation = transformUtils.poseFromTransform(robotInMap)
            else:
                robotPosition, robotOrientation = transformUtils.poseFromTransform(robotTransform)


            jointNamesIn = list(self.subscriberRos.getJointNames())
            jointPositionsIn = self.subscriberRos.getJointPositions()

            #if self.ignoreOldStateMessages and self.lastRobotStateMessage is not None and msg.utime < self.lastRobotStateMessage.utime:
            #    return
            poseName = channelName
            pose = robotstate.convertStateMessageToDrakePoseBasic(robotPosition, robotOrientation, jointNamesIn, jointPositionsIn)
            #self.lastRobotStateMessage = msg

            # use joint name/positions from robot_state_t and append base_{x,y,z,roll,pitch,yaw}
            jointPositions = np.hstack((jointPositionsIn, pose[:6]))
            jointNames = jointNamesIn + robotstate.getDrakePoseJointNames()[:6]

            self.setPose(poseName, pose, pushToModel=False)
            for model in self.models:
                model.model.setJointPositions(jointPositions, jointNames)

        def remove_launchfile_generated_args(arg_strings):
            # remove args created by ROS
            new_arg_strings = []
            for arg_string in arg_strings:
                if not arg_string.startswith('__name:=') and not arg_string.startswith('__log:='):
                    if "director/director" not in arg_string: # application name
                        new_arg_strings.append(arg_string)
            return new_arg_strings



        new_arg_strings = remove_launchfile_generated_args(sys.argv)
        self.subscriberRos = PythonQt.dd.ddROSStateSubscriber(new_arg_strings, "/state_estimator/anymal_state")
        self.subscriberRos.connect('messageReceived(const QString&)', onRobotStateMessageRos)
        self.subscriberRos.setSpeedLimit(60)
        self.fixedFrame = "map"





class JointControlTestRamp(TimerCallback):

    def __init__(self, jointController):
        TimerCallback.__init__(self)
        self.controller = jointController
        self.testTime = 2.0

    def testJoint(self, jointId):
        self.jointId = jointId
        self.testTimer = SimpleTimer()
        self.start()

    def tick(self):

        if self.testTimer.elapsed() > self.testTime:
            self.stop()
            return

        jointPosition = math.sin( (self.testTimer.elapsed() / self.testTime) * math.pi) * math.pi
        self.controller.setJointPosition(self.jointId, math.degrees(jointPosition))
