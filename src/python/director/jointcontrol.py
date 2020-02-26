import os
import math
from director.timercallback import TimerCallback
from director.simpletimer import SimpleTimer
from director import robotstate
from director import getDRCBaseDir
from director import transformUtils
from director import visualization as vis
import numpy as np


class JointController(object):

    def __init__(self, models, poseCollection=None, jointNames=None, robotName="", pushToModel=True):
        self.robotState = robotstate.getRobotState(robotName)
        self.robotName = robotName
        self.jointNames = jointNames or self.robotState.getDrakePoseJointNames()
        self.numberOfJoints = len(self.jointNames)
        self.models = list(models)
        self.poses = {}
        self.poseCollection = poseCollection
        self.currentPoseName = None
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
