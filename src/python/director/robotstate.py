import numpy as np
import time
import re
from director import drcargs
from director import transformUtils

robotStates = {}


def getRobotState(robotName=None):
    global robotStates
    dictKey = robotName or "default";
    if not robotStates.get(dictKey):
        robotStates[dictKey] = RobotState(robotName)

    return robotStates[dictKey]


class RobotState(object):

    def __init__(self, robotName=None):
        self._robotStateToDrakePoseJointMap = None
        self._drakePoseToRobotStateJointMap = None
        self._drakePoseJointNames = None
        self._robotStateJointNames = None
        self._numPositions = None
        self.robotName = robotName

    def getRollPitchYawFromRobotState(self, robotState):
        return transformUtils.quaternionToRollPitchYaw(robotState[3:7])


    def getPositionFromRobotState(self, robotState):
        return robotState[0:4]


    def getRobotStateToDrakePoseJointMap(self):

        if self._robotStateToDrakePoseJointMap is None:
            robotStateJointNames = self.getRobotStateJointNames()
            drakePoseJointNames = self.getDrakePoseJointNames()

            self._robotStateToDrakePoseJointMap = dict()

            for robotStateJointIdx, robotStateJointName in enumerate(robotStateJointNames):
                drakeJointIdx = drakePoseJointNames.index(robotStateJointName)
                self._robotStateToDrakePoseJointMap[robotStateJointIdx] = drakeJointIdx

        return self._robotStateToDrakePoseJointMap


    def getDrakePoseToRobotStateJointMap(self):

        if self._drakePoseToRobotStateJointMap is None:
            self._drakePoseToRobotStateJointMap = dict()
            for key, value in self.getRobotStateToDrakePoseJointMap().iteritems():
                self._drakePoseToRobotStateJointMap[value] = key

        return self._drakePoseToRobotStateJointMap


    def convertStateMessageToDrakePose(self, msg, strict=True):
        '''
        If strict is true, then the state message must contain a joint_position
        for each named joint in the drake pose joint names.  If strict is false,
        then a default value of 0.0 is used to fill joint positions that are
        not specified in the robot state msg argument.
        '''

        jointMap = {}
        for name, position in zip(msg.joint_name, msg.joint_position):
            jointMap[name] = position

        jointPositions = []
        for name in self.getDrakePoseJointNames()[6:]:
            if strict:
                jointPositions.append(jointMap[name])
            else:
                jointPositions.append(jointMap.get(name, 0.0))

        trans = msg.pose.translation
        quat = msg.pose.rotation
        trans = [trans.x, trans.y, trans.z]
        quat = [quat.w, quat.x, quat.y, quat.z]
        rpy = transformUtils.quaternionToRollPitchYaw(quat)

        pose = np.hstack((trans, rpy, jointPositions))
        assert len(pose) == self.getNumPositions()
        return pose

    def convertStateMessageToDrakePoseBasic(self, trans, quat, jointNamesIn, jointPositionsIn, strict=True):
        '''
        If strict is true, then the state message must contain a joint_position
        for each named joint in the drake pose joint names.  If strict is false,
        then a default value of 0.0 is used to fill joint positions that are
        not specified in the robot state msg argument.
        This version is used for ROS messages
        '''

        jointMap = {}
        for name, position in zip(jointNamesIn, jointPositionsIn):
            jointMap[name] = position

        jointPositions = []
        for name in self.getDrakePoseJointNames()[6:]:
            if strict:
                jointPositions.append(jointMap[name])
            else:
                jointPositions.append(jointMap.get(name, 0.0))

        rpy = transformUtils.quaternionToRollPitchYaw(quat)

        pose = np.hstack((trans, rpy, jointPositions))
        assert len(pose) == self.getNumPositions()
        return pose


    def atlasCommandToDrakePose(self, msg):
        jointIndexMap = self.getRobotStateToDrakePoseJointMap()
        drakePose = np.zeros(len(self.getDrakePoseJointNames()))
        for jointIdx, drakeIdx in jointIndexMap.iteritems():
            drakePose[drakeIdx] = msg.position[jointIdx]
        return drakePose.tolist()


    def robotStateToDrakePose(self, robotState):

        drakePose = range(self.getNumPositions())
        jointIndexMap = self.getRobotStateToDrakePoseJointMap()

        pos = self.getPositionFromRobotState(robotState)
        rpy = self.getRollPitchYawFromRobotState(robotState)
        robotState = robotState[7:]

        assert len(jointIndexMap) == self.getNumJoints()
        assert len(robotState) >= len(jointIndexMap)

        for jointIdx in xrange(len(jointIndexMap)):
            drakePose[jointIndexMap[jointIdx]] = robotState[jointIdx]

        drakePose[0] = pos[0]
        drakePose[1] = pos[1]
        drakePose[2] = pos[2]
        drakePose[3] = rpy[0]
        drakePose[4] = rpy[1]
        drakePose[5] = rpy[2]

        return drakePose


    def matchJoints(self, regex):
        search = re.compile(regex).search
        return [name for name in self.getDrakePoseJointNames() if search(name)]


    def getDrakePoseJointNames(self):

        if not self._drakePoseJointNames:
            if not self.robotName:
                self._drakePoseJointNames = drcargs.getDirectorConfig()['drakeJointNames']
            else:
                self._drakePoseJointNames = drcargs.getDirectorConfig()[self.robotName]['drakeJointNames']

        return self._drakePoseJointNames

    def getRobotStateJointNames(self):

        if not self._robotStateJointNames:
            if not self.robotName:
                self._robotStateJointNames = drcargs.getDirectorConfig()['robotStateJointNames']
            else:
                self._robotStateJointNames = drcargs.getDirectorConfig()[self.robotName]['robotStateJointNames']

        return self._robotStateJointNames

    def getNumPositions(self):

        if self._numPositions is None:
            self._numPositions = len(self.getDrakePoseJointNames())

        return self._numPositions

    def getNumJoints(self):
        return self.getNumPositions() - 6
