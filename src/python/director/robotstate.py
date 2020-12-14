import numpy as np
import time
import re
from director import drcargs
from director import transformUtils

robotStates = {}


def getRobotState(robotName=""):
    global robotStates
    dictKey = robotName
    if not robotStates.get(dictKey):
        robotStates[dictKey] = RobotState(robotName)

    return robotStates[dictKey]


class RobotState(object):

    # All robots have a position in 3d space which is represented by these values
    base_pose_names = [
        "base_x",
        "base_y",
        "base_z",
        "base_roll",
        "base_pitch",
        "base_yaw",
    ]

    def __init__(self, robotName=""):
        self._jointNames = None
        self.robotName = robotName

    def convertStateMessageToDrakePoseBasic(
        self, trans, quat, jointNamesIn, jointPositionsIn, strict=True
    ):
        """
        If strict is true, then the state message must contain a joint_position
        for each named joint in the drake pose joint names.  If strict is false,
        then a default value of 0.0 is used to fill joint positions that are
        not specified in the robot state msg argument.
        This version is used for ROS messages
        """

        jointMap = {}
        for name, position in zip(jointNamesIn, jointPositionsIn):
            jointMap[name] = position

        jointPositions = []
        for name in self.getStateJointNames():
            if strict:
                jointPositions.append(jointMap[name])
            else:
                jointPositions.append(jointMap.get(name, 0.0))

        rpy = transformUtils.quaternionToRollPitchYaw(quat)

        pose = np.hstack((trans, rpy, jointPositions))
        assert len(pose) == self.getStateLength()
        return pose

    def getStateJointNames(self):
        if not self._jointNames:
            self._jointNames = drcargs.getRobotConfig(self.robotName)["jointNames"]

        return self._jointNames

    def getStateAndPoseFieldNames(self):
        return self.getStateJointNames() + self.getPoseFieldNames()

    def getPoseFieldNames(self):
        return self.base_pose_names

    def getStateLength(self):
        return self.getNumJoints() + len(self.base_pose_names)

    def getNumJoints(self):
        return len(self._jointNames)
