import abc
import threading
import time

import director.objectmodel as om
import director.rosutils as rosutils
from director import visualization as vis
import director.applogic as app
from director import transformUtils
from director.debugVis import DebugData
from director.shallowCopy import deepCopy
from director.timercallback import TimerCallback
import vtkRosPython as vtkRos

from PythonQt import QtCore, QtGui

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseArray
import rospy
import tf, tf2_ros

import vtk



class PathSource(om.ContainerItem):
    """
        A class used to draw a nav_msgs/Path
    """
    def __init__(self, name, topicName, rootFrame):
        om.ContainerItem.__init__(self, name)


        self.topicName = topicName
        self.readerPath = vtkRos.vtkRosPathSubscriber()
        self.readerPath.SetFixedFrame(rootFrame)
        self.timerPath = TimerCallback()
        self.timerPath.callback = self.showPath
        defaultTubeWidth = 0.03
        defaultScale = 0.35
        useTube = False
        self.addProperty('Tube Width', defaultTubeWidth, attributes=om.PropertyAttributes(decimals=3, minimum=0.001, maximum=10, singleStep=0.01, hidden=True))
        self.addProperty('Tube', useTube)
        self.addProperty('Scale', defaultScale, attributes=om.PropertyAttributes(decimals=2, minimum=0.01, maximum=100, singleStep=0.1, hidden=False))
        self.readerPath.SetTubeWidth(defaultTubeWidth)
        self.readerPath.UseTube(useTube)
        self.readerPath.SetScale(defaultScale)

        om.collapse(self)

    def setRootFrame(self, frame):
        self.rootFrame = frame
        self.readerPath.SetFixedFrame(frame)


    def unregister(self):
        self.timerPath.stop()
        self.readerPath.Stop()
        om.removeFromObjectModel(self)

    def register(self):
        om.addToObjectModel(self)
        self.timerPath.start()
        self.readerPath.Start(self.topicName)

    def showPath(self):
        frames = vtk.vtkPolyData()
        lines = vtk.vtkPolyData()
        self.readerPath.GetMeshes(frames, lines, True)
        self._displayPolyData(frames, "frames")
        self._displayPolyData(lines, "lines")


    def _displayPolyData(self, polyData, name):
        if polyData.GetNumberOfPoints() != 0:
            obj = vis.updatePolyData(polyData, name, parent=self)
            # set the color of path
            colorList = obj.properties.getPropertyAttribute('Color By', 'enumNames')
            colorIndex = colorList.index('Color') if 'Color' in colorList else 0
            obj.setProperty('Color By', colorIndex)

    def _onPropertyChanged(self, propertySet, propertyName):
        om.ContainerItem._onPropertyChanged(self, propertySet, propertyName)
        propertyValue = self.getProperty(propertyName)
        if propertyName == 'Tube Width':
            self.readerPath.SetTubeWidth(propertyValue)
        elif propertyName == 'Tube':
            self.readerPath.UseTube(propertyValue)
            self.properties.setPropertyAttribute('Tube Width', 'hidden', not self.getProperty(propertyName))
        elif propertyName == 'Scale':
            self.readerPath.SetScale(propertyValue)










