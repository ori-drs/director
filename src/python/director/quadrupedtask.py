import os
import sys
import vtkAll as vtk
import math
import time
import types
import functools
import numpy as np

from director import transformUtils
from director import lcmUtils
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
from director import quadrupedplanner

from director.tasks.taskuserpanel import TaskUserPanel
from director.tasks.taskuserpanel import ImageBasedAffordanceFit

import director.tasks.robottasks as rt
import director.tasks.taskmanagerwidget as tmw

import drc as lcmdrc
import copy

from PythonQt import QtCore, QtGui



class QuadrupedTaskPlanner(object):

    def __init__(self, robotSystem):
        self.robotSystem = robotSystem
        self.robotModel = robotSystem.robotStateModel
        self.ikPlanner = robotSystem.ikPlanner
        self.lockBackForManip = False
        self.lockBaseForManip = True


class ImageFitter(ImageBasedAffordanceFit):

    def __init__(self, quadrupedPlanner):
        ImageBasedAffordanceFit.__init__(self, numberOfPoints=1)
        self.quadrupedPlanner = quadrupedPlanner
        self.fitFunc = None
        self.pickLineRadius = 0.05
        self.pickNearestToCamera = False

        self.useLocalPlaneFit = True
        self.useVoxelGrid = True

    def fit(self, polyData, points):
        if self.fitFunc:
            self.fitFunc(polyData, points)

    def fitSwitchBox(self, polyData, points):
        boxPosition = points[0]
        wallPoint = points[1]


        # find a frame that is aligned with wall
        searchRadius = 0.2
        planePoints, normal = segmentation.applyLocalPlaneFit(polyData, points[0], searchRadius=np.linalg.norm(points[1] - points[0]), searchRadiusEnd=1.0)

        obj = vis.updatePolyData(planePoints, 'wall plane points', color=[0,1,0], visible=False)
        obj.setProperty('Point Size', 7)

        viewDirection = segmentation.SegmentationContext.getGlobalInstance().getViewDirection()
        if np.dot(normal, viewDirection) < 0:
            normal = -normal

        origin = segmentation.computeCentroid(planePoints)

        zaxis = [0,0,1]
        xaxis = normal
        yaxis = np.cross(zaxis, xaxis)
        yaxis /= np.linalg.norm(yaxis)
        zaxis = np.cross(xaxis, yaxis)
        zaxis /= np.linalg.norm(zaxis)

        t = transformUtils.getTransformFromAxes(xaxis, yaxis, zaxis)

        # translate that frame to the box position
        t.PostMultiply()
        t.Translate(boxPosition)

        boxFrame = transformUtils.copyFrame(t)
        self.quadrupedPlanner.spawnBoxAffordanceAtFrame(boxFrame)

class QuadrupedTaskPanel(TaskUserPanel):

    def __init__(self, robotSystem):

        TaskUserPanel.__init__(self, windowTitle='Quadruped Task')

        self.planner = QuadrupedTaskPlanner(robotSystem)
        self.quadrupedPlanner = quadrupedplanner.QuadrupedPlanner(robotSystem)
        self.fitter = ImageFitter(self.quadrupedPlanner)
        #self.initImageView(self.fitter.imageView)

        self.addDefaultProperties()
        self.addButtons()
        self.addTasks()


    def test(self):
        print 'test'

    def addButtons(self):

        self.addManualSpacer()
        self.addManualButton('body low', self.quadrupedPlanner.planBodyLow)
        self.addManualButton('look up', self.quadrupedPlanner.planLookUp)
        self.addManualButton('body nominal', self.quadrupedPlanner.planHomeNominal)
        self.addManualButton('spawn box affordance', self.quadrupedPlanner.spawnBoxAffordance)
        self.addManualButton('spawn box (distant)', self.quadrupedPlanner.spawnBoxAffordanceAtDistance)
        self.addManualButton('spawn footstep frame', self.quadrupedPlanner.spawnFootstepFrame)

    def onPlanPinchReach(self):
        self.quadrupedPlanner.planPinchReach(maxDegreesPerSecond=self.maxDegreesPerSecond)

    def addDefaultProperties(self):
        self.params.addProperty('max degrees per second', 10, attributes=om.PropertyAttributes(singleStep=1, decimals=2))


    def onPropertyChanged(self, propertySet, propertyName):

        self.syncProperties()

    def syncProperties(self):
        self.maxDegreesPerSecond = self.params.getProperty('max degrees per second')

    def setParamsPreTeleop(self):
        self.params.setProperty('max degrees per second', 30)

    def setParamsTeleop(self):
        self.params.setProperty('max degrees per second', 10)

    def addTasks(self):

        # some helpers
        self.folder = None
        def addTask(task, parent=None):
            parent = parent or self.folder
            self.taskTree.onAddTask(task, copy=False, parent=parent)
        def addFunc(name, func, parent=None):
            addTask(rt.CallbackTask(callback=func, name=name), parent=parent)
        def addFolder(name, parent=None):
            self.folder = self.taskTree.addGroup(name, parent=parent)
            return self.folder

        def addManipTask(name, planFunc, userPrompt=False):

            prevFolder = self.folder
            addFolder(name, prevFolder)
            addFunc('plan', planFunc)
            if not userPrompt:
                addTask(rt.CheckPlanInfo(name='check manip plan info'))
            else:
                addTask(rt.UserPromptTask(name='approve manip plan', message='Please approve manipulation plan.'))
            addFunc('execute manip plan', self.quadrupedPlanner.commitManipPlan)
            addTask(rt.WaitForManipulationPlanExecution(name='wait for manip execution'))
            self.folder = prevFolder


        self.taskTree.removeAllTasks()

        addFolder('Fit Box Affordance')
        addFunc('spawn affordance (distant)',self.quadrupedPlanner.spawnBoxAffordanceAtDistance)
        #addFunc('fit switch box affordance', self.fitSwitchBox)
        #addTask(rt.UserPromptTask(name='verify/adjust affordance', message='verify/adjust affordance.'))

        # walk to drill
        addFolder('Walk to Target')
        addFunc('plan footstep frame', self.quadrupedPlanner.spawnFootstepFrame)
        addTask(rt.RequestFootstepPlan(name='walk to drill', stanceFrameName='switch box stance frame'))
        #addTask(rt.UserPromptTask(name='approve footsteps', message='Please approve footstep plan.'))
        #addTask(rt.CommitFootstepPlan(name='walk to switch box', planName='switch box stance frame footstep plan'))
        #addTask(rt.WaitForWalkExecution(name='wait for walking'))

        twistAndShout = addFolder('Look at target')
        self.folder = twistAndShout
        #addFunc('plan body low', self.quadrupedPlanner.planBodyLow)
        addFunc('plan look up', self.quadrupedPlanner.planLookUp)
        addFunc('plan home nominal', self.quadrupedPlanner.planHomeNominal)


    def fitSwitchBox(self):
        print 'fitting switch box'
        self.fitter.imagePicker.numberOfPoints = 2
        self.fitter.pointCloudSource = 'lidar'
        self.fitter.fitFunc = self.fitter.fitSwitchBox