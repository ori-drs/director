# This script is executed in the main console namespace so
# that all the variables defined here become console variables.

from __future__ import division

import director
#from director import irisdriver

import os
import sys
import PythonQt
import json
from PythonQt import QtCore, QtGui
from time import time
import imp
import director.applogic as app
from director import drcargs
from director import vtkAll as vtk
from director import jointcontrol
from director import callbacks
from director import camerabookmarks
from director import cameracontrol
from director import cameracontrolpanel
from director import objectmodel as om
from director import transformUtils
from director import skybox
from director import perception
from director import segmentation
from director import cameraview
from director.fieldcontainer import FieldContainer
from director import robotstate
from director import roboturdf
from director import robotsystem
from director import affordancepanel
from director import filterUtils
from director import footstepsdriver
from director import footstepsdriverpanel
from director import framevisualization
from director import tasklaunchpanel
from director import viewcolors
from director import viewframes

from director import planplayback
from director import playbackpanel
from director import screengrabberpanel
from director import splinewidget
from director import vtkNumpy as vnp
from director import visualization as vis
from director import actionhandlers
from director.timercallback import TimerCallback
from director.pointpicker import PointPicker, ImagePointPicker
from director import segmentationpanel
from director.utime import getUtime
from director.shallowCopy import shallowCopy

from director import segmentationroutines


from director.tasks import taskmanagerwidget
from director.tasks.descriptions import loadTaskDescriptions

from collections import OrderedDict
import functools
import math

import numpy as np
from director.debugVis import DebugData
from director import ioutils as io

# remove feb 2019, when moving to roslaunch
#drcargs.requireStrict()
drcargs.args()
app.startup(globals())
om.init(app.getMainWindow().objectTree(), app.getMainWindow().propertiesPanel())
actionhandlers.init()

quit = app.quit
exit = quit
view = app.getDRCView()
camera = view.camera()
tree = app.getMainWindow().objectTree()
orbit = cameracontrol.OrbitController(view)
showPolyData = segmentation.showPolyData
updatePolyData = segmentation.updatePolyData


###############################################################################


robotSystem = robotsystem.create(view)
globals().update(dict(robotSystem))


useRobotState = True
usePerception = True
useGrid = True
useFootsteps = True
useHands = False
usePlanning = True
useCollections = False
useLightColorScheme = True
useNavigationPanel = True
useCameraFrustumVisualizer = True
useForceDisplay = True
useDataFiles = True

useSkybox = False
useBlackoutText = False
useLimitJointsSentToPlanner = False
useFeetlessRobot = False


poseCollection = PythonQt.dd.ddSignalMap()
costCollection = PythonQt.dd.ddSignalMap()

if 'userConfig' in drcargs.getDirectorConfig():
    if 'fixedBaseArm' in drcargs.getDirectorConfig()['userConfig']:
        ikPlanner.fixedBaseArm = True

if 'disableComponents' in drcargs.getDirectorConfig():
    for component in drcargs.getDirectorConfig()['disableComponents']:
        print "Disabling", component
        locals()[component] = False

if 'enableComponents' in drcargs.getDirectorConfig():
    for component in drcargs.getDirectorConfig()['enableComponents']:
        print "Enabling", component
        locals()[component] = True


if usePerception:
    segmentationpanel.init()
    cameraview.init()

    cameraview.cameraView.rayCallback = segmentation.extractPointsAlongClickRay


if useGrid:
    grid = vis.showGrid(view, color=[0,0,0], alpha=0.1)
    grid.setProperty('Surface Mode', 'Surface with edges')

app.setBackgroundColor([0.3, 0.3, 0.35], [0.95,0.95,1])

viewOptions = vis.ViewOptionsItem(view)
om.addToObjectModel(viewOptions, parentObj=om.findObjectByName('sensors'))

viewBackgroundLightHandler = viewcolors.ViewBackgroundLightHandler(viewOptions, grid,
                                app.getToolsMenuActions()['ActionToggleBackgroundLight'])

viewFramesHandler = viewframes.ViewFramesSizeHandler(app.getToolsMenuActions()['ActionToggleFramesSize'])

if not useLightColorScheme:
    viewBackgroundLightHandler.action.trigger()

# reset time button and connections
button = QtGui.QPushButton('Reset time')
button.setObjectName("resettime")
button.connect('clicked()', pointCloudSource.resetTime)
button.connect('clicked()', gridMapSource.resetTime)
button.connect('clicked()', gridMapLidarSource.resetTime)
button.connect('clicked()', headCameraPointCloudSource.resetTime)
button.connect('clicked()', groundCameraPointCloudSource.resetTime)
button.connect('clicked()', cameraview.cameraView.resetTime)
app.getMainWindow().statusBar().addPermanentWidget(button)

if useFootsteps:
    footstepsPanel = footstepsdriverpanel.init(footstepsDriver, robotStateModel, robotStateJointController)
else:
    app.removeToolbarMacro('ActionFootstepPanel')

if usePlanning:
    def showPose(pose):
        playbackRobotModel.setProperty('Visible', True)
        playbackJointController.setPose('show_pose', pose)

    def playPlan(plan):
        playPlans([plan])

    def playPlans(plans):
        planPlayback.stopAnimation()
        playbackRobotModel.setProperty('Visible', True)
        planPlayback.playPlans(plans, playbackJointController)

    def playManipPlan():
        playPlan(manipPlanner.lastManipPlan)

    def playWalkingPlan():
        playPlan(footstepsDriver.lastWalkingPlan)

    def plotManipPlan():
        planPlayback.plotPlan(manipPlanner.lastManipPlan)

    def planStand():
        ikPlanner.computeStandPlan(robotStateJointController.q)

    def planNominal():
        ikPlanner.computeNominalPlan(robotStateJointController.q)

    def planHomeStand():
        ''' Move the robot back to a safe posture, 1m above its feet, w/o moving the hands '''
        ikPlanner.computeHomeStandPlan(robotStateJointController.q, footstepsDriver.getFeetMidPoint(robotStateModel), 1.0167)

    def planHomeNominal():
        ''' Move the robot back to a safe posture, 1m above its feet, w/o moving the hands '''
        ikPlanner.computeHomeNominalPlan(robotStateJointController.q, footstepsDriver.getFeetMidPoint(robotStateModel), 1.0167)

    def planHomeNominalHyq():
        ''' Move the robot back to a safe posture, 0.627m above its feet '''
        ikPlanner.computeHomeNominalPlanQuadruped(robotStateJointController.q, footstepsDriver.getFeetMidPoint(robotStateModel), 0.627)

    def planHomeNominalAnymal():
        ''' Move the robot back to a safe posture, above the mid point of its 4 feet '''
        ikPlanner.computeHomeNominalPlanQuadruped(robotStateJointController.q, footstepsDriver.getFeetMidPoint(robotStateModel), 0.5)

    def approveRefit():
        for obj in om.getObjects():
            if isinstance(obj, segmentation.BlockAffordanceItem):
                if 'refit' in obj.getProperty('Name'):
                    originalObj = om.findObjectByName(obj.getProperty('Name').replace(' refit', ''))
                    if originalObj:
                        originalObj.params = obj.params
                        originalObj.polyData.DeepCopy(obj.polyData)
                        originalObj.actor.GetUserTransform().SetMatrix(obj.actor.GetUserTransform().GetMatrix())
                        originalObj.actor.GetUserTransform().Modified()
                        obj.setProperty('Visible', False)


    handJoints = []
    if drcargs.args().directorConfigFile.find('atlas') != -1:
        handJoints = roboturdf.getRobotiqJoints() + ['neck_ay']
    else:
        for handModel in ikPlanner.handModels:
            handJoints += handModel.handModel.model.getJointNames()
        # filter base joints out
        handJoints = [ joint for joint in handJoints if joint.find('base')==-1 ]

    if useLimitJointsSentToPlanner:
        planningUtils.clampToJointLimits = True

    def fitPosts():
        segmentation.fitVerticalPosts(segmentation.getCurrentRevolutionData())
        affordancePanel.onGetRaycastTerrain()

    ikPlanner.addPostureGoalListener(robotStateJointController)

    playbackpanel.addPanelToMainWindow(playbackPanel)

    if useBlackoutText:
        blackoutMonitor = blackoutmonitor.BlackoutMonitor(robotStateJointController, view, cameraview, mapServerSource)


    taskPanels = OrderedDict()

    splinewidget.init(view, handFactory, robotStateModel)


    taskManagerPanel = taskmanagerwidget.init()

    for taskDescription in loadTaskDescriptions():
        taskManagerPanel.taskQueueWidget.loadTaskDescription(taskDescription[0], taskDescription[1])
    taskManagerPanel.taskQueueWidget.setCurrentQueue('Task library')

    for obj in om.getObjects():
        obj.setProperty('Deletable', False)


if useSkybox:

    skyboxDataDir = os.path.expanduser('~/Downloads/skybox')
    imageMap = skybox.getSkyboxImages(skyboxDataDir)
    skyboxObjs = skybox.createSkybox(imageMap, view)
    skybox.connectSkyboxCamera(view)
    #skybox.createTextureGround(os.path.join(skyboxDataDir, 'Dirt_seamless.jpg'), view)
    #view.camera().SetViewAngle(60)


class RobotLinkHighligher(object):

    def __init__(self, robotModel):
        self.robotModel = robotModel
        self.previousColors = {}

    def highlightLink(self, linkName, color):

        currentColor = self.robotModel.model.getLinkColor(linkName)
        if not currentColor.isValid():
            return

        if linkName not in self.previousColors:
            self.previousColors[linkName] = currentColor

        alpha = self.robotModel.getProperty('Alpha')
        newColor = QtGui.QColor(color[0]*255, color[1]*255, color[2]*255, alpha*255)

        self.robotModel.model.setLinkColor(linkName, newColor)

    def dehighlightLink(self, linkName):

        color = self.previousColors.pop(linkName, None)
        if color is None:
            return

        color.setAlpha(self.robotModel.getProperty('Alpha')*255)
        self.robotModel.model.setLinkColor(linkName, color)

robotHighlighter = RobotLinkHighligher(robotStateModel)




if useDataFiles:

    for filename in drcargs.args().data_files:
        actionhandlers.onOpenFile(filename)


class ImageOverlayManager(object):

    def __init__(self):
        monoCameras = drcargs.getDirectorConfig()['monoCameras']
        self.viewName = monoCameras[0]
        self.desiredWidth = 400
        self.position = [0, 0]
        self.usePicker = False
        self.imageView = None
        self.imagePicker = None
        self._prevParent = None
        self._updateAspectRatio()

    def setWidth(self, width):
        self.desiredWidth = width
        self._updateAspectRatio()
        self.hide()
        self.show()

    def _updateAspectRatio(self):
        imageExtent = cameraview.imageManager.images[self.viewName].GetExtent()
        if imageExtent[1] != -1 and imageExtent[3] != -1:
            self.imageSize = [imageExtent[1]+1, imageExtent[3]+1]
            imageAspectRatio = self.imageSize[0] / self.imageSize[1]
            self.size = [self.desiredWidth, self.desiredWidth / imageAspectRatio]

    def show(self):
        if self.imageView:
            return

        imageView = cameraview.views[self.viewName]
        self.imageView = imageView
        self._prevParent = imageView.view.parent()

        self._updateAspectRatio()

        imageView.view.hide()
        imageView.view.setParent(view)
        imageView.view.resize(self.size[0], self.size[1])
        imageView.view.move(self.position[0], self.position[1])
        imageView.view.show()

        if self.usePicker:
            self.imagePicker = ImagePointPicker(imageView)
            self.imagePicker.start()

    def hide(self):
        if self.imageView:
            self.imageView.view.hide()
            self.imageView.view.setParent(self._prevParent)
            self.imageView.view.show()
            self.imageView = None
        if self.imagePicker:
            self.imagePicker.stop()


class ToggleImageViewHandler(object):

    def __init__(self, manager):
        self.action = app.getToolsMenuActions()['ActionToggleImageView']
        self.action.connect('triggered()', self.toggle)
        self.manager = manager

    def toggle(self):
        if self.action.checked:
            self.manager.show()
        else:
            self.manager.hide()


monoCameras = drcargs.getDirectorConfig()['monoCameras']
imageOverlayManager = ImageOverlayManager()
imageWidget = cameraview.ImageWidget(cameraview.imageManager, monoCameras, view, visible=False)
imageViewHandler = ToggleImageViewHandler(imageWidget)

screengrabberpanel.init(view, imageWidget)
#framevisualization.init(view)
affordancePanel = affordancepanel.init(view, affordanceManager, robotStateJointController)
cameraBooksmarksPanel = camerabookmarks.init(view)

cameraControlPanel = cameracontrolpanel.CameraControlPanel(view)
app.addWidgetToDock(cameraControlPanel.widget, action=None).hide()


def getLinkFrame(linkName, model=None):
    model = model or robotStateModel
    return model.getLinkFrame(linkName)


def getBotFrame(frameName):
    t = vtk.vtkTransform()
    t.PostMultiply()
    cameraview.imageManager.queue.getTransform(frameName, 'local', t)
    return t


def showLinkFrame(linkName, model=None):
    frame = getLinkFrame(linkName, model)
    if not frame:
        raise Exception('Link not found: ' + linkName)
    return vis.updateFrame(frame, linkName, parent='link frames')


def sendEstRobotState(pose=None):
    if pose is None:
        pose = robotStateJointController.q
    msg = robotstate.drakePoseToRobotState(pose)
    lcmUtils.publish('EST_ROBOT_STATE', msg)

estRobotStatePublisher = TimerCallback(callback=sendEstRobotState)



app.setCameraTerrainModeEnabled(view, True)
app.resetCamera(viewDirection=[-1,0,0], view=view)


import signal
def sendMatlabSigint():
    ikServer.comm.client.proc.send_signal(signal.SIGINT)


#app.addToolbarMacro('Ctrl+C MATLAB', sendMatlabSigint)


def drawCenterOfMass(model):
    stanceFrame = footstepsDriver.getFeetMidPoint(model)
    com = list(model.model.getCenterOfMass())
    com[2] = stanceFrame.GetPosition()[2]
    d = DebugData()
    d.addSphere(com, radius=0.015)
    obj = vis.updatePolyData(d.getPolyData(), 'COM %s' % model.getProperty('Name'), color=[1,0,0], visible=False, parent=model)




class RobotMoverWidget(object):
    def __init__(self, jointController):
        self.jointController = jointController
        pos, rpy = jointController.q[:3], jointController.q[3:6]
        t = transformUtils.frameFromPositionAndRPY(pos, np.degrees(rpy))
        self.frame = vis.showFrame(t, 'mover widget', scale=0.3)
        self.frame.setProperty('Edit', True)
        self.frame.connectFrameModified(self.onFrameModified)

    def onFrameModified(self, frame):
        pos, rpy = self.frame.transform.GetPosition(), transformUtils.rollPitchYawFromTransform(self.frame.transform)
        q = self.jointController.q.copy()
        q[:3] = pos
        q[3:6] = rpy
        self.jointController.setPose('moved_pose', q)


class RobotGridUpdater(object):

    def __init__(self, gridFrame, robotModel, jointController):
        self.gridFrame = gridFrame
        self.robotModel = robotModel
        self.jointController = jointController
        self.robotModel.connectModelChanged(self.updateGrid)
        self.z_offset = 0.627 # for Husky # 0.85 for Atlas

    def setZOffset(self, z_offset):
        self.z_offset = z_offset
        self.updateGrid(None)

    def updateGrid(self, model):
        pos = self.jointController.q[:3]

        x = int(np.round(pos[0])) / 10
        y = int(np.round(pos[1])) / 10
        z = np.round( (pos[2]-self.z_offset)*10.0 ) / 10.0


        t = vtk.vtkTransform()
        t.Translate((x*10,y*10,z))
        self.gridFrame.copyFrame(t)

gridUpdater = RobotGridUpdater(grid.getChildFrame(), robotStateModel, robotStateJointController)

if useFeetlessRobot:
    ikPlanner.robotNoFeet = True

for scriptArgs in drcargs.args().scripts:
    execfile(scriptArgs[0])
