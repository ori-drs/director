# This script is executed in the main console namespace so
# that all the variables defined here become console variables.

from __future__ import division

import os
from collections import OrderedDict

import PythonQt
import director.applogic as app
import numpy as np
from PythonQt import QtGui
from director import actionhandlers
from director import affordancepanel
from director import atlasdriverpanel
from director import blackoutmonitor
from director import camerabookmarks
from director import cameracontrol
from director import cameracontrolpanel
from director import cameraview
from director import continuouswalkingdemo
from director import coursemodel
from director import debrisdemo
from director import drcargs
from director import drilldemo
from director import footstepsdriverpanel
from director import framevisualization
from director import gamepad
from director import handcontrolpanel
from director import motionplanningpanel
from director import multisensepanel
from director import navigationpanel
from director import objectmodel as om
from director import perception
from director import playbackpanel
from director import quadrupedtask
from director import robotsystem
from director import roboturdf
from director import screengrabberpanel
from director import segmentation
from director import segmentationpanel
from director import skybox
from director import splinewidget
from director import spreadsheet
from director import surprisetask
from director import teleoppanel
from director import terraintask
from director import tfvisualization as tf_vis
from director import viewcolors
from director import viewframes
from director import visualization as vis
from director import vtkAll as vtk
from director import walkingtestdemo
from director.debugVis import DebugData
from director.jointpropagator import JointPropagator
from director.pointpicker import PointPicker, ImagePointPicker
from director.tasks import robottasks as rt
from director.tasks import taskmanagerwidget
from director.tasks.descriptions import loadTaskDescriptions
from director.timercallback import TimerCallback


class RobotLinkHighlighter(object):

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
        newColor = QtGui.QColor(color[0] * 255, color[1] * 255, color[2] * 255, alpha * 255)

        self.robotModel.model.setLinkColor(linkName, newColor)

    def dehighlightLink(self, linkName):

        color = self.previousColors.pop(linkName, None)
        if color is None:
            return

        color.setAlpha(self.robotModel.getProperty('Alpha') * 255)
        self.robotModel.model.setLinkColor(linkName, color)


class ControllerRateLabel(object):
    """
    Displays a controller frequency in the status bar
    """

    def __init__(self, atlasDriver, statusBar):
        self.atlasDriver = atlasDriver
        self.label = QtGui.QLabel('')
        statusBar.addPermanentWidget(self.label)

        self.timer = TimerCallback(targetFps=1)
        self.timer.callback = self.showRate
        self.timer.start()

    def showRate(self):
        rate = self.atlasDriver.getControllerRate()
        rate = 'unknown' if rate is None else '%d hz' % rate
        self.label.text = 'Controller rate: %s' % rate


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
            self.imageSize = [imageExtent[1] + 1, imageExtent[3] + 1]
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


class IgnoreOldStateMessagesSelector(object):

    def __init__(self, jointController):
        self.jointController = jointController
        self.action = app.addMenuAction('Tools', 'Ignore Old State Messages')
        self.action.setCheckable(True)
        self.action.setChecked(self.jointController.ignoreOldStateMessages)
        self.action.connect('triggered()', self.toggle)

    def toggle(self):
        self.jointController.ignoreOldStateMessages = bool(self.action.checked)


class RobotGridUpdater(object):

    def __init__(self, gridFrame, robotModel, jointController):
        self.gridFrame = gridFrame
        self.robotModel = robotModel
        self.jointController = jointController
        self.robotModel.connectModelChanged(self.updateGrid)
        self.z_offset = 0.627  # for Husky # 0.85 for Atlas

    def setZOffset(self, z_offset):
        self.z_offset = z_offset
        self.updateGrid(None)

    def updateGrid(self, model):
        pos = self.jointController.q[:3]

        x = int(np.round(pos[0])) / 10
        y = int(np.round(pos[1])) / 10
        z = np.round((pos[2] - self.z_offset) * 10.0) / 10.0

        t = vtk.vtkTransform()
        t.Translate((x * 10, y * 10, z))
        self.gridFrame.copyFrame(t)


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

robotSystem = robotsystem.create(view)

useIk = True
useRobotState = True
usePerception = True
useGrid = True
useSpreadsheet = True
useFootsteps = True
useHands = False
usePlanning = True
useHumanoidDRCDemos = False
useAtlasDriver = False
useOctomap = True
useCollections = False
useLightColorScheme = True
useLoggingWidget = False
useDrakeVisualizer = False
useNavigationPanel = True
useFallDetectorVis = True
useCameraFrustumVisualizer = True
useControllerRate = True
useForceDisplay = True
useDataFiles = True
useGamepad = False
useRandomWalk = True
useCOPMonitor = False

useQuadrupedDemos = False
useSkybox = False
useFootContactVis = False
useBlackoutText = False
useCourseModel = False
useLimitJointsSentToPlanner = False
useFeetlessRobot = False
useCOMMonitor = True

# Sensor Flags
useKinect = False
useMultisense = True
useOpenniDepthImage = False
usePointCloudSource = True

poseCollection = PythonQt.dd.ddSignalMap()
costCollection = PythonQt.dd.ddSignalMap()

if 'userConfig' in drcargs.getDirectorConfig():
    if 'fixedBaseArm' in drcargs.getDirectorConfig()['userConfig']:
        robotSystem.ikPlanner.fixedBaseArm = True

if 'disableComponents' in drcargs.getDirectorConfig():
    for component in drcargs.getDirectorConfig()['disableComponents']:
        print "Disabling", component
        locals()[component] = False

if 'enableComponents' in drcargs.getDirectorConfig():
    for component in drcargs.getDirectorConfig()['enableComponents']:
        print "Enabling", component
        locals()[component] = True

if useSpreadsheet:
    spreadsheet.init(poseCollection, costCollection)

if useIk:
    def onIkStartup(ikServer, startSuccess):
        if startSuccess:
            app.getMainWindow().statusBar().showMessage('Planning server started.', 2000)
        else:
            app.showErrorMessage('Error detected while starting the matlab planning server. '
                                 'Please check the output console for more information.', title='Error starting matlab')


    robotSystem.ikServer.outputConsole = app.getOutputConsole()
    robotSystem.ikServer.infoFunc = app.displaySnoptInfo
    robotSystem.ikServer.connectStartupCompleted(onIkStartup)
    robotSystem.startIkServer()

if useAtlasDriver:
    atlasdriverpanel.init(robotSystem.atlasDriver)
else:
    app.removeToolbarMacro('ActionAtlasDriverPanel')

if usePerception:
    segmentationpanel.init()
    cameraview.init()

    cameraview.cameraView.rayCallback = segmentation.extractPointsAlongClickRay

    if not useMultisense:
        app.removeToolbarMacro('ActionMultisensePanel')

if useGrid:
    grid = vis.showGrid(view, color=[0, 0, 0], alpha=0.1)
    grid.setProperty('Surface Mode', 'Surface with edges')

app.setBackgroundColor([0.3, 0.3, 0.35], [0.95, 0.95, 1])

viewOptions = vis.ViewOptionsItem(view)
om.addToObjectModel(viewOptions, parentObj=om.findObjectByName('sensors'))

viewBackgroundLightHandler = viewcolors.ViewBackgroundLightHandler(viewOptions, grid,
                                                                   app.getToolsMenuActions()[
                                                                       'ActionToggleBackgroundLight'])

viewFramesHandler = viewframes.ViewFramesSizeHandler(app.getToolsMenuActions()['ActionToggleFramesSize'])

if not useLightColorScheme:
    viewBackgroundLightHandler.action.trigger()

# reset time button and connections
button = QtGui.QPushButton('')
button.text = 'Reset time'
button.connect('clicked()', robotSystem.pointCloudSource.resetTime)
button.connect('clicked()', robotSystem.gridMapSource.resetTime)
button.connect('clicked()', robotSystem.headCameraPointCloudSource.resetTime)
button.connect('clicked()', robotSystem.groundCameraPointCloudSource.resetTime)
button.connect('clicked()', robotSystem.robotStateJointController.resetTime)
button.connect('clicked()', cameraview.cameraView.resetTime)
button.connect('clicked()', tf_vis.TfFrameSync.resetTime)
app.getMainWindow().statusBar().addPermanentWidget(button)

if useHands:
    handcontrolpanel.init(robotSystem.lHandDriver, robotSystem.rHandDriver, robotSystem.robotStateModel,
                          robotSystem.robotStateJointController, view)
else:
    app.removeToolbarMacro('ActionHandControlPanel')

if useFootsteps:
    footstepsPanel = footstepsdriverpanel.init(robotSystem.footstepsDriver, robotSystem.robotStateModel,
                                               robotSystem.robotStateJointController)
else:
    app.removeToolbarMacro('ActionFootstepPanel')

if useNavigationPanel:
    navigationPanel = navigationpanel.init(robotSystem.robotStateJointController, robotSystem.footstepsDriver)
    picker = PointPicker(view, callback=navigationPanel.pointPickerStoredFootsteps, numberOfPoints=2)
    # picker.start()

if usePlanning:
    def showPose(pose):
        robotSystem.playbackRobotModel.setProperty('Visible', True)
        robotSystem.playbackJointController.setPose('show_pose', pose)


    def playPlan(plan):
        playPlans([plan])


    def playPlans(plans):
        robotSystem.planPlayback.stopAnimation()
        robotSystem.playbackRobotModel.setProperty('Visible', True)
        robotSystem.planPlayback.playPlans(plans, robotSystem.playbackJointController)


    def playManipPlan():
        playPlan(robotSystem.manipPlanner.lastManipPlan)


    def playWalkingPlan():
        playPlan(robotSystem.footstepsDriver.lastWalkingPlan)


    def plotManipPlan():
        robotSystem.planPlayback.plotPlan(robotSystem.manipPlanner.lastManipPlan)


    def planStand():
        robotSystem.ikPlanner.computeStandPlan(robotSystem.robotStateJointController.q)


    def planNominal():
        robotSystem.ikPlanner.computeNominalPlan(robotSystem.robotStateJointController.q)


    def planHomeStand():
        """ Move the robot back to a safe posture, 1m above its feet, w/o moving the hands """
        robotSystem.ikPlanner.computeHomeStandPlan(robotSystem.robotStateJointController.q,
                                                   robotSystem.footstepsDriver.getFeetMidPoint(
                                                       robotSystem.robotStateModel), 1.0167)


    def planHomeNominal():
        """ Move the robot back to a safe posture, 1m above its feet, w/o moving the hands """
        robotSystem.ikPlanner.computeHomeNominalPlan(robotSystem.robotStateJointController.q,
                                                     robotSystem.footstepsDriver.getFeetMidPoint(
                                                         robotSystem.robotStateModel), 1.0167)


    def planHomeNominalHyq():
        """ Move the robot back to a safe posture, 0.627m above its feet """
        robotSystem.ikPlanner.computeHomeNominalPlanQuadruped(robotSystem.robotStateJointController.q,
                                                              robotSystem.footstepsDriver.getFeetMidPoint(
                                                                  robotSystem.robotStateModel), 0.627)


    def planHomeNominalAnymal():
        """ Move the robot back to a safe posture, above the mid point of its 4 feet """
        robotSystem.ikPlanner.computeHomeNominalPlanQuadruped(robotSystem.robotStateJointController.q,
                                                              robotSystem.footstepsDriver.getFeetMidPoint(
                                                                  robotSystem.robotStateModel), 0.5)


    if useMultisense:
        def fitDrillMultisense():
            pd = om.findObjectByName('Multisense').model.revPolyData
            om.removeFromObjectModel(om.findObjectByName('debug'))
            segmentation.findAndFitDrillBarrel(pd)


        def refitBlocks(autoApprove=True):
            polyData = om.findObjectByName('Multisense').model.revPolyData
            segmentation.updateBlockAffordances(polyData)
            if autoApprove:
                approveRefit()


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
        for handModel in robotSystem.ikPlanner.handModels:
            handJoints += handModel.handModel.model.getJointNames()
        # filter base joints out
        handJoints = [joint for joint in handJoints if joint.find('base') == -1]

    teleopJointPropagator = JointPropagator(robotSystem.robotStateModel, robotSystem.teleopRobotModel, handJoints)
    playbackJointPropagator = JointPropagator(robotSystem.robotStateModel, robotSystem.playbackRobotModel, handJoints)


    def doPropagation(model=None):
        if robotSystem.teleopRobotModel.getProperty('Visible'):
            teleopJointPropagator.doPropagation()
        if robotSystem.playbackRobotModel.getProperty('Visible'):
            playbackJointPropagator.doPropagation()


    robotSystem.robotStateModel.connectModelChanged(doPropagation)

    if useLimitJointsSentToPlanner:
        robotSystem.planningUtils.clampToJointLimits = True

    jointLimitChecker = teleoppanel.JointLimitChecker(robotSystem.robotStateModel,
                                                      robotSystem.robotStateJointController)
    jointLimitChecker.setupMenuAction()
    jointLimitChecker.start()

    if useMultisense:
        spindleSpinChecker = multisensepanel.SpindleSpinChecker(spindleMonitor)
        spindleSpinChecker.setupMenuAction()

    postureShortcuts = teleoppanel.PosturePlanShortcuts(robotSystem.robotStateJointController, robotSystem.ikPlanner,
                                                        robotSystem.planningUtils)

    if useMultisense:
        def drillTrackerOn():
            om.findObjectByName('Multisense').model.showRevolutionCallback = fitDrillMultisense


        def drillTrackerOff():
            om.findObjectByName('Multisense').model.showRevolutionCallback = None


    def fitPosts():
        segmentation.fitVerticalPosts(segmentation.getCurrentRevolutionData())
        affordancePanel.onGetRaycastTerrain()


    robotSystem.ikPlanner.addPostureGoalListener(robotSystem.robotStateJointController)

    playbackpanel.addPanelToMainWindow(robotSystem.playbackPanel)
    teleoppanel.addPanelToMainWindow(robotSystem.teleopPanel)

    motionPlanningPanel = motionplanningpanel.init(robotSystem.planningUtils, robotSystem.robotStateModel,
                                                   robotSystem.robotStateJointController,
                                                   robotSystem.teleopRobotModel, robotSystem.teleopJointController,
                                                   robotSystem.ikPlanner, robotSystem.manipPlanner,
                                                   robotSystem.affordanceManager, robotSystem.playbackPanel.setPlan,
                                                   robotSystem.playbackPanel.hidePlan, robotSystem.footstepsDriver)

    if useGamepad:
        gamePad = gamepad.Gamepad(robotSystem.teleopPanel, robotSystem.teleopJointController, robotSystem.ikPlanner,
                                  view)

    if useBlackoutText:
        blackoutMonitor = blackoutmonitor.BlackoutMonitor(robotSystem.robotStateJointController, view, cameraview,
                                                          mapServerSource)

    taskPanels = OrderedDict()

    if useHumanoidDRCDemos:
        debrisDemo = debrisdemo.DebrisPlannerDemo(robotSystem.robotStateModel, robotSystem.robotStateJointController,
                                                  robotSystem.playbackRobotModel,
                                                  robotSystem.ikPlanner, robotSystem.manipPlanner,
                                                  robotSystem.atlasDriver.driver, robotSystem.lHandDriver,
                                                  perception.multisenseDriver, refitBlocks)

        drillDemo = drilldemo.DrillPlannerDemo(robotSystem.robotStateModel, robotSystem.playbackRobotModel,
                                               robotSystem.teleopRobotModel, robotSystem.footstepsDriver,
                                               robotSystem.manipPlanner, robotSystem.ikPlanner,
                                               robotSystem.lHandDriver, robotSystem.rHandDriver,
                                               robotSystem.atlasDriver.driver,
                                               perception.multisenseDriver,
                                               fitDrillMultisense, robotSystem.robotStateJointController,
                                               playPlans, robotSystem.teleopPanel.showPose, cameraview,
                                               segmentationpanel)
        drillTaskPanel = drilldemo.DrillTaskPanel(drillDemo)

        continuouswalkingDemo = continuouswalkingdemo.ContinousWalkingDemo(robotSystem.robotStateModel, footstepsPanel,
                                                                           robotSystem.footstepsDriver,
                                                                           robotSystem.playbackPanel,
                                                                           robotSystem.robotStateJointController,
                                                                           robotSystem.ikPlanner,
                                                                           robotSystem.teleopJointController,
                                                                           navigationPanel,
                                                                           cameraview)
        continuousWalkingTaskPanel = continuouswalkingdemo.ContinuousWalkingTaskPanel(continuouswalkingDemo)

        walkingDemo = walkingtestdemo.walkingTestDemo(robotSystem.robotStateModel, robotSystem.playbackRobotModel,
                                                      robotSystem.teleopRobotModel,
                                                      robotSystem.footstepsDriver, robotSystem.manipPlanner,
                                                      robotSystem.ikPlanner,
                                                      robotSystem.lHandDriver, robotSystem.rHandDriver,
                                                      robotSystem.atlasDriver.driver,
                                                      perception.multisenseDriver,
                                                      robotSystem.robotStateJointController,
                                                      playPlans, showPose)

        terrainTaskPanel = terraintask.TerrainTaskPanel(robotSystem)
        terrainTask = terrainTaskPanel.terrainTask

        surpriseTaskPanel = surprisetask.SurpriseTaskPanel(robotSystem)
        surpriseTask = surpriseTaskPanel.planner

        taskPanels['Terrain'] = terrainTaskPanel.widget
        taskPanels['Continuous Walking'] = continuousWalkingTaskPanel.widget

    if useQuadrupedDemos:
        quadrupedTaskPanel = quadrupedtask.QuadrupedTaskPanel(robotSystem)
        quadrupedTask = quadrupedTaskPanel.planner

        taskPanels['Quadruped'] = quadrupedTaskPanel.widget

    splinewidget.init(view, robotSystem.handFactory, robotSystem.robotStateModel)

    rt.robotSystem = robotSystem
    taskManagerPanel = taskmanagerwidget.init()

    for taskDescription in loadTaskDescriptions():
        taskManagerPanel.taskQueueWidget.loadTaskDescription(taskDescription[0], taskDescription[1])
    taskManagerPanel.taskQueueWidget.setCurrentQueue('Task library')

    for obj in om.getObjects():
        obj.setProperty('Deletable', False)

useControllerRate = False
if useControllerRate:
    controllerRateLabel = ControllerRateLabel(robotSystem.atlasDriver, app.getMainWindow().statusBar())

if useSkybox:
    skyboxDataDir = os.path.expanduser('~/Downloads/skybox')
    imageMap = skybox.getSkyboxImages(skyboxDataDir)
    skyboxObjs = skybox.createSkybox(imageMap, view)
    skybox.connectSkyboxCamera(view)

robotHighlighter = RobotLinkHighlighter(robotSystem.robotStateModel)

if useDataFiles:

    for filename in drcargs.args().data_files:
        actionhandlers.onOpenFile(filename)

monoCameras = drcargs.getDirectorConfig()['monoCameras']
imageOverlayManager = ImageOverlayManager()
imageWidget = cameraview.ImageWidget(cameraview.imageManager, monoCameras, view, visible=False)
imageViewHandler = ToggleImageViewHandler(imageWidget)

screengrabberpanel.init(view, imageWidget)
framevisualization.init(view)
affordancePanel = affordancepanel.init(view, robotSystem.affordanceManager, robotSystem.robotStateJointController)
cameraBooksmarksPanel = camerabookmarks.init(view)

cameraControlPanel = cameracontrolpanel.CameraControlPanel(view)
app.addWidgetToDock(cameraControlPanel.widget, action=None).hide()

app.setCameraTerrainModeEnabled(view, True)
app.resetCamera(viewDirection=[-1, 0, 0], view=view)


def drawCenterOfMass(model):
    stanceFrame = robotSystem.footstepsDriver.getFeetMidPoint(model)
    com = list(model.model.getCenterOfMass())
    com[2] = stanceFrame.GetPosition()[2]
    d = DebugData()
    d.addSphere(com, radius=0.015)
    obj = vis.updatePolyData(d.getPolyData(), 'COM %s' % model.getProperty('Name'), color=[1, 0, 0], visible=False,
                             parent=model)


def initCenterOfMassVisualization():
    for model in [robotSystem.robotStateModel, robotSystem.teleopRobotModel,
                  robotSystem.robotSystem.playbackRobotModel]:
        model.connectModelChanged(drawCenterOfMass)
        drawCenterOfMass(model)


if useCOMMonitor:
    initCenterOfMassVisualization()

gridUpdater = RobotGridUpdater(grid.getChildFrame(), robotSystem.robotStateModel, robotSystem.robotStateJointController)

IgnoreOldStateMessagesSelector(robotSystem.robotStateJointController)

if useCourseModel:
    courseModel = coursemodel.CourseModel()

if useKinect:
    imageOverlayManager.viewName = "KINECT_RGB"

if useFeetlessRobot:
    robotSystem.ikPlanner.robotNoFeet = True

for scriptArgs in drcargs.args().scripts:
    execfile(scriptArgs[0])
