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

    def __init__(self, monoCamerasConfig, robotName):
        self.viewName = monoCamerasConfig[0]
        self.desiredWidth = 400
        self.position = [0, 0]
        self.usePicker = False
        self.imageView = None
        self.imagePicker = None
        self.robotName = robotName
        self._prevParent = None
        self._updateAspectRatio()

    def setWidth(self, width):
        self.desiredWidth = width
        self._updateAspectRatio()
        self.hide()
        self.show()

    def _updateAspectRatio(self):
        imageExtent = cameraview.imageManager.images[self.robotName][self.viewName].GetExtent()
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


class RobotSelector(QtGui.QWidget):
    """
    An object which stores various UI components associated with a robot and provides a combo box with the names
     of robots to facilitate switching between them and modifying UI appearance so that only components associated
     with the selected robot are displayed at any one time.
    """

    # Object types that can be associated with the object. Each one has a different method of hiding or changing
    # behaviour so that all actions affect only the selected robot.
    _objectTypes = ["widgets", "views", "models", "viewbehaviors"]

    def __init__(self, robotNames=[]):
        """
        Initialise the selector object
        :param robotNames: The names of robots to add to the combobox to start with
        """
        super(RobotSelector, self).__init__()
        self.objectName = "RobotSelector"
        self.robotNames = robotNames
        self.associatedWidgets = {}  # associated objects are stored here. Keys are robot names

        self.robotSelectLabel = QtGui.QLabel("Controlling:")

        self.robotSelectCombo = QtGui.QComboBox()

        for robotName in self.robotNames:
            self.addRobot(robotName)

        self.horizLayout = QtGui.QHBoxLayout(self)
        self.horizLayout.addWidget(self.robotSelectLabel)
        self.horizLayout.addWidget(self.robotSelectCombo)

        self.robotSelectCombo.connect("currentIndexChanged(QString)", self.showAssociatedComponents)

    def addRobot(self, robotName):
        self.robotNames.append(robotName)
        self.robotSelectCombo.addItem(robotName)
        self.associatedWidgets[robotName] = {}
        for objtype in self._objectTypes:
            self.associatedWidgets[robotName][objtype] = []

    def _associateObjectWithRobot(self, object, robotName, type):
        """
        Associate an object of a given type with a robot
        :param object: An object to associate with the robot
        :param robotName: The name of the robot with which to associate the object
        :param type: The type of the object - one of the keys in self.associatedWidgets
        :return:
        """
        if (robotName in self.robotNames) and (object not in self.associatedWidgets[robotName][type]):
            self.associatedWidgets[robotName][type].append(object)

    def associateWidgetWithRobot(self, widget, robotName):
        """
        Associate a qt widget with the given robot. If the robot name is one that has not been added, or if the widget
        has already been added to that robot, the widget will not be added
        :param widget: The widget to associate
        :param robotName: The robot name with which to associate the widget
        :return:
        """
        self._associateObjectWithRobot(widget, robotName, "widgets")

    def associateViewWithRobot(self, view, robotName):
        """
        Associate a view with a given robot. A view is something created by cameraview.py when adding a new camera
        perspective. If the robot name is one that has not been added, or if the view has already been added to
        that robot, the view will not be added
        :param view: The view to associate
        :param robotName: The robot name with which to associate the view
        :return:
        """
        self._associateObjectWithRobot(view, robotName, "views")

    def associateModelWithRobot(self, model, robotName):
        """
        Associate a model with a given robot. Models are PolyDataItem objects which can be displayed in the DRC view of
        director to display something.
        :param model: The model to associate
        :param robotName: The robot with which to associate the model
        :return:
        """
        self._associateObjectWithRobot(model, robotName, "models")

    def associateViewBehaviorWithRobot(self, viewBehavior, robotName):
        """
        Associate a RobotViewBehaviors object which controls UI input for a specific robot
        :param viewBehavior: A RobotViewBehaviors object to associate
        :param robotName: The robot with which to associate the RobotViewBehaviors object
        :return:
        """
        self._associateObjectWithRobot(viewBehavior, robotName, "viewbehaviors")

    def selectRobot(self, robotName):
        index = self.robotSelectCombo.findText(robotName)
        if index >= 0:
            self.robotSelectCombo.setCurrentIndex(index)

    def selectedRobotName(self):
        return self.robotSelectCombo.currentText

    def showAssociatedComponents(self, robotName):
        # If we have nothing in the dictionary, do nothing, otherwise may crash due to empty dicts.
        # TODO better checking of internal state of viewmanager to remove this check
        if not self.associatedWidgets:
            return
        # Have to update the page index cache before moving any of the components so that the hidden tabs are placed
        # in the correct order when they are shown
        app.getViewManager().updatePageIndexCache()
        # If there is an open dock widget, we must hide it independently of the action that it is attached to.
        # For simplicity, just hide all dock widgets when switching.
        # TODO remember open docks associated with each robot so that UI state is saved
        app.hideDockWidgets()
        # TODO use an object with a hide/show method to store widgets and other components so that the hiding method
        #  is associated with the object rather than being implemented here
        for robot in self.associatedWidgets.keys():
            for widget in self.associatedWidgets[robot]["widgets"]:
                widget.setVisible(robot == robotName)

            for view in self.associatedWidgets[robot]["views"]:
                if robot == robotName:
                    app.getViewManager().showView(view)
                else:
                    app.getViewManager().hideView(view, False)

            for model in self.associatedWidgets[robot]["models"]:
                model.setProperty('Visible', robot == robotName)

            for viewBehavior in self.associatedWidgets[robot]["viewbehaviors"]:
                # Setting the enabled flag to false will cause the event filter not to filter any events, allowing them
                # to pass to the event filter which is enabled, i.e. the one for the currently selected robot
                viewBehavior.robotViewBehaviors.setEnabled(robot == robotName)

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

directorConfigFull = drcargs.getDirectorConfig()

selector = RobotSelector()
# To hide the selector if there is only one robot we actually need to hide the action that is created by the
# toolbar's addwidget
selectorAction = app.getMainWindow().toolBar().addWidget(selector)

# If this is a single robot configuration, we expect modelName as a top level key. Otherwise it will be a second
# level one.
if "modelName" not in directorConfigFull:
    robotSystems = []
    for robot in directorConfigFull:
        robotSystems.append(robotsystem.create(view, robotName=robot))

    app.getMainWindow().toolBar().addSeparator()
else:
    robotSystems = [robotsystem.create(view)]
    selectorAction.setVisible(False)

setupScene = True  # The scene setup is done only once, unset this flag once it is done

for robotSystem in robotSystems:
    selector.addRobot(robotSystem.robotName)
    selector.associateViewBehaviorWithRobot(robotSystem.viewBehaviors, robotSystem.robotName)
    directorConfig = directorConfigFull[robotSystem.robotName] if robotSystem.robotName else directorConfigFull

    useIk = True
    usePerception = True
    useSpreadsheet = True
    useFootsteps = True
    useHands = False
    usePlanning = True
    useHumanoidDRCDemos = False
    useAtlasDriver = False
    useLightColorScheme = True
    useNavigationPanel = True
    useControllerRate = True
    useDataFiles = True
    useGamepad = False

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

    poseCollection = PythonQt.dd.ddSignalMap()
    costCollection = PythonQt.dd.ddSignalMap()

    if 'userConfig' in directorConfig:
        if 'fixedBaseArm' in directorConfig['userConfig']:
            robotSystem.ikPlanner.fixedBaseArm = True

    if 'disableComponents' in directorConfig:
        for component in directorConfig['disableComponents']:
            print "Disabling", component
            locals()[component] = False

    if 'enableComponents' in directorConfig:
        for component in directorConfig['enableComponents']:
            print "Enabling", component
            locals()[component] = True

    if useSpreadsheet:
        spreadsheet.init(poseCollection, costCollection, robotSystem.robotName)

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
        atlasdriverpanel.init(robotSystem.atlasDriver, robotSystem.robotName)

    if usePerception:
        segmentationpanel.init()
        cameraview.init(robotName=robotSystem.robotName)

        cameraview.cameraView.rayCallback = segmentation.extractPointsAlongClickRay

        if not useMultisense:
            app.removeToolbarMacro('ActionMultisensePanel')

    if setupScene:
        sceneRoot = om.getOrCreateContainer('scene')
        grid = vis.showGrid(view, color=[0, 0, 0], alpha=0.1, parent=sceneRoot)
        grid.setProperty('Surface Mode', 'Surface with edges')

        app.setBackgroundColor([0.3, 0.3, 0.35], [0.95, 0.95, 1])

        viewOptions = vis.ViewOptionsItem(view)
        om.addToObjectModel(viewOptions, parentObj=sceneRoot)

        viewBackgroundLightHandler = viewcolors.ViewBackgroundLightHandler(viewOptions, grid,
                                                                           app.getToolsMenuActions()[
                                                                               'ActionToggleBackgroundLight'])

        viewFramesHandler = viewframes.ViewFramesSizeHandler(app.getToolsMenuActions()['ActionToggleFramesSize'])

        if not useLightColorScheme:
            viewBackgroundLightHandler.action.trigger()
        setupScene = False

    # reset time button and connections
    button = QtGui.QPushButton('Reset time')
    button.setObjectName("resettime")
    button.connect('clicked()', robotSystem.pointCloudSource.resetTime)
    button.connect('clicked()', robotSystem.gridMapSource.resetTime)
    button.connect('clicked()', robotSystem.gridMapLidarSource.resetTime)
    button.connect('clicked()', robotSystem.headCameraPointCloudSource.resetTime)
    button.connect('clicked()', robotSystem.groundCameraPointCloudSource.resetTime)
    button.connect('clicked()', cameraview.cameraView.resetTime)
    app.getMainWindow().statusBar().addPermanentWidget(button)
    app.getRobotSelector().associateWidgetWithRobot(button, robotSystem.robotName)

    if useHands:
        handcontrolpanel.init(robotSystem.lHandDriver, robotSystem.rHandDriver, robotSystem.robotStateModel,
                              robotSystem.robotStateJointController, view)

    if useFootsteps:
        footstepsPanel = footstepsdriverpanel.init(robotSystem.footstepsDriver, robotSystem.robotStateModel,
                                                   robotSystem.robotStateJointController,
                                                   robotName=robotSystem.robotName)

    if useNavigationPanel:
        navigationPanel = navigationpanel.init(robotSystem.robotStateJointController, robotSystem.footstepsDriver,
                                               robotSystem.robotName)
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

    cameras = [camera['name'] for camera in directorConfig['sensors']['camera']['color']]
    imageOverlayManager = ImageOverlayManager(cameras, robotSystem.robotName)
    imageWidget = cameraview.ImageWidget(cameraview.imageManager, cameras, view, visible=False)
    imageViewHandler = ToggleImageViewHandler(imageWidget)

    screengrabberpanel.init(view, imageWidget, robotSystem.robotName)
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

    if useCourseModel:
        courseModel = coursemodel.CourseModel()

    if useKinect:
        imageOverlayManager.viewName = "KINECT_RGB"

    if useFeetlessRobot:
        robotSystem.ikPlanner.robotNoFeet = True

    for scriptArgs in drcargs.args().scripts:
        execfile(scriptArgs[0])

    selector.selectRobot(robotSystem.robotName)
