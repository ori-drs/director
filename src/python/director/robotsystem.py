from director.componentgraph import ComponentFactory
from director.fieldcontainer import FieldContainer
from director import drcargs

class RobotSystemFactory(object):

    def getComponents(self):

        components = {
            'DirectorConfig' : [],
            'RobotState' : ['DirectorConfig'],
            'Segmentation' : [],
            'SegmentationRobotState' : ['Segmentation', 'RobotState'],
            'SegmentationAffordances' : ['Segmentation', 'Affordances'],
            'PerceptionDrivers' : ['RobotState'],
            'Footsteps' : ['RobotState'],
            'Planning' : ['RobotState'],#x
            'Playback' : ['Planning'],#x
            'Teleop' : ['Planning', 'Playback', 'Affordances'],#x
            'ConvexHullModel' : ['Playback'],#x
            'FootstepsPlayback' : ['Footsteps', 'Playback'],#x
            'Affordances' : [],
            #'ViewBehaviors' : ['Footsteps', 'PerceptionDrivers', 'Planning', 'Affordances'],
            'ViewBehaviors' : ['Footsteps', 'PerceptionDrivers', 'Affordances'], 
            'RobotLinkSelector' : ['ViewBehaviors']} #x

        disabledComponents = [
            'ConvexHullModel',
            'Teleop',
            'Planning',
            'Playback',
            'FootstepsPlayback',
            'RobotLinkSelector']

        return components, disabledComponents

    def initDirectorConfig(self, robotSystem):

        directorConfig = drcargs.getDirectorConfig()

        if 'colorMode' not in directorConfig:
            defaultColorMode = 'URDF Colors'
            directorConfig['colorMode'] = defaultColorMode

        assert directorConfig['colorMode'] in ['URDF Colors', 'Solid Color', 'Textures']

        return FieldContainer(directorConfig=directorConfig)

    def initRobotState(self, robotSystem):

        from director import roboturdf

        robotStateModel, robotStateJointController = roboturdf.loadRobotModel(
            'robot state model',
            robotSystem.view,
            #urdfFile=robotSystem.directorConfig['urdfConfig']['robotState'],
            color=roboturdf.getRobotGrayColor(),
            colorMode=robotSystem.directorConfig['colorMode'],
            parent='sensors',
            visible=True)

        #robotStateJointController.setPose('EST_ROBOT_STATE', robotStateJointController.getPose('q_nom'))
        #roboturdf.startModelPublisherListener([(robotStateModel, robotStateJointController)])

        return FieldContainer(robotStateModel=robotStateModel,
                                robotStateJointController=robotStateJointController)

    def initSegmentation(self, robotSystem):
        from director import segmentation

    def initSegmentationRobotState(self, robotSystem):

        from director import segmentationroutines
        segmentationroutines.SegmentationContext.initWithRobot(robotSystem.robotStateModel)

    def initPerceptionDrivers(self, robotSystem):

        from director import perception
        from director import robotstate


        pointCloudSource, gridMapSource, gridMapLidarSource, headCameraPointCloudSource, groundCameraPointCloudSource = perception.init(robotSystem.view,  robotSystem.robotStateJointController)


        spindleJoint = 'hokuyo_joint'

        def getSpindleAngleFunction():
            msg = robotSystem.robotStateJointController.lastRobotStateMessage
            if msg and spindleJoint in msg.joint_name:
                index = msg.joint_name.index(spindleJoint)
                return (float(msg.utime)/(1e6), msg.joint_position[index])
            else:
                return (0, 0)

        spindleMonitor = None
        #spindleMonitor = perception.SpindleMonitor(getSpindleAngleFunction)
        #robotSystem.robotStateModel.connectModelChanged(spindleMonitor.onRobotStateChanged)

        return FieldContainer(pointCloudSource=pointCloudSource,
                              gridMapSource=gridMapSource,
                              gridMapLidarSource=gridMapLidarSource,
                              headCameraPointCloudSource=headCameraPointCloudSource,
                              groundCameraPointCloudSource=groundCameraPointCloudSource)

    def initFootsteps(self, robotSystem):

        if 'useFootsteps' in drcargs.getDirectorConfig()['disableComponents']:
            footstepsDriver = None
        else:
            from director import footstepsdriver
            footstepsDriver = footstepsdriver.FootstepsDriver(robotSystem.robotStateJointController)

        return FieldContainer(footstepsDriver=footstepsDriver)

    def initPlanning(self, robotSystem):

        from director import objectmodel as om
        from director import roboturdf
        from director import ikplanner


        directorConfig = robotSystem.directorConfig

        ikRobotModel, ikJointController = roboturdf.loadRobotModel('ik model', urdfFile=directorConfig['urdfConfig']['ik'], parent=None)
        om.removeFromObjectModel(ikRobotModel)
        ikJointController.addPose('q_end', ikJointController.getPose('q_nom'))
        ikJointController.addPose('q_start', ikJointController.getPose('q_nom'))

        handFactory = roboturdf.HandFactory(robotSystem.robotStateModel)
        handModels = []

        for side in ['left', 'right']:
            if side in handFactory.defaultHandTypes:
                handModels.append(handFactory.getLoader(side))

        ikPlanner = ikplanner.IKPlanner(ikRobotModel, ikJointController, handModels)

        return FieldContainer(
            ikRobotModel=ikRobotModel,
            ikJointController=ikJointController,
            handFactory=handFactory,
            handModels=handModels,
            ikPlanner=ikPlanner,
            )

    def initConvexHullModel(self, robotSystem):

        from director import roboturdf

        directorConfig = robotSystem.directorConfig
        chullRobotModel, chullJointController = roboturdf.loadRobotModel('convex hull model', view, urdfFile=directorConfig['urdfConfig']['chull'], parent='planning', color=roboturdf.getRobotOrangeColor(), colorMode=directorConfig['colorMode'], visible=False)

        robotSystem.playbackJointController.models.append(chullRobotModel)

        return FieldContainer(
            chullRobotModel=chullRobotModel,
            chullJointController=chullJointController
            )

    def initPlayback(self, robotSystem):

        from director import roboturdf
        from director import planplayback
        from director import playbackpanel
        from director import robotplanlistener

        directorConfig = robotSystem.directorConfig

        manipPlanner = robotplanlistener.ManipulationPlanDriver(robotSystem.ikPlanner)

        playbackRobotModel, playbackJointController = roboturdf.loadRobotModel('playback model', robotSystem.view, urdfFile=directorConfig['urdfConfig']['playback'], parent='planning', color=roboturdf.getRobotOrangeColor(), visible=False, colorMode=directorConfig['colorMode'])

        planPlayback = planplayback.PlanPlayback()

        playbackPanel = playbackpanel.PlaybackPanel(planPlayback, playbackRobotModel, playbackJointController,
                                          robotSystem.robotStateModel, robotSystem.robotStateJointController, manipPlanner)

        manipPlanner.connectPlanReceived(playbackPanel.setPlan)


        return FieldContainer(
            playbackRobotModel=playbackRobotModel,
            playbackJointController=playbackJointController,
            planPlayback=planPlayback,
            manipPlanner=manipPlanner,
            playbackPanel=playbackPanel
            )

    def initTeleop(self, robotSystem):

        from director import roboturdf
        from director import teleoppanel

        directorConfig = robotSystem.directorConfig

        teleopRobotModel, teleopJointController = roboturdf.loadRobotModel('teleop model', robotSystem.view, urdfFile=directorConfig['urdfConfig']['teleop'], parent='planning', color=roboturdf.getRobotBlueColor(), visible=False, colorMode=directorConfig['colorMode'])


        teleopPanel = teleoppanel.TeleopPanel(robotSystem.robotStateModel, robotSystem.robotStateJointController, teleopRobotModel, teleopJointController,
                          robotSystem.ikPlanner, robotSystem.manipPlanner, robotSystem.affordanceManager, robotSystem.playbackPanel.setPlan, robotSystem.playbackPanel.hidePlan, robotSystem.planningUtils)

        return FieldContainer(
            teleopRobotModel=teleopRobotModel,
            teleopJointController=teleopJointController,
            teleopPanel=teleopPanel,
            )

    def initFootstepsPlayback(self, robotSystem):
        if 'useFootsteps' not in drcargs.getDirectorConfig()['disableComponents']:
            robotSystem.footstepsDriver.walkingPlanCallback = robotSystem.playbackPanel.setPlan

    def initAffordances(self, robotSystem):

        from director import affordancemanager
        from director import affordanceitems

        affordanceManager = affordancemanager.AffordanceObjectModelManager(robotSystem.view)
        affordanceitems.MeshAffordanceItem.getMeshManager()

        return FieldContainer(
            affordanceManager=affordanceManager,
            )

    def initSegmentationAffordances(self, robotSystem):

        from director import segmentation
        segmentation.affordanceManager = robotSystem.affordanceManager


    def initRobotLinkSelector(self, robotSystem):

        from director import robotlinkselector
        robotLinkSelector = robotlinkselector.RobotLinkSelector()
        robotSystem.viewBehaviors.addHandler(
            robotSystem.viewBehaviors.LEFT_DOUBLE_CLICK_EVENT,
            robotLinkSelector.onLeftDoubleClick)
        return FieldContainer(robotLinkSelector=robotLinkSelector)

    def initViewBehaviors(self, robotSystem):

        from director import applogic
        from director import robotviewbehaviors

        viewBehaviors = robotviewbehaviors.RobotViewBehaviors(robotSystem.view, robotSystem)
        applogic.resetCamera(viewDirection=[-1,0,0], view=robotSystem.view)
        return FieldContainer(viewBehaviors=viewBehaviors)


def create(view=None, globalsDict=None, options=None, planningOnly=False):
    '''
    Convenience function for initializing a robotSystem
    with the default options and populating a globals()
    dictionary with all the constructed objects.
    '''

    from director import applogic

    view = view or applogic.getCurrentRenderView()

    factory = ComponentFactory()
    factory.register(RobotSystemFactory)
    options = options or factory.getDefaultOptions()

    if planningOnly:
        options = factory.getDisabledOptions()
        factory.setDependentOptions(options, usePlannerPublisher=True, useTeleop=True)

    robotSystem = factory.construct(options, view=view)

    if globalsDict is not None:
        globalsDict.update(dict(robotSystem))

    return robotSystem
