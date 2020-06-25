from director.componentgraph import ComponentFactory
from director.fieldcontainer import FieldContainer
from director import drcargs


class RobotSystemFactory(object):
    def getComponents(self):

        components = {
            "DirectorConfig": [],
            "RobotState": ["DirectorConfig"],
            "Segmentation": [],
            "SegmentationRobotState": ["Segmentation", "RobotState"],
            "SegmentationAffordances": ["Segmentation", "Affordances"],
            "PerceptionDrivers": ["RobotState"],
            "ConvexHullModel": [],  # x
            "Affordances": [],
            #'ViewBehaviors' : ['Footsteps', 'PerceptionDrivers', 'Planning', 'Affordances'],
            "ViewBehaviors": ["PerceptionDrivers", "Affordances"],
            "RobotLinkSelector": ["ViewBehaviors"],
        }  # x

        disabledComponents = ["ConvexHullModel", "RobotLinkSelector"]

        return components, disabledComponents

    def initDirectorConfig(self, robotSystem):

        directorConfig = drcargs.getRobotConfig(robotSystem.robotName)

        if "colorMode" not in directorConfig:
            defaultColorMode = "URDF Colors"
            directorConfig["colorMode"] = defaultColorMode

        assert directorConfig["colorMode"] in ["URDF Colors", "Solid Color", "Textures"]

        return FieldContainer(
            directorConfig=directorConfig, robotType=directorConfig["modelName"]
        )

    def initRobotState(self, robotSystem):

        from director import roboturdf
        from director import objectmodel as om

        robotStateModel, robotStateJointController = roboturdf.loadRobotModel(
            "robot state model",
            robotSystem.view,
            # urdfFile=robotSystem.directorConfig['urdfConfig']['robotState'],
            color=roboturdf.getRobotGrayColor(),
            colorMode=robotSystem.directorConfig["colorMode"],
            parent=om.getOrCreateContainer(
                "sensors", om.getOrCreateContainer(robotSystem.robotName)
            ),
            visible=True,
            robotName=robotSystem.robotName,
        )

        # robotStateJointController.setPose('EST_ROBOT_STATE', robotStateJointController.getPose('q_nom'))
        # roboturdf.startModelPublisherListener([(robotStateModel, robotStateJointController)])

        return FieldContainer(
            robotStateModel=robotStateModel,
            robotStateJointController=robotStateJointController,
        )

    def initSegmentation(self, robotSystem):
        from director import segmentation

    def initSegmentationRobotState(self, robotSystem):

        from director import segmentationroutines

        segmentationroutines.SegmentationContext.initWithRobot(
            robotSystem.robotStateModel, robotSystem.robotName
        )

    def initPerceptionDrivers(self, robotSystem):

        from director import perception

        perceptionSources = perception.init(
            robotSystem.view, robotSystem.robotStateJointController
        )

        # Expand dict to keyword args, robotSystem object will have objects accessible via keys set in config
        return FieldContainer(**perceptionSources)

    def initConvexHullModel(self, robotSystem):

        from director import roboturdf

        directorConfig = robotSystem.directorConfig
        chullRobotModel, chullJointController = roboturdf.loadRobotModel(
            "convex hull model",
            view,
            urdfFile=directorConfig["urdfConfig"]["chull"],
            parent="planning",
            color=roboturdf.getRobotOrangeColor(),
            colorMode=directorConfig["colorMode"],
            visible=False,
        )

        robotSystem.playbackJointController.models.append(chullRobotModel)

        return FieldContainer(
            chullRobotModel=chullRobotModel, chullJointController=chullJointController
        )

    def initAffordances(self, robotSystem):

        from director import affordancemanager
        from director import affordanceitems

        affordanceManager = affordancemanager.AffordanceObjectModelManager(
            robotSystem.view
        )
        affordanceitems.MeshAffordanceItem.getMeshManager()

        return FieldContainer(affordanceManager=affordanceManager,)

    def initSegmentationAffordances(self, robotSystem):

        from director import segmentation

        segmentation.affordanceManager = robotSystem.affordanceManager

    def initRobotLinkSelector(self, robotSystem):

        from director import robotlinkselector

        robotLinkSelector = robotlinkselector.RobotLinkSelector()
        robotSystem.viewBehaviors.addHandler(
            robotSystem.viewBehaviors.LEFT_DOUBLE_CLICK_EVENT,
            robotLinkSelector.onLeftDoubleClick,
        )
        return FieldContainer(robotLinkSelector=robotLinkSelector)

    def initViewBehaviors(self, robotSystem):

        from director import applogic
        from director import robotviewbehaviors

        viewBehaviors = robotviewbehaviors.RobotViewBehaviors(
            robotSystem.view, robotSystem
        )
        applogic.resetCamera(viewDirection=[-1, 0, 0], view=robotSystem.view)
        return FieldContainer(viewBehaviors=viewBehaviors)


def create(view=None, globalsDict=None, options=None, planningOnly=False, **kwargs):
    """
    Convenience function for initializing a robotSystem
    with the default options and populating a globals()
    dictionary with all the constructed objects.
    """

    from director import applogic

    view = view or applogic.getCurrentRenderView()

    factory = ComponentFactory()
    factory.register(RobotSystemFactory)
    options = options or factory.getDefaultOptions()

    if planningOnly:
        options = factory.getDisabledOptions()
        factory.setDependentOptions(options, usePlannerPublisher=True, useTeleop=True)

    robotSystem = factory.construct(options, view=view, **kwargs)

    if globalsDict is not None:
        globalsDict.update(dict(robotSystem))

    return robotSystem
