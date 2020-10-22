import director
import os
import sys
import argparse
import yaml


class DRCArgParser(object):
    def __init__(self):
        self._args = None
        self._parser = None
        self.strict = False

    def getArgs(self):
        if self._args is None:
            self.parseArgs()
        return self._args

    def getParser(self):
        if self._parser is None:
            self._parser = argparse.ArgumentParser()
            self.addDefaultArgs(self._parser)
        return self._parser

    def parseArgs(self):
        parser = self.getParser()
        sys.argv = [str(v) for v in sys.argv]

        if not self.strict:
            self._args, unknown = parser.parse_known_args()
        else:
            self._args = parser.parse_args()

        def flatten(l):
            return [item for sublist in l for item in sublist]

        # now flatten some list of lists
        self._args.data_files = flatten(self._args.data_files)

    def getDefaultDirectorConfigFile(self):
        return self.getDefaultAnymalDirectorConfigFile()

    def getDefaultAtlasV3DirectorConfigFile(self):
        return os.path.join(
            director.getDRCBaseDir(), "models/atlas_v3/director_config.yaml"
        )

    def getDefaultAtlasV4DirectorConfigFile(self):
        return os.path.join(
            director.getDRCBaseDir(), "models/atlas_v4/director_config.yaml"
        )

    def getDefaultAtlasV5DirectorConfigFile(self):
        return os.path.join(
            director.getDRCBaseDir(), "models/atlas_v5/director_config.yaml"
        )

    def getDefaultValkyrieDirectorConfigFile(self):
        return os.path.join(
            director.getDRCBaseDir(), "models/val_description/director_config.yaml"
        )

    def getDefaultValkyrieSimpleDirectorConfigFile(self):
        return os.path.join(
            director.getDRCBaseDir(),
            "models/val_description/director_config_simple.yaml",
        )

    def getDefaultHyQDirectorConfigFile(self):
        return os.path.join(
            director.getDRCBaseDir(), "models/hyq_description/director_config.yaml"
        )

    def getDefaultAnymalDirectorConfigFile(self):
        return os.path.join(
            director.getDRCBaseDir(), "config/anymal/director_config.yaml"
        )

    def getDefaultKukaLWRConfigFile(self):
        return os.path.join(
            director.getDRCBaseDir(), "models/lwr_defs/director_config.yaml"
        )

    def getDefaultHuskyConfigFile(self):
        return os.path.join(
            director.getDRCBaseDir(), "config/husky/director_config.yaml"
        )

    def getDefaultDualArmHuskyConfigFile(self):
        return os.path.join(
            director.getDRCBaseDir(),
            "models/dual_arm_husky_description/director_config.yaml",
        )

    def _isPyDrakeAvailable(self):
        try:
            import pydrake
        except ImportError:
            return False
        if "DRAKE_RESOURCE_ROOT" not in os.environ:
            return False
        return True

    def addDrakeConfigShortcuts(self, directorConfig):

        import pydrake

        directorConfig.add_argument(
            "--iiwa-drake",
            dest="directorConfigFile",
            action="store_const",
            const=pydrake.common.FindResourceOrThrow(
                "drake/examples/kuka_iiwa_arm/director_config.yaml"
            ),
            help="Use KUKA IIWA from drake/examples",
        )

    def addDefaultArgs(self, parser):
        class IgnoreROSArgsAction(argparse.Action):
            """
            This action allows us to ignore ROS arguments, which are always added and prefixed by __. Also ignores
            remapping arguments, which contain :=
            """

            def __call__(self, parser, namespace, values, option_string=None):
                valid_args = [arg for arg in values if not arg.startswith("__") and ":=" not in arg]
                if getattr(namespace, self.dest):
                    # Already received a config argument previously, need to append to the list
                    getattr(namespace, self.dest).append(valid_args)
                else:
                    setattr(namespace, self.dest, [valid_args])

        parser.add_argument(
            "--robot-config",
            dest="robotConfigs",
            action=IgnoreROSArgsAction,
            nargs="+",
            metavar=("file_path", "robot_name"),
            help="YAML files specifying configurations for robots to display. Can be provided "
            "multiple times to display multiple robots. The second argument specifying the robot "
            "name is required if using the same configuration file more than once.",
        )

        if self._isPyDrakeAvailable():
            self.addDrakeConfigShortcuts(directorConfig)

        parser.add_argument(
            "--data",
            type=str,
            nargs="+",
            dest="data_files",
            default=[],
            action="append",
            metavar="filename",
            help="data files to load at startup",
        )

        parser.add_argument(
            "--script",
            "--startup",
            type=str,
            nargs="+",
            dest="scripts",
            default=[],
            action="append",
            metavar="filename",
            help="python scripts to run at startup",
        )


_argParser = None


def getGlobalArgParser():
    global _argParser
    if not _argParser:
        _argParser = DRCArgParser()
    return _argParser


def requireStrict():
    global _argParser
    _argParser = None
    getGlobalArgParser().strict = True


def args():
    return getGlobalArgParser().getArgs()


class RobotConfig(object):
    def __init__(self, config):
        config_file = config[0]
        if not os.path.isfile(config_file):
            raise Exception("Robot config file not found: %s" % config_file)

        self.dirname = os.path.dirname(os.path.abspath(config_file))
        self.config = yaml.safe_load(open(config_file))

        if "fixedPointFile" in self.config:
            self.config["fixedPointFile"] = os.path.join(
                self.dirname, self.config["fixedPointFile"]
            )

        # we received a robot name along with the config file
        if len(config) > 1:
            self.config["robotName"] = config[1]

        urdfConfig = self.config["urdfConfig"]
        for key, urdf in list(urdfConfig.items()):
            urdfConfig[key] = os.path.join(self.dirname, urdf)

    def __getitem__(self, key):
        """Used for dictionary accesses to on a robotconfig object"""
        return self.config[key]

    def __contains__(self, item):
        """Used for 'in' statements on a robotconfig object"""
        return item in self.config

    def get(self, key, default=None):
        """
        Some places call get rather than using []

        :param key: The key to retrieve from the configuration
        :param default: The default value to return if the key does not exist
        :return: The value of the key, or the default value if the key is not present
        """

        if key in self.config:
            return self.config[key]
        else:
            print("Key {} was not found in director config, returning default value '{}'".format(key, default))
            return default


class DirectorConfig(object):

    _defaultInstance = None

    def __init__(self, robotConfigs):
        self.robotConfigs = {}

        for config in robotConfigs:
            robotConfig = RobotConfig(config)
            if robotConfig["robotName"] in self.robotConfigs:
                raise ValueError(
                    "At least two robot configuration files given have the same robot name. Make sure "
                    "all names are unique by providing '--robot-config config_file.yaml robot_name' in the "
                    "arguments to the director node for the conflicting robots."
                )
            else:
                self.robotConfigs[robotConfig["robotName"]] = robotConfig

    def getConfig(self, robotName=None):
        """
        Get the configuration for a robot with the given name. If no name is given and there is only a single
        configuration, the function will return it. Otherwise, throw a ValueError

        :param robotName: The name of the robot whose configuration is to be retrieved
        :return:
        """
        if not robotName:
            if len(self.robotConfigs) == 1:
                import warnings

                warnings.warn(
                    "DirectorConfig.getConfig should be called with a valid robot name and not an empty "
                    "string. Be specific about the robot you want the configuration for."
                )
                return self.robotConfigs[self.robotConfigs.keys()[0]]
            else:
                raise ValueError(
                    "DirectorConfig.getConfig was called without a valid robotName specified and had "
                    "more than one robot. A name should be specified when calling."
                )
        else:
            return self.robotConfigs[robotName]

    @classmethod
    def getDefaultInstance(cls):
        if cls._defaultInstance is None:
            if not args().robotConfigs:
                raise Exception(
                    "Robot config files not defined. You must pass at least one. "
                    "Use --robot-config <filename> on the command line."
                )
            cls._defaultInstance = DirectorConfig(args().robotConfigs)
        return cls._defaultInstance


def getRobotConfig(robotName=""):
    return DirectorConfig.getDefaultInstance().getConfig(robotName)
