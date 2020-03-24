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
        return self.getDefaultAnymalDirectorConfigFile();

    def getDefaultAtlasV3DirectorConfigFile(self):
        return os.path.join(director.getDRCBaseDir(),
                            'models/atlas_v3/director_config.yaml')

    def getDefaultAtlasV4DirectorConfigFile(self):
        return os.path.join(director.getDRCBaseDir(),
                            'models/atlas_v4/director_config.yaml')

    def getDefaultAtlasV5DirectorConfigFile(self):
        return os.path.join(director.getDRCBaseDir(),
                            'models/atlas_v5/director_config.yaml')

    def getDefaultValkyrieDirectorConfigFile(self):
        return os.path.join(director.getDRCBaseDir(),
                            'models/val_description/director_config.yaml')

    def getDefaultValkyrieSimpleDirectorConfigFile(self):
        return os.path.join(director.getDRCBaseDir(),
                            'models/val_description/director_config_simple.yaml')

    def getDefaultHyQDirectorConfigFile(self):
        return os.path.join(director.getDRCBaseDir(),
                            'models/hyq_description/director_config.yaml')

    def getDefaultAnymalDirectorConfigFile(self):
        return os.path.join(director.getDRCBaseDir(),
                            'config/anymal/director_config.yaml')

    def getDefaultKukaLWRConfigFile(self):
        return os.path.join(director.getDRCBaseDir(),
                            'models/lwr_defs/director_config.yaml')

    def getDefaultHuskyConfigFile(self):
        return os.path.join(director.getDRCBaseDir(),
                            'config/husky/director_config.yaml')

    def getDefaultDualArmHuskyConfigFile(self):
        return os.path.join(director.getDRCBaseDir(),
                            'models/dual_arm_husky_description/director_config.yaml')


    def _isPyDrakeAvailable(self):
        try:
            import pydrake
        except ImportError:
            return False
        if 'DRAKE_RESOURCE_ROOT' not in os.environ:
            return False
        return True

    def addDrakeConfigShortcuts(self, directorConfig):

        import pydrake

        directorConfig.add_argument(
            '--iiwa-drake',
            dest='directorConfigFile',
            action='store_const',
            const=pydrake.common.FindResourceOrThrow('drake/examples/kuka_iiwa_arm/director_config.yaml'),
            help='Use KUKA IIWA from drake/examples')

    def addOpenHumanoidsConfigShortcuts(self, directorConfig):

        directorConfig.add_argument('-v3', '--atlas_v3', dest='directorConfigFile',
                            action='store_const',
                            const=self.getDefaultAtlasV3DirectorConfigFile(),
                            help='Use Atlas V3')

        directorConfig.add_argument('-v4', '--atlas_v4', dest='directorConfigFile',
                            action='store_const',
                            const=self.getDefaultAtlasV4DirectorConfigFile(),
                            help='Use Atlas V4')

        directorConfig.add_argument('-v5', '--atlas_v5', dest='directorConfigFile',
                            action='store_const',
                            const=self.getDefaultAtlasV5DirectorConfigFile(),
                            help='Use Atlas V5')

        directorConfig.add_argument('-val', '--valkyrie', dest='directorConfigFile',
                            action='store_const',
                            const=self.getDefaultValkyrieDirectorConfigFile(),
                            help='Use Valkyrie (Default)')

        directorConfig.add_argument('-val_simple', '--valkyrie_simple', dest='directorConfigFile',
                            action='store_const',
                            const=self.getDefaultValkyrieSimpleDirectorConfigFile(),
                            help='Use Valkyrie (Simple/Primitive Shapes)')

        directorConfig.add_argument('-hyq', '--hyq', dest='directorConfigFile',
                            action='store_const',
                            const=self.getDefaultHyQDirectorConfigFile(),
                            help='Use HyQ')

        directorConfig.add_argument('-anymal', '--anymal', dest='directorConfigFile',
                            action='store_const',
                            const=self.getDefaultAnymalDirectorConfigFile(),
                            help='Use Anymal')

        directorConfig.add_argument('-lwr', '--lwr', dest='directorConfigFile',
                            action='store_const',
                            const=self.getDefaultKukaLWRConfigFile(),
                            help='Use Kuka LWR')

        directorConfig.add_argument('-husky', '--husky', dest='directorConfigFile',
                            action='store_const',
                            const=self.getDefaultHuskyConfigFile(),
                            help='Use Husky')

        directorConfig.add_argument('-dual_arm_husky', '--dual_arm_husky', dest='directorConfigFile',
                            action='store_const',
                            const=self.getDefaultDualArmHuskyConfigFile(),
                            help='Use Dual Arm Husky')

    def addDefaultArgs(self, parser):

        parser.add_argument('--matlab-host', metavar='hostname', type=str,
                            help='hostname to connect with external matlab server')

        class IgnoreROSArgsAction(argparse.Action):
            """
            This action allows us to ignore ROS arguments, which are always added and prefixed by __
            """
            def __call__(self, parser, namespace, values, option_string=None):
                valid_paths = [path for path in values if not path.startswith("__")]
                setattr(namespace, self.dest, valid_paths)

        directorConfig = parser.add_mutually_exclusive_group(required=False)
        directorConfig.add_argument('--director-config', '--director_config', dest='directorConfigFile',
                                    default=[], metavar='filename', nargs='+', action=IgnoreROSArgsAction,
                                    help='YAML files specifying configurations for robots to display')

        if director.getDRCBaseIsSet():
            self.addOpenHumanoidsConfigShortcuts(directorConfig)
            parser.set_defaults(directorConfigFile=self.getDefaultDirectorConfigFile())

        if self._isPyDrakeAvailable():
            self.addDrakeConfigShortcuts(directorConfig)

        parser.add_argument('--data', type=str, nargs='+', dest='data_files',
                            default=[], action='append', metavar='filename',
                            help='data files to load at startup')

        parser.add_argument('--script', '--startup', type=str, nargs='+', dest='scripts',
                            default=[], action='append', metavar='filename',
                            help='python scripts to run at startup')


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

    def __init__(self, configPath):
        if not os.path.isfile(configPath):
            raise Exception('Director config file not found: %s' % configPath)

        self.dirname = os.path.dirname(os.path.abspath(configPath))
        self.config = yaml.safe_load(open(configPath))

        self.config['fixedPointFile'] = os.path.join(self.dirname, self.config['fixedPointFile'])

        urdfConfig = self.config['urdfConfig']
        for key, urdf in list(urdfConfig.items()):
            urdfConfig[key] = os.path.join(self.dirname, urdf)

    def __getitem__(self, key):
        """Used for dictionary accesses to on a robotconfig object"""
        return self.config[key]

    def __contains__(self, item):
        """Used for 'in' statements on a robotconfig object"""
        return item in self.config

    def get(self, item):
        """Some places call get rather than using []"""
        return self[item]

class DirectorConfig(object):

    _defaultInstance = None

    def __init__(self, filePaths):

        self.robotConfigs = {}

        self.file_paths = filePaths
        for path in self.file_paths:
            robotConfig = RobotConfig(path)
            self.robotConfigs[robotConfig['robotName']] = robotConfig

    def getConfig(self, robotName):
        """
        Get the configuration for a robot with the given name. If no name is given and there is only a single
        configuration, the function will return it. Otherwise, throw a ValueError

        :param robotName: The name of the robot whose configuration is to be retrieved
        :return:
        """
        if not robotName:
            if len(self.robotConfigs) == 1:
                import warnings
                warnings.warn("DirectorConfig.getConfig should be called with a valid robot name and not an empty "
                              "string. Be specific about the robot you want the configuration for.")
                return self.robotConfigs[self.robotConfigs.keys()[0]]
            else:
                raise ValueError("DirectorConfig.getConfig was called without a valid robotName specified and had "
                                 "more than one robot. A name should be specified when calling.")
        else:
            return self.robotConfigs[robotName]

    @classmethod
    def getDefaultInstance(cls):
        if cls._defaultInstance is None:
            if not args().directorConfigFile:
                raise Exception('Director config file is not defined. You must pass at least one.'
                                'Use --director-config <filename> on the command line.')
            cls._defaultInstance = DirectorConfig(args().directorConfigFile)
        return cls._defaultInstance


def getRobotConfig(robotName=""):
    return DirectorConfig.getDefaultInstance().getConfig(robotName)
