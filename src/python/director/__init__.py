import os
import sys
import rospkg


def _initCoverage():
    if  'COVERAGE_PROCESS_START' in os.environ:
        try:
            import coverage
            coverage.process_startup()
        except ImportError:
            pass


def getDRCBaseDir():
    rp = rospkg.RosPack()
    return rp.get_path('director_drs')

def getDRCBaseIsSet():
    try:
        rp = rospkg.RosPack()
        rp.get_path('director_drs')
        return True
    except rospkg.ResourceNotFound:
        return False

def updateSysPath(path):
    if path and os.path.isdir(path) and path not in sys.path:
        sys.path.insert(0, path)
        return True
    return False


_initCoverage()

# this is for mac homebrew users
#updateSysPath('/usr/local/opt/vtk7/lib/python2.7/site-packages')
