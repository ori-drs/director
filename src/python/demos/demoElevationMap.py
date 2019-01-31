'''
directorPython demoElevationMap.py --interactive
Listens to the Elevation Map and draws it in a UI
'''

from director import segmentation
from director.consoleapp import ConsoleApp
from director import drcargs
from director import vtkAll as vtk

from director import applogic
from director import visualization as vis
from director.timercallback import TimerCallback

vtkRos= vtk.vtkRosInit()
vtkRos.Start()

reader= vtk.vtkRosGridMapSubscriber()
reader.Start()
print reader


import time

app = ConsoleApp()
view = app.createView()

def spin():
    polyData = vtk.vtkPolyData()
    reader.GetMesh(polyData)
    vis.updatePolyData(polyData,'mesh')
    print "Number of points (a)",  polyData.GetNumberOfPoints()
    if (polyData.GetNumberOfPoints() == 0):
        return

    polyDataPC = vtk.vtkPolyData()
    reader.GetPointCloud(polyDataPC)
    vis.updatePolyData(polyDataPC, 'output')
    print "Number of points (b)",  polyDataPC.GetNumberOfPoints()


    
quitTimer = TimerCallback(targetFps=5.0)
quitTimer.callback = spin
quitTimer.start()

if app.getTestingInteractiveEnabled():
    view.show()
    app.showObjectModel()
    app.start()


