'''
directorPython demoElevationMap.py --interactive
Listens to the Elevation Map and draws it in a UI
'''

from director import segmentation
from director.consoleapp import ConsoleApp
from director import drcargs
from director import vtkAll as vtk
from director import vtkDRCFiltersPython as drc

from director import applogic
from director import visualization as vis
from director.timercallback import TimerCallback

reader= drc.vtkRosGridMapSubscriber()
reader.Start()
print reader


import time

app = ConsoleApp()
view = app.createView()

def spin():
    polyData = vtk.vtkPolyData()
    reader.GetMesh(polyData)
    n = polyData.GetNumberOfPoints()
    vis.updatePolyData(polyData,'mesh')
    print "Number of points",  polyData.GetNumberOfPoints()

    polyData2=segmentation.applyVoxelGrid(polyData)
    vis.updatePolyData(polyData2,'point cloud')

    
quitTimer = TimerCallback(targetFps=5.0)
quitTimer.callback = spin
quitTimer.start()

if app.getTestingInteractiveEnabled():
    view.show()
    app.showObjectModel()
    app.start()


