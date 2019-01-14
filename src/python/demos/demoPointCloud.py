'''
directorPython demoPointCloud.py --interactive
Listens to a point cloud topics and draws it in a UI
'''

from director import segmentation
from director.consoleapp import ConsoleApp
from director import drcargs
from director import vtkAll as vtk
from director import vtkDRCFiltersPython as drc

from director import applogic
from director import visualization as vis
from director.timercallback import TimerCallback

reader= drc.vtkRosPointCloudSubscriber()
reader.Start("/velodyne/point_cloud_filtered")
print reader


import time

app = ConsoleApp()
view = app.createView()

def spin():
    polyData = vtk.vtkPolyData()
    reader.GetPointCloud(polyData)
    frame_id = reader.GetFrameId()
    sec = reader.GetSec()
    nsec = reader.GetNsec()
    message = str(polyData.GetNumberOfPoints()) + " points, "
    message += frame_id + ", " + str(sec) + "." + str(nsec)
    print message

    vis.updatePolyData(polyData,'point cloud')

    
quitTimer = TimerCallback(targetFps=5.0)
quitTimer.callback = spin
quitTimer.start()

if app.getTestingInteractiveEnabled():
    view.show()
    app.showObjectModel()
    app.start()


