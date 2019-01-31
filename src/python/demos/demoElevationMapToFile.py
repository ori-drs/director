'''
directorPython demoElevationMapToFile.py
Listens to the Elevation Map and writes the first one to file (as a point cloud)
'''

from director import vtkAll as vtk
from director import ioUtils
from director import vtkDRCFiltersPython as drc
from director.shallowCopy import shallowCopy

vtkRos= drc.vtkRosInit()
vtkRos.Start()

reader= drc.vtkRosGridMapSubscriber()
reader.Start()
print reader

import time

continueLoop = True
while (continueLoop):
    polyData = vtk.vtkPolyData()
    reader.GetMesh(polyData)
    n = polyData.GetNumberOfPoints()
    print "Number of points",  polyData.GetNumberOfPoints()

    polyDataPC = vtk.vtkPolyData()
    reader.GetPointCloud(polyDataPC)
    print "Number of output points",  polyDataPC.GetNumberOfPoints()



    if (polyDataPC.GetNumberOfPoints() > 0):
        ioUtils.writePolyData(polyDataPC, 'out.ply')
        continueLoop = False

    time.sleep(0.1)
