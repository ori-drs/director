'''
directorPython demoElevationMapToFile.py
Listens to the Elevation Map and writes the first one to file (as a point cloud)
'''

from director import vtkAll as vtk
from director import ioUtils
from director import vtkDRCFiltersPython as drc
from director.shallowCopy import shallowCopy

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

    leafSize = 0.005
    v = vtk.vtkPCLVoxelGrid()
    v.SetLeafSize(leafSize, leafSize, leafSize)
    v.SetInputData(polyData)
    v.Update()
    polyData2 = shallowCopy(v.GetOutput())


    if (polyData2.GetNumberOfPoints() > 0):
        ioUtils.writePolyData(polyData2, 'out.ply')
        continueLoop = False

    time.sleep(0.1)
