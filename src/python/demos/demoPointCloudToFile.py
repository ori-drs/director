'''
directorPython demoPointCloudToFile.py
Listens to the requested topic and writes the first one to file
'''

import time
from director import segmentation
from director import vtkAll as vtk
from director import ioUtils

reader= vtk.vtkRosPointCloudSubscriber()
reader.Start("/velodyne/point_cloud_filtered")

continueLoop = True
while (continueLoop):
    polyData = vtk.vtkPolyData()
    reader.GetPointCloud(polyData)
    frame_id = reader.GetFrameId()
    sec = reader.GetSec()
    nsec = reader.GetNsec()
    message = str(polyData.GetNumberOfPoints()) + " points, "
    message += frame_id + ", " + str(sec) + "." + str(nsec)
    print message

    if (polyData.GetNumberOfPoints() > 0):
        ioUtils.writePolyData(polyData, 'out.vtk')
        continueLoop = False

    time.sleep(0.1)

