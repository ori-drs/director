import director.applogic as app
from director import lcmUtils
from director import transformUtils
from director import visualization as vis
from director import filterUtils
from director import drcargs
from director.shallowCopy import shallowCopy
from director.timercallback import TimerCallback
from director import vtkNumpy
from director import objectmodel as om
import director.vtkAll as vtk
from director.debugVis import DebugData

import PythonQt
from PythonQt import QtCore, QtGui
import bot_core as lcmbotcore
import numpy as np
from director.simpletimer import SimpleTimer
from director import ioUtils
import sys
import drc as lcmdrc
from director.consoleapp import ConsoleApp


class PointCloudItem(om.ObjectModelItem):

    def __init__(self, model):

        om.ObjectModelItem.__init__(self, 'PointCloud', om.Icons.Laser)

        self.model = model
        self.scalarBarWidget = None
        self.addProperty('Color By', 5,
                         attributes=om.PropertyAttributes(enumNames=['Solid Color', 'intensity', 'ring', 'x', 'y', 'z']))
        self.addProperty('Visible', model.visible)
        self.addProperty('Alpha', model.alpha,
                         attributes=om.PropertyAttributes(decimals=2, minimum=0, maximum=1.0, singleStep=0.1, hidden=False))
        self.addProperty('Point Size', model.pointSize,
                         attributes=om.PropertyAttributes(decimals=0, minimum=1, maximum=20, singleStep=1, hidden=False))
        self.addProperty('Updates Enabled', True)
        self.addProperty('Framerate', model.targetFps,
                         attributes=om.PropertyAttributes(decimals=0, minimum=1.0, maximum=30.0, singleStep=1, hidden=False))
        self.addProperty('Number of Point Clouds', model.numberOfPointClouds,
                         attributes=om.PropertyAttributes(decimals=0, minimum=1, maximum=100, singleStep=1, hidden=False))
        self.addProperty('Copy PointCloud', False)

        
    def _onPropertyChanged(self, propertySet, propertyName):
        om.ObjectModelItem._onPropertyChanged(self, propertySet, propertyName)

        if propertyName == 'Updates Enabled':
            if self.getProperty('Updates Enabled'):
                self.model.start()
            else:
                self.model.stop()

        if propertyName == 'Copy PointCloud':
            if self.getProperty('Copy PointCloud'):
                self.model.copyPointCloud()
                #self.setProperty('Copy PointCloud', False)
                self.setProperty('Visible', False)

        elif propertyName == 'Alpha':
            self.model.setAlpha(self.getProperty(propertyName))

        elif propertyName == 'Visible':
            self.model.setVisible(self.getProperty(propertyName))

        elif propertyName == 'Point Size':
            self.model.setPointSize(self.getProperty(propertyName))

        elif propertyName == 'Number of Point Clouds':
            self.model.numberOfPointClouds = self.getProperty(propertyName)
            self.model.initPointClouds()

        elif propertyName == 'Framerate':
            self.model.setFramerate(self.getProperty('Framerate'))

        elif propertyName == 'Color By':
            self._updateColorBy()

        #self.model.polyDataObj._renderAllViews()


    def _updateColorBy(self):
        colorBy = self.getPropertyEnumValue('Color By')
        self.model.colorBy = colorBy
        self.model._updateColorBy(colorBy)


class PointCloudSource(TimerCallback):

    def __init__(self, view, _PointCloudQueue):
        self.view = view
        self.PointCloudQueue = _PointCloudQueue

        self.visible = True
        self.colorBy = 'z'
        self.pointSize = 1
        self.alpha = 1.0

        self.lastPointCloud = 0
        self.numberOfPointClouds = 20
        self.nextPointCloudId = 0
        self.pointClouds = []
        self.initPointClouds()
        
        self.p = vtk.vtkPolyData()
        self.previousUtime = 0

        sensorsFolder = om.getOrCreateContainer('sensors')

        self.queue = PythonQt.dd.ddBotImageQueue(lcmUtils.getGlobalLCMThread())
        self.queue.init(lcmUtils.getGlobalLCMThread(), drcargs.args().config_file)

        self.targetFps = 30
        self.timerCallback = TimerCallback(targetFps=self.targetFps)
        self.timerCallback.callback = self._updateSource
        #self.timerCallback.start()
        
    def initPointClouds(self):

        for pointCloud in self.pointClouds:
            pointCloud.removeFromAllViews()

        self.pointClouds = []
        self.nextPointCloudId = 0
        self.lastPointCloud = max(self.lastPointCloud - self.numberOfPointClouds, 0)

        for i in xrange(self.numberOfPointClouds):
            polyData = vtk.vtkPolyData()
            pointCloud = vis.PolyDataItem('scan line %d' % i, polyData, self.view)
            if (i==0):
                pointCloud.actor.SetPickable(1)
            else:
                pointCloud.actor.SetPickable(0)

            pointCloud.setSolidColor((1,0,0))
            self.pointClouds.append(pointCloud)

    def start(self):
        self.timerCallback.start()

    def stop(self):
        self.timerCallback.stop()

    def copyPointCloud(self):
        appendPolyData = vtk.vtkAppendPolyData()
        for pointCloud in self.pointClouds:
            appendPolyData.AddInputData(pointCloud.polyData)

        appendPolyData.Update()
        combinedPolyData = appendPolyData.GetOutput()
        vis.showPolyData(combinedPolyData,'pointcloud copy')


    def setPointSize(self, pointSize):
        for pointCloud in self.pointClouds:
            pointCloud.setProperty('Point Size', pointSize)

    def setFramerate(self, framerate):
        self.targetFps = framerate
        self.timerCallback.stop()
        self.timerCallback.targetFps = framerate
        self.timerCallback.start()

    def setAlpha(self, alpha):
        self.alpha = alpha
        for pointCloud in self.pointClouds:
            pointCloud.setProperty('Alpha', alpha)

    def setVisible(self, visible):
        self.visible = visible
        for pointCloud in self.pointClouds:
            pointCloud.setProperty('Visible', visible)

    def _updateSource(self):
        p = vtk.vtkPolyData()
        utime = self.PointCloudQueue.getPointCloudFromPointCloud(p)

        if not p.GetNumberOfPoints():
            return

        if (self.previousUtime==utime):
            return

        self.previousUtime = utime

        # get point cloud and transform to local/map frame
        # TODO read a list of point cloud channels
        sensorToLocalFused = vtk.vtkTransform()
        self.queue.getTransform('VELODYNE', 'local', utime, sensorToLocalFused)
        p = filterUtils.transformPolyData(p,sensorToLocalFused)

        # add x,y,z labels
        points = vtkNumpy.getNumpyFromVtk(p, 'Points')
        vtkNumpy.addNumpyToVtk(p, points[:,0].copy(), 'x')
        vtkNumpy.addNumpyToVtk(p, points[:,1].copy(), 'y')
        vtkNumpy.addNumpyToVtk(p, points[:,2].copy(), 'z')

        bodyToLocal = vtk.vtkTransform()
        self.queue.getTransform('body', 'local', utime, bodyToLocal)
        bodyHeight = bodyToLocal.GetPosition()[2]

        # add point cloud into circular buffer
        self.nextPointCloudId = self.nextPointCloudId + 1
        i = self.nextPointCloudId % self.numberOfPointClouds

        self.pointClouds[i].setPolyData(p)
        self.pointClouds[i].setRangeMap('z', [bodyHeight-0.5, bodyHeight+0.5])

        self._updateColorBy(self.colorBy)

        if self.pointClouds[0].getProperty('Visible'):
            self.view.render()

    def _updateColorBy(self, colorBy):
        for pointCloud in self.pointClouds:
            enumNames = pointCloud.properties._attributes['Color By'].enumNames
            if colorBy in enumNames:
                pointCloud.setProperty('Color By', colorBy)

def init(view):
    global PointCloudQueue, _pointcloudItem, _pointcloudSource
    PointCloudQueue = PythonQt.dd.ddPointCloudLCM(lcmUtils.getGlobalLCMThread())
    PointCloudQueue.init(lcmUtils.getGlobalLCMThread(), drcargs.args().config_file)
    
    _pointcloudSource = PointCloudSource(view, PointCloudQueue)
    _pointcloudSource.start()

    sensorsFolder = om.getOrCreateContainer('sensors')

    _pointcloudItem = PointCloudItem(_pointcloudSource)
    om.addToObjectModel(_pointcloudItem, sensorsFolder)
