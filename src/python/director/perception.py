import os
import abc
import sys
import vtk
import math
import time
import PythonQt
from PythonQt import QtCore, QtGui
import director.objectmodel as om
from director import cameraview
from director import drcargs
from director import robotstate
from director.timercallback import TimerCallback
from director.utime import getUtime
from director.simpletimer import MovingAverageComputer
import vtkDRCFiltersPython as drc
from director.debugVis import DebugData
import director.visualization as vis
from director import perceptionmeta
import functools
from director import vtkNumpy as vnp
import numpy as np
import rospy


class CheckProvider(object):
    """
    A decorator class to use to ensure that functions which require a provider for their data source are not called
    if that data source has not yet been initialised. Also counts how many times a function has been skipped.
    """
    def __init__(self, func):
        functools.update_wrapper(self, func)
        self.func = func
        self.num_calls = 0

    def __get__(self, obj, objtype):
        """Support instance methods
        https://stackoverflow.com/questions/5469956/python-decorator-self-is-mixed-up
        https://stackoverflow.com/questions/2365701/decorating-python-class-methods-how-do-i-pass-the-instance-to-the-decorator
        """
        return functools.partial(self.__call__, obj)

    def __call__(self, *args, **kwargs):
        if args[0].provider:
            return self.func(*args, **kwargs)
        else:
            if self.num_calls % 50 == 0:
                print("Provider not yet intialised, skipping execution of {}.{}"
                      " (skipped {} times)".format(args[0].__class__.__name__, self.func.__name__, self.num_calls))
            self.num_calls += 1
            return


class MultisenseItem(om.ObjectModelItem):

    def __init__(self, model):

        om.ObjectModelItem.__init__(self, 'Multisense', om.Icons.Laser)

        self.model = model
        self.scalarBarWidget = None
        self.addProperty('Color By', 0,
                         attributes=om.PropertyAttributes(enumNames=['Solid Color', 'Intensity', 'Z Coordinate', 'Range', 'Spindle Angle', 'Azimuth', 'Camera RGB', 'Scan Delta']))
        self.addProperty('Show Scalar Bar', False)
        self.addProperty('Updates Enabled', True)
        self.addProperty('Min Range', model.reader.GetDistanceRange()[0],
                         attributes=om.PropertyAttributes(decimals=2, minimum=0.0, maximum=100.0, singleStep=0.25, hidden=False))
        self.addProperty('Max Range', model.reader.GetDistanceRange()[1],
                         attributes=om.PropertyAttributes(decimals=2, minimum=0.0, maximum=100.0, singleStep=0.25, hidden=False))
        self.addProperty('Edge Filter Angle', model.reader.GetEdgeAngleThreshold(),
                         attributes=om.PropertyAttributes(decimals=0, minimum=0.0, maximum=60.0, singleStep=1, hidden=False))
        self.addProperty('Number of Scan Lines', model.numberOfScanLines,
                         attributes=om.PropertyAttributes(decimals=0, minimum=0, maximum=100, singleStep=1, hidden=False))
        self.addProperty('Visible', model.visible)
        self.addProperty('Point Size', model.pointSize,
                         attributes=om.PropertyAttributes(decimals=0, minimum=1, maximum=20, singleStep=1, hidden=False))
        self.addProperty('Alpha', model.alpha,
                         attributes=om.PropertyAttributes(decimals=2, minimum=0, maximum=1.0, singleStep=0.1, hidden=False))
        self.addProperty('Min Height', model.reader.GetHeightRange()[0],
                         attributes=om.PropertyAttributes(decimals=2, minimum=-80.0, maximum=80.0, singleStep=0.25, hidden=False))
        self.addProperty('Max Height', model.reader.GetHeightRange()[1],
                         attributes=om.PropertyAttributes(decimals=2, minimum=-80.0, maximum=80.0, singleStep=0.25, hidden=False))

        #self.addProperty('Color', QtGui.QColor(255,255,255))
        #self.addProperty('Scanline Color', QtGui.QColor(255,0,0))

    def _onPropertyChanged(self, propertySet, propertyName):
        om.ObjectModelItem._onPropertyChanged(self, propertySet, propertyName)

        if propertyName == 'Updates Enabled':
            if self.getProperty('Updates Enabled'):
                self.model.start()
            else:
                self.model.stop()

        elif propertyName == 'Edge Filter Angle':
            self.model.reader.SetEdgeAngleThreshold(self.getProperty('Edge Filter Angle'))
            self.model.showRevolution(self.model.displayedRevolution)

        elif propertyName == 'Alpha':
            self.model.setAlpha(self.getProperty(propertyName))

        elif propertyName == 'Visible':
            self.model.setVisible(self.getProperty(propertyName))

        elif propertyName == 'Point Size':
            self.model.setPointSize(self.getProperty(propertyName))

        elif propertyName == 'Number of Scan Lines':
            self.model.numberOfScanLines = self.getProperty(propertyName)
            self.model.initScanLines()

        elif propertyName in ('Min Range', 'Max Range'):
            self.model.reader.SetDistanceRange(self.getProperty('Min Range'), self.getProperty('Max Range'))
            self.model.showRevolution(self.model.displayedRevolution)

        elif propertyName in ('Min Height', 'Max Height'):
            self.model.reader.SetHeightRange(self.getProperty('Min Height'), self.getProperty('Max Height'))
            self.model.showRevolution(self.model.displayedRevolution)

        elif propertyName == 'Color By':
            self._updateColorBy()

        elif propertyName == 'Show Scalar Bar':
            self._updateScalarBar()

        self.model.polyDataObj._renderAllViews()


    def _updateColorBy(self):

        arrayMap = {
          0 : 'Solid Color',
          1 : 'intensity',
          2 : 'z',
          3 : 'distance',
          4 : 'spindle_angle',
          5 : 'azimuth',
          6 : 'rgb',
          7 : 'scan_delta'
          }

        colorBy = self.getProperty('Color By')
        arrayName = arrayMap.get(colorBy)

        if arrayName == 'rgb' and arrayName not in self.model.polyDataObj.getArrayNames():
            self.model.colorizeCallback()
            self.model.polyDataObj._updateColorByProperty()
        self.model.polyDataObj.setProperty('Color By', arrayName)
        self._updateScalarBar()

    def hasDataSet(self, dataSet):
        return self.model.polyDataObj.hasDataSet(dataSet)

    def _updateScalarBar(self):
        self.model.polyDataObj.setProperty('Show Scalar Bar', self.getProperty('Show Scalar Bar'))


class LidarItem(om.ObjectModelItem):

    def __init__(self, model):
        om.ObjectModelItem.__init__(self, model.sensorName, om.Icons.EyeOff)

        self.model = model
        self.scalarBarWidget = None
        self.addProperty('Color By', 0,
                         attributes=om.PropertyAttributes(enumNames=['Solid Color', 'Intensity', 'Z Coordinate', 'Range', 'Spindle Angle', 'Azimuth', 'Camera RGB', 'Scan Delta']))
        self.addProperty('Show Scalar Bar', False)
        self.addProperty('Updates Enabled', True)
        self.addProperty('Min Range', model.reader.GetDistanceRange()[0],
                         attributes=om.PropertyAttributes(decimals=2, minimum=0.0, maximum=100.0, singleStep=0.25, hidden=False))
        self.addProperty('Max Range', model.reader.GetDistanceRange()[1],
                         attributes=om.PropertyAttributes(decimals=2, minimum=0.0, maximum=100.0, singleStep=0.25, hidden=False))
        self.addProperty('Edge Filter Angle', model.reader.GetEdgeAngleThreshold(),
                         attributes=om.PropertyAttributes(decimals=0, minimum=0.0, maximum=60.0, singleStep=1, hidden=False))
        self.addProperty('Number of Scan Lines', model.numberOfScanLines,
                         attributes=om.PropertyAttributes(decimals=0, minimum=0, maximum=5000, singleStep=1, hidden=False))
        self.addProperty('Visible', model.visible)
        self.addProperty('Point Size', model.pointSize,
                         attributes=om.PropertyAttributes(decimals=0, minimum=-1, maximum=20, singleStep=1, hidden=False))
        self.addProperty('Alpha', model.alpha,
                         attributes=om.PropertyAttributes(decimals=2, minimum=0, maximum=1.0, singleStep=0.1, hidden=False))
        self.addProperty('Min Height', model.reader.GetHeightRange()[0],
                         attributes=om.PropertyAttributes(decimals=2, minimum=-80.0, maximum=80.0, singleStep=0.25, hidden=False))
        self.addProperty('Max Height', model.reader.GetHeightRange()[1],
                         attributes=om.PropertyAttributes(decimals=2, minimum=-80.0, maximum=80.0, singleStep=0.25, hidden=False))

        #self.addProperty('Color', QtGui.QColor(255,255,255))
        #self.addProperty('Scanline Color', QtGui.QColor(255,0,0))

    def _onPropertyChanged(self, propertySet, propertyName):
        om.ObjectModelItem._onPropertyChanged(self, propertySet, propertyName)

        if propertyName == 'Updates Enabled':
            if self.getProperty('Updates Enabled'):
                self.model.start()
            else:
                self.model.stop()

        elif propertyName == 'Edge Filter Angle':
            self.model.reader.SetEdgeAngleThreshold(self.getProperty('Edge Filter Angle'))
            #    self.model.showRevolution(self.model.displayedRevolution)

        elif propertyName == 'Alpha':
            self.model.setAlpha(self.getProperty(propertyName))

        elif propertyName == 'Visible':
            self.model.setVisible(self.getProperty(propertyName))

        elif propertyName == 'Point Size':
            self.model.setPointSize(self.getProperty(propertyName))

        elif propertyName == 'Number of Scan Lines':
            self.model.numberOfScanLines = self.getProperty(propertyName)
            self.model.initScanLines()

        elif propertyName in ('Min Range', 'Max Range'):
            self.model.reader.SetDistanceRange(self.getProperty('Min Range'), self.getProperty('Max Range'))
            #    self.model.showRevolution(self.model.displayedRevolution)

        elif propertyName in ('Min Height', 'Max Height'):
            self.model.reader.SetHeightRange(self.getProperty('Min Height'), self.getProperty('Max Height'))
            #    self.model.showRevolution(self.model.displayedRevolution)

        elif propertyName == 'Color By':
            self._updateColorBy()

        elif propertyName == 'Show Scalar Bar':
            self._updateScalarBar()

        self.model.polyDataObj._renderAllViews()


    def _updateColorBy(self):

        arrayMap = {
          0 : 'Solid Color',
          1 : 'intensity',
          2 : 'z',
          3 : 'distance',
          4 : 'spindle_angle',
          5 : 'azimuth',
          6 : 'rgb',
          7 : 'scan_delta'
          }

        colorBy = self.getProperty('Color By')
        arrayName = arrayMap.get(colorBy)

        self.model.setColorBy(arrayName)
        self._updateScalarBar()

    def hasDataSet(self, dataSet):
        return self.model.polyDataObj.hasDataSet(dataSet)

    def _updateScalarBar(self):
        self.model.polyDataObj.setProperty('Show Scalar Bar', self.getProperty('Show Scalar Bar'))


class SpindleAxisDebug(vis.PolyDataItem):

    def __init__(self, frameProvider):
        vis.PolyDataItem.__init__(self, 'spindle axis', vtk.vtkPolyData(), view=None)
        self.frameProvider = frameProvider
        self.timer = TimerCallback()
        self.timer.callback = self.update
        self.setProperty('Color', QtGui.QColor(0, 255, 0))
        self.setProperty('Visible', False)

    def _onPropertyChanged(self, propertySet, propertyName):
        vis.PolyDataItem._onPropertyChanged(self, propertySet, propertyName)

        if propertyName == 'Visible':
            if self.getProperty(propertyName):
                self.timer.start()
            else:
                self.timer.stop()

    def onRemoveFromObjectModel(self):
        vis.PolyDataItem.onRemoveFromObjectModel(self)
        self.timer.stop()

    def update(self):

        t = self.frameProvider.getFrame('MULTISENSE_SCAN')

        p1 = [0.0, 0.0, 0.0]
        p2 = [2.0, 0.0, 0.0]

        p1 = t.TransformPoint(p1)
        p2 = t.TransformPoint(p2)

        d = DebugData()
        d.addSphere(p1, radius=0.01, color=[0,1,0])
        d.addLine(p1, p2, color=[0,1,0])
        self.setPolyData(d.getPolyData())





class LidarSource(TimerCallback):

    def __init__(self, view, channelName, coordinateFrame, sensorName, intensityRange=(400,4000)):
        TimerCallback.__init__(self)
        self.view = view
        self.channelName = channelName
        self.reader = None
        self.displayedRevolution = -1
        self.lastScanLine = 0
        self.numberOfScanLines = 100
        self.nextScanLineId = 0
        self.scanLines = []
        self.pointSize = 1
        self.alpha = 0.5
        self.visible = True
        self.colorBy = 'Solid Color'
        self.intensityRange = intensityRange
        self.initScanLines()
        self.sensorName = sensorName
        self.coordinateFrame = coordinateFrame

        self.revPolyData = vtk.vtkPolyData()
        self.polyDataObj = vis.PolyDataItem('Lidar Sweep', self.revPolyData, view)
        self.polyDataObj.actor.SetPickable(1)

        self.polyDataObj.setRangeMap('intensity', intensityRange)

        self.setPointSize(self.pointSize)
        self.setAlpha(self.alpha)
        self.targetFps = 60
        self.colorizeCallback = None

    def initScanLines(self):

        for scanLine in self.scanLines:
            scanLine.removeFromAllViews()

        self.scanLines = []
        self.nextScanLineId = 0
        self.lastScanLine = max(self.lastScanLine - self.numberOfScanLines, 0)

        for i in xrange(self.numberOfScanLines):
            polyData = vtk.vtkPolyData()
            scanLine = vis.PolyDataItem('scan line %d' % i, polyData, self.view)
            scanLine.rangeMap["intensity"] = self.intensityRange

            scanLine.actor.SetPickable(0)
            #scanLine.setSolidColor((0,1,0))
            self.scanLines.append(scanLine)

    def getScanToLocal(self):
        return None

    def setPointSize(self, pointSize):
        for scanLine in self.scanLines:
            scanLine.setProperty('Point Size', pointSize + 2)
        self.polyDataObj.setProperty('Point Size', pointSize)

    def setAlpha(self, alpha):
        self.alpha = alpha
        for scanLine in self.scanLines:
            scanLine.setProperty('Alpha', alpha)
        self.polyDataObj.setProperty('Alpha', alpha)

    def setVisible(self, visible):
        self.visible = visible
        for scanLine in self.scanLines:
            scanLine.setProperty('Visible', visible)
        self.polyDataObj.setProperty('Visible', visible)

    def setColorBy(self, colorBy):
        self.colorBy = colorBy
        for scanLine in self.scanLines:
            if colorBy and colorBy in scanLine.getArrayNames():
                scanLine.colorBy(self.colorBy)
            elif colorBy == "Solid Color":
                scanLine.setSolidColor((1,1,1))

    def start(self):
        if self.reader is None:
            self.reader = drc.vtkLidarSource()
            self.reader.subscribe(self.channelName)
            self.reader.setCoordinateFrame(self.coordinateFrame)
            self.reader.InitBotConfig(drcargs.args().config_file)
            self.reader.SetDistanceRange(0.25, 80.0)
            self.reader.SetHeightRange(-80.0, 80.0)
            self.reader.Start()

        TimerCallback.start(self)

    def updateScanLines(self):

        if not self.numberOfScanLines:
            return

        currentScanLine = self.reader.GetCurrentScanLine() - 1
        scanLinesToUpdate = currentScanLine - self.lastScanLine
        scanLinesToUpdate = min(scanLinesToUpdate, self.numberOfScanLines)

        if not scanLinesToUpdate:
            return

        #print 'current scanline:', currentScanLine
        #print 'scan lines to update:', scanLinesToUpdate
        #print 'updating actors:', self.nextScanLineId, (self.nextScanLineId + (scanLinesToUpdate-1)) % self.numberOfActors
        #print 'updating scan lines:', self.lastScanLine + 1, self.lastScanLine + 1 + (scanLinesToUpdate-1)

        for i in xrange(scanLinesToUpdate):
            scanLine = self.scanLines[(self.nextScanLineId + i) % self.numberOfScanLines]
            self.reader.GetDataForScanLine(self.lastScanLine + i + 1, scanLine.polyData)
            if self.colorBy and self.colorBy in scanLine.getArrayNames():
                scanLine.colorBy(self.colorBy)

        self.lastScanLine = currentScanLine
        self.nextScanLineId = (self.nextScanLineId + scanLinesToUpdate) % self.numberOfScanLines

        if self.scanLines[0].getProperty('Visible'):
            self.view.render()

    def getPolyData(self):
        self.revPolyData = vtk.vtkPolyData()
        self.reader.GetDataForHistory(self.numberOfScanLines, self.revPolyData)
        vis.updatePolyData( self.revPolyData , 'point cloud', colorByName=self.colorBy)

    def tick(self):
        self.updateScanLines()

    def setIntensityRange(self, lowerBound, upperBound):
        self.polyDataObj.setRangeMap('intensity', [lowerBound, upperBound])


class SpindleMonitor(object):
    def __init__(self, getSpindleAngleFunction):
        self.lastSpindleAngle = 0
        self.lastStateTime = 0
        self.spindleSpinRateAverager = MovingAverageComputer()
        self.spindleSpinRateAverager.timeWindow = 0.5
        self._getSpindleAngleFunction = getSpindleAngleFunction

    def onRobotStateChanged(self, newState):
        t, newAngle = self._getSpindleAngleFunction()
        elapsed = t - self.lastStateTime
        if (elapsed > 0.001 and elapsed < 100):
            # unwrap
            diff = newAngle - self.lastSpindleAngle
            if (abs(diff - 2*math.pi) < abs(diff)):
                diff = diff - 2*math.pi
            if (abs(diff + 2*math.pi) < abs(diff)):
                diff = diff + 2*math.pi
            velocity = diff / elapsed
            self.spindleSpinRateAverager.update(velocity)
            # if avg veloicty is bad panic
        self.lastStateTime = t
        self.lastSpindleAngle = newAngle

    def getAverageSpindleVelocity(self):
        return self.spindleSpinRateAverager.getAverage()



class RosGridMap(vis.PolyDataItem):

    _requiredProviderClass = perceptionmeta.RosGridMapMeta

    def __init__(self, robotStateJointController, name, callbackFunc=None, provider=None):
        vis.PolyDataItem.__init__(self, name, vtk.vtkPolyData(), view=None)
        self.firstData = True
        self.robotStateJointController = robotStateJointController
        self.timer = TimerCallback()
        self.timer.callback = self.showMap
        self.timer.start()
        self.callbackFunc = callbackFunc

        if provider:
            self.setProvider(provider)
        else:
            self.provider = None

    def setProvider(self, provider):
        if not issubclass(provider.__class__, self._requiredProviderClass):
            raise TypeError("Attempted to set {} provider to {}, but it was not a"
                            " subclass of {} as is required.".format(self.__class__,
                                                                     provider.__class__,
                                                                     self._requiredProviderClass.__class__))

        self.provider = provider
        self.provider.set_consumer(self)

    def _onPropertyChanged(self, propertySet, propertyName):
        vis.PolyDataItem._onPropertyChanged(self, propertySet, propertyName)
        if propertyName == 'Visible':
            if self.getProperty(propertyName):
                self.timer.start()
            else:
                self.timer.stop()
        elif propertyName == 'Color By':
            color= self.getPropertyEnumValue(propertyName)
            self.provider.set_color_layer(color)
            #only_new_data = False because the poly_date need to be redraw with the new color layer
            self.showMap(only_new_data = False)
            self._updateColorBy()

        if self.provider:
            self.provider._on_property_changed(propertySet, propertyName)


    @CheckProvider
    def showMap(self, only_new_data = True):

        polyData = vtk.vtkPolyData()
        self.provider.get_mesh(polyData, only_new_data)
        if polyData.GetNumberOfPoints() == 0:
            return

        bodyHeight = self.robotStateJointController.q[2]
        self.setRangeMap('z', [bodyHeight-0.5, bodyHeight])

        if self.callbackFunc:
            self.callbackFunc()
        #update view
        self.setPolyData(polyData)

        if self.firstData:
            self.firstData = False
            colorList = self.properties.getPropertyAttribute('Color By', 'enumNames')
            zIndex = colorList.index('z') if 'z' in colorList else 0
            self.properties.setProperty('Color By', zIndex)

    @CheckProvider
    def resetTime(self):
        self.provider.reset_time()

    @CheckProvider
    def getPointCloud(self):
        polyData = vtk.vtkPolyData()
        self.provider.get_point_cloud(polyData)
        if polyData.GetNumberOfPoints() == 0:
            return None
        else:
            return polyData


class MarkerSource(vis.PolyDataItem):

    _requiredProviderClass = perceptionmeta.MarkerSourceMeta

    def __init__(self, name, callbackFunc=None, provider=None):
        vis.PolyDataItem.__init__(self, name, vtk.vtkPolyData(), view=None)
        self.timer = TimerCallback()
        self.timer.callback = self.showData
        self.timer.start()

        self.callbackFunc = callbackFunc
        self.resetColor = True

        if provider:
            self.setProvider(provider)
        else:
            self.provider = None

    def setProvider(self, provider):
        if not issubclass(provider.__class__, self._requiredProviderClass):
            raise TypeError("Attempted to set {} provider to {}, but it was not a"
                            " subclass of {} as is required.".format(self.__class__,
                                                                     provider.__class__,
                                                                     self._requiredProviderClass.__class__))

        self.provider = provider
        self.provider.set_consumer(self)

    def _onPropertyChanged(self, propertySet, propertyName):
        vis.PolyDataItem._onPropertyChanged(self, propertySet, propertyName)
        if propertyName == 'Visible':
            if self.getProperty(propertyName):
                self.timer.start()
            else:
                self.timer.stop()
        elif propertyName == 'Subscribe':
            if self.getProperty(propertyName):
                self.timer.start()
            else:
                self.timer.stop()

        if self.provider:
            self.provider._on_property_changed(propertySet, propertyName)

    @CheckProvider
    def resetTime(self):
        self.provider.reset_time()

    @CheckProvider
    def showData(self):
        polyData = vtk.vtkPolyData()
        self.provider.get_mesh(polyData)

        if polyData.GetNumberOfPoints() == 0:
            #if an empty message is received, we will reset the default color when the next message is received
            self.resetColor = True

        if self.callbackFunc:
            self.callbackFunc()

        #update view
        self.setPolyData(polyData)

        if self.resetColor and polyData.GetNumberOfPoints() != 0:
            self.resetColor = False
            colorList = self.properties.getPropertyAttribute('Color By', 'enumNames')
            index = colorList.index('color') if 'color' in colorList else 0
            self.properties.setProperty('Color By', index)


class MarkerArraySource(vis.PolyDataItemList):

    _requiredProviderClass = perceptionmeta.MarkerArraySourceMeta

    def __init__(self, name, singlePolyData=False, callbackFunc=None, provider=None):
        vis.PolyDataItemList.__init__(self, name, 'color')
        # if singlePolyData is True, it means that all the markers received are merged into a single one
        self.singlePolyData = singlePolyData
        self.timer = TimerCallback()
        self.timer.callback = self.showData
        self.timer.start()
        self.callbackFunc = callbackFunc
        #self.topicName = topicName
        if provider:
            self.setProvider(provider)
        else:
            self.provider = None

    def setProvider(self, provider):
        if not issubclass(provider.__class__, self._requiredProviderClass):
            raise TypeError("Attempted to set {} provider to {}, but it was not a"
                            " subclass of {} as is required.".format(self.__class__,
                                                                     provider.__class__,
                                                                     self._requiredProviderClass.__class__))

        self.provider = provider
        self.provider.set_consumer(self)

    def _onPropertyChanged(self, propertySet, propertyName):
        vis.PolyDataItemList._onPropertyChanged(self, propertySet, propertyName)
        if propertyName == 'Visible':
            if self.getProperty(propertyName):
                self.timer.start()
            else:
                self.timer.stop()
        elif propertyName == 'Subscribe':
            if self.getProperty(propertyName):
                self.timer.start()
            else:
                self.timer.stop()
        if self.provider:
            self.provider._on_property_changed(propertySet, propertyName)

    def resetTime(self):
        self.provider.ResetTime()

    @CheckProvider
    def showData(self):
        numPoly = self.provider.get_number_of_mesh()
        polyDataList = []
        if self.singlePolyData:
            polyData = vtk.vtkPolyData()
            self.provider.get_mesh(polyData)
            if polyData.GetNumberOfPoints() > 0:
                polyDataList.append(polyData)
        else:
            for i in range(0, numPoly):
                polyData = vtk.vtkPolyData()
                self.provider.get_mesh(polyData, i)
                polyDataList.append(polyData)

        if self.callbackFunc:
            self.callbackFunc()

        self.setPolyData(polyDataList)


class PointCloudSource(vis.PolyDataItem):

    _requiredProviderClass = perceptionmeta.PointCloudSourceMeta

    def __init__(self, robotStateJointController, callbackFunc=None, provider=None):
        vis.PolyDataItem.__init__(self, 'point cloud', vtk.vtkPolyData(), view=None)
        self.firstData = True
        self.robotStateJointController = robotStateJointController
        self.timer = TimerCallback()
        self.timer.callback = self.showPointCloud
        self.timer.start()
        self.callbackFunc = callbackFunc
        if provider:
            self.setProvider(provider)
        else:
            self.provider = None

    def setProvider(self, provider):
        if not issubclass(provider.__class__, self._requiredProviderClass):
            raise TypeError("Attempted to set {} provider to {}, but it was not a"
                            " subclass of {} as is required.".format(self.__class__,
                                                                     provider.__class__,
                                                                     self._requiredProviderClass.__class__))

        self.provider = provider
        self.provider.set_consumer(self)
        self.addProperty('Updates Enabled', True)
        self.addProperty('Number of Point Clouds', 10,
                         attributes=om.PropertyAttributes(decimals=0, minimum=1, maximum=100, singleStep=1, hidden=False))


    def _onPropertyChanged(self, propertySet, propertyName):
        vis.PolyDataItem._onPropertyChanged(self, propertySet, propertyName)
        if propertyName == 'Visible' or propertyName == 'Updates Enabled':
            if self.getProperty(propertyName):
                self.timer.start()
            else:
                self.timer.stop()
        elif propertyName == 'Number of Point Clouds':
            numberOfPointCloud = self.getProperty(propertyName)
            self.provider.SetNumberOfPointClouds(numberOfPointCloud)

        if self.provider:
            self.provider._on_property_changed(propertySet, propertyName)

    @CheckProvider
    def getPointCloud(self):
        polyData = vtk.vtkPolyData()
        self.provider.get_point_cloud(polyData)
        if polyData.GetNumberOfPoints() == 0:
            return None
        else:
            return polyData

    @CheckProvider
    def resetTime(self):
        self.provider.reset_time()

    @CheckProvider
    def showPointCloud(self):
        polyData = vtk.vtkPolyData()
        self.provider.get_point_cloud(polyData, True)
        if polyData.GetNumberOfPoints() == 0:
            return

        bodyHeight = self.robotStateJointController.q[2]
        self.setRangeMap('z', [bodyHeight-0.5, bodyHeight+0.5])

        if self.callbackFunc:
            self.callbackFunc()
        #update view
        self.setPolyData(polyData)

        if self.firstData:
            self.firstData = False
            colorList = self.properties.getPropertyAttribute('Color By', 'enumNames')
            zIndex = colorList.index('z') if 'z' in colorList else 0
            self.properties.setProperty('Color By', zIndex)



class DepthImagePointCloudSource(vis.PolyDataItem):

    _requiredProviderClass = perceptionmeta.DepthImageSourceMeta

    def __init__(self, name, imagesChannel, cameraName, imageManager, robotStateJointController, provider=None):
        vis.PolyDataItem.__init__(self, name, vtk.vtkPolyData(), view=None)

        self.robotStateJointController = robotStateJointController
        self.addProperty('Channel', imagesChannel)
        self.addProperty('Camera name', cameraName)

        self.addProperty('Decimation', 1, attributes=om.PropertyAttributes(enumNames=['1', '2', '4', '8', '16']))
        self.addProperty('Remove Size', 1000, attributes=om.PropertyAttributes(decimals=0, minimum=0, maximum=100000.0, singleStep=1000))
        self.addProperty('Target FPS', 5.0, attributes=om.PropertyAttributes(decimals=1, minimum=0.1, maximum=30.0, singleStep=0.1))
        self.addProperty('Max Range', 5.0,  attributes=om.PropertyAttributes(decimals=2, minimum=0., maximum=30.0, singleStep=0.25))

        self.imageManager = imageManager
        self.cameraName = cameraName
        self.firstData = True
        self.timer = TimerCallback()
        self.timer.callback = self.update
        self.lastUtime = 0
        self.lastDataReceivedTime = 0

        if provider:
            self.setProvider(provider)
        else:
            self.provider = None

    def setProvider(self, provider):
        if not issubclass(provider.__class__, self._requiredProviderClass):
            raise TypeError("Attempted to set {} provider to {}, but it was not a"
                            " subclass of {} as is required.".format(self.__class__,
                                                                     provider.__class__,
                                                                     self._requiredProviderClass.__class__))

        self.provider = provider
        self.provider.set_consumer(self)

        decimation = int(self.properties.getPropertyEnumValue('Decimation'))
        removeSize = int(self.properties.getProperty('Remove Size'))
        rangeThreshold = float(self.properties.getProperty('Max Range'))
        self.addProperty('Remove Stale Data', False)
        self.addProperty('Stale Data Timeout', 5.0,
                         attributes=om.PropertyAttributes(decimals=1, minimum=0.1, maximum=30.0, singleStep=0.1))

        self.provider.set_decimate(int(decimation))
        self.provider.set_remove_size(removeSize)
        self.provider.set_range_threshold(rangeThreshold)
        self.setProperty('Visible', True)
        self.lastDataReceivedTime = time.time()

    def _onPropertyChanged(self, propertySet, propertyName):
        vis.PolyDataItem._onPropertyChanged(self, propertySet, propertyName)

        if propertyName == 'Visible':
            if self.getProperty(propertyName):
                self.timer.start()
            else:
                self.timer.stop()

        if propertyName in ('Decimation', 'Remove outliers', 'Max Range'):
            self.lastUtime = 0
        if propertyName == 'Decimation':
            decimate = self.getPropertyEnumValue(propertyName)
            self.provider.set_decimate(int(decimate))
        elif propertyName == 'Remove Size':
            remove_size = self.getProperty(propertyName)
            self.provider.set_remove_size(remove_size)
        elif propertyName == 'Max Range':
            max_range = self.getProperty(propertyName)
            self.provider.set_range_threshold(max_range)

    @CheckProvider
    def getPointCloud(self):
        polyData = vtk.vtkPolyData()
        self.provider.get_point_cloud(polyData)
        if polyData.GetNumberOfPoints() == 0:
            return None
        else:
            return polyData

    def onRemoveFromObjectModel(self):
        vis.PolyDataItem.onRemoveFromObjectModel(self)
        self.timer.stop()

    @CheckProvider
    def resetTime(self):
        self.provider.reset_time()

    @CheckProvider
    def update(self):
        #utime = self.imageManager.queue.getCurrentImageTime(self.cameraName)
        utime =  self.provider.get_sec() *1E6 + round( self.provider.get_nsec() *1E-3)

        if utime == self.lastUtime:
            if self.getProperty('Remove Stale Data') and ((time.time()-self.lastDataReceivedTime) > self.getProperty('Stale Data Timeout')):
                if self.polyData.GetNumberOfPoints() > 0:
                    self.setPolyData(vtk.vtkPolyData())
            return

        if (utime < self.lastUtime):
            temp=0 # dummy
        elif (utime - self.lastUtime < 1E6/self.getProperty('Target FPS')):
            return

        polyData = vtk.vtkPolyData()
        new_data = self.provider.get_point_cloud(polyData, True)
        if polyData.GetNumberOfPoints() == 0:
            return

        # currently disabled
        #bodyToLocal = vtk.vtkTransform()
        #self.imageManager.queue.getTransform('body', 'local', utime, bodyToLocal)
        #bodyHeight = bodyToLocal.GetPosition()[2]

        bodyHeight = self.robotStateJointController.q[2]
        self.setRangeMap('z',[bodyHeight-0.5, bodyHeight+0.5])

        self.setPolyData(polyData)

        if self.firstData:
            self.firstData = False
            colorList = self.properties.getPropertyAttribute('Color By', 'enumNames')
            zIndex = colorList.index('z') if 'z' in colorList else 0
            self.properties.setProperty('Color By', zIndex)

        self.lastDataReceivedTime = time.time()
        self.lastUtime = utime



def init(view, robotStateJointController):
    global _multisenseItem

    # If the robot name is blank, put Robot in instead so that it's not just a blank box.
    sensorsFolder = om.getOrCreateContainer('sensors', om.getOrCreateContainer(robotStateJointController.robotName or
                                                                               "Robot"))


    #queue = PythonQt.dd.ddPointCloudLCM(lcmUtils.getGlobalLCMThread())
    #queue.init(lcmUtils.getGlobalLCMThread(), drcargs.args().config_file)
    #lidarNames = queue.getLidarNames()
    #for lidar in lidarNames:
    #    if queue.displayLidar(lidar):
    #        
    #        l = LidarSource(view, queue.getLidarChannelName(lidar), queue.getLidarCoordinateFrame(lidar), queue.getLidarFriendlyName(lidar), queue.getLidarIntensity(lidar))
    #        l.start()
    #        lidarDriver = l
    #        _lidarItem = LidarItem(l)
    #        om.addToObjectModel(_lidarItem, sensorsFolder)


    #useMapServer = hasattr(drc, 'vtkMapServerSource')
    #if useMapServer:
    #    mapServerSource = MapServerSource(view, callbackFunc=view.render)
    #    mapsServerContainer = om.ObjectModelItem('Map Server', icon=om.Icons.Robot)
    #    mapsServerContainer.source = mapServerSource
    #    om.addToObjectModel(mapsServerContainer, parentObj=sensorsFolder)
    #    mapServerSource.start()
    #else:
    #    mapServerSource = None

    #om.addToObjectModel(rosInit, sensorsFolder)

    #elevation map

    gridMapSource = RosGridMap(robotStateJointController, 'elevation map', callbackFunc=view.render)
    gridMapSource.addToView(view)
    om.addToObjectModel(gridMapSource, sensorsFolder)
    
    #lidar elevation map        

    gridMapLidarSource = RosGridMap(robotStateJointController, 'lidar elevation map', callbackFunc=view.render)
    gridMapLidarSource.setProperty('Visible', False)
    gridMapLidarSource.addToView(view)
    om.addToObjectModel(gridMapLidarSource, sensorsFolder)

    pointCloudSource = PointCloudSource(robotStateJointController, callbackFunc=view.render)
    pointCloudSource.addToView(view)
    om.addToObjectModel(pointCloudSource, sensorsFolder)

    if not robotStateJointController.robotName:
        depthCameras = drcargs.getDirectorConfig()['depthCameras']
        depthCamerasShortName = drcargs.getDirectorConfig()['depthCamerasShortName']
    else:
        depthCameras = drcargs.getDirectorConfig()[robotStateJointController.robotName]['depthCameras']
        depthCamerasShortName = drcargs.getDirectorConfig()[robotStateJointController.robotName]['depthCamerasShortName']

    headCameraPointCloudSource = DepthImagePointCloudSource(depthCamerasShortName[0], depthCameras[0], str(depthCameras[0] + '_LEFT'), None,
                                 robotStateJointController)
    headCameraPointCloudSource.addToView(view)
    om.addToObjectModel(headCameraPointCloudSource, sensorsFolder)

    groundCameraPointCloudSource = DepthImagePointCloudSource(depthCamerasShortName[1], depthCameras[1], str(depthCameras[1] + '_LEFT'), None,
                                                              robotStateJointController)
    groundCameraPointCloudSource.addToView(view)
    om.addToObjectModel(groundCameraPointCloudSource, sensorsFolder)

    #if (i==0):
    #    mainDisparityPointCloud = disparityPointCloud

    #def createPointerTracker():
    #    return trackers.PointerTracker(robotStateModel, mainDisparityPointCloud)

    return pointCloudSource, gridMapSource, gridMapLidarSource, headCameraPointCloudSource, groundCameraPointCloudSource
