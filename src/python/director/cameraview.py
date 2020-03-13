import director.applogic as app
from director import transformUtils
from director import visualization as vis
from director import filterUtils
from director import drcargs
from director import perceptionmeta
from director.shallowCopy import shallowCopy
from director.timercallback import TimerCallback
from director import vtkNumpy
from director import objectmodel as om
import director.vtkAll as vtk
from director.debugVis import DebugData
import vtkRosPython as vtkRos

import PythonQt
from PythonQt import QtCore, QtGui
import numpy as np
from director.simpletimer import SimpleTimer
from director import ioutils
import sys
import rospy


def clipRange(dataObj, arrayName, thresholdRange):
    if not dataObj.GetPointData().GetArray(arrayName):
        raise Exception('clipRange: could not locate array: %s' % arrayName)

    dataObj.GetPointData().SetScalars(dataObj.GetPointData().GetArray(arrayName))

    f = vtk.vtkClipPolyData()
    f.SetInputData(dataObj)
    f.SetValue(thresholdRange[0])
    f.SetInputArrayToProcess(0, 0, 0, vtk.vtkDataObject.FIELD_ASSOCIATION_POINTS, arrayName)

    f2 = vtk.vtkClipPolyData()
    f2.AddInputConnection(f.GetOutputPort())
    f2.SetValue(thresholdRange[1])
    f2.InsideOutOn()
    f2.SetInputArrayToProcess(0, 0, 0, vtk.vtkDataObject.FIELD_ASSOCIATION_POINTS, arrayName)

    f2.Update()
    return shallowCopy(f2.GetOutput())



def makeSphere(radius, resolution):

    s = vtk.vtkSphereSource()
    s.SetThetaResolution(resolution)
    s.SetPhiResolution(resolution)
    s.SetRadius(radius)
    s.SetEndPhi(85)
    s.Update()
    return shallowCopy(s.GetOutput())



def rayDebug(position, ray):
    d = DebugData()
    d.addLine(position, position+ray*5.0)
    drcView = app.getViewManager().findView('DRC View')
    obj = vis.updatePolyData(d.getPolyData(), 'camera ray', view=drcView, color=[0,1,0])
    obj.actor.GetProperty().SetLineWidth(2)


class ImageManager(object):

    _requiredProviderClass = perceptionmeta.ImageSourceMeta

    def __init__(self):

        self.images = {}
        self.imageUtimes = {}
        self.textures = {}
        self.imageRotations180 = {}
        self.providerClass = None

        self.queue = {}

    #def getFrameNames(self):
    #    if len(self.queue) == 0:
    #        return
    #    else:
    #        key = next(iter(self.queue))
    #        frames = self.queue[key].GetFrameNames()
    #        return frames.split(',')

    def resetTime(self):
        for cameraName, subscriber in self.queue.iteritems():
            subscriber.reset_time()

    def addImage(self, name):

        if name in self.images:
            return

        image = vtk.vtkImageData()
        tex = vtk.vtkTexture()
        tex.SetInputData(image)
        tex.EdgeClampOn()
        tex.RepeatOff()

        self.imageUtimes[name] = 0
        self.images[name] = image
        self.textures[name] = tex
        self.imageRotations180[name] = False

        if self.providerClass:
            self.queue[name] = self.providerClass.initialise_from_name(name)
        else:
            print("Could not initialise camera {} as the provider class is not initialised.".format(name))
            self.queue[name] = None

    def setProviderClass(self, provider):
        if not issubclass(provider, self._requiredProviderClass):
            raise TypeError("Attempted to set {} provider to {}, but it was not a"
                            " subclass of {} as is required.".format(self.__class__,
                                                                     provider.__class__,
                                                                     self._requiredProviderClass.__class__))

        self.providerClass = provider
        # Initialise the provider for names which were added to the object before this point
        for name in self.images.keys():
            print("Initialising image provider for {}".format(name))
            self.queue[name] = self.providerClass.initialise_from_name(name)

    def writeImage(self, imageName, outFile):
        writer = vtk.vtkPNGWriter()
        writer.SetInput(self.images[imageName])
        writer.SetFileName(outFile)
        writer.Write()

    def updateImage(self, imageName):
        # If the given imagename has not had its provider initialised, use the original utime to indicate that no
        # data was received.
        if not self.queue[imageName]:
            return self.imageUtimes[imageName]

        imageUtime = self.queue[imageName].get_current_image_time()
        if imageUtime != self.imageUtimes[imageName]:
            image = self.images[imageName]
            self.imageUtimes[imageName] = self.queue[imageName].get_image(image)

            if self.imageRotations180[imageName]:
                self.images[imageName].ShallowCopy(filterUtils.rotateImage180(image))

        return imageUtime

    def updateImages(self):
        for imageName in self.images.keys():
            self.updateImage(imageName)

    def setImageRotation180(self, imageName):
        assert imageName in self.images
        self.imageRotations180[imageName] = True

    def hasImage(self, imageName):
        return imageName in self.images

    def getImage(self, imageName):
        return self.images[imageName]

    def getUtime(self, imageName):
        return self.imageUtimes[imageName]

    def getTexture(self, imageName):
        return self.textures[imageName]


def disableCameraTexture(obj):
    obj.actor.SetTexture(None)
    obj.actor.GetProperty().LightingOn()
    obj.actor.GetProperty().SetColor(obj.getProperty('Color'))


class CameraView(object):

    def __init__(self, imageManager, view=None):

        self.imageManager = imageManager
        self.updateUtimes = {}
        self.robotModel = None
        self.sphereObjects = {}
        self.sphereImages = drcargs.getDirectorConfig()['monoCameras']

        for name in self.sphereImages:
            imageManager.addImage(name)
            self.updateUtimes[name] = 0

        self.initView(view)
        self.initEventFilter()
        self.rayCallback = rayDebug

        self.timerCallback = TimerCallback()
        self.timerCallback.targetFps = 60
        self.timerCallback.callback = self.updateView
        self.timerCallback.start()

    def resetTime(self):
        self.imageManager.resetTime()

    def filterEvent(self, obj, event):
        if event.type() == QtCore.QEvent.KeyPress:
            if str(event.text()).lower() == 'p':
                self.eventFilter.setEventHandlerResult(True)
            elif str(event.text()).lower() == 'r':
                self.eventFilter.setEventHandlerResult(True)
                self.resetCamera()

    def initEventFilter(self):
        self.eventFilter = PythonQt.dd.ddPythonEventFilter()
        qvtkwidget = self.view.vtkWidget()
        qvtkwidget.installEventFilter(self.eventFilter)
        self.eventFilter.addFilteredEventType(QtCore.QEvent.MouseButtonDblClick)
        self.eventFilter.addFilteredEventType(QtCore.QEvent.KeyPress)
        self.eventFilter.connect('handleEvent(QObject*, QEvent*)', self.filterEvent)

    def initView(self, view):

        self.view = view or app.getViewManager().createView('Camera View', 'VTK View')

        self.renderers = [self.view.renderer()]
        renWin = self.view.renderWindow()
        renWin.SetNumberOfLayers(3)
        for i in [1, 2]:
            ren = vtk.vtkRenderer()
            ren.SetLayer(2)
            ren.SetActiveCamera(self.view.camera())
            renWin.AddRenderer(ren)
            self.renderers.append(ren)

        def applyCustomBounds():
            self.view.addCustomBounds([-100, 100, -100, 100, -100, 100])
        self.view.connect('computeBoundsRequest(ddQVTKWidgetView*)', applyCustomBounds)

        app.setCameraTerrainModeEnabled(self.view, True)
        self.resetCamera()

    def resetCamera(self):
        self.view.camera().SetViewAngle(90)
        self.view.camera().SetPosition(2.1, 0.75, -1.0)
        self.view.camera().SetFocalPoint(11.0, 0.76, -3.9)
        self.view.camera().SetViewUp(0.0, 0.0, 1.0)
        self.view.render()

    def getSphereGeometry(self, imageName):

        sphereObj = self.sphereObjects.get(imageName)
        if sphereObj:
            return sphereObj

        if not self.imageManager.getImage(imageName).GetDimensions()[0]:
            return None

        sphereResolution = 50
        sphereRadii = 20

        geometry = makeSphere(sphereRadii, sphereResolution)
        self.imageManager.queue[imageName].compute_texture_coords(imageName, geometry)

        tcoordsArrayName = 'tcoords_%s' % imageName
        vtkNumpy.addNumpyToVtk(geometry, vtkNumpy.getNumpyFromVtk(geometry, tcoordsArrayName)[:,0].copy(), 'tcoords_U')
        vtkNumpy.addNumpyToVtk(geometry, vtkNumpy.getNumpyFromVtk(geometry, tcoordsArrayName)[:,1].copy(), 'tcoords_V')
        geometry = clipRange(geometry, 'tcoords_U', [0.0, 1.0])
        geometry = clipRange(geometry, 'tcoords_V', [0.0, 1.0])
        geometry.GetPointData().SetTCoords(geometry.GetPointData().GetArray(tcoordsArrayName))

        sphereObj = vis.showPolyData(geometry, imageName, view=self.view, parent='cameras')
        sphereObj.actor.SetTexture(self.imageManager.getTexture(imageName))
        sphereObj.actor.GetProperty().LightingOff()

        self.view.renderer().RemoveActor(sphereObj.actor)
        rendererId = 2 - self.sphereImages.index(imageName)
        self.renderers[rendererId].AddActor(sphereObj.actor)

        self.sphereObjects[imageName] = sphereObj
        return sphereObj

    def updateSphereGeometry(self):

        for imageName in self.sphereImages:
            sphereObj = self.getSphereGeometry(imageName)
            if not sphereObj:
                continue

            transform = vtk.vtkTransform()
            self.imageManager.queue[imageName].get_body_to_camera_transform(transform)
            sphereObj.actor.SetUserTransform(transform.GetLinearInverse())

    def updateImages(self):

        updated = False
        for imageName, lastUtime in self.updateUtimes.iteritems():
            currentUtime = self.imageManager.updateImage(imageName)
            if currentUtime != lastUtime:
                self.updateUtimes[imageName] = currentUtime
                updated = True

        return updated

    def updateView(self):

        if not self.view.isVisible():
            return

        if not self.updateImages():
            return

        self.updateSphereGeometry()
        self.view.render()


class ImageWidget(object):

    def __init__(self, imageManager, imageNames, view, visible=True):
        self.view = view
        self.imageManager = imageManager
        self.imageNames = imageNames
        self.visible = visible
        self.widgetWidth = 400
        self.showNonMainImages = True # if false, show only imageNames[0]

        self.updateUtime = 0
        self.initialized = False

        #these two data structures are initialized when data is received
        self.imageWidgets = [None for i in range(0, len(self.imageNames))]
        self.flips = [None for i in range(0, len(self.imageNames))]

        self.eventFilter = PythonQt.dd.ddPythonEventFilter()
        self.view.installEventFilter(self.eventFilter)
        self.eventFilter.addFilteredEventType(QtCore.QEvent.Resize)
        self.eventFilter.connect('handleEvent(QObject*, QEvent*)', self.onResizeEvent)

        self.timerCallback = TimerCallback()
        self.timerCallback.targetFps = 60
        self.timerCallback.callback = self.updateView
        self.timerCallback.start()

    def initImageFlip(self, i):
        if i >= len(self.flips):
            return
        if self.flips[i]: #already initialized
            return

        self.flips[i] = vtk.vtkImageFlip()
        self.flips[i].SetFilteredAxis(1)
        self.flips[i].SetInputData(imageManager.getImage(self.imageNames[i]))

        self.imageWidgets[i] = vtk.vtkLogoWidget()
        self.imageWidgets[i].ResizableOff()
        self.imageWidgets[i].SelectableOn()
        self.imageWidgets[i].SetInteractor(self.view.renderWindow().GetInteractor())

        imageRep = self.imageWidgets[i].GetRepresentation()
        imageRep.GetImageProperty().SetOpacity(1.0)
        imageRep.SetImage(self.flips[i].GetOutput())


    def setWidgetSize(self, desiredWidth=400):

        offsetY = 0
        for i, imageName in enumerate(self.imageNames):
            if not self.imageWidgets[i]:
                continue

            image = self.imageManager.getImage(imageName)
            dims = image.GetDimensions()
            if 0 in dims:
                continue

            aspectRatio = float(dims[0])/dims[1]
            imageWidth, imageHeight = desiredWidth, desiredWidth/aspectRatio
            viewWidth, viewHeight = self.view.width, self.view.height

            rep = self.imageWidgets[i].GetBorderRepresentation()
            rep.SetShowBorderToOff()
            coord = rep.GetPositionCoordinate()
            coord2 = rep.GetPosition2Coordinate()
            coord.SetCoordinateSystemToDisplay()
            coord2.SetCoordinateSystemToDisplay()
            coord.SetValue(0, viewHeight-imageHeight-offsetY)
            coord2.SetValue(imageWidth, imageHeight)
            offsetY += imageHeight

        self.view.render()

    def onResizeEvent(self):
        self.setWidgetSize(self.widgetWidth)

    #def setImageName(self, imageName):
    #    self.imageName = imageName
    #    self.flip.SetInputData(imageManager.getImage(imageName))

    def setOpacity(self, opacity=1.0):
        for imageWidget in self.imageWidgets:
            if not imageWidget:
                continue
            imageWidget.GetRepresentation().GetImageProperty().SetOpacity(opacity)

    def hide(self):
        self.visible = False
        for i, imageWidget in enumerate(self.imageWidgets):
            if (imageWidget is not None):
                imageWidget.Off()
        self.view.render()

    def show(self):
        self.visible = True
        if self.haveImage():
            for i, imageWidget in enumerate(self.imageWidgets):
                if (i==0 or self.showNonMainImages) and imageWidget:
                    imageWidget.On()
            self.view.render()

    def haveImage(self):
        for imageName in self.imageNames:
            image = self.imageManager.getImage(imageName)
            dims = image.GetDimensions()
            if 0 not in dims:
                return True
        return False

    def updateView(self):
        if not self.visible or not self.view.isVisible():
            return

        currentUtime = 0
        for imageName in self.imageNames:
            currentUtime = max(self.imageManager.updateImage(imageName), currentUtime)


        if currentUtime == 0:
            #no data received
            return

        if currentUtime != self.updateUtime:
            self.updateUtime = currentUtime
            for i in range(0, len(self.flips)):
                image = self.imageManager.getImage(self.imageNames[i])
                if 0 not in image.GetDimensions():
                    #the image is not empty
                    self.initImageFlip(i)
                    self.flips[i].Update()
            self.view.render()

            if not self.initialized and self.visible and self.haveImage():
                self.show()
                self.setWidgetSize(self.widgetWidth)
                self.initialized = True


class CameraImageView(object):

    def __init__(self, imageManager, imageName, viewName=None, view=None):

        imageManager.addImage(imageName)

        self.cameraRoll = None
        self.imageManager = imageManager
        self.viewName = viewName or imageName
        self.imageName = imageName
        self.imageInitialized = False
        self.updateUtime = 0
        self.useImageColorMap = False
        self.imageMapToColors = None
        self.initView(view)
        self.initEventFilter()


    def getImagePixel(self, displayPoint, restrictToImageDimensions=True):

        worldPoint = [0.0, 0.0, 0.0, 0.0]
        vtk.vtkInteractorObserver.ComputeDisplayToWorld(self.view.renderer(), displayPoint[0], displayPoint[1], 0, worldPoint)

        imageDimensions = self.getImage().GetDimensions()

        if 0.0 <= worldPoint[0] <= imageDimensions[0] and 0.0 <= worldPoint[1] <= imageDimensions[1] or not restrictToImageDimensions:
            return [worldPoint[0], worldPoint[1], 0.0]
        else:
            return None


    def getWorldPositionAndRay(self, imagePixel, imageUtime=None):
        #disabled
        return np.array([0, 0, 0]), np.array([0, 0, 0])
        '''
        Given an XY image pixel, computes an equivalent ray in the world
        coordinate system using the camera to local transform at the given
        imageUtime.  If imageUtime is None, then the utime of the most recent
        image is used.

        Returns the camera xyz position in world, and a ray unit vector.
        '''
        if imageUtime is None:
            imageUtime = self.imageManager.getUtime(self.imageName)

        # input is pixel u,v, output is unit x,y,z in camera coordinates
        cameraPoint = self.imageManager.queue.unprojectPixel(self.imageName, imagePixel[0], imagePixel[1])

        cameraToLocal = vtk.vtkTransform()
        self.imageManager.queue.getTransform(self.imageName, 'local', imageUtime, cameraToLocal)

        p = np.array(cameraToLocal.TransformPoint(cameraPoint))
        cameraPosition = np.array(cameraToLocal.GetPosition())
        ray = p - cameraPosition
        ray /= np.linalg.norm(ray)

        return cameraPosition, ray


    def filterEvent(self, obj, event):
        if self.eventFilterEnabled and event.type() == QtCore.QEvent.MouseButtonDblClick:
            self.eventFilter.setEventHandlerResult(True)

        elif event.type() == QtCore.QEvent.KeyPress:
            if str(event.text()).lower() == 'p':
                self.eventFilter.setEventHandlerResult(True)
            elif str(event.text()).lower() == 'r':
                self.eventFilter.setEventHandlerResult(True)
                self.resetCamera()

    def onRubberBandPick(self, obj, event):
        displayPoints = self.interactorStyle.GetStartPosition(), self.interactorStyle.GetEndPosition()
        imagePoints = [vis.pickImage(point, self.view)[1] for point in displayPoints]

    def getImage(self):
        return self.imageManager.getImage(self.imageName)

    def initView(self, view):
        self.view = view or app.getViewManager().createView(self.viewName, 'VTK View')
        self.view.installImageInteractor()
        #self.interactorStyle = self.view.renderWindow().GetInteractor().GetInteractorStyle()
        #self.interactorStyle.AddObserver('SelectionChangedEvent', self.onRubberBandPick)

        self.imageActor = vtk.vtkImageActor()
        self.imageActor.SetInputData(self.getImage())
        self.imageActor.SetVisibility(False)
        self.view.renderer().AddActor(self.imageActor)

        self.view.orientationMarkerWidget().Off()
        self.view.backgroundRenderer().SetBackground(0,0,0)
        self.view.backgroundRenderer().SetBackground2(0,0,0)

        self.timerCallback = TimerCallback()
        self.timerCallback.targetFps = 60
        self.timerCallback.callback = self.updateView
        self.timerCallback.start()

    def initEventFilter(self):
        self.eventFilter = PythonQt.dd.ddPythonEventFilter()
        qvtkwidget = self.view.vtkWidget()
        qvtkwidget.installEventFilter(self.eventFilter)
        self.eventFilter.addFilteredEventType(QtCore.QEvent.MouseButtonDblClick)
        self.eventFilter.addFilteredEventType(QtCore.QEvent.KeyPress)
        self.eventFilter.connect('handleEvent(QObject*, QEvent*)', self.filterEvent)
        self.eventFilterEnabled = True

    def setCameraRoll(self, roll):
        self.cameraRoll = roll
        self.resetCamera()

    def resetCamera(self):
        camera = self.view.camera()
        camera.ParallelProjectionOn()
        camera.SetFocalPoint(0,0,0)
        camera.SetPosition(0,0,-1)
        camera.SetViewUp(0,-1, 0)

        if self.cameraRoll is not None:
            camera.SetRoll(self.cameraRoll)

        self.view.resetCamera()
        self.fitImageToView()
        self.view.render()

    def fitImageToView(self):

        camera = self.view.camera()
        image = self.getImage()
        imageWidth, imageHeight, _ = image.GetDimensions()

        viewWidth, viewHeight = self.view.renderWindow().GetSize()
        aspectRatio = float(viewWidth)/viewHeight
        parallelScale = max(imageWidth/aspectRatio, imageHeight) / 2.0
        camera.SetParallelScale(parallelScale)

    def setImageName(self, imageName):
        if imageName == self.imageName:
            return

        assert self.imageManager.hasImage(imageName)

        self.imageName = imageName
        self.imageInitialized = False
        self.updateUtime = 0
        self.imageActor.SetInputData(self.imageManager.getImage(self.imageName))
        self.imageActor.SetVisibility(False)
        self.view.render()

    def initImageColorMap(self):

        self.depthImageColorByRange = self.getImage().GetScalarRange()

        lut = vtk.vtkLookupTable()
        lut.SetNumberOfColors(256)
        lut.SetHueRange(0, 0.667) # red to blue
        lut.SetRange(self.depthImageColorByRange) # map red (near) to blue (far)
        lut.SetRampToLinear()
        lut.Build()

        im = vtk.vtkImageMapToColors()
        im.SetLookupTable(lut)
        im.SetInputData(self.getImage())
        im.Update()
        self.depthImageLookupTable = lut
        self.imageMapToColors = im
        self.imageActor.SetInputData(im.GetOutput())

    def updateView(self):

        if not self.view.isVisible():
            return

        if self.useImageColorMap and self.imageMapToColors:
            self.imageMapToColors.Update()

        currentUtime = self.imageManager.updateImage(self.imageName)
        if currentUtime != self.updateUtime:
            self.updateUtime = currentUtime
            self.view.render()

            if not self.imageInitialized and self.getImage().GetDimensions()[0]:

                if self.useImageColorMap:
                    self.initImageColorMap()

                self.imageActor.SetVisibility(True)
                self.resetCamera()
                self.imageInitialized = True




views = {}


def addCameraView(channel, viewName=None, cameraName=None, imageType=-1, addToView=True):
    cameraName = cameraName or channel

    if (addToView):
        #print "will add to view", cameraName
        imageManager.addImage(cameraName)
        view = CameraImageView(imageManager, cameraName, viewName)
        global views
        views[channel] = view
        return view
    else:
        print "will NOT camera add to view", cameraName
        return None





def init(view=None,addToView=True):
    global imageManager
    imageManager = ImageManager()

    global cameraView
    cameraView = CameraView(imageManager,view)

    if "modelName" in drcargs.getDirectorConfig():
        _modelName = drcargs.getDirectorConfig()['modelName']
        cameraNames = imageManager.images

        monoCameras = drcargs.getDirectorConfig()['monoCameras']
        monoCamerasShortName = drcargs.getDirectorConfig()['monoCamerasShortName']

        for i in range( len(monoCameras)):
            monoCamera = monoCameras[i]
            monoCameraShortName = monoCamerasShortName[i]
            if monoCamera in cameraNames:
                addCameraView(monoCamera, monoCameraShortName, addToView=addToView)

