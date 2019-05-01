import weakref
import itertools

from director import transformUtils
from director import frameupdater
import director.applogic as app
import director.objectmodel as om
import director.vtkAll as vtk
from director import visualization as vis


class TfFrameSync(object):

    class FrameData(object):
        def __init__(self, **kwargs):
            self.__dict__.update(kwargs)

    def __init__(self, frame):
        self.items = {}
        self.frame = frame
        self._blockCallbacks = False
        self._ids = itertools.count()

    def addItem(self, item, timestamp, ignoreIncoming=False):

        if item is None:
            return

        if self._findItemId(item) is not None:
            return

        itemId = self._ids.next()
        callbackId = item.connectFrameModified(self._onFrameModified)

        self.items[itemId] = TfFrameSync.FrameData(
            ref=weakref.ref(item),
            baseTransform=self._computeBaseTransform(item),
            callbackId=callbackId, timestamp=timestamp,
            ignoreIncoming=ignoreIncoming)

    def removeItem(self, item):

        itemId = self._findItemId(item)
        if itemId is None:
            raise KeyError(item)

        item.disconnectFrameModified(self.items[itemId].callbackId)
        self._removeItemId(itemId)

    def _computeBaseTransform(self, item):

        currentDelta = None
        for itemId, frameData in self.items.items():

            if frameData.ref() is None:
                self._removeItemId(itemId)
            elif frameData.ref() is item:
                continue
            else:
                currentDelta = transformUtils.copyFrame(frameData.baseTransform.GetLinearInverse())
                currentDelta.Concatenate(transformUtils.copyFrame(frameData.ref().transform))
                break

        t = transformUtils.copyFrame(item.transform)
        t.PostMultiply()
        if currentDelta:
            t.Concatenate(currentDelta.GetLinearInverse())

        return t

    def _removeItemId(self, itemId):
        del self.items[itemId]

    def _findItemId(self, frame):

        for itemId, frameData in self.items.items():

            if frameData.ref() is None:
                self._removeItemId(itemId)
            elif frameData.ref() is frame:
                return itemId

    def _moveFrame(self, itemId, modifiedItemId):

        frameData = self.items[itemId]
        modifiedFrameData = self.items[modifiedItemId]

        t = vtk.vtkTransform()
        t.PostMultiply()
        t.Concatenate(frameData.baseTransform)
        t.Concatenate(modifiedFrameData.baseTransform.GetLinearInverse())
        t.Concatenate(modifiedFrameData.ref().transform)
        frameData.ref().copyFrame(t)

    def _onFrameModified(self, frame):

        if self._blockCallbacks:
            return

        modifiedItemId = self._findItemId(frame)
        assert modifiedItemId is not None

        #print self, 'onFrameModified:', self.items[modifiedItemId].ref().getProperty('Name')

        if self.items[modifiedItemId].ignoreIncoming:
            self.items[modifiedItemId].baseTransform = self._computeBaseTransform(frame)
            return

        self._blockCallbacks = True

        for itemId, frameData in self.items.items():
            if frameData.ref() is None:
                self._removeItemId(itemId)
            elif itemId != modifiedItemId:

                #print '  ', self, 'moving:', self.items[itemId].ref().getProperty('Name')
                self._moveFrame(itemId, modifiedItemId)

        self._blockCallbacks = False



class TfMovableItem(vis.PolyDataItem):

    def __init__(self, name, transform, view):

        vis.PolyDataItem.__init__(self, name, vtk.vtkPolyData(), view)

        self.transform = transform
        self._blockSignals = False
        self._frameSync = None

        self.callbacks.addSignal('FrameModified')
        self.onTransformModifiedCallback = None
        self.observerTag = self.transform.AddObserver('ModifiedEvent', self.onTransformModified)



    def connectFrameModified(self, func):
        return self.callbacks.connect('FrameModified', func)

    def disconnectFrameModified(self, callbackId):
        self.callbacks.disconnect(callbackId)

    def onTransformModified(self, transform, event):
        if not self._blockSignals:
            if self.onTransformModifiedCallback:
                self.onTransformModifiedCallback(self)
            self.callbacks.process('FrameModified', self)

    def addToView(self, view):
        vis.PolyDataItem.addToView(self, view)

    def copyFrame(self, transform):
        self._blockSignals = True
        self.transform.SetMatrix(transform.GetMatrix())
        self._blockSignals = False
        self.transform.Modified()
        parent = self.parent()
        if (parent and parent.getProperty('Visible')) or self.getProperty('Visible'):
            self._renderAllViews()

    def getFrameSync(self):
        return self._frameSync


    def onRemoveFromObjectModel(self):
        vis.PolyDataItem.onRemoveFromObjectModel(self)

        self.transform.RemoveObserver(self.observerTag)

        self.widget.SetInteractor(None)
        self.widget.EnabledOff()
        for view in self.views:
            view.renderer().RemoveActor(self.actor)
            view.render()


class TfFrameItem(TfMovableItem):

    def __init__(self, name, transform, view):

        TfMovableItem.__init__(self, name, transform, view)


        self.actor.SetUserTransform(transform)

        self.widget = vtk.vtkFrameWidget()
        self.widget.CreateDefaultRepresentation()
        self.widget.EnabledOff()
        self.rep = self.widget.GetRepresentation()
        self.rep.SetTransform(transform)
        self.traceData = None
        self._frameSync = None

        self.addProperty('Scale', 1.0, attributes=om.PropertyAttributes(decimals=2, minimum=0.01, maximum=100, singleStep=0.1, hidden=False))
        self.addProperty('Edit', False)
        self.addProperty('Trace', False)
        self.addProperty('Tube', False)
        self.addProperty('Tube Width', 0.002, attributes=om.PropertyAttributes(decimals=3, minimum=0.001, maximum=10, singleStep=0.01, hidden=True))

        self.properties.setPropertyIndex('Edit', 0)
        self.properties.setPropertyIndex('Trace', 1)
        self.properties.setPropertyIndex('Tube', 2)


        self._updateAxesGeometry()
        self.setProperty('Color By', 'Axes')
        self.setProperty('Icon', om.Icons.Axes)


    def _updateAxesGeometry(self):
        scale = self.getProperty('Scale')
        self.rep.SetWorldSize(scale)
        self.setPolyData(vis.createAxesPolyData(scale, self.getProperty('Tube'), self.getProperty('Tube Width')))

    def _onPropertyChanged(self, propertySet, propertyName):
        vis.PolyDataItem._onPropertyChanged(self, propertySet, propertyName)

        if propertyName == 'Scale':
            scale = self.getProperty(propertyName)
            self.rep.SetWorldSize(scale)
            self._updateAxesGeometry()
        elif propertyName == 'Edit':
            view = app.getCurrentRenderView()
            if view not in self.views:
                view = self.views[0]
            self.widget.SetInteractor(view.renderWindow().GetInteractor())

            self.widget.SetEnabled(self.getProperty(propertyName))
            isEditing = self.getProperty(propertyName)
            if isEditing:
                frameupdater.registerFrame(self)
        elif propertyName == 'Trace':
            trace = self.getProperty(propertyName)
            if trace and not self.traceData:
                self.traceData = vis.FrameTraceVisualizer(self)
            elif not trace and self.traceData:
                om.removeFromObjectModel(self.traceData.getTraceData())
                self.traceData = None
        elif propertyName == 'Tube':
            self.properties.setPropertyAttribute('Tube Width', 'hidden', not self.getProperty(propertyName))
            self._updateAxesGeometry()

def updatePolyData(polyData, name, **kwargs):

    #obj = om.findObjectByName(name)
    #obj = obj or showPolyData(polyData, name, **kwargs)
    #obj.setPolyData(polyData)
    return obj

def showFrame(frame, name, view=None, parent='segmentation', scale=0.35, visible=True):

    view = view or app.getCurrentRenderView()
    assert view

    if isinstance(parent, str):
        parentObj = om.getOrCreateContainer(parent)
    else:
        parentObj = parent

    item = TfFrameItem(name, frame, view)
    om.addToObjectModel(item, parentObj)
    item.setProperty('Visible', visible)
    item.setProperty('Scale', scale)
    return item


def updateFrame(frame, name, **kwargs):

    obj = om.findObjectByName(name)
    obj = obj or showFrame(frame, name, **kwargs)
    obj.copyFrame(frame)
    return obj