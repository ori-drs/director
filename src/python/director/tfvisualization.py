import weakref
import itertools
import traceback
import sys
import abc
from threading import Lock

import rospy
import tf, tf2_ros
from tf.msg import tfMessage
from geometry_msgs.msg import Pose
from director import rosutils

from director import transformUtils
from director import frameupdater
import director.applogic as app
import director.objectmodel as om
import director.vtkAll as vtk
from director import visualization as vis
from director.timercallback import TimerCallback

import vtkRosPython as vtkRos


class TfFrameSync(object):
    """
        A class used to draw items in a custom global frame
        Each item is represented in a local frame, this class synchronized the item thanks to tfListener
        in order to draw it in another frame
    """

    listener = None

    class FrameData(object):
        def __init__(self, **kwargs):
            self.__dict__.update(kwargs)

    def __init__(self, frame):
        self.items = {}
        # a base item represent the transform between the frame of an item and rootFrame
        # one for each item ( because of timestamp )
        self.baseItems = []
        self.baseItemsToUpdate = []
        self._rootFrame = frame
        self._blockCallbacks = False
        self._ids = itertools.count()
        #baseItems are can be modified in self.callback and in other methods at the same time
        #that why we need this mutex
        self.mutex = Lock()
        self.timer = TimerCallback(targetFps=1, callback=self.callback)
        self.timer.start()

        if not TfFrameSync.listener:
            TfFrameSync.listener = tf.TransformListener()

    @staticmethod
    def resetTime():
        TfFrameSync.listener.clear()

    def callback(self):
        """
        Callback of the timer used to regular check if the items are synchronized
        """
        if not self.baseItemsToUpdate:
            return

        self.mutex.acquire()
        newBaseItemsToUpdate = []
        for i, baseItem in enumerate(self.baseItemsToUpdate):
            try:
                baseItem.baseTransform = self._getTransform(self._rootFrame, baseItem.frame, baseItem.timestamp, i == 0)
                self._onBaseItemModified(baseItem)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException,
                     tf2_ros.TransformException) as e:
                newBaseItemsToUpdate.append(baseItem)

        self.baseItemsToUpdate = newBaseItemsToUpdate
        self.mutex.release()


    def hasItem(self, item):
        if self._findItemId(item) is not None:
            return True
        else:
            return False

    def addItem(self, item, timestamp, ignoreIncoming=True):

        if item is None:
            return

        if self._findItemId(item) is not None:
            return

        item._frameSync = self
        itemId = self._ids.next()
        callbackId = item.connectItemModified(self._onItemModified)

        self.items[itemId] = TfFrameSync.FrameData(
            ref=weakref.ref(item),
            callbackId=callbackId,
            ignoreIncoming=ignoreIncoming)

        self.mutex.acquire()
        baseItem = self._addBaseItem(item, itemId, timestamp)
        self._onBaseItemModified(baseItem)
        self.mutex.release()

    def _addBaseItem(self, item, itemId, timestamp):

        self.baseItems.append(TfFrameSync.FrameData(
            item=self.items[itemId],
            frame=item.frame,
            baseTransform=transformUtils.frameFromPositionAndRPY([0,0,0],[0,0,0]),
            timestamp=timestamp,
            ignoreIncoming=False))

        baseItem = self.baseItems[-1]
        try:
            baseItem.baseTransform = self._getTransform(self._rootFrame, baseItem.frame, baseItem.timestamp)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException,
                tf2_ros.TransformException) as e:
            baseItem.baseTransform = self._catchBaseItemTfException(baseItem)

        return baseItem

    def updateItemTimestamp(self, item, timestamp):

        self.mutex.acquire()
        savedBaseItem = None
        for baseItem in self.baseItems:
            if baseItem.item.ref() is item:
                baseItem.timestamp = timestamp
                try:
                    baseItem.baseTransform = self._getTransform(self._rootFrame, baseItem.frame, baseItem.timestamp)
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException,
                        tf2_ros.TransformException) as e:
                    baseItem.baseTransform = self._catchBaseItemTfException(baseItem)
                savedBaseItem = baseItem
                break

        if not savedBaseItem:
            self.addItem(item, timestamp)
        else:
            self._onBaseItemModified(savedBaseItem)
        self.mutex.release()

    def setRootFrame(self, frame):
        """
        Set the global or root frame of the TfFrameSync object
        """
        self._rootFrame = frame
        self.mutex.acquire()
        for baseItem in self.baseItems:
            try:
                baseItem.baseTransform = self._getTransform(self._rootFrame, baseItem.frame, baseItem.timestamp)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, tf2_ros.TransformException) as e:
                baseItem.baseTransform = self._catchBaseItemTfException(baseItem)

        self._onBaseItemsModified()
        self.mutex.release()

    def _catchBaseItemTfException(self, baseItem):
        """
        The method is called when the transform between self._rootFrame and the local transform of an item
        cannot be computed
        """
        if baseItem not in self.baseItemsToUpdate:
            self.baseItemsToUpdate.append(baseItem)

        return transformUtils.frameFromPositionAndRPY([0,0,0],[0,0,0])

    def _getTransform(self, targetFrame, sourceFrame, timestamp, waitForTransform=True):
        """
        :return: the transform between sourceFrame and targetFrame
        """
        if waitForTransform:
            TfFrameSync.listener.waitForTransform(targetFrame, sourceFrame, timestamp, rospy.Duration(0.1))
        (trans, rot) = TfFrameSync.listener.lookupTransform(targetFrame, sourceFrame, timestamp)
        pose = Pose()
        pose.position.x = trans[0]
        pose.position.y = trans[1]
        pose.position.z = trans[2]
        pose.orientation.x = rot[0]
        pose.orientation.y = rot[1]
        pose.orientation.z = rot[2]
        pose.orientation.w = rot[3]
        return rosutils.rosPoseToTransform(pose)


    def removeItem(self, item):
        itemId = self._findItemId(item)
        if itemId is None:
            raise KeyError(item)

        #removing baseItem
        baseItemToRemove = None
        for baseItem in self.baseItems:
            if baseItem.item.ref() is item:
                baseItemToRemove = baseItem
        self.baseItems.remove(baseItemToRemove)
        if baseItemToRemove.frame in self.baseItemsToUpdate:
            self.baseItemsToUpdate.remove(baseItem)

        item.disconnectItemModified(self.items[itemId].callbackId)
        item._frameSync = None
        self._removeItemId(itemId)


    def _removeItemId(self, itemId):
        del self.items[itemId]

    def _findItemId(self, item):

        for itemId, itemData in self.items.items():

            if itemData.ref() is None:
                self._removeItemId(itemId)
            elif itemData.ref() is item:
                return itemId


    def _onBaseItemModified(self, baseItem):
        baseItem.item.ref().applyTransform(baseItem.baseTransform)

    def _onBaseItemsModified(self):

        for baseItem in self.baseItems:
            self._onBaseItemModified(baseItem)


    def _onItemModified(self, item):
        for baseItem in self.baseItems:
            if baseItem.item.ref() is item:
                self._onBaseItemModified(baseItem)
                break



class TfMovableItem(vis.PolyDataItem):
    """
        An abstract class used to represent an item synchronized by a TfFrameSync object
    """

    def __init__(self, name, transform, frame, view, polyData=vtk.vtkPolyData()):

        vis.PolyDataItem.__init__(self, name, polyData, view)

        self.localTransform = transformUtils.copyFrame(transform)
        self._blockSignals = False
        self._frameSync = None
        #the localTransform of the item is in this frame
        self.frame = frame

        self.callbacks.addSignal('ItemModified')
        self.observerTag = None


    def connectItemModified(self, func):
        return self.callbacks.connect('ItemModified', func)

    def disconnectItemModified(self, callbackId):
        self.callbacks.disconnect(callbackId)

    def addToView(self, view):
        vis.PolyDataItem.addToView(self, view)

    @abc.abstractmethod
    def applyTransform(self, baseTransform):
        """
        Apply a new transform given by TfFrameSync
        :param baseTransform: the transformation between self.localTransform and the global frame
                              of the TfFrameSync
        """
        pass



class TfFrameItem(TfMovableItem):
    """
        An class used to represent a frame item synchronized by a TfFrameSync object
    """
    def __init__(self, name, transform, frame, view):

        TfMovableItem.__init__(self, name, transform, frame, view)

        self.transform = transformUtils.copyFrame(self.localTransform)
        self.actor.SetUserTransform(self.transform)

        self.widget = vtk.vtkFrameWidget()
        self.widget.CreateDefaultRepresentation()
        self.widget.EnabledOff()
        self.rep = self.widget.GetRepresentation()
        self.rep.SetTransform(self.transform)
        self.traceData = None

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
        self.observerTag = self.localTransform.AddObserver('ModifiedEvent', self.onTransformModified)


    def onTransformModified(self, transform, event):
        """
        This callback is called when self._localTransform is modified
        """
        if not self._blockSignals:
            self.callbacks.process('ItemModified', self)

    def _updateAxesGeometry(self):
        scale = self.getProperty('Scale')
        self.rep.SetWorldSize(scale)
        self.setPolyData(vis.createAxesPolyData(scale, self.getProperty('Tube'), self.getProperty('Tube Width')))

    def setLocalTransform(self, transform):
        #obj.localTransform = transform
        #obj.localTransform.SetMatrix(transform.GetMatrix())
        #obj._frameSync._onItemModified(obj)
        self.localTransform.DeepCopy(transform)
        self.localTransform.Modified()

    def applyTransform(self, baseTransform):
        self._blockSignals = True
        new_transform = transformUtils.concatenateTransforms([self.localTransform, baseTransform])
        self.transform.SetMatrix(new_transform.GetMatrix())
        #
        self.actor.SetUserTransform(self.transform)
        self.rep.SetTransform(self.transform)
        #
        self._blockSignals = False
        parent = self.parent()
        if (parent and parent.getProperty('Visible')) or self.getProperty('Visible'):
            self._renderAllViews()

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

    def onRemoveFromObjectModel(self):
        vis.PolyDataItem.onRemoveFromObjectModel(self)

        self.localTransform.RemoveObserver(self.observerTag)

        self.widget.SetInteractor(None)
        self.widget.EnabledOff()
        for view in self.views:
            view.renderer().RemoveActor(self.actor)
            view.render()


class TfPolyDataItem(TfMovableItem):
    """
        An class used to represent a polydata item synchronized by a TfFrameSync object
    """
    def __init__(self, name, polyData, frame, view):

        transform = transformUtils.frameFromPositionAndRPY([0, 0, 0], [0, 0, 0])
        TfMovableItem.__init__(self, name, transform, frame, view, polyData=polyData)

        #self.polyData is represented in the global frame and we need to keep a version of the
        #polydata represented in the local frame
        self._notTransformedPolyData = polyData

        self.observerTag = self._notTransformedPolyData.AddObserver('ModifiedEvent', self.onPolyDataModified)

    def onPolyDataModified(self, transform, event):
        """
        This callback is called when self._notTransformedPolyData is modified
        """
        if not self._blockSignals:
            self.callbacks.process('ItemModified', self)


    def applyTransform(self, baseTransform):
        transformFilter = vtk.vtkTransformPolyDataFilter()
        transformFilter.SetTransform(baseTransform)
        transformFilter.SetInputData(self._notTransformedPolyData)
        transformFilter.Update()

        vis.PolyDataItem.setPolyData(self, transformFilter.GetOutput())

    def setPolyData(self, polyData):
        #self._notTransformedPolyData = polyData
        #self._frameSync._onItemModified(self)
        self._notTransformedPolyData.DeepCopy(polyData)
        self._notTransformedPolyData.Modified()

    def onRemoveFromObjectModel(self):
        vis.PolyDataItem.onRemoveFromObjectModel(self)

        self._notTransformedPolyData.RemoveObserver(self.observerTag)


def showPolyData(polyData, name, frame, color=None, colorByName=None, colorByRange=None, alpha=1.0, visible=True, view=None, parent=None):
    """
    Create a new TfPolyDataItem
    """
    view = view or app.getCurrentRenderView()
    assert view

    item = TfPolyDataItem(name, polyData, frame, view)

    if isinstance(parent, str):
        parentObj = om.getOrCreateContainer(parent)
    else:
        parentObj = parent

    om.addToObjectModel(item, parentObj)
    item.setProperty('Visible', visible)
    item.setProperty('Alpha', alpha)

    if colorByName and colorByName not in item.getArrayNames():
        print 'showPolyData(colorByName=%s): array not found' % colorByName
        colorByName = None

    if colorByName:
        item.setProperty('Color By', colorByName)
        item.colorBy(colorByName, colorByRange)

    else:
        color = [1.0, 1.0, 1.0] if color is None else color
        item.setProperty('Color', [float(c) for c in color])
        item.colorBy(None)

    return item

def updatePolyData(polyData, name, frame, **kwargs):
    obj = om.findObjectByName(name)
    if obj:
        obj.setPolyData(polyData)
    else:
        obj = showPolyData(polyData, name, frame, **kwargs)
    return obj

def showFrame(name, transform, frame, view=None, parent=None, scale=0.35, visible=True):
    """
    Create a new TfFrameItem
    """
    view = view or app.getCurrentRenderView()
    assert view

    if isinstance(parent, str):
        parentObj = om.getOrCreateContainer(parent)
    else:
        parentObj = parent

    item = TfFrameItem(name, transform, frame, view)
    om.addToObjectModel(item, parentObj)
    item.setProperty('Visible', visible)
    item.setProperty('Scale', scale)
    return item


def updateFrame(name, transform, frame, **kwargs):

    obj = om.findObjectByName(name)
    if obj:
        obj.setLocalTransform(transform)
    else:
        obj = showFrame(name, transform, frame, **kwargs)

    return obj