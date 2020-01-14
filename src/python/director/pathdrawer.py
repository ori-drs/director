import abc

import director.objectmodel as om
import director.rosutils as rosutils
from director import tfdrawer as tf_draw
import director.applogic as app

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseArray


class PosesSource(om.ContainerItem):

    def __init__(self, name, topicName, tfDrawer, messageClass):
        om.ContainerItem.__init__(self, name)


        self.topicName = topicName
        self.messageClass = messageClass
        self.subscriber = None
        self.lineContainer = tf_draw.PolyDataContainer('lines')
        self.register()
        self.tfDrawer = tfDrawer
        self.lines = []
        self.frames = []
        self.arrows = []

        self.areContainerInitialized = False
        self.prevTfFrame = None
        self.prevIndexReceived = -1

        self.addProperty('Topic name', topicName)
        self.addProperty('Subscribe', True)
        self.addProperty('Style', 0, attributes=om.PropertyAttributes(enumNames=['Frames', 'Arrows']))
        self.addProperty('Draw entirety of received messages', False)
        om.collapse(self)

    def unregister(self):
        self.subscriber.unsubscribe()
        om.removeFromObjectModel(self)
        om.removeFromObjectModel(self.lineContainer)
        #reset the index, it means all the message received will be drawn if the register method is called
        self.prevIndexReceived = -1
        self.areContainerInitialized = False
        self.prevTfFrame = None

    def register(self):
        om.addToObjectModel(self)
        om.addToObjectModel(self.lineContainer, parentObj=self)
        self.subscriber = rosutils.addSubscriber(self.topicName, self.messageClass, self._posesCallback)

    def _posesCallback(self, msg):
        view = app.getCurrentRenderView()
        name = self.getProperty('Name')

        drawEverything = self.getProperty('Draw entirety of received messages')

        # self.prevIndexReceived+1 > len(msg.poses) means that the number of poses received is lower that the previous
        # number of received poses, it shouldn't happen, so everything is redrawn
        if drawEverything or self.prevIndexReceived+1 > len(msg.poses):
            # clear lines, frames and arrows
            self.prevIndexReceived = -1
            self.areContainerInitialized = False
            self.prevTfFrame = None
            
            for obj in self.lines:
                obj.disconnect()
                om.removeFromObjectModel(obj)

            del self.lines[:]
            for obj in self.arrows + self.frames:
                om.removeFromObjectModel(obj)
            del self.frames[:]
            del self.arrows[:]

        # only render last poses received, much faster than redrawing everything but sometimes it doesn't make sense
        # so toggle property "draw entirety of received messages to change this behaviour
        newLines = []
        for i in range(self.prevIndexReceived+1, len(msg.poses)):
            pose = self._getPose(msg, i)

            transform = rosutils.rosPoseToTransform(pose)

            tfFrame = self.tfDrawer.drawFrame(transform, name + " - pose " + str(i), msg.header.stamp, msg.header.frame_id,
                                    parent=self)
            self.frames.append(tfFrame)
            tfArrow = self.tfDrawer.drawArrow(transform, name + " - arrow " + str(i), msg.header.stamp, msg.header.frame_id,
                                              parent=self)
            self.arrows.append(tfArrow)

            if self.prevTfFrame:
                line = tf_draw.LineItem(name + ' - line ' + str(i), self.prevTfFrame, tfFrame)
                line.addToView(view)
                om.addToObjectModel(line, self.lineContainer)
                self.lines.append(line)
                newLines.append(line)

            self.prevTfFrame = tfFrame

        self.prevIndexReceived = len(msg.poses)-1

        if not self.areContainerInitialized:
            if self.lines:
                self._copyPropertiesToContainer(self.lineContainer, self.lines[0])
                self.areContainerInitialized = True
        else:
            self._copyContainerPropertiesToObjects(self.lineContainer, newLines)


    @abc.abstractmethod
    def _getPose(self, msg, index):
        pass

    """
       Display lines and arrows according to the properties of the container
    """
    def _displayTfObjects(self):
        propertyValue = self.getPropertyEnumValue('Style')
        visible = self.getProperty('Visible')
        if propertyValue == 'Frames' and visible:
            for frame in self.frames:
                frame.setProperty('Visible', True)
            for arrow in self.arrows:
                arrow.setProperty('Visible', False)
        elif propertyValue == 'Arrows' and visible:
            for frame in self.frames:
                frame.setProperty('Visible', False)
            for arrow in self.arrows:
                arrow.setProperty('Visible', True)
        else:
            for obj in self.frames + self.arrows:
                obj.setProperty('Visible', False)

    def _copyContainerPropertiesToObjects(self, container, objects):

        for obj in objects:
            for propertyName in container.propertyNames():
                if propertyName in ['Name', 'Icon']:
                    continue

                propertyValue = container.getProperty(propertyName)
                if obj.hasProperty(propertyName):
                    obj.setProperty(propertyName, propertyValue)

    def _copyPropertiesToContainer(self, container, obj):
        for propertyName in obj.propertyNames():
            if propertyName in ['Name', 'Icon', 'Visible']:
                continue

            if propertyName == 'Color By':
                enumNames = ['Solid Color'] + obj.getArrayNames()
                currentValue = obj.getProperty('Color By')
                if currentValue >= len(enumNames):
                    container.setProperty('Color By', 0)
                container.setPropertyAttribute('Color By', 'enumNames', enumNames)

            propertyValue = obj.getProperty(propertyName)
            if container.hasProperty(propertyName):
                attribute = obj.getPropertyAttribute(propertyName, 'hidden')
                container.setProperty(propertyName, propertyValue)
                container.setPropertyAttribute(propertyName, 'hidden', attribute)
            else:
                #shouldn't go there
                container.addProperty(propertyName, propertyValue)


    def _onPropertyChanged(self, propertySet, propertyName):
        om.ContainerItem._onPropertyChanged(self, propertySet, propertyName)

        if propertyName == 'Topic name':
            self.topicName = self.getProperty(propertyName)
            if self.getProperty('Subscribe'):
                self.subscriber = rosutils.addSubscriber(self.topicName, self.messageClass, self._posesCallback)
        elif propertyName == 'Subscribe':
            if self.getProperty(propertyName):
                self.subscriber = rosutils.addSubscriber(self.topicName, self.messageClass, self._posesCallback)
            else:
                self.subscriber.unsubscribe()
        elif propertyName == 'Style':
            self._displayTfObjects()
        elif propertyName == 'Visible':
            self._displayTfObjects()


class PathSource(PosesSource):
    """
        A class used to draw a nav_msgs/Path
    """
    def __init__(self, name, topicName, tfDrawer):
        super(PathSource, self).__init__(name, topicName, tfDrawer, Path)

    def _getPose(self, msg, index):
        return msg.poses[index].pose

    def __del__(self):
        print("PathSource del")


class ArraySource(PosesSource):
    """
        A class used to draw a PoseArray
    """
    def __init__(self, name, topicName, tfDrawer):
        super(ArraySource, self).__init__(name, topicName, tfDrawer, PoseArray)

    def _getPose(self, msg, index):
        return msg.poses[index]