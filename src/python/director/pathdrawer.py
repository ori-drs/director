import director.objectmodel as om
import director.rosutils as rosutils
from director import tfdrawer as tf_draw
import director.applogic as app

from nav_msgs.msg import Path


class AicpPoseSource(om.ContainerItem):

    def __init__(self, name, topicName, tfDrawer):
        om.ContainerItem.__init__(self, name)

        om.addToObjectModel(self)
        self.topicName = topicName
        self.subscriber = rosutils.addSubscriber(self.topicName, Path, self._posesCallback)
        self.tfDrawer = tfDrawer
        self.lines = []
        self.frames = []
        self.arrows = []
        self.lineContainer = tf_draw.PolyDataContainer('lines')
        om.addToObjectModel(self.lineContainer, parentObj=self)
        self.areContainerInitialized = False
        self.prevTfFrame = None
        self.prevIndexReceived = -1

        self.addProperty('Topic name', topicName)
        self.addProperty('Subscribe', True)
        self.addProperty('Style', 0, attributes=om.PropertyAttributes(enumNames=['Frames', 'Arrows']))


    def _posesCallback(self, msg):
        view = app.getCurrentRenderView()
        #only render last poses received

        print len(msg.poses)-self.prevIndexReceived

        for i in range(self.prevIndexReceived+1, len(msg.poses)):
            pose = msg.poses[i]

            transform = rosutils.rosPoseToTransform(pose.pose)
            tfFrame = self.tfDrawer.drawFrame(transform, "pose " + str(i), msg.header.stamp, msg.header.frame_id,
                                    parent=self)
            self.frames.append(tfFrame)
            tfArrow = self.tfDrawer.drawArrow(transform, "arrow " + str(i), msg.header.stamp, msg.header.frame_id,
                                              parent=self)
            self.arrows.append(tfArrow)

            if self.prevTfFrame:
                line = tf_draw.LineItem('line ' + str(i), self.prevTfFrame, tfFrame)
                line.addToView(view)
                om.addToObjectModel(line, self.lineContainer)
                self.lines.append(line)

            self.prevTfFrame = tfFrame

        self.prevIndexReceived = len(msg.poses)-1

        if not self.areContainerInitialized:
            if self.lines:
                self._copyPropertiesToContainer(self.lineContainer, self.lines[0])
                self.areContainerInitialized = True
        else:
            self._copyContainerPropertiesToObjects(self.lineContainer, self.lines)

        self._displayTfObjects()


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
                self.subscriber = rosutils.addSubscriber(self.topicName, Path, self._posesCallback)
        elif propertyName == 'Subscribe':
            if self.getProperty(propertyName):
                self.subscriber = rosutils.addSubscriber(self.topicName, Path, self._posesCallback)
            else:
                self.subscriber.unsubscribe()
        elif propertyName == 'Style':
            self._displayTfObjects()
        elif propertyName == 'Visible':
            self._displayTfObjects()


