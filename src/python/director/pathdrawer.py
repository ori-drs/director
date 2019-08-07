import director.objectmodel as om
import director.rosutils as rosutils
from director import tfdrawer as tf_draw
import director.applogic as app

from nav_msgs.msg import Path

class AicpPoseSource(om.ContainerItem):

    def __init__(self, name, topicName, fixedFrame):
        om.ContainerItem.__init__(self, name)

        om.addToObjectModel(self)
        self.topicName = topicName
        self.subscriber = rosutils.addSubscriber(self.topicName, Path, self._posesCallback)
        self.tfDrawer = tf_draw.TfDrawer(fixedFrame)
        self.lines = []
        self.frames = []
        self.lineContainer = om.getOrCreateContainer('lines', parentObj=self)

        self.addProperty('Topic name', topicName)
        self.addProperty('Subscribe', True)
        self.addProperty('Visible', True)
        self.addProperty('Draw Lines', True)
        self.addProperty('Draw Frames', True)


    def _posesCallback(self, msg):

        view = app.getCurrentRenderView()

        # clear lines and frames
        for line in self.lines:
            line.disconnect()
        del self.lines[:]
        del self.frames[:]


        prevTfFrame = None
        for i, pose in enumerate(msg.poses):
            transform = rosutils.rosPoseToTransform(pose.pose)
            tfFrame = self.tfDrawer.drawFrame(transform, "pose " + str(i), msg.header.stamp, msg.header.frame_id,
                                    parent=self)
            self.frames.append(tfFrame)

            if prevTfFrame:
                line = tf_draw.LineItem('line ' + str(i), prevTfFrame, tfFrame)
                line.addToView(view)
                om.addToObjectModel(line, self.lineContainer)
                self.lines.append(line)
            prevTfFrame = tfFrame


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
                self.subscriber = None
        elif propertyName == 'Draw Lines':
            for line in self.lines:
                line.setProperty('Visible', self.getProperty(propertyName))
        elif propertyName == 'Draw Frames':
            for frame in self.frames:
                frame.setProperty('Visible', self.getProperty(propertyName))
        elif propertyName == 'Visible':
            self.setProperty('Draw Lines', self.getProperty(propertyName))
            self.setProperty('Draw Frames', self.getProperty(propertyName))


