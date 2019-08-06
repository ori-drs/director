import director.objectmodel as om
import director.rosutils as rosutils
from director import tfdrawer as tf_draw

from nav_msgs.msg import Path

class AicpPoseSource(om.ContainerItem):

    def __init__(self, name, topicName, fixedFrame):
        om.ContainerItem.__init__(self, name)

        self.topicName = topicName
        self.subscriber = rosutils.addSubscriber(self.topicName, Path, self._posesCallback)
        self.tfDrawer = tf_draw.TfDrawer(fixedFrame)
        self.objects = [] # objects drawn by this class

        self.addProperty('Topic name', topicName)
        self.addProperty('Subscribe', True)
        self.addProperty('Visible', True)


    def _posesCallback(self, msg):

        for i, pose in enumerate(msg.poses):
            transform = rosutils.rosPoseToTransform(pose.pose)
            self.tfDrawer.drawFrame(transform, "pose " + str(i), msg.header.stamp, msg.header.frame_id,
                                    parent=self)

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

