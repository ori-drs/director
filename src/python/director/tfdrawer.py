from director import tfvisualization as tf_vis
from director import visualization as vis
from director.debugVis import DebugData
from director import objectmodel as om

class TfDrawer(object):

    def __init__(self, frame):
        self.frameSync = tf_vis.TfFrameSync(frame)

    def drawArrow(self, transform, name, timestamp, frame, color=[1, 0, 0], parent=None):
        obj = om.findObjectByName(name)
        if obj and not self.frameSync.hasItem(obj):
            self.frameSync.addItem(obj, timestamp, frame)

        d = DebugData()
        d.addArrowWithFrame(transform, scale=0.5, color=color, headRadius=0.1, tubeRadius=0.04)
        obj = tf_vis.updatePolyData(d.getPolyData(), name, frame, colorByName='RGB255', parent=parent)

        if not self.frameSync.hasItem(obj):
            self.frameSync.addItem(obj, timestamp, frame)
        else:
            self.frameSync.updateItemTimestamp(obj, timestamp)

        return obj

    def drawArrowWithoutTimestamp(self, frame, name, color=[1,0,0], parent=None):
        d = DebugData()
        d.addArrowWithFrame(frame, scale=0.5, color=color, headRadius=0.1, tubeRadius=0.04)
        obj = vis.updatePolyData(d.getPolyData(), name, colorByName='RGB255', parent=parent)
        return obj

    def drawFrame(self, transform, name, timestamp, frame, **kwargs):

        obj = tf_vis.updateFrame(name, transform, frame, **kwargs)
        if not self.frameSync.hasItem(obj):
            self.frameSync.addItem(obj, timestamp, frame)
        else:
            self.frameSync.updateItemTimestamp(obj, timestamp)

    def setRootFrame(self, frame):
        self.frameSync.setRootFrame(frame)