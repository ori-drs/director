from director import tfvisualization as tf_vis
from director import visualization as vis
from director.debugVis import DebugData
from director import objectmodel as om

import vtk


class LineItem(vis.PolyDataItem):
    def __init__(self, name, tfFrame1, tfFrame2):
        vis.PolyDataItem.__init__(self, name, vtk.vtkPolyData(), view=None)
        self.tfFrame1 = tfFrame1
        self.tfFrame2 = tfFrame2
        self.callbackId1 = tfFrame1.connectTransformModified(self.redraw())
        self.callbackId2 = tfFrame2.connectTransformModified(self.redraw())
        self.draw()

    def draw(self):
        d = DebugData()
        p1 = self.tfFrame1.transform.GetPosition()
        p2 = self.tfFrame2.transform.GetPosition()
        d.addLine(p1, p2, color=[0, 1, 0])
        polyData = d.getPolyData()
        self.setPolyData(polyData)
        self.setProperty('Color By', 'RGB255')

    def disconnect(self):
        self.tfFrame1.disconnectCallback(self.callbackId1)
        self.tfFrame2.disconnectCallback(self.callbackId2)

    def redraw(self):
        #callback called when a TfFrame1 or TfFrame2 is modified
        self.draw()



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


    def drawFrame(self, transform, name, timestamp, frame, color=None, **kwargs):

        obj = tf_vis.updateFrame(name, transform, frame, **kwargs)
        if timestamp and frame:
            if not self.frameSync.hasItem(obj):
                self.frameSync.addItem(obj, timestamp, frame)
            else:
                self.frameSync.updateItemTimestamp(obj, timestamp)

        if color:
            obj.setProperty('Color By', 'Solid Color')
            obj.setProperty('Color', color)
        return obj

    def setRootFrame(self, frame):
        self.frameSync.setRootFrame(frame)