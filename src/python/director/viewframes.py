from director import objectmodel as om
import director.visualization as vis
import director.tfvisualization as tf_vis

class ViewFramesSizeHandler(object):
    """
        An class used to change the size of Frames objects
    """
    def __init__(self, action):
        self.action = action
        self.action.checkable = True
        self.action.connect('triggered()', self._onChecked)


    def _onChecked(self):
        for obj in om.getObjects():
            if type(obj) is tf_vis.TfFrameItem:
                if self.action.checked:
                    obj.setBigSizeProperties()
                else:
                    obj.setDefaultSizeProperties()