from director import objectmodel as om
import director.visualization as vis
import director.vtkAll as vtk


class ViewFramesSizeHandler(object):
    """
        An class used to change the size of Frames objects
    """
    def __init__(self, action):
        self.action = action
        self.action.checkable = True
        self.action.connect('triggered()', self._onChecked)

    def _onChecked(self):

        if self.action.checked:
            vtk.vtkFrameWidgetRepresentation.SetBigSizeHandles()
        else:
            vtk.vtkFrameWidgetRepresentation.SetDefaultSizeHandles()

        for obj in om.getObjects():
            if obj.getProperty('Name') == 'walking goal' and type(obj) is vis.FrameItem:
                # increase size of the tool to move walking goal
                rep = obj.widget.GetRepresentation()
                rep.RedrawHandles()
                obj._renderAllViews()
