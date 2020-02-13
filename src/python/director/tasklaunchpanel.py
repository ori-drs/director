from PythonQt import QtCore, QtGui
from director import applogic as app
import functools

class TaskLaunchPanel(object):

    def __init__(self, widgetMap):

        self.widget = QtGui.QTabWidget()
        self.widget.setWindowTitle('Task Panel')

        for name, widget in widgetMap.iteritems():
            self.addTaskPanel(name, widget)

    def getTaskPanelNames(self):
        return [self.widget.tabText(i) for i in xrange(self.widget.count)]

    def removeTaskPanel(self, taskPanelName):
        names = self.getTaskPanelNames()
        assert taskPanelName in names
        self.widget.removeTab(names.index(taskPanelName))

    def clear(self):
        self.widget.clear()

    def addTaskPanel(self, taskPanelName, taskPanelWidget):
        self.widget.addTab(taskPanelWidget, taskPanelName)

    def showTaskLaunchPanel(self):

        if not self.widget.visible:
            self.widget.show()
            self.widget.raise_()
            self.widget.activateWindow()
        else:
            self.widget.hide()


def init(widgetMap, robotName=""):
    global panels

    if 'panels' not in globals():
        panels = {}

    import os
    panel = TaskLaunchPanel(widgetMap)
    action = app.addDockAction('ActionTaskLauncher' + robotName, 'Task Launcher',
                               os.path.join(os.path.dirname(__file__), 'images/task_icon.png'), append=True)
    action.connect('triggered()', panel.showTaskLaunchPanel)
    app.getRobotSelector().associateWidgetWithRobot(action, robotName)

    panels[robotName] = panel

    return panel
