import PythonQt
from PythonQt import QtCore, QtGui, QtUiTools
from director import applogic as app
from director import visualization as vis
from director.debugpolydata import DebugData
from director.timercallback import TimerCallback
from director.simpletimer import FPSCounter
from director import cameraview
from director import objectmodel as om
import director.vtkAll as vtk


def addWidgetsToDict(widgets, d):

    for widget in widgets:
        if widget.objectName:
            d[str(widget.objectName)] = widget
        addWidgetsToDict(widget.children(), d)


class WidgetDict(object):
    def __init__(self, widgets):
        addWidgetsToDict(widgets, self.__dict__)


class FrameUpdater(object):
    def __init__(self, folderName, listWidget):

        self.folderName = folderName
        self.listWidget = listWidget
        self.itemMap = {}
        self.initialised = False  # TODO: not an ideal method...
        self.initListWidget()

        self.trace = {}

    def getFrameTransform(self, frameName):
        return vtk.vtkTransform()

    def getFramesNames(self):
        return []

    def initListWidget(self):

        frameNames = self.getFramesNames()
        # If the model is not yet initialised don't do anything. This function will be called once it's initialised
        if not frameNames:
            return

        for name in frameNames:
            item = QtGui.QListWidgetItem(name)
            item.setData(QtCore.Qt.CheckStateRole, QtCore.Qt.Unchecked)
            self.listWidget.addItem(item)
            self.itemMap[name] = item

        self.listWidget.connect("itemChanged(QListWidgetItem*)", self.onItemChecked)
        self.initialised = True

    def onItemChecked(self, item):
        name = str(item.text())
        isChecked = item.checkState() == QtCore.Qt.Checked

        for frameObj in self.getFrameObjects():
            if frameObj.getProperty("Name") == name:
                frameObj.setProperty("Visible", isChecked)
                break
        else:
            if isChecked:
                self.addFrame(name)

    def getEnabledFrameNames(self):
        enabledFrames = set()
        for name, item in self.itemMap.iteritems():
            isChecked = item.checkState() == QtCore.Qt.Checked
            if isChecked:
                enabledFrames.add(name)
        return set(enabledFrames)

    def updateFrame(self, frameName, frameObj):
        t = self.getFrameTransform(frameName)
        frameObj.copyFrame(t)

    def addFrame(self, frameName):
        t = self.getFrameTransform(frameName)
        folder = self.getFramesFolder()
        frame = vis.showFrame(t, frameName, parent=folder, scale=0.2)
        frame.setProperty("Trace", True)

    def getFramesFolder(self):
        return om.getOrCreateContainer(self.folderName)

    def getFrameObjects(self):
        if not om.findObjectByName(self.folderName):
            return []
        return self.getFramesFolder().children()

    def hideAllFrames(self):
        for frame in self.getFrameObjects():
            frame.setProperty("Visible", False)

    def updateFrames(self):

        frames = self.getFrameObjects()
        enabledFrames = self.getEnabledFrameNames()

        for frame in frames:

            frameName = frame.getProperty("Name")
            isChecked = (
                QtCore.Qt.Checked
                if frame.getProperty("Visible")
                else QtCore.Qt.Unchecked
            )
            self.itemMap[frameName].setCheckState(isChecked)

            try:
                enabledFrames.remove(frameName)
            except:
                pass

            if not frame.getProperty("Visible"):
                continue

            self.updateFrame(frameName, frame)

        # add new frames if needed
        for frameName in enabledFrames:
            self.addFrame(frameName)


# Depreciated after bot frames was disabled
# TODO: add support for tf frame visualization
# class BotFrameUpdater(FrameUpdater):
#
#    def __init__(self, listWidget):
#        FrameUpdater.__init__(self, 'Bot Frames', listWidget)
#
#    def getFrameTransform(self, frameName):
#        t = vtk.vtkTransform()
#        t.PostMultiply()
#        cameraview.imageManager.queue.getTransform(frameName, 'local', t)
#        return t


class LinkFrameUpdater(FrameUpdater):
    def __init__(self, robotModel, listWidget):
        self.robotModel = robotModel
        FrameUpdater.__init__(self, "Link Frames", listWidget)
        robotModel.connectModelChanged(self.onModelChanged)

    def getFrameTransform(self, frameName):
        return self.robotModel.getLinkFrame(frameName)

    def getFramesNames(self):
        # Do this to allow for deferred initialisation of robot model
        if not self.robotModel.model:
            return []
        return sorted(list(self.robotModel.model.getLinkNames()))

    def onModelChanged(self, model):
        if not self.initialised:
            self.initListWidget()
        self.updateFrames()


class FrameVisualizationPanel(object):
    def __init__(self, view, robotSystem):

        self.view = view

        loader = QtUiTools.QUiLoader()
        uifile = QtCore.QFile(":/ui/ddFrameVisualization.ui")
        assert uifile.open(uifile.ReadOnly)

        self.widget = loader.load(uifile)
        self.ui = WidgetDict(self.widget.children())

        # self.botFrameUpdater = BotFrameUpdater(self.ui.botFramesListWidget)

        self.linkFrameUpdater = LinkFrameUpdater(
            robotSystem.robotStateModel, self.ui.linkFramesListWidget
        )

        self.eventFilter = PythonQt.dd.ddPythonEventFilter()
        self.ui.scrollArea.installEventFilter(self.eventFilter)
        self.ui.allFramesCheckBox.connect("stateChanged(int)", self.toggleAllFrames)
        self.eventFilter.addFilteredEventType(QtCore.QEvent.Resize)
        self.eventFilter.connect("handleEvent(QObject*, QEvent*)", self.onEvent)

        # PythonQt.dd.ddGroupBoxHider(self.ui.botFramesGroup)
        # PythonQt.dd.ddGroupBoxHider(self.ui.linkFramesGroup)

        # self.updateTimer = TimerCallback(targetFps=60)
        # self.updateTimer.callback = self.updateFrames
        # self.updateTimer.start()

    def onEvent(self, obj, event):
        minSize = (
            self.ui.scrollArea.widget().minimumSizeHint.width()
            + self.ui.scrollArea.verticalScrollBar().width
        )
        self.ui.scrollArea.setMinimumWidth(minSize)

    def toggleAllFrames(self, ignored):
        newCheckState = (
            QtCore.Qt.Checked
            if self.ui.allFramesCheckBox.isChecked()
            else QtCore.Qt.Unchecked
        )
        for _, item in self.linkFrameUpdater.itemMap.items():
            item.setData(QtCore.Qt.CheckStateRole, newCheckState)

    # def updateFrames(self):
    #    self.botFrameUpdater.updateFrames()

    # def getNameFilter(self):
    #    return str(self.ui.botFramesFilterEdit.text)

    # def onNameFilterChanged(self):
    #    filter = self.getNameFilter()


def init(view, robotSystem):

    global panels

    if "panels" not in globals():
        panels = {}

    panel = FrameVisualizationPanel(view, robotSystem)
    dock = app.addWidgetToDock(
        panel.widget, action=None, associatedRobotName=robotSystem.robotName
    )
    dock.hide()

    panels[robotSystem.robotName] = (panel, dock)

    return panel
