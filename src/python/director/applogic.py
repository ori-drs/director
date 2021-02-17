import os
import time
import math
import director.vtkAll as vtk
import director
import PythonQt
from PythonQt import QtCore
from PythonQt import QtGui
from director import getDRCBaseDir as getDRCBase
import functools

_mainWindow = None
_defaultRenderView = None


def getMainWindow():

    global _mainWindow
    if _mainWindow is None:
        for widget in QtGui.QApplication.topLevelWidgets():
            if isinstance(widget, PythonQt.QtGui.QMainWindow):
                _mainWindow = widget
                break

    return _mainWindow


def getRobotSelector():
    """
    Return the robotselector object for this director instance. Specifying a robot to display will show UI components
    corresponding only to that robot.
    :return: RobotSelector object
    """
    return getMainWindow().toolBar().findChild("QWidget", "RobotSelector")


def quit():
    QtGui.QApplication.instance().quit()


def getViewManager():
    return getMainWindow().viewManager()


def getDRCView():
    return _defaultRenderView or getMainWindow().viewManager().findView("DRC View")


def getSpreadsheetView():
    return getMainWindow().viewManager().findView("Spreadsheet View")


def getCurrentView():
    return _defaultRenderView or getMainWindow().viewManager().currentView()


def getCurrentRenderView():
    view = getCurrentView()
    if hasattr(view, "camera"):
        return view


def getOutputConsole():
    return getMainWindow().outputConsole()


def getPythonConsole():
    return PythonQt.dd._pythonManager.consoleWidget()


def showPythonConsole():
    getPythonConsole().show()


_exclusiveDockWidgets = {}


def hideDockWidgets(action=None):
    for a, wList in _exclusiveDockWidgets.items():
        if a is not action:
            for dock, widget in wList:
                if not dock.isFloating():
                    dock.hide()


def addDockAction(actionName, actionText, iconPath, append=False):
    """
    Get a dock action in the right-hand action toolbar. If it does not exist, it will be created

    :param actionName: The name of the action, the objectName field of the QAction
    :param actionText: The text shown on mouseover of the action and also when viewing the right-click menu
    :param iconPath: The path to the icon to display for this action
    :param append: If true, put this action at the bottom of the toolbar rather than at the top
    :return: The QAction added to the dock, or the existing action if it was already there
    """

    action = getToolBarActions().get(actionName)

    if action is None:
        assert os.path.isfile(iconPath)

        action = QtGui.QAction(QtGui.QIcon(iconPath), actionText, None)
        action.objectName = actionName
        action.checkable = True

        toolbar = getMainWindow().panelToolBar()

        if append:
            toolbar.addAction(action)
        else:
            toolbar.insertAction(toolbar.actions()[0], action)

    return action


def addWidgetToDock(
    widget, dockArea=QtCore.Qt.RightDockWidgetArea, action=None, associatedRobotName=""
):
    """
    This function adds the given widget to the specified dock area. The widgets can then be accesses by clicking the
    icon. It also adds the widgets to the view menu

    :param widget: The widget to add
    :param dockArea: The dock area to which the widget should be added
    :param action: The action associated with the widget, which is required for it to work TODO: what is the actual relationship
    :param associatedRobotName: The name of the robot associated with this action. This is used to set the name of
    the submenu of the view menu to which this widget will also be added
    :return:
    """
    dock = QtGui.QDockWidget()
    dock.setWidget(widget)
    dock.setWindowTitle(widget.windowTitle)
    getMainWindow().addDockWidget(dockArea, dock)

    if dockArea == QtCore.Qt.RightDockWidgetArea and action:
        if action in _exclusiveDockWidgets:
            _exclusiveDockWidgets[action].append((dock, widget))
        else:
            _exclusiveDockWidgets[action] = [(dock, widget)]
        action.connect("triggered()", functools.partial(hideDockWidgets, action))

    if action is None:
        getMainWindow().addWidgetToViewMenu(dock, associatedRobotName)
    else:
        getMainWindow().addWidgetToViewMenu(dock, action, associatedRobotName)

    return dock


def resetCamera(viewDirection=None, view=None):

    view = view or getCurrentRenderView()
    assert view

    if viewDirection is not None:
        camera = view.camera()
        camera.SetPosition([0, 0, 0])
        camera.SetFocalPoint(viewDirection)
        camera.SetViewUp([0, 0, 1])

    view.resetCamera()
    view.render()


def setBackgroundColor(color, color2=None, view=None):
    view = view or getCurrentRenderView()
    assert view

    if color2 is None:
        color2 = color
    ren = view.backgroundRenderer()
    ren.SetBackground(color)
    ren.SetBackground2(color2)


def displaySnoptInfo(info):
    if getMainWindow() is not None:
        getMainWindow().statusBar().showMessage("Info: %d" % info)


def toggleStereoRender():
    view = getCurrentRenderView()
    assert view

    renderWindow = view.renderWindow()
    renderWindow.SetStereoRender(not renderWindow.GetStereoRender())
    view.render()


def getCameraTerrainModeEnabled(view):
    return isinstance(
        view.renderWindow().GetInteractor().GetInteractorStyle(),
        vtk.vtkInteractorStyleTerrain2,
    )


def setCameraTerrainModeEnabled(view, enabled):

    if getCameraTerrainModeEnabled(view) == enabled:
        return

    if enabled:
        view.renderWindow().GetInteractor().SetInteractorStyle(
            vtk.vtkInteractorStyleTerrain2()
        )
        view.camera().SetViewUp(0, 0, 1)
    else:
        view.renderWindow().GetInteractor().SetInteractorStyle(
            vtk.vtkInteractorStyleTrackballCamera()
        )

    view.render()


def toggleCameraTerrainMode(view=None):

    view = view or getCurrentRenderView()
    assert view

    setCameraTerrainModeEnabled(view, not getCameraTerrainModeEnabled(view))
    updateToggleTerrainAction(view)


def findMenu(menuTitle, mainWindow=None):
    mainWindow = mainWindow or getMainWindow()
    menus = mainWindow.findChildren("QMenu")
    for menu in menus:
        title = str(menu.title)
        if title.startswith("&"):
            title = title[1:]
        if title == menuTitle:
            return menu


def findToolBar(title, mainWindow=None):
    mainWindow = mainWindow or getMainWindow()
    bars = mainWindow.findChildren("QToolBar")
    for bar in bars:
        if title == str(bar.windowTitle):
            return bar


def addMenuAction(menuTitle, actionName):
    menu = findMenu(menuTitle)
    assert menu
    return menu.addAction(actionName)


def getToolBarActions():
    return getActionsDict(getMainWindow().toolBarActions())


def getToolsMenuActions():
    return getActionsDict(getMainWindow().toolsMenu().actions())


def getActionsDict(actions):
    actionsDict = {}
    for action in actions:
        if action.name:
            actionsDict[action.name] = action
    return actionsDict


def updateToggleTerrainAction(view):
    if not getMainWindow():
        return
    isTerrainMode = False
    if hasattr(view, "renderWindow"):
        isTerrainMode = isinstance(
            view.renderWindow().GetInteractor().GetInteractorStyle(),
            vtk.vtkInteractorStyleTerrain2,
        )
    getToolBarActions()["ActionToggleCameraTerrainMode"].checked = isTerrainMode


class ActionToggleHelper(object):
    """
    This class manages a checkable action and forwards checked events
    to user selected callbacks.
    """

    def __init__(self, action, getEnabledFunc, setEnabledFunc):
        self.getEnabled = getEnabledFunc
        self.setEnabled = setEnabledFunc

        self.action = action
        self.action.setCheckable(True)
        self.action.checked = getEnabledFunc()
        self.action.connect("triggered()", self.onActionChanged)

    def updateAction(self):
        self.action.checked = self.getEnabled()

    def onActionChanged(self):
        if self.action.checked:
            self.setEnabled(True)
        else:
            self.setEnabled(False)
        self.updateAction()


class MenuActionToggleHelper(ActionToggleHelper):
    def __init__(self, menuName, actionName, getEnabledFunc, setEnabledFunc):
        self.action = addMenuAction(menuName, actionName)
        ActionToggleHelper.__init__(self, self.action, getEnabledFunc, setEnabledFunc)


def onCurrentViewChanged(previousView, currentView):
    updateToggleTerrainAction(currentView)


def addToolbarMacro(name, func, robotName=""):
    toolbar = getMainWindow().macrosToolBar()
    action = toolbar.addAction(name)
    action.connect("triggered()", func)
    getRobotSelector().associateWidgetWithRobot(action, robotName)


def removeToolbarMacro(name):
    action = getToolBarActions().get(name)
    if action:
        getMainWindow().panelToolBar().removeAction(action)


def addShortcut(widget, keySequence, func):
    shortcut = QtGui.QShortcut(QtGui.QKeySequence(keySequence), widget)
    shortcut.connect("activated()", func)
    shortcut.connect("activatedAmbiguously()", func)
    return shortcut


def showErrorMessage(message, title="Error"):
    QtGui.QMessageBox.warning(getMainWindow(), title, message)


def showInfoMessage(message, title="Info"):
    QtGui.QMessageBox.information(getMainWindow(), title, message)


def showViewTabContextMenu(view, tabBar, menuPosition):
    def onPopOut():
        getViewManager().popOut(view)

    menu = QtGui.QMenu(tabBar)
    menu.addAction("Pop out").connect("triggered()", onPopOut)
    menu.popup(menuPosition)


def onTabWidgetContextMenu(mouseClick):

    tabBar = getViewManager().findChildren("QTabBar")[0]
    tabIndex = tabBar.tabAt(mouseClick)
    viewName = tabBar.tabText(tabIndex)
    view = getViewManager().findView(viewName)
    if view:
        showViewTabContextMenu(view, tabBar, tabBar.mapToGlobal(mouseClick))


def setupViewManager():

    vm = getViewManager()
    vm.connect("currentViewChanged(ddViewBase*, ddViewBase*)", onCurrentViewChanged)

    tabBar = vm.findChildren("QTabBar")[0]
    tabBar.setContextMenuPolicy(QtCore.Qt.CustomContextMenu)
    tabBar.connect("customContextMenuRequested(const QPoint &)", onTabWidgetContextMenu)


def startup(globals):

    global _mainWindow
    _mainWindow = globals["_mainWindow"]

    if not director.getDRCBaseIsSet():
        showErrorMessage("director_drs package cannot be found")
        return

    _mainWindow.connect("resetCamera()", resetCamera)
    _mainWindow.connect("toggleStereoRender()", toggleStereoRender)
    _mainWindow.connect("toggleCameraTerrainMode()", toggleCameraTerrainMode)

    setupViewManager()
