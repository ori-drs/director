from director import applogic
from director import cameracontrol

from PythonQt import QtCore, QtGui


class CameraBookmarks(object):

    def __init__(self, view):
        self.bookmarks = {}
        self.view = view
        self.flyer = cameracontrol.Flyer(view)
        self.flyer.flyTime = 1.0

    def storeCameraBookmark(self, key):
        camera = self.view.camera()
        focal, position = camera.GetFocalPoint(), camera.GetPosition()
        self.storeFocalAndPosition(key, focal, position)

    def storeFocalAndPosition(self, key, focal, position):
        self.bookmarks[key] = (focal, position)

    def clear(self):
        self.bookmarks = {}

    def getBookmark(self, key):
        return self.bookmarks.get(key)

    def flyToBookmark(self, key):
        focal, position = self.getBookmark(key)
        self.flyer.zoomTo(focal, position)


class CameraBookmarkWidget(object):

    def __init__(self, view):
        self.bookmarks = CameraBookmarks(view)
        self.widget = QtGui.QScrollArea()
        self.widget.setWindowTitle('Camera Bookmarks')
        self.storeMapper = QtCore.QSignalMapper()
        self.flyMapper = QtCore.QSignalMapper()
        self.storeMapper.connect('mapped(QObject*)', self.onStoreCamera)
        self.flyMapper.connect('mapped(QObject*)', self.onFlyToCamera)
        self.numberOfBookmarks = 8
        self.updateLayout()

    def updateLayout(self):
        self.storeButtons = []
        self.flyButtons = []
        self.textEdits = []

        w = QtGui.QWidget()
        l = QtGui.QGridLayout(w)

        for i in xrange(self.numberOfBookmarks):
            storeButton = QtGui.QPushButton('Set')
            flyButton = QtGui.QPushButton('Fly')
            textEdit = QtGui.QLineEdit('camera %d' % i)
            storeButton.connect('clicked()', self.storeMapper, 'map()')
            flyButton.connect('clicked()', self.flyMapper, 'map()')
            self.storeMapper.setMapping(storeButton, storeButton)
            self.flyMapper.setMapping(flyButton, flyButton)
            self.storeButtons.append(storeButton)
            self.flyButtons.append(flyButton)
            self.textEdits.append(textEdit)
            l.addWidget(storeButton, i, 0)
            l.addWidget(flyButton, i, 1)
            l.addWidget(textEdit, i, 2)
            flyButton.setEnabled(False)

        self.flySpeedSpinner = QtGui.QDoubleSpinBox()
        self.flySpeedSpinner.setMinimum(0)
        self.flySpeedSpinner.setMaximum(60)
        self.flySpeedSpinner.setDecimals(1)
        self.flySpeedSpinner.setSingleStep(0.5)
        self.flySpeedSpinner.setSuffix(' seconds')
        self.flySpeedSpinner.setValue(1.0)

        l.addWidget(QtGui.QLabel('Fly speed:'), i+1, 0, 2)
        l.addWidget(self.flySpeedSpinner, i+1, 2)

        loadButton = QtGui.QPushButton('Load Stored')
        loadButton.connect('clicked()', self.loadStoredValues)
        l.addWidget(loadButton, i+2, 0, 2)
        printButton = QtGui.QPushButton('Print Camera')
        printButton.connect('clicked()', self.printCurrentCamera)
        l.addWidget(printButton, i+2, 2)

        self.widget.setWidget(w)

    def onStoreCamera(self, button):
        index = self.storeButtons.index(button)
        self.bookmarks.storeCameraBookmark(index)
        self.flyButtons[index].setEnabled(True)

    def onFlyToCamera(self, button):
        index = self.flyButtons.index(button)
        self.bookmarks.flyer.flyTime = self.flySpeedSpinner.value
        self.bookmarks.flyToBookmark(index)

    def setStoredCamera(self, index, focal, position, label):
        self.textEdits[index].setText(label)
        self.bookmarks.storeFocalAndPosition(index, focal, position)
        self.flyButtons[index].setEnabled(True)

    def loadStoredValues(self):
        self.setStoredCamera(0, [7.89,-0.65,0.50], [8.16,1.88,48.64], "top down")
        self.setStoredCamera(1, [6.27,3.08,-1.75], [7.68,16.79,21.63], "approach")
        self.setStoredCamera(2, [3.64,-0.98,-0.26], [11.61,-3.51,2.86], "lever1")
        self.setStoredCamera(3, [-3.98,-0.27,-0.90], [6.06,-9.81,9.71], "middle area")
        self.setStoredCamera(4, [-10.04,-0.14,0.77], [-13.83,-7.51,7.24], "middle area 2")
        self.setStoredCamera(5, [-11.35,1.41,2.19], [-12.15,-6.20,13.42], "narrow")
        self.setStoredCamera(6, [7.20,-1.28,1.24], [6.93,-3.67,46.78], "top end")

    def printCurrentCamera(self):
        camera = self.bookmarks.view.camera()
        focal, position = camera.GetFocalPoint(), camera.GetPosition()
        #print focal
        #print position
        focal_str='[' + '{:.2f}'.format(focal[0]) + ',' + '{:.2f}'.format(focal[1]) + ',' + '{:.2f}'.format(focal[2]) + ']'
        position_str='[' + '{:.2f}'.format(position[0]) + ',' + '{:.2f}'.format(position[1]) + ',' + '{:.2f}'.format(position[2]) + ']'
        full_str = 'self.setStoredCamera(0, ' + focal_str + ', ' + position_str + ', "label")'
        print full_str


def init(view):
    global widget, dock
    widget = CameraBookmarkWidget(view)
    dock = applogic.addWidgetToDock(widget.widget, action=None)
    dock.hide()

    return widget
