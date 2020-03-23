import json
from collections import OrderedDict
from director.thirdparty import numpyjsoncoder
from director import callbacks
from director.utime import getUtime
from director.uuidutil import newUUID

class ObjectCollection(object):

    DESCRIPTION_UPDATED_SIGNAL = 'DESCRIPTION_UPDATED_SIGNAL'
    DESCRIPTION_REMOVED_SIGNAL = 'DESCRIPTION_REMOVED_SIGNAL'

    def __init__(self, channel):
        self.collection = OrderedDict()
        self.collectionId = newUUID()
        self.sentCommands = set()
        self.sentRequest = None
        self.channel = channel
        self.callbacks = callbacks.CallbackRegistry([self.DESCRIPTION_UPDATED_SIGNAL,
                                                     self.DESCRIPTION_REMOVED_SIGNAL])
        self.sub = None
        self._modified()

    def connectDescriptionUpdated(self, func):
        return self.callbacks.connect(self.DESCRIPTION_UPDATED_SIGNAL, func)

    def disconnectDescriptionUpdated(self, callbackId):
        self.callbacks.disconnect(callbackId)

    def connectDescriptionRemoved(self, func):
        return self.callbacks.connect(self.DESCRIPTION_REMOVED_SIGNAL, func)

    def disconnectDescriptionRemoved(self, callbackId):
        self.callbacks.disconnect(callbackId)

    def getDescriptionId(self, desc):
        return desc['uuid']

    def prettyPrintCollection(self):
        print json.dumps(json.loads(numpyjsoncoder.encode(self.collection)), indent=2)

    def getDescription(self, descriptionId):
        return self.collection[descriptionId]

    def updateDescription(self, desc, publish=True, notify=True):
        self.collection[self.getDescriptionId(desc)] = desc
        self._modified()

        if notify:
            self.callbacks.process(self.DESCRIPTION_UPDATED_SIGNAL, self, self.getDescriptionId(desc))

    def removeDescription(self, descriptionId, publish=True, notify=True):

        try:
            del self.collection[descriptionId]
            self._modified()
        except KeyError:
            pass

        if notify:
            self.callbacks.process(self.DESCRIPTION_REMOVED_SIGNAL, self, descriptionId)

    def _modified(self):
        self.mtime = getUtime()