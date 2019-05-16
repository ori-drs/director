from director import transformUtils
import numpy as np
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
import PythonQt

def rosPoseToTransform(pose):
    '''
    From ros Pose to vtk Transform
    '''


    pos = [pose.position.x,pose.position.y,pose.position.z]
    quat = [pose.orientation.w,pose.orientation.x,pose.orientation.y,pose.orientation.z]
    transform = transformUtils.transformFromPose(pos,quat)
    return transform

def rosPoseFromTransform(transform):
    '''
    From vtk Transform to ros Pose
    '''
    pos,quat = transformUtils.poseFromTransform(transform)
    # This is unsatisfactory, but something is wrong with the orientation from poseFromTransform
    quat = transformUtils.rollPitchYawToQuaternion(np.array(transform.GetOrientation())*np.pi/180)

    pose = Pose()
    pose.position.x = pos[0]
    pose.position.y = pos[1]
    pose.position.z = pos[2]

    # NB: vtk Ordering is wxyz
    pose.orientation.w = quat[0]
    pose.orientation.x = quat[1]
    pose.orientation.y = quat[2]
    pose.orientation.z = quat[3]

    return pose


def addSubscriber(channel, messageClass=None, callback=None, historicalLoader=None, callbackNeedsChannel=False):

    subscriber = PythonQt.dd.ddROSSubscriber(channel)

    def handleMessage(messageData, channel, md5sum):

        if (md5sum != messageClass()._md5sum):
            print "Message on topic", channel, "cannot be decoded. md5sum has changed."
            print "Msg definition :", messageClass()._md5sum
            print "  Incoming msg :", md5sum
            subscriber.unsubscribe()
            return

        #print "received ROS msg",channel
        #print messageClass
        msg = messageClass().deserialize(messageData.data())

        # alternative approach:
        #a,_,_ = rostopic.get_topic_class(channel)
        #msg = a().deserialize(msg_raw.data())

        #print msg
        #print channel

        callback(msg)


    subscriber.connect('messageReceived(const QByteArray&, const QString&, const QString&)', handleMessage)

    return subscriber