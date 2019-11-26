import PythonQt
import numpy as np
import vtkNumpy

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from sensor_msgs import point_cloud2

from director import transformUtils
from director import segmentation

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


def rosPoseArrayFromTransformArray(goalTransformList, fixedFrame, rosTime):
    '''
    From list of vtkTransform to ros PoseArray
    '''
    msg = PoseArray()
    msg.header.stamp = rosTime
    msg.header.frame_id = fixedFrame

    for transform in goalTransformList:
        msg.poses.append( rosPoseFromTransform( transform ) )

    return msg


def convertPointCloud2ToPolyData(msg, addXYZ=False):
    # alternative:
    # https://answers.ros.org/question/230680/extracting-the-xyz-coordinates-from-pointcloud2-data-in-python/
    #reader = point_cloud2.read_points(msg)
    #x.append( reader.next()[0:3] )

    numPoints = msg.width * msg.height
    points=np.zeros(shape=(numPoints,3))
    for i, point in enumerate(point_cloud2.read_points(msg)):
        points[i] = point[0:3]

    polyData = vtkNumpy.numpyToPolyData(points, createVertexCells=True)

    if (addXYZ):
        polyData = segmentation.addCoordArraysToPolyDataXYZ(polyData)
    return polyData


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
