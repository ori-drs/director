#ifndef __ddROSTfListener_h
#define __ddROSTfListener_h

#include <QObject>
#include <QVector>
#include <ddMacros.h>

#include <string>

#include "ddAppConfigure.h"

#include <memory>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>


class DD_APP_EXPORT ddROSTfListener : public QObject
 {
  Q_OBJECT

public:

  ddROSTfListener(QObject* parent=NULL) 
    : QObject(parent)
  {
    if(!mTfListener)
    {
      mTfBuffer = std::make_shared<tf2_ros::Buffer>();
      mTfListener = std::make_shared<tf2_ros::TransformListener>(*mTfBuffer);
    }

  }

  virtual ~ddROSTfListener(){}
  
  void resetTime()
  {
    mTfBuffer->clear();
  }

  QString getAllFrames() const {
    return QString(mTfBuffer->allFramesAsYAML().c_str());
  }

  QVector<double> computeTransform(const QString& targetFrame, const QString& sourceFrame) const
  {
    // returns transform from target frame to source frame in x,y,z,qx,qy,qz,qw
    // note the ordering matches ROS
    QVector<double> tfTransform;
    tfTransform.resize(7);
    tfTransform[0] = 0;
    tfTransform[1] = 0;
    tfTransform[2] = 0;
    tfTransform[3] = 0;
    tfTransform[4] = 0;
    tfTransform[5] = 0;
    tfTransform[6] = 1;

    geometry_msgs::TransformStamped transform;
    try {
      transform = mTfBuffer->lookupTransform(targetFrame.toStdString(), sourceFrame.toStdString(), ros::Time(0));
    }
    catch (tf2::TransformException ex){
      ROS_ERROR("%s",ex.what());
      return tfTransform; // this isn't correct
    }

    tfTransform[0] = transform.transform.translation.x;
    tfTransform[1] = transform.transform.translation.y;
    tfTransform[2] = transform.transform.translation.z;
    tfTransform[3] = transform.transform.rotation.x;
    tfTransform[4] = transform.transform.rotation.y;
    tfTransform[5] = transform.transform.rotation.z;
    tfTransform[6] = transform.transform.rotation.w;

    return tfTransform;
  }

protected:

  static std::shared_ptr<tf2_ros::TransformListener> mTfListener;
  static std::shared_ptr<tf2_ros::Buffer> mTfBuffer;
};

#endif
