#ifndef __ddROSTfListener_h
#define __ddROSTfListener_h

#include <QObject>
#include <QVector>
#include <ddMacros.h>

#include <string>

#include "ddAppConfigure.h"

#include <boost/shared_ptr.hpp>
#include <ros/ros.h>
#include <tf/transform_listener.h>


class DD_APP_EXPORT ddROSTfListener : public QObject
 {
  Q_OBJECT

public:

  ddROSTfListener(QObject* parent=NULL) 
    : QObject(parent), odomToMap(7)
  {
    odomToMap[0] = 0;
    odomToMap[1] = 0;
    odomToMap[2] = 0;
    odomToMap[3] = 0;
    odomToMap[4] = 0;
    odomToMap[5] = 0;
    odomToMap[6] = 1;

    mTfListener = boost::make_shared<tf::TransformListener>();

  }

  virtual ~ddROSTfListener()
  {
  }

  
  void resetTime()
  {
    mTfListener->clear();
  }

  QVector<double> getOdomInMap() const
  {
    return odomToMap;
  }      

  QVector<double> computeOdomInMap() const
  {
    // returns odom_in_map in x,y,z,qx,qy,qz,qw
    // note the ordering matches ROS
    QVector<double> mapInOdomTranslation;
    mapInOdomTranslation.resize(7);
    mapInOdomTranslation[0] = 0;
    mapInOdomTranslation[1] = 0;
    mapInOdomTranslation[2] = 0;
    mapInOdomTranslation[3] = 0;
    mapInOdomTranslation[4] = 0;
    mapInOdomTranslation[5] = 0;
    mapInOdomTranslation[6] = 1;

    tf::StampedTransform transform;
    try {
      mTfListener->lookupTransform("map", "odom", ros::Time(0), transform);
    }
    catch (tf::TransformException& ex){
      ROS_ERROR("%s",ex.what());
      return mapInOdomTranslation; // this isn't correct
    }

    tf::Vector3 origin = transform.getOrigin();
    tf::Quaternion quaternion = transform.getRotation();

    mapInOdomTranslation[0] = origin.getX();
    mapInOdomTranslation[1] = origin.getY();
    mapInOdomTranslation[2] = origin.getZ();
    mapInOdomTranslation[3] = quaternion.x();
    mapInOdomTranslation[4] = quaternion.y();
    mapInOdomTranslation[5] = quaternion.z();
    mapInOdomTranslation[6] = quaternion.w();

    return mapInOdomTranslation;
  }

  QVector<double> computeTransform(const QString& targetFrame, const QString& sourceFrame) const
  {
    // returns odom_in_map in x,y,z,qx,qy,qz,qw
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

    

    tf::StampedTransform transform;
    try {
      mTfListener->lookupTransform(targetFrame.toStdString(), sourceFrame.toStdString(), ros::Time(0), transform);
    }
    catch (tf::TransformException& ex){
      ROS_ERROR("%s",ex.what());
      return tfTransform; // this isn't correct
    }

    tf::Vector3 origin = transform.getOrigin();
    tf::Quaternion quaternion = transform.getRotation();

    tfTransform[0] = origin.getX();
    tfTransform[1] = origin.getY();
    tfTransform[2] = origin.getZ();
    tfTransform[3] = quaternion.x();
    tfTransform[4] = quaternion.y();
    tfTransform[5] = quaternion.z();
    tfTransform[6] = quaternion.w();

    return tfTransform;
  }

signals:

  void messageReceived(const QByteArray& messageData, const QString& channel, const QString& md5sum);

protected:

  boost::shared_ptr<tf::TransformListener> mTfListener;

  /**
   * @brief odomToMap is the transformation between map and odom
   */
  QVector<double> odomToMap;  



};

#endif
