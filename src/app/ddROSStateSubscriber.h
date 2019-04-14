#ifndef __ddROSStateSubscriber_h
#define __ddROSStateSubscriber_h

#include <QObject>
#include <ddMacros.h>

#include <string>

#include <PythonQt.h>
#include <QMutex>
#include <QMutexLocker>
#include <QWaitCondition>
#include <QTime>

#include <boost/shared_ptr.hpp>
#include <ros/ros.h>
#include <tf/transform_listener.h>

#include "ddFPSCounter.h"
#include "ddAppConfigure.h"

#include <quadruped_msgs/QuadrupedState.h>


class DD_APP_EXPORT ddROSStateSubscriber : public QObject
 {
  Q_OBJECT

public:

  ddROSStateSubscriber(const QList<QString>& argv2, const QString& channel, QObject* parent=NULL) : QObject(parent)
  {
    mChannel = channel;
    this->mEmitMessages = true;
    this->mNotifyAllMessages = false;
    this->mRequiredElapsedMilliseconds = 0;
    this->mTimer.start();
    this->connect(this, SIGNAL(messageReceivedInQueue(const QString&)), SLOT(onMessageInQueue(const QString&)));

    if (!ros::isInitialized()) {
      std::cout << "ddROSStateSubscriber: roscpp not initialized. Running init.\n";

      // guarantee contiguous, null terminated strings
      std::vector<std::vector<char>> vstrings;
      // pointers to rhose strings
      std::vector<char*> cstrings;
      vstrings.reserve(argv2.size());
      cstrings.reserve(argv2.size());

      for(size_t i = 0; i < argv2.size(); ++i)
      {
        std::string argv_string = argv2[i].toStdString();
        vstrings.emplace_back(argv_string.begin(), argv_string.end());
        vstrings.back().push_back('\0');
        cstrings.push_back(vstrings.back().data());
      }
      int argc = cstrings.size();

      ros::init(argc, cstrings.data(), "director_dd", ros::init_options::NoSigintHandler |
                ros::init_options::AnonymousName);


      //int argc = 0;
      //char** argv = 0;
      //ros::init(argc, argv, "director_dd", ros::init_options::NoSigintHandler |
      //          ros::init_options::AnonymousName);
    }else{
      std::cout << "ddROSStateSubscriber: ROS is Initialized. Not running init. This should not happen\n";
    }

    ros::NodeHandle n;
    mSubscriber = boost::make_shared<ros::Subscriber>(
    n.subscribe("/state_estimator/quadruped_state", 1000, &ddROSStateSubscriber::messageHandler, this));
    mTfListener = boost::make_shared<tf::TransformListener>();

  }

  virtual ~ddROSStateSubscriber()
  {
  }

  virtual void subscribe()
  {
    // Used to contain LCM subscribe
  }

  virtual void unsubscribe()
  {
    // Used to contain LCM unsubscribe
  }

  void resetTime()
  {
    mTfListener->clear();
  }

  const QString& channel() const
  {
    return mChannel;
  }

  QList<QString> getJointNames() const
  {
    return mJointNames;
  }


  QVector<double> getJointPositions() const
  {
    return mJointPositions;
  }

  QVector<double> getRobotPose() const
  {
    // returns base_in_odom in x,y,z,qx,qy,qz,qw
    // note the ordering matches ROS
    return mRobotPose;
  }

  QVector<double> getOdomInMap() const
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
    ros::Time time = ros::Time(mSec, mNsec);
    mTfListener->waitForTransform("map", "odom", time, ros::Duration(2.0));
    try {
      mTfListener->lookupTransform("map", "odom", time, transform);
    }
    catch (tf::TransformException& ex){
      ROS_ERROR("%s",ex.what());
      return mapInOdomTranslation; // this isnt correct
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

    /*
    std::cout << mSec << ":" << mNsec << " - "
              << mapInOdomTranslation[0] << ", "
              << mapInOdomTranslation[1] << ", "
              << mapInOdomTranslation[2] << ", "
              << mapInOdomTranslation[3] << ", "
              << mapInOdomTranslation[4] << ", "
              << mapInOdomTranslation[5] << ", "
              << mapInOdomTranslation[6] << "\n";
    */

    return mapInOdomTranslation;
  }

  void setCallbackEnabled(bool enabled)
  {
    this->mEmitMessages = enabled;
  }

  bool callbackIsEnabled() const
  {
    return this->mEmitMessages;
  }

  // See notifyAllMessagesIsEnabled()
  void setNotifyAllMessagesEnabled(bool enabled)
  {
    this->mNotifyAllMessages = enabled;
  }

  // If the main thread is busy while several LCM messages are received by this
  // subscriber on the LCM thread, then this flag determines which messages the
  // main thread will see when it becomes ready to process messages.  If this
  // flag is true, then the main thread will be notified, via the messageReceived()
  // signal, for each message.  If this flag is false, then the main thread will
  // be notified only once with the most recently received message.  Set this
  // flag to true if it is important to never miss a message.  The default is
  // false, meaning that messages will be dropped if the main thread is not
  // available to process them before a new message is received.
  bool notifyAllMessagesIsEnabled() const
  {
    return this->mNotifyAllMessages;
  }

  void setSpeedLimit(double hertz)
  {
    if (hertz <= 0.0)
    {
      this->mRequiredElapsedMilliseconds = 0;
    }
    else
    {
      this->mRequiredElapsedMilliseconds = static_cast<int>(1000.0 / hertz);
    }
  }

  double getMessageRate()
  {
    return this->mFPSCounter.averageFPS();
  }

  QByteArray getNextMessage(int timeout)
  {

    QMutexLocker locker(&this->mMutex);

    QByteArray msg = this->mLastMessage;
    this->mLastMessage.clear();


    if (msg.size())
    {
      return msg;
    }

    bool haveNewMessage = this->mWaitCondition.wait(&this->mMutex, timeout);

    if (!haveNewMessage)
    {
      return QByteArray();
    }

    msg = this->mLastMessage;
    this->mLastMessage.clear();

    return msg;
  }

signals:

  void messageReceived(const QString& channel);
  void messageReceivedInQueue(const QString& channel);

protected slots:

  void onMessageInQueue(const QString& channel)
  {
    emit this->messageReceived(channel);
  }


protected:

  void updateState(const quadruped_msgs::QuadrupedState& message){
    mSec = message.header.stamp.sec;
    mNsec = message.header.stamp.nsec;

    mRobotPose.resize(7);
    mRobotPose[0] = message.pose.pose.position.x;
    mRobotPose[1] = message.pose.pose.position.y;
    mRobotPose[2] = message.pose.pose.position.z;
    mRobotPose[3] = message.pose.pose.orientation.x;
    mRobotPose[4] = message.pose.pose.orientation.y;
    mRobotPose[5] = message.pose.pose.orientation.z;
    mRobotPose[6] = message.pose.pose.orientation.w;

    int numJoints = message.joints.name.size();
    mJointPositions.resize(numJoints);
    QList<QString> names;
    names.reserve(numJoints);

    for (int i = 0; i < numJoints; i++)
    {
       mJointPositions[i] = message.joints.position[i];
       names.append(QString( message.joints.name[i].c_str()  ));
    }
    mJointNames = names;

  }

  void messageHandler(const quadruped_msgs::QuadrupedState& message) {

    mFPSCounter.update();

    if (this->mEmitMessages)
    {
      if (this->mRequiredElapsedMilliseconds == 0 || mTimer.elapsed() > this->mRequiredElapsedMilliseconds)
      {
        this->mTimer.restart();

        if (this->mNotifyAllMessages)
        {
          updateState(message);
          emit this->messageReceived( mChannel );
        }
        else
        {
          this->mMutex.lock();
          updateState(message);
          bool doEmit = true;//= !this->mLastMessage.size();
          this->mMutex.unlock();

          if (doEmit)
          {
            emit this->messageReceived( mChannel );
          }
        }

      }
    }
    else
    {
      this->mMutex.lock();
      this->mMutex.unlock();
      this->mWaitCondition.wakeAll();
    }

  }

  bool mEmitMessages;
  bool mNotifyAllMessages;
  int mRequiredElapsedMilliseconds;
  mutable QMutex mMutex;
  QWaitCondition mWaitCondition;
  QByteArray mLastMessage;
  ddFPSCounter mFPSCounter;
  QTime mTimer;
  QString mChannel;

  boost::shared_ptr<ros::Subscriber> mSubscriber;
  boost::shared_ptr<tf::TransformListener> mTfListener;

  QVector<double> mRobotPose;
  QVector<double> mJointPositions;
  QList<QString> mJointNames;
  long mSec;
  long mNsec;


};

#endif
