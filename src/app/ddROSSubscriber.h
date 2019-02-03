#ifndef __ddROSSubscriber_h
#define __ddROSSubscriber_h

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

#include "ddFPSCounter.h"
#include "ddAppConfigure.h"

#include <quadruped_msgs/QuadrupedState.h>


class DD_APP_EXPORT ddROSSubscriber : public QObject
 {
  Q_OBJECT

public:

  ddROSSubscriber(const QString& channel, QObject* parent=NULL) : QObject(parent)
  {
    mChannel = channel;
    this->mEmitMessages = true;
    this->mNotifyAllMessages = false;
    this->mRequiredElapsedMilliseconds = 0;
    this->mTimer.start();
    this->connect(this, SIGNAL(messageReceivedInQueue(const QString&)), SLOT(onMessageInQueue(const QString&)));

  if (!ros::isInitialized()) {
    std::cout << "ddROSSubscriber is not init\n";
    int argc = 0;
    char** argv = 0;

    ros::init(argc, argv, "director_dd", ros::init_options::NoSigintHandler |
              ros::init_options::AnonymousName);    
  }else{
    std::cout << "ddROSSubscriber is init\n";
  }


    ros::NodeHandle n;
    subscriber_ = boost::make_shared<ros::Subscriber>(
    n.subscribe("/state_estimator/quadruped_state", 1000, &ddROSSubscriber::messageHandler, this));

    mRobotPosition = QVector<double>(3, 0.0);
    mRobotOrientation = QVector<double>(4, 0.0);
    mRobotOrientation[0] = 1; // w,x,y,z

    std::cout << "called ros subscribe" << "\n";

  }

  virtual ~ddROSSubscriber()
  {
  }

  virtual void subscribe()//lcm::LCM* lcmHandle)
  {

    //mSubscription = lcmHandle->subscribe(mChannel.toLocal8Bit().data(), &ddROSSubscriber::messageHandler, this);
  }

  virtual void unsubscribe()//lcm::LCM* lcmHandle)
  {
    //lcmHandle->unsubscribe(mSubscription);
    //mSubscription = 0;
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

  QVector<double> getRobotPosition() const
  {
    return mRobotPosition;
  }

  QVector<double> getRobotOrientation() const
  {
    return mRobotOrientation;
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

  //void messageReceived(const QByteArray& messageData, const QString& channel);
  void messageReceived(const QString& channel);
  void messageReceivedInQueue(const QString& channel);

protected slots:

  void onMessageInQueue(const QString& channel)
  {
    //QByteArray msg = this->getNextMessage(0);
    emit this->messageReceived(channel);
  }


protected:

  void updateState(const quadruped_msgs::QuadrupedState& message){
    mRobotPosition[0] = message.pose.pose.position.x;
    mRobotPosition[1] = message.pose.pose.position.y;
    mRobotPosition[2] = message.pose.pose.position.z;

    mRobotOrientation[0] = message.pose.pose.orientation.w;
    mRobotOrientation[1] = message.pose.pose.orientation.x;
    mRobotOrientation[2] = message.pose.pose.orientation.y;
    mRobotOrientation[3] = message.pose.pose.orientation.z;

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
  //lcm::Subscription* mSubscription;

  boost::shared_ptr<ros::Subscriber> subscriber_;

  QVector<double> mRobotPosition;
  QVector<double> mRobotOrientation;
  QVector<double> mJointPositions;
  QList<QString> mJointNames;

};

#endif
