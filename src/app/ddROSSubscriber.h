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
#include <tf/transform_listener.h>

#include "ddFPSCounter.h"
#include "ddAppConfigure.h"

#include <topic_tools/shape_shifter.h>


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
      std::cout << "ddROSSubscriber: roscpp not initialized. This should not happen.\n";
    }else{
      std::cout << "ddROSSubscriber: roscpp is Initialized. Not running init\n";
    }

    ros::NodeHandle n;
    std::string channel_str = channel.toLocal8Bit().data();
    boost::function<void(const topic_tools::ShapeShifter::ConstPtr&) > callback;
    callback = boost::bind( &ddROSSubscriber::messageHandler, this, _1, channel_str ) ;

    mSubscriber = boost::make_shared<ros::Subscriber>(
    n.subscribe(channel.toLocal8Bit().data(), 1000, callback));

  }

  virtual ~ddROSSubscriber()
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

  const QString& channel() const
  {
    return mChannel;
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

  void messageReceived(const QByteArray& messageData, const QString& channel);
  void messageReceivedInQueue(const QString& channel);

protected slots:

  void onMessageInQueue(const QString& channel)
  {
    QByteArray msg = this->getNextMessage(0);
    emit this->messageReceived(msg, channel);
  }


protected:

  QByteArray decodeMessage(const topic_tools::ShapeShifter::ConstPtr& msg)
  {
    //if (!mDecodeCallback)
    //{
    //  return QVariant();
    //}

    // Once again, it would be nice to access directly the 
    // private member ShapeShifter::msgBuf, unfortunately we can't.
    // This oblige us to do a copy of the buffer using ShapeShifter::write
    static std::vector<uint8_t> buffer;
    buffer.resize( msg->size() );
    ros::serialization::OStream stream(buffer.data(), buffer.size());
    msg->write(stream);

    //std::cout << (int) buffer.size() << " is buffer size\n";

    return QByteArray((char*)buffer.data(), buffer.size());
  }

  void messageHandler(const topic_tools::ShapeShifter::ConstPtr& msg,
                              const std::string& channel )
  {
    //std::cout << channel << " in messageHandler\n";

    ddNotUsed(channel);

    QByteArray messageBytes = decodeMessage(msg);

    mFPSCounter.update();

    if (this->mEmitMessages)
    {
      if (this->mRequiredElapsedMilliseconds == 0 || mTimer.elapsed() > this->mRequiredElapsedMilliseconds)
      {
        this->mTimer.restart();

        if (this->mNotifyAllMessages)
        {
          emit this->messageReceived(messageBytes, QString(channel.c_str()));
        }
        else
        {
          this->mMutex.lock();
          bool doEmit = !this->mLastMessage.size();
          this->mLastMessage = messageBytes;
          this->mMutex.unlock();

          if (doEmit)
          {
            emit this->messageReceivedInQueue(QString(channel.c_str()));
          }
        }

      }
    }
    else
    {
      this->mMutex.lock();
      this->mLastMessage = messageBytes;
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

};

#endif
