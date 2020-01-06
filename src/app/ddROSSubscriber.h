#ifndef __ddROSStateSubscriber_h
#define __ddROSStateSubscriber_h

#include <QObject>
#include <ddMacros.h>

#include <string>

#include <PythonQt.h>

#include <boost/shared_ptr.hpp>
#include <ros/ros.h>

#include "ddAppConfigure.h"

#include <topic_tools/shape_shifter.h>

class DD_APP_EXPORT ddROSSubscriber : public QObject
 {
  Q_OBJECT

public:

  ddROSSubscriber(const QList<QString>& argv2, const QString& channel, QObject* parent=NULL) 
    : QObject(parent)
  {
    mChannel = channel;

    if (!ros::isInitialized()) {
      // guarantee contiguous, null terminated strings
      std::vector<std::vector<char>> vstrings;
      // pointers to rhose strings
      std::vector<char*> cstrings;
      vstrings.reserve(argv2.size());
      cstrings.reserve(argv2.size());

      for(size_t i = 0; i < argv2.size(); ++i) {
        std::string argv_string = argv2[i].toStdString();
        vstrings.emplace_back(argv_string.begin(), argv_string.end());
        vstrings.back().push_back('\0');
        cstrings.push_back(vstrings.back().data());
      }
      int argc = cstrings.size();

      ros::init(argc, cstrings.data(), "director_dd", ros::init_options::NoSigintHandler |
                ros::init_options::AnonymousName);
    }

    ros::NodeHandle n;
    std::string channel_str = channel.toLocal8Bit().data();
    boost::function<void(const topic_tools::ShapeShifter::ConstPtr&) > callback;
    callback = boost::bind( &ddROSSubscriber::messageHandler, this, _1, channel_str ) ;

    mSubscriber = boost::make_shared<ros::Subscriber>(
    n.subscribe(channel.toLocal8Bit().data(), 1000, callback));
  }

  virtual void unsubscribe()
  {
    mSubscriber->shutdown();
  }

signals:

  void messageReceived(const QByteArray& messageData, const QString& channel, const QString& md5sum);

protected:
QByteArray decodeMessage(const topic_tools::ShapeShifter::ConstPtr& msg)
  {
    // Once again, it would be nice to access directly the 
    // private member ShapeShifter::msgBuf, unfortunately we can't.
    // This oblige us to do a copy of the buffer using ShapeShifter::write
    static std::vector<uint8_t> buffer;
    buffer.resize( msg->size() );
    ros::serialization::OStream stream(buffer.data(), buffer.size());
    msg->write(stream);

    return QByteArray((char*)buffer.data(), buffer.size());
  }

  void messageHandler(const topic_tools::ShapeShifter::ConstPtr& msg, const std::string& channel)
  {
    ddNotUsed(channel);

    QByteArray messageBytes = decodeMessage(msg);
    mMD5Sum = QString(msg->getMD5Sum().c_str() );
    emit this->messageReceived(messageBytes, QString(channel.c_str()), mMD5Sum );
  }

  QString mChannel;
  QString mMD5Sum;

  boost::shared_ptr<ros::Subscriber> mSubscriber;
};

#endif
