#ifndef __ddROSInit_h
#define __ddROSInit_h

#include <QObject>
#include <ddMacros.h>
#include <PythonQt.h>
#include <memory>

#include <ros/ros.h>

#include "ddAppConfigure.h"

class DD_APP_EXPORT ddROSInit : public QObject {
    Q_OBJECT
public:

    ~ddROSInit(){
        ros::shutdown();
    }

    ddROSInit(const QList<QString>& argv2)
    {
        if (!ros::isInitialized()) {
            ROS_INFO("Initialising dd ROS.");
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
        } else {
            ROS_INFO("dd ROS is already initialised.");
        }

        if (!spinner) {
            spinner = std::make_shared<ros::AsyncSpinner>(8);
        }
        spinner->start();
    }

private:
    std::shared_ptr<ros::AsyncSpinner> spinner;
};

#endif    