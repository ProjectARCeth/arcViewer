#ifndef ___ROSINTERFACE_HPP___
#define ___ROSINTERFACE_HPP___

#include <QMutex>
#include <QStringList>
#include <QtCore>
#include <QThread>
#include <stdlib.h>
#include <iostream>
#include "assert.h"
#include <ros/ros.h>
#include <ros/network.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>

class RosInterface : public QObject {
	Q_OBJECT
public:
    RosInterface(int argc, char **pArgv, const char *topic_pose);
    virtual ~RosInterface();
    bool init();
    void poseCallback(const nav_msgs::Odometry & msg);
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    Q_SLOT void run();
    Q_SIGNAL void newPose(double x,double y,double z);
private:
    int m_Init_argc;
    char** m_pInit_argv;
    const char * m_topic_pose;

    double m_xPos;
    double m_yPos;
    double m_zPos;

    QThread * m_pThread;

    ros::Subscriber pose_listener;
};
#endif

