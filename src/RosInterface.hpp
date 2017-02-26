#ifndef ___ROSINTERFACE_HPP___
#define ___ROSINTERFACE_HPP___

#include "arc_msgs/State.h"
#include "arc_tools/coordinate_transform.hpp"

#include "Eigen/Dense"

#include <iostream>
#include <geometry_msgs/Vector3.h>
#include <ros/ros.h>
#include <stdlib.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

#include <QMutex>
#include <QStringList>
#include <QtCore>
#include <QThread>

class RosInterface : public QObject {
	Q_OBJECT
public:
    RosInterface(int argc, char **pArgv);
    virtual ~RosInterface();
    bool init();
    void notstop();
    void devCallback(const std_msgs::Float64::ConstPtr& msg);
    void devVelCallback(const std_msgs::Float64::ConstPtr& msg);
    void velCallback(const arc_msgs::State::ConstPtr& msg);
    Q_SLOT void run();
    Q_SIGNAL void newVel(double x,double y,double z);
    Q_SIGNAL void newDev(double deviation, double relative);
    Q_SIGNAL void newVelDev(double vel_deviation);

private:
    int init_argc_;
    char** init_argv_;
    ros::Publisher notstop_pub;
    ros::Subscriber deviation_sub_;
    ros::Subscriber deviation_vel_sub_;
    ros::Subscriber velocity_sub_;
    float MAX_DEVIATION_FROM_TEACH_PATH;
    std::string NOTSTOP_TOPIC;
    std::string STATE_TOPIC;
    std::string DEVIATION_TOPIC;
    std::string DEVIATION_VELOCITY_TOPIC;
    int QUEUE_LENGTH;
    QThread * thread_;
};
#endif

