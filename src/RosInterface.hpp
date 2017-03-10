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
#include <std_msgs/Float32MultiArray.h>
#include <vector>

#include <QMutex>
#include <QStringList>
#include <QtCore>
#include <QThread>

class RosInterface : public QObject {
	Q_OBJECT
public:
    RosInterface(int argc, char **pArgv);
    virtual ~RosInterface();
    bool getInitMode();
    bool init();
    void notstop();
    void shutdown();
    void devCallback(const std_msgs::Float64::ConstPtr& msg);
    void devVelCallback(const std_msgs::Float64::ConstPtr& msg);
    void obstacleDistanceCallback(const std_msgs::Float64::ConstPtr& msg);
    void modeCallback(const std_msgs::Bool::ConstPtr& msg);
    void notstopCallback(const std_msgs::Bool::ConstPtr& msg);
    void purePursuitCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);
    void steeringCallback(const std_msgs::Float64::ConstPtr& msg);
    void velCallback(const arc_msgs::State::ConstPtr& msg);
    Q_SLOT void run();
    Q_SIGNAL void newDev(double deviation);
    Q_SIGNAL void newObstacleDis(double distance);
    Q_SIGNAL void newMode(bool mode);
    Q_SIGNAL void newNotstop();
    Q_SIGNAL void newPurePursuitInfo(std::vector<double> info);
    Q_SIGNAL void newSteering(double angle);
    Q_SIGNAL void newVel(double x,double y,double z);
    Q_SIGNAL void newVelDev(double vel_deviation);

private:
    int init_argc_;
    char** init_argv_;
    //Publisher and subscriber.
    ros::Publisher notstop_pub_;
    ros::Publisher shutdown_pub_;
    ros::Subscriber deviation_sub_;
    ros::Subscriber deviation_vel_sub_;
    ros::Subscriber obstacle_distance_sub;
    ros::Subscriber mode_sub_;
    ros::Subscriber notstop_sub_;
    ros::Subscriber pure_pursuit_info_sub_;
    ros::Subscriber steering_sub_;
    ros::Subscriber velocity_sub_;
    //Yaml constants.
    bool MODE_INIT;
    int MIN_PUBLISH_NOTSTOP_COUNT;
    std::string DEVIATION_TOPIC;
    std::string DEVIATION_VELOCITY_TOPIC;
    std::string OBSTACLE_DISTANCE_TOPIC;
    std::string MODE_TOPIC;
    std::string NOTSTOP_TOPIC;
    std::string PURE_PURSUIT_INFO_TOPIC;
    std::string SHUTDOWN_TOPIC;
    std::string STATE_TOPIC;
    std::string STEERING_TOPIC;
    int QUEUE_LENGTH;
    //Thread.
    QThread * thread_;
};
#endif

