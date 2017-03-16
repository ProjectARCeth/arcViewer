#include "RosInterface.hpp"

RosInterface::RosInterface(int argc, char **pArgv){
    init_argc_ = argc;
    init_argv_ = pArgv;
    MODE_INIT = true;
    if(strlen(*(init_argv_ + 1)) == 5) MODE_INIT = false;
}

RosInterface::~RosInterface(){
    if (ros::isStarted()){
        ros::shutdown();
        ros::waitForShutdown();
    }
    thread_->wait();
}

bool RosInterface::getInitMode(){return MODE_INIT;}

bool RosInterface::init(){
    //Init thread.
    thread_ = new QThread();
    this->moveToThread(thread_);
    connect(thread_, &QThread::started, this, &RosInterface::run);
    //Init ros.
    ros::init(init_argc_,init_argv_, "arc_gui");
    if (!ros::master::check()) return false;
    ros::start();
    ros::Time::init();
    ros::NodeHandle node;
    //Get parameters.
    node.getParam("/safety/MIN_PUBLISH_NOTSTOP_COUNT", MIN_PUBLISH_NOTSTOP_COUNT);
    node.getParam("/general/QUEUE_LENGTH", QUEUE_LENGTH);
    node.getParam("/topic/OBSTACLE_DISTANCE", OBSTACLE_DISTANCE_TOPIC);
    node.getParam("/topic/NOTSTOP", NOTSTOP_TOPIC);
    node.getParam("/topic/NAVIGATION_INFO", PURE_PURSUIT_INFO_TOPIC);
    node.getParam("/topic/READY_FOR_DRIVING", READY_FOR_LAUNCHING_TOPIC);
    node.getParam("/topic/SHUTDOWN", SHUTDOWN_TOPIC);
    node.getParam("/topic/STATE", STATE_TOPIC);
    node.getParam("/topic/STATE_STEERING_ANGLE", STEERING_TOPIC);
    node.getParam("/topic/TRACKING_ERROR", DEVIATION_TOPIC);
    node.getParam("/topic/TRACKING_ERROR_VELOCITY", DEVIATION_VELOCITY_TOPIC);
    node.getParam("/topic/VCU_LAUNCHING_COMMAND", LAUNCHING_COMMAND_TOPIC);
    //Init publisher and subscriber.
    launch_command_pub_ = node.advertise<std_msgs::Bool>(LAUNCHING_COMMAND_TOPIC, QUEUE_LENGTH);
    notstop_pub_ = node.advertise<std_msgs::Bool>(NOTSTOP_TOPIC, QUEUE_LENGTH);
    shutdown_pub_ = node.advertise<std_msgs::Bool>(SHUTDOWN_TOPIC, QUEUE_LENGTH);
    deviation_sub_ = node.subscribe(DEVIATION_TOPIC, QUEUE_LENGTH, &RosInterface::devCallback, this);
    deviation_vel_sub_ = node.subscribe(DEVIATION_VELOCITY_TOPIC, QUEUE_LENGTH, &RosInterface::devVelCallback, this);
    obstacle_distance_sub_ = node.subscribe(OBSTACLE_DISTANCE_TOPIC, QUEUE_LENGTH, &RosInterface::obstacleDistanceCallback, this);
    notstop_sub_ = node.subscribe(NOTSTOP_TOPIC, QUEUE_LENGTH, &RosInterface::notstopCallback, this);
    pure_pursuit_info_sub_ = node.subscribe(PURE_PURSUIT_INFO_TOPIC, QUEUE_LENGTH, &RosInterface::purePursuitCallback, this);
    ready_for_launching_sub_ = node.subscribe(READY_FOR_LAUNCHING_TOPIC, QUEUE_LENGTH, &RosInterface::readyForLaunchingCallback, this);
    steering_sub_ = node.subscribe(STEERING_TOPIC, QUEUE_LENGTH, &RosInterface::steeringCallback, this);
    velocity_sub_ = node.subscribe(STATE_TOPIC, QUEUE_LENGTH, &RosInterface::velCallback, this);
    //Start thread.
    thread_->start();
    return true;
}

void RosInterface::run(){
    ros::Rate loop_rate(100);
    while (ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void RosInterface::launching(){
    std::cout << std::endl << "GUI: System launched " << std::endl;
    std_msgs::Bool launching_msg;
    launching_msg.data = true;
    launch_command_pub_.publish(launching_msg);
}

void RosInterface::notstop(){
    std::cout << std::endl << "GUI: NOTSTOP !!!!!!!!" << std::endl;
    std_msgs::Bool notstop_msg;
    notstop_msg.data = true;
    for (int i=0; i<MIN_PUBLISH_NOTSTOP_COUNT; ++i) notstop_pub_.publish(notstop_msg);
}

void RosInterface::shutdown(){
    std::cout << std::endl << "GUI: Controlled Shutdown !!!" << std::endl;
    std_msgs::Bool shutdown_msg;
    shutdown_msg.data = true;
    for (int i=0; i<MIN_PUBLISH_NOTSTOP_COUNT; ++i) shutdown_pub_.publish(shutdown_msg);
}

void RosInterface::devCallback(const std_msgs::Float64::ConstPtr& msg){
    QMutex * pMutex = new QMutex();
    pMutex->lock();
    double deviation = msg->data;
    pMutex->unlock();
    delete pMutex;
    Q_EMIT newDev(deviation);
}

void RosInterface::devVelCallback(const std_msgs::Float64::ConstPtr& msg){
    QMutex * pMutex = new QMutex();
    pMutex->lock();
    double vel_deviation = msg->data;
    pMutex->unlock();
    delete pMutex;
    Q_EMIT newVelDev(vel_deviation);
}

void RosInterface::obstacleDistanceCallback(const std_msgs::Float64::ConstPtr& msg){
    QMutex * pMutex = new QMutex();
    pMutex->lock();
    double distance = msg->data;
    pMutex->unlock();
    delete pMutex;
    Q_EMIT newObstacleDis(distance);
}

void RosInterface::notstopCallback(const std_msgs::Bool::ConstPtr& msg){
    if(msg->data == true) Q_EMIT newNotstop();
}

void RosInterface::purePursuitCallback(const std_msgs::Float32MultiArray::ConstPtr& msg){
    QMutex * pMutex = new QMutex();
    pMutex->lock();
    std::vector<double> info(10, 0);
    for (int i = 0; i < 10; ++i){
        info[i] = msg->data[i];
    }
    pMutex->unlock();
    delete pMutex;
    Q_EMIT newPurePursuitInfo(info);
}

void RosInterface::readyForLaunchingCallback(const std_msgs::Bool::ConstPtr& msg){
    if(msg->data == true) Q_EMIT newLaunchCommand(MODE_INIT);
}

void RosInterface::steeringCallback(const std_msgs::Float64::ConstPtr& msg){
    QMutex * pMutex = new QMutex();
    pMutex->lock();
    double angle = msg->data;
    pMutex->unlock();
    delete pMutex;
    Q_EMIT newSteering(angle);
}

void RosInterface::velCallback(const arc_msgs::State::ConstPtr& msg){
    QMutex * pMutex = new QMutex();
    pMutex->lock();
    double velocity = msg->pose_diff;
    pMutex->unlock();
    delete pMutex;
    Q_EMIT newVel(velocity);
}