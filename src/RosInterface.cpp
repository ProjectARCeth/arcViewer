#include "RosInterface.hpp"

RosInterface::RosInterface(int argc, char **pArgv){
    init_argc_ = argc;
    init_argv_ = pArgv;
}

RosInterface::~RosInterface(){
    if (ros::isStarted()){
        ros::shutdown();
        ros::waitForShutdown();
    }
    thread_->wait();
}

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
    node.getParam("/safety/MAX_DEVIATION_FROM_TEACH_PATH", MAX_DEVIATION_FROM_TEACH_PATH);
    node.getParam("/general/QUEUE_LENGTH", QUEUE_LENGTH);
    node.getParam("/topic/NOTSTOP", NOTSTOP_TOPIC);
    node.getParam("/topic/STATE", STATE_TOPIC);
    node.getParam("/topic/TRACKING_ERROR", DEVIATION_TOPIC);
    node.getParam("/topic/TRACKING_ERROR_VELOCITY", DEVIATION_VELOCITY_TOPIC);
    //Init publisher and subscriber.
    notstop_pub = node.advertise<std_msgs::Bool>(NOTSTOP_TOPIC, QUEUE_LENGTH);
    velocity_sub_ = node.subscribe(STATE_TOPIC, QUEUE_LENGTH, &RosInterface::velCallback, this);
    deviation_sub_ = node.subscribe(DEVIATION_TOPIC, QUEUE_LENGTH, &RosInterface::devCallback, this);
    deviation_vel_sub_ = node.subscribe(DEVIATION_VELOCITY_TOPIC, QUEUE_LENGTH, &RosInterface::devVelCallback, this);
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

void RosInterface::notstop(){
    std::cout << std::endl << "GUI: NOTSTOP !!!!!!!!" << std::endl;
    std_msgs::Bool notstop;
    notstop.data = true;
    notstop_pub.publish(notstop);
}

void RosInterface::velCallback(const arc_msgs::State::ConstPtr& msg){
    QMutex * pMutex = new QMutex();
    pMutex->lock();
    Eigen::Vector3d velocity = arc_tools::transformVectorMessageToEigen(msg->pose_diff.twist.linear);
    pMutex->unlock();
    delete pMutex;
    Q_EMIT newVel(velocity(0), velocity(1), velocity(2));
}

void RosInterface::devCallback(const std_msgs::Float64::ConstPtr& msg){
    QMutex * pMutex = new QMutex();
    pMutex->lock();
    double deviation = msg->data;
    double relative = deviation/MAX_DEVIATION_FROM_TEACH_PATH;
    pMutex->unlock();
    delete pMutex;
    Q_EMIT newDev(deviation, relative);
}

void RosInterface::devVelCallback(const std_msgs::Float64::ConstPtr& msg){
    QMutex * pMutex = new QMutex();
    pMutex->lock();
    double vel_deviation = msg->data;
    pMutex->unlock();
    delete pMutex;
    Q_EMIT newVelDev(vel_deviation);
}

