#include "RosInterface.hpp"

RosInterface::RosInterface(int argc, char **pArgv, const char *topic_pose){
    m_Init_argc = argc;
    m_pInit_argv = pArgv;
    m_topic_pose = topic_pose;
}

RosInterface::~RosInterface(){
    if (ros::isStarted()){
        ros::shutdown();
        ros::waitForShutdown();
    }
    m_pThread->wait();
}

bool RosInterface::init(){
    //Init thread.
    m_pThread = new QThread();
    this->moveToThread(m_pThread);
    connect(m_pThread, &QThread::started, this, &RosInterface::run);
    //Init ros.
    ros::init(m_Init_argc, m_pInit_argv, "arc_gui");
    if (!ros::master::check()) return false;
    ros::start();
    ros::Time::init();
    ros::NodeHandle nh;
    //Init publisher and subscriber.
    pose_listener = nh.subscribe(m_topic_pose, 10, &RosInterface::poseCallback, this);
    //Start thread.
    m_pThread->start();
    return true;
}

void RosInterface::poseCallback(const nav_msgs::Odometry & msg){
    QMutex * pMutex = new QMutex();
    pMutex->lock();
    m_xPos = msg.pose.pose.position.x;
    m_yPos = msg.pose.pose.position.y;
    m_zPos = msg.pose.pose.position.z;
    pMutex->unlock();
    delete pMutex;
    Q_EMIT newPose(m_xPos, m_yPos, m_zPos);
}

void RosInterface::run(){
    ros::Rate loop_rate(100);
    while (ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }
}
