#include "ControlWindow.hpp"

ControlWindow::ControlWindow(int argc, char **argv, QWidget *parent)
    : QWidget(parent),
      RosInterface_(argc, argv)
{
    //Init ROSInterface..
    RosInterface_.init();
    INIT_MODE = RosInterface_.getInitMode(); //O: Teach, 1: Repeat.
    //Getting launching programs.
    USE_GPS = true;
    if(strlen(*(argv + 2)) == 5) USE_GPS = false;
    USE_VI = true;
    if(strlen(*(argv + 3)) == 5) USE_VI = false;
    USE_VELODYNE = true;
    if(strlen(*(argv + 4)) == 5) USE_VELODYNE = false;
    USE_NI_CLIENT = true;
    if(strlen(*(argv + 5)) == 5) USE_NI_CLIENT = false;
    USE_STATE_ESTIMATION = true;
    if(strlen(*(argv + 6)) == 5) USE_STATE_ESTIMATION = false;
    USE_OBSTACLE_DETECTION = true;
    if(strlen(*(argv + 7)) == 5) USE_OBSTACLE_DETECTION = false;
    USE_GUARD = true;
    if(strlen(*(argv + 8)) == 5) USE_GUARD = false;
    USE_CONTROLLING = true;
    if(strlen(*(argv + 9)) == 5) USE_CONTROLLING = false;
    //Init big layouts.
    leftLayout = new QVBoxLayout();
    rightLayout = new QVBoxLayout();
    lowerLayout = new QHBoxLayout();
    upperLayout = new QHBoxLayout();
    mainLayout = new QGridLayout();
    //Init small layouts.
    abs_vel_display_ = new QLineEdit();
    x_pose_display_ = new QLineEdit();
    y_pose_display_ = new QLineEdit();
    array_pose_display_ = new QLineEdit();
    dev_display_ = new QLineEdit();
    vel_dev_display_ = new QLineEdit();
    obstacle_dis_display_ = new QLineEdit();
    distance_start_display_ = new QLineEdit();
    distance_end_display_ = new QLineEdit();
    steering_reference_index_display_ = new QLineEdit();
    steering_should_display_ = new QLineEdit();
    radius_reference_index_display_ = new QLineEdit();
    radius_path_display_ = new QLineEdit();
    velocity_bound_physical_display_ = new QLineEdit();
    velocity_bound_teach_display_ = new QLineEdit();
    velocity_should_display_ = new QLineEdit();
    braking_distance_display_ = new QLineEdit();
    steering_ist_display_ = new QLineEdit();
    wheel_left_display_ = new QLineEdit();
    wheel_right_display_ = new QLineEdit();
    mode_display_ = new QLineEdit();
    //Init checkboxes for launched programs.
    controlling_box_ = new QRadioButton();
    gps_box_ = new QRadioButton();
    guard_box_ = new QRadioButton();
    obstacle_detection_box_ = new QRadioButton();
    orbslam_box_ = new QRadioButton();
    ni_client_box_ = new QRadioButton();
    rovio_box_ = new QRadioButton();
    state_estimation_box_ = new QRadioButton();
    velodyne_box_ = new QRadioButton();
    vi_box_ = new QRadioButton();
    //Building up interface.
    buildInterface(INIT_MODE);
    //Merging main layout.
    upperLayout->addLayout(leftLayout);
    upperLayout->addLayout(rightLayout);
    mainLayout->addLayout(upperLayout, 0, 0);
    mainLayout->addLayout(lowerLayout, 1, 0);
    setLayout(mainLayout);
    //Init window.
    show();
    setWindowTitle(tr("Control Window"));
    //Connecting actions.
    connect(&RosInterface_, &RosInterface::newDev, this, &ControlWindow::updateDevDisplay);
    connect(&RosInterface_, &RosInterface::newLaunchCommand, this, &ControlWindow::setLaunchable);
    connect(&RosInterface_, &RosInterface::newLaunching1, this, &ControlWindow::updateLaunchedProgrammes1);
    connect(&RosInterface_, &RosInterface::newLaunching2, this, &ControlWindow::updateLaunchedProgrammes2);
    connect(&RosInterface_, &RosInterface::newObstacleDis, this, &ControlWindow::updateObstacleDisDisplay);
    connect(&RosInterface_, &RosInterface::newNotstop, this, &ControlWindow::popUpNotstop);
    connect(&RosInterface_, &RosInterface::newPathInfo, this, &ControlWindow::updatePathInfoDisplay);
    connect(&RosInterface_, &RosInterface::newState, this, &ControlWindow::updateStateDisplay);
    connect(&RosInterface_, &RosInterface::newSteering, this, &ControlWindow::updateSteeringDisplay);
    connect(&RosInterface_, &RosInterface::newStellgroessen, this, &ControlWindow::updateStellgroessenDisplay);
    connect(&RosInterface_, &RosInterface::newVelDev, this, &ControlWindow::updateVelDevDisplay);
    connect(&RosInterface_, &RosInterface::newVelInfo, this, &ControlWindow::updateVelInfoDisplay);
    connect(&RosInterface_, &RosInterface::newWheelLeft, this, &ControlWindow::updateWheelLeftDisplay);
    connect(&RosInterface_, &RosInterface::newWheelRight, this, &ControlWindow::updateWheelRightDisplay);
    connect(launch_button_, &QPushButton::clicked, this, &ControlWindow::converting);
    connect(shutdown_button_, &QPushButton::clicked, this, &ControlWindow::shutdown);
    connect(stop_button_, &QPushButton::clicked, this, &ControlWindow::notstop);
}

void ControlWindow::buildInterface(bool mode){
    //Delete current interface.
    deleteWidgets();
    //Create color.
    QColor black = Qt::black;
    QColor blue =  Qt::blue;
    QColor cyan = Qt::cyan;
    QColor green = Qt::green;
    QColor magenta = Qt::magenta;
    QColor red = Qt::red;
    QColor white = Qt::white;
    QColor yellow = Qt::yellow;
    //Adding mode.
    setModeDisplay(mode);
    //Adding labels.
    setVelocityDisplay();
    if(mode){
        setUpDisplay(velocity_should_display_, "Vel should", leftLayout, green, black);
        setUpDisplay(steering_should_display_, "Steering should", leftLayout, green, black);
    }
    setUpDisplay(steering_ist_display_, "Steering ist", leftLayout, cyan, black);
    setUpDisplay(wheel_left_display_, "Wheel left", leftLayout, cyan, black);
    setUpDisplay(wheel_right_display_, "Wheel right", leftLayout, cyan, black); 
    setUpDisplay(x_pose_display_, "X Position", leftLayout, cyan, black);
    setUpDisplay(y_pose_display_, "Y Position", leftLayout, cyan, black);
    setUpDisplay(array_pose_display_, "Array Position", leftLayout, cyan, black); 
    if(mode){
        setUpDisplay(radius_reference_index_display_, "Radius ref", leftLayout, magenta, black);
        setUpDisplay(steering_reference_index_display_, "Steering ref", leftLayout, magenta, black);
        setUpDisplay(radius_path_display_, "Radius path", leftLayout, magenta, black);
        setUpDisplay(velocity_bound_physical_display_, "Vel bound phys", leftLayout, magenta, black);
        setUpDisplay(velocity_bound_teach_display_, "Vel bound teach", leftLayout, magenta, black);
    }           
    if(mode){
        setUpDisplay(distance_start_display_, "Distance start", rightLayout, yellow, black);
        setUpDisplay(distance_end_display_, "Distance end", rightLayout, yellow, black);
    }
    if(mode){
        setUpDisplay(braking_distance_display_, "Braking distance", rightLayout, red, black);
        setUpDisplay(obstacle_dis_display_, "Obstacle distance", rightLayout, red, black);
        setUpDisplay(dev_display_, "Path deviation", rightLayout, red, black);
        setUpDisplay(vel_dev_display_, "Vel deviation", rightLayout, red, black);
    }
    setShutdownButton();
    setStopButton();
    setLaunchButton();
    setLaunchingProgrammsDisplay();
}

void ControlWindow::checkAndChangeBox(QRadioButton *button){
    button->setChecked(true);
    QString style_string("background: green;"); 
    button->setStyleSheet(style_string);
}

void ControlWindow::converting(){
    //Launch system only one time.
    if(system_launched_){
        RosInterface_.launching();
        system_launched_ = false;
        //Adapt to manuell button.
        QString style_string("background: blue;"); 
        launch_button_->setStyleSheet(style_string);
        if(INIT_MODE){
            autonomous_mode_ = true;
            launch_button_->setText(tr("&Manuell Mode"));
        }
        else launch_button_->setText(tr("&System started"));
    }
    //Change to manuell iff autonomous.
    if(autonomous_mode_ && INIT_MODE){
        RosInterface_.changeToManuell();
        autonomous_mode_ = false;
        //Reset to launchable mode.
        system_launched_ = true;
    }
}

void ControlWindow::deleteWidgets(){
    QLayoutItem* item;
    while ( ( item = mainLayout->takeAt( 0 ) ) != NULL ){
        delete item->widget();
        delete item;
    }
}

void ControlWindow::notstop(){
    RosInterface_.notstop();
}

void ControlWindow::popUpNotstop(){
    QMessageBox::warning(this, "CAUTION", "CAUTION. Emergency STOP !");
}

void ControlWindow::setLaunchable(){
    //Change launch button on green color and change text.
    QString style_string("background: green;"); 
    launch_button_->setStyleSheet(style_string);
    launch_button_->setText(tr("&Ready !"));
    launch_button_->update();
    //Permit system launch.
    system_launched_ = true;
}

void ControlWindow::shutdown(){
    RosInterface_.shutdown();
}

void ControlWindow::updateDevDisplay(double deviation){
    QString dev;
    dev.setNum(deviation);
    dev_display_->setText(dev);
}

void ControlWindow::updateLaunchedProgrammes1(bool gps, bool vi, bool velodyne, bool rovio,
                                              bool state_estimation, bool orbslam){
    if(USE_GPS && gps) checkAndChangeBox(gps_box_);
    if(USE_STATE_ESTIMATION && rovio) checkAndChangeBox(rovio_box_);
    if(USE_STATE_ESTIMATION && state_estimation) checkAndChangeBox(state_estimation_box_);
    if(USE_STATE_ESTIMATION && orbslam) checkAndChangeBox(orbslam_box_);
    if(USE_VELODYNE && velodyne) checkAndChangeBox(velodyne_box_);
    if(USE_VI && vi) checkAndChangeBox(vi_box_);
}

void ControlWindow::updateLaunchedProgrammes2(bool controlling, bool ni_client, bool obstacle_detection,
                                             bool guard){
    if(USE_CONTROLLING && controlling) checkAndChangeBox(controlling_box_);
    if(USE_NI_CLIENT && ni_client) checkAndChangeBox(ni_client_box_);
    if(USE_OBSTACLE_DETECTION && obstacle_detection) checkAndChangeBox(obstacle_detection_box_);
    if(USE_GUARD && guard) checkAndChangeBox(guard_box_);
}

void ControlWindow::updateVelDevDisplay(double deviation){
    QString vel_dev;
    vel_dev.setNum(deviation);
    vel_dev_display_->setText(vel_dev);
}

void ControlWindow::updateObstacleDisDisplay(double distance){
    QString obs_dis;
    obs_dis.setNum(distance);
    obstacle_dis_display_->setText(obs_dis);
}

void ControlWindow::updatePathInfoDisplay(float dis_begin, float dis_end, float radius, float ref_steering){
    QString dis_begin_string, dis_end_string, radius_string, ref_steering_string;
    dis_begin_string.setNum(dis_begin);
    dis_end_string.setNum(dis_end);
    radius_string.setNum(radius);
    ref_steering_string.setNum(ref_steering);
    distance_start_display_->setText(dis_begin_string);
    distance_end_display_->setText(dis_end_string);
    radius_path_display_->setText(radius_string);
    steering_reference_index_display_->setText(ref_steering_string);
}

void ControlWindow::updateStateDisplay(double x, double y, double velocity, int array_position){
    QString x_string, y_string, vel_string, array_pose_string;
    x_string.setNum(x);
    y_string.setNum(y);
    vel_string.setNum(velocity);
    array_pose_string.setNum(array_position);
    x_pose_display_->setText(x_string);
    y_pose_display_->setText(y_string);
    abs_vel_display_->setText(vel_string);
    array_pose_display_->setText(array_pose_string);
}

void ControlWindow::updateSteeringDisplay(double angle){
    QString steering_dis;
    steering_dis.setNum(angle);
    steering_ist_display_->setText(steering_dis);
}

void ControlWindow::updateStellgroessenDisplay(double vel_should, double steering_should){
    QString vel_should_string, steering_should_string;
    vel_should_string.setNum(vel_should);
    steering_should_string.setNum(steering_should);
    velocity_should_display_->setText(vel_should_string);
    steering_should_display_->setText(steering_should_string);
}

void ControlWindow::updateVelInfoDisplay(float ref_vel, float bound_phys, float braking_dis, float bound_teach){
    QString ref_vel_string, bound_phys_string, braking_dis_string, bound_teach_string;
    ref_vel_string.setNum(ref_vel);
    bound_phys_string.setNum(bound_phys);
    braking_dis_string.setNum(braking_dis);
    bound_teach_string.setNum(bound_teach);
    radius_reference_index_display_->setText(ref_vel_string);
    velocity_bound_physical_display_->setText(bound_phys_string);
    braking_distance_display_->setText(braking_dis_string);
    velocity_bound_teach_display_->setText(bound_teach_string);
}

void ControlWindow::updateWheelLeftDisplay(double wheel_left){
    QString leftVel;
    leftVel.setNum(wheel_left);
    wheel_left_display_->setText(leftVel);
}

void ControlWindow::updateWheelRightDisplay(double wheel_right){
    QString rightVel;
    rightVel.setNum(wheel_right);
    wheel_right_display_->setText(rightVel);
}

void ControlWindow::setLaunchButton(){
    launch_button_ = new QPushButton(tr("&System Booting"));
    launch_button_->setMinimumHeight(60);
    QString style_string("background: orange;"); 
    launch_button_->setStyleSheet(style_string);
    QHBoxLayout *launching_layout = new QHBoxLayout();
    launching_layout->addWidget(launch_button_);
    rightLayout->addLayout(launching_layout);
    //Reset launchable and autonomous.
    system_launched_ = false;
    autonomous_mode_ = false;
}

void ControlWindow::setLaunchingProgrammsDisplay(){
   if(USE_CONTROLLING && INIT_MODE) setUpLaunchableProgrammBox(controlling_box_, "CONTROLLER");
   if(USE_GPS) setUpLaunchableProgrammBox(gps_box_, "GPS");
   if(USE_NI_CLIENT) setUpLaunchableProgrammBox(ni_client_box_, "NI");
   if(USE_OBSTACLE_DETECTION && INIT_MODE) setUpLaunchableProgrammBox(obstacle_detection_box_, "OBSTACLES");
   if(USE_GUARD && INIT_MODE) setUpLaunchableProgrammBox(guard_box_, "GUARD");
   if(USE_STATE_ESTIMATION){
    setUpLaunchableProgrammBox(rovio_box_, "ROVIO");
    setUpLaunchableProgrammBox(state_estimation_box_, "SE");
    setUpLaunchableProgrammBox(orbslam_box_, "RSLAM");
   }
   if(USE_VELODYNE) setUpLaunchableProgrammBox(velodyne_box_, "LASER");
   if(USE_VI) setUpLaunchableProgrammBox(vi_box_, "VI");
}

void ControlWindow::setModeDisplay(bool mode){
    QHBoxLayout *mode_layout = new QHBoxLayout();
    mode_display_ = new QLineEdit();
    QPalette palette;
    palette.setColor(QPalette::Base,Qt::black);
    palette.setColor(QPalette::Text,Qt::white);
    mode_display_->setPalette(palette);
    if (!mode) mode_display_->setText(tr("TEACH"));
    else mode_display_->setText(tr("REPEAT"));
    mode_layout->addWidget(mode_display_);
    //Adding to layout.
    rightLayout->addLayout(mode_layout);
}

void ControlWindow::setStopButton(){
    stop_button_ = new QPushButton();
    stop_button_->setIcon(QIcon(":/images/stop.png"));
    stop_button_->setIconSize(QSize(50, 50));
    QHBoxLayout *stop_layout = new QHBoxLayout();
    stop_layout->addWidget(stop_button_);
    rightLayout->addLayout(stop_layout);
}

void ControlWindow::setShutdownButton(){
    shutdown_button_ = new QPushButton(tr("&Controlled Shutdown"));
    shutdown_button_->setIconSize(QSize(50, 50));
    QHBoxLayout *shutdown_layout = new QHBoxLayout();
    shutdown_layout->addWidget(shutdown_button_);
    rightLayout->addLayout(shutdown_layout);
}

void ControlWindow::setUpDisplay(QLineEdit *display, std::string info, QVBoxLayout *bigLayout){
    QHBoxLayout *layout = new QHBoxLayout();
    QLabel *label = new QLabel();
    QString label_string = QString::fromStdString(info + ":");
    label->setText(label_string);
    display->setText(tr("0.0"));
    layout->addWidget(label);
    layout->addWidget(display);
    bigLayout->addLayout(layout);
}

void ControlWindow::setUpDisplay(QLineEdit *display, std::string info, QVBoxLayout *bigLayout, 
                                 QColor &base_color, QColor &text_color){
    QHBoxLayout *layout = new QHBoxLayout();
    QLabel *label = new QLabel();
    QPalette palette;
    palette.setColor(QPalette::Base,base_color);
    palette.setColor(QPalette::Text,text_color);
    display->setPalette(palette);
    display->setText(tr("0.0"));
    QString label_string = QString::fromStdString(info + ":");
    label->setText(label_string);
    layout->addWidget(label);
    layout->addWidget(display);
    bigLayout->addLayout(layout);
}

void ControlWindow::setUpLaunchableProgrammBox(QRadioButton *button, std::string name){
    QVBoxLayout *layout = new QVBoxLayout();
    QLabel *label = new QLabel();
    QString label_string = QString::fromStdString(name);
    label->setText(label_string);
    QString style_string("background: red;"); 
    button->setStyleSheet(style_string);
    button->setCheckable(false);
    button->setAutoExclusive(false);
    layout->addWidget(button);
    layout->addWidget(label);
    layout->setAlignment(Qt::AlignHCenter);
    lowerLayout->addLayout(layout); 
}

void ControlWindow::setVelocityDisplay(){
    //Set up the Velocity Display - absolute Value.
    QHBoxLayout *abs_layout = new QHBoxLayout();
    QPalette palette;
    palette.setColor(QPalette::Base,Qt::black);
    palette.setColor(QPalette::Text,Qt::green);
    abs_vel_display_->setPalette(palette);
    QFont sansFont("Helvetica [Cronyx]", 36);
    abs_vel_display_->setFont(sansFont);
    abs_vel_display_->setText(tr("0.0"));
    abs_layout->addWidget(abs_vel_display_);
    //Adding to layout.
    leftLayout->addLayout(abs_layout);
}