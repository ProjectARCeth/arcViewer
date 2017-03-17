#include "ControlWindow.hpp"

ControlWindow::ControlWindow(int argc, char **argv, QWidget *parent)
    : QWidget(parent),
      RosInterface_(argc, argv)
{
    //Init ROSInterface..
    RosInterface_.init();
    bool mode = RosInterface_.getInitMode(); //O: Teach, 1: Repeat.
    //Init big layouts.
    leftLayout = new QVBoxLayout();
    rightLayout = new QVBoxLayout();
    mainLayout = new QHBoxLayout();
    //Init small layouts.
    abs_vel_display_ = new QLineEdit();
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
    mode_display_ = new QLineEdit();
    //Building up interface.
    buildInterface(mode);
    //Merging main layout.
    mainLayout->addLayout(leftLayout);
    mainLayout->addLayout(rightLayout);
    setLayout(mainLayout);
    //Init window.
    show();
    setWindowTitle(tr("Control Window"));
    //Connecting actions.
    connect(&RosInterface_, &RosInterface::newVel, this, &ControlWindow::updateVelDisplay);
    connect(&RosInterface_, &RosInterface::newDev, this, &ControlWindow::updateDevDisplay);
    connect(&RosInterface_, &RosInterface::newLaunchCommand, this, &ControlWindow::setLaunchable);
    connect(&RosInterface_, &RosInterface::newObstacleDis, this, &ControlWindow::updateObstacleDisDisplay);
    connect(&RosInterface_, &RosInterface::newNotstop, this, &ControlWindow::popUpNotstop);
    connect(&RosInterface_, &RosInterface::newPurePursuitInfo, this, &ControlWindow::updatePurePursuitDisplay);
    connect(&RosInterface_, &RosInterface::newSteering, this, &ControlWindow::updateSteeringDisplay);
    connect(&RosInterface_, &RosInterface::newVelDev, this, &ControlWindow::updateVelDevDisplay);
    connect(stop_button_, &QPushButton::clicked, this, &ControlWindow::notstop);
    connect(shutdown_button_, &QPushButton::clicked, this, &ControlWindow::shutdown);
    connect(launch_button_, &QPushButton::clicked, this, &ControlWindow::launching);
}

void ControlWindow::buildInterface(bool mode){
    //Delete current interface.
    deleteWidgets();
    //Adding modem shutdown and notstop.
    setModeDisplay(mode);
    setShutdownButton();
    setStopButton();
    //Adding labels.
    setVelocityDisplay();
    if(mode) setUpDisplay(velocity_should_display_, "Vel should", leftLayout);
    setUpDisplay(steering_ist_display_, "Steering ist", leftLayout);
    if(mode){
        setUpDisplay(steering_should_display_, "Steering should", leftLayout);
        setUpDisplay(steering_reference_index_display_, "Steering ref", leftLayout);
    }    
    if(mode){
        setUpDisplay(radius_reference_index_display_, "Radius ref", leftLayout);
        setUpDisplay(radius_path_display_, "Radius path", leftLayout);
        setUpDisplay(velocity_bound_physical_display_, "Vel bound phys", leftLayout);
        setUpDisplay(velocity_bound_teach_display_, "Vel bound teach", leftLayout);
    }           
    if(mode){
        setUpDisplay(distance_start_display_, "Distance start", leftLayout);
        setUpDisplay(distance_end_display_, "Distance end", leftLayout);
    }
    if(mode){
        setUpDisplay(braking_distance_display_, "Braking distance", rightLayout);
        setUpDisplay(obstacle_dis_display_, "Obstacle distance", rightLayout);
        setUpDisplay(dev_display_, "Path deviation", rightLayout);
        setUpDisplay(vel_dev_display_, "Vel deviation", rightLayout);
    }
    setLaunchButton();
}

void ControlWindow::deleteWidgets(){
    QLayoutItem* item;
    while ( ( item = mainLayout->takeAt( 0 ) ) != NULL ){
        delete item->widget();
        delete item;
    }
}

void ControlWindow::launching(){
    //Launch system only one time.
    if(system_launched_){
        RosInterface_.launching();
        system_launched_ = false;
    }
}

void ControlWindow::notstop(){
    RosInterface_.notstop();
}

void ControlWindow::popUpNotstop(){
    QMessageBox::warning(this, "ACHTUNG NOTSTOP", "NOTSTOP! Festhalten.");
}

void ControlWindow::setLaunchable(bool mode){
    //Change launch button on green color and change text.
    QPalette palette;
    palette.setColor(QPalette::Button,Qt::green);
    launch_button_->setPalette(palette);
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

void ControlWindow::updatePurePursuitDisplay(std::vector <double> infos){
    QString dis_start, dis_end, steering_ref, steering_should, radius_ref, radius_path,
            braking_dis, bound_phys, bound_teach, vel_should;
    dis_start.setNum(infos[0]);
    dis_end.setNum(infos[1]);
    steering_ref.setNum(infos[2]);
    steering_should.setNum(infos[3]);
    radius_ref.setNum(infos[4]);
    radius_path.setNum(infos[5]);
    bound_phys.setNum(infos[6]);
    braking_dis.setNum(infos[7]);
    bound_teach.setNum(infos[8]);
    vel_should.setNum(infos[9]);    
    distance_start_display_->setText(dis_start);
    distance_end_display_->setText(dis_end);
    steering_reference_index_display_->setText(steering_ref);
    steering_should_display_->setText(steering_should);
    radius_reference_index_display_->setText(radius_ref);
    radius_path_display_->setText(radius_path);
    velocity_bound_physical_display_->setText(bound_phys);
    braking_distance_display_->setText(braking_dis);
    velocity_bound_teach_display_->setText(bound_teach);
    velocity_should_display_->setText(vel_should);
}

void ControlWindow::updateSteeringDisplay(double angle){
    QString steering_dis;
    steering_dis.setNum(angle);
    steering_ist_display_->setText(steering_dis);
}

void ControlWindow::updateVelDisplay(double velocity){
    QString absVel;
    absVel.setNum(velocity);
    abs_vel_display_->setText(absVel);
}

void ControlWindow::setLaunchButton(){
    launch_button_ = new QPushButton(tr("&System Booting"));
    launch_button_->setIconSize(QSize(50, 50));
    QPalette palette;
    palette.setColor(QPalette::Button,Qt::red);
    launch_button_->setPalette(palette);
    QHBoxLayout *launching_layout = new QHBoxLayout();
    launching_layout->addWidget(launch_button_);
    rightLayout->addLayout(launching_layout);
    //Reset launchable.
    system_launched_ = false;
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