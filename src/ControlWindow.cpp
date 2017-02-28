#include "ControlWindow.hpp"

ControlWindow::ControlWindow(int argc, char **argv, QWidget *parent)
    : QWidget(parent),
      RosInterface_(argc, argv)
{
    //Init ROSInterface..
    RosInterface_.init();
    bool mode = RosInterface_.getInitMode(); //O: Teach, 1: Repeat.
    //Init layouts.
    leftLayout = new QVBoxLayout();
    rightLayout = new QVBoxLayout();
    mainLayout = new QHBoxLayout();
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
    connect(&RosInterface_, &RosInterface::newVelDev, this, &ControlWindow::updateVelDevDisplay);
    connect(&RosInterface_, &RosInterface::newObstacleDis, this, &ControlWindow::updateObstacleDisDisplay);
    connect(&RosInterface_, &RosInterface::newMode, this, &ControlWindow::buildInterface);
    connect(&RosInterface_, &RosInterface::newNotstop, this, &ControlWindow::popUpNotstop);
    connect(stop_button_, &QPushButton::clicked, this, &ControlWindow::notstop);
    connect(shutdown_button_, &QPushButton::clicked, this, &ControlWindow::shutdown);
}

void ControlWindow::buildInterface(bool mode){
    //Delete current interface.
    deleteWidgets();
    //Adding velocity labels.
    setVelocityDisplay();
    //Adding distance to obstacle.
    if(mode) setObstacleDistanceDisplay();
    //Adding mode label.
    setModeDisplay(mode);
    //Shutdown Button.
    setShutdownButton();
    //Stop Button.
    setStopButton();
    //Deviation only in repeat mode.
    if(mode) setDeviationDisplay();
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
    QMessageBox::warning(this, "ACHTUNG NOTSTOP", "NOTSTOP! Festhalten.");
}

void ControlWindow::shutdown(){
    RosInterface_.shutdown();
}

void ControlWindow::updateVelDisplay(double x, double y, double z){
    QString absVel, xVel, yVel, zVel;
    absVel.setNum(sqrt(x*x + y*y + z*z));
    xVel.setNum(x);
    yVel.setNum(y);
    zVel.setNum(z);
    abs_vel_display_->setText(absVel);
    x_vel_display_->setText(xVel);
    y_vel_display_->setText(yVel);
    z_vel_display_->setText(zVel);
}

void ControlWindow::updateDevDisplay(double deviation, double relative){
    QString dev, rel;
    dev.setNum(deviation);
    rel.setNum(relative);
    dev_display_->setText(dev);
    rel_dev_display_->setText(rel);
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

void ControlWindow::setVelocityDisplay(){
    //Set up the Velocity Display - absolute Value.
    QHBoxLayout *abs_layout = new QHBoxLayout();
    abs_vel_display_ = new QLineEdit();
    QPalette palette;
    palette.setColor(QPalette::Base,Qt::black);
    palette.setColor(QPalette::Text,Qt::green);
    abs_vel_display_->setPalette(palette);
    QFont sansFont("Helvetica [Cronyx]", 36);
    abs_vel_display_->setFont(sansFont);
    abs_vel_display_->setText(tr("0.0"));
    abs_layout->addWidget(abs_vel_display_);
    //Set up the Velocity Display - X.
    QHBoxLayout *x_layout = new QHBoxLayout();
    QLabel *x_label = new QLabel();
    x_label->setText(tr("V_x:"));
    x_vel_display_ = new QLineEdit();
    x_vel_display_->setText(tr("0.0"));
    x_layout->addWidget(x_label);
    x_layout->addWidget(x_vel_display_);
    //Set up the Velocity Display - Y.
    QHBoxLayout *y_layout = new QHBoxLayout();
    QLabel *y_label = new QLabel();
    y_label->setText(tr("V_y:"));
    y_vel_display_ = new QLineEdit();
    y_vel_display_->setText(tr("0.0"));
    y_layout->addWidget(y_label);
    y_layout->addWidget(y_vel_display_);
    //Set up the Velocity Display - Z.
    QHBoxLayout *z_layout = new QHBoxLayout();
    QLabel *z_label = new QLabel();
    z_label->setText(tr("V_z:"));
    z_vel_display_ = new QLineEdit();
    z_vel_display_->setText(tr("0.0"));
    z_layout->addWidget(z_label);
    z_layout->addWidget(z_vel_display_);
    //Adding to layout.
    leftLayout->addLayout(abs_layout);
    leftLayout->addLayout(x_layout);
    leftLayout->addLayout(y_layout);
    leftLayout->addLayout(z_layout);
}

void ControlWindow::setDeviationDisplay(){
    //Set up deviation from path display.
    QHBoxLayout *dev_layout = new QHBoxLayout();
    QLabel *dev_label = new QLabel();
    dev_label->setText(tr("Deviation:"));
    dev_display_ = new QLineEdit();
    dev_display_->setText(tr("0.0"));
    dev_layout->addWidget(dev_label);
    dev_layout->addWidget(dev_display_);
    //Set up relative deviation display.
    QHBoxLayout *rel_layout = new QHBoxLayout();
    QLabel *rel_label = new QLabel();
    rel_label->setText(tr("Relative Deviation:"));
    rel_dev_display_ = new QLineEdit();
    rel_dev_display_->setText(tr("0.0"));
    rel_layout->addWidget(rel_label);
    rel_layout->addWidget(rel_dev_display_);
    //Set up velocity deviation.
    QHBoxLayout *vel_dev_layout = new QHBoxLayout();
    QLabel *vel_dev_label = new QLabel();
    vel_dev_label->setText(tr("Velocity Deviation:"));
    vel_dev_display_ = new QLineEdit();
    vel_dev_display_->setText(tr("0.0"));
    vel_dev_layout->addWidget(vel_dev_label);
    vel_dev_layout->addWidget(vel_dev_display_);
    //Adding to layout.
    leftLayout->addLayout(dev_layout);
    leftLayout->addLayout(rel_layout);
    leftLayout->addLayout(vel_dev_layout);
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

void ControlWindow::setObstacleDistanceDisplay(){
    QHBoxLayout *obs_dis_layout = new QHBoxLayout();
    QLabel *obs_dis_label = new QLabel();
    obs_dis_label->setText(tr("Distance to obstacle:"));
    obstacle_dis_display_ = new QLineEdit();
    obstacle_dis_display_->setText(tr("0.0"));
    obs_dis_layout->addWidget(obs_dis_label);
    obs_dis_layout->addWidget(obstacle_dis_display_);
    //Adding to layout.
    leftLayout->addLayout(obs_dis_layout);
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