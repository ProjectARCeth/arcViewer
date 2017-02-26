#include "ControlWindow.hpp"

ControlWindow::ControlWindow(int argc, char **argv, QWidget *parent)
    : QWidget(parent),
      RosInterface_(argc, argv)
{
    //Init layouts.
    leftLayout = new QVBoxLayout();
    rightLayout = new QVBoxLayout();
    mainLayout = new QHBoxLayout();
    //Adding velocity labels.
    setVelocityDisplay();
    //Adding deviation from path and velocity labels.
    setDeviationDisplay();
    //Stop Button.
    setStopButton();
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
    // connect(stop_button_,  SIGNAL(clicked()), this, SLOT(&RosInterface::notstop()));
    connect(stop_button_, &QPushButton::clicked, this, &ControlWindow::notstop);
    //Init subscriber.
    RosInterface_.init();
}

void ControlWindow::notstop(){
    RosInterface_.notstop();
}

void ControlWindow::updateVelDisplay(double x, double y, double z){
    QString xVel, yVel, zVel;
    xVel.setNum(x);
    yVel.setNum(y);
    zVel.setNum(z);
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

void ControlWindow::setVelocityDisplay(){
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
    vel_dev_label->setText(tr("Relative Deviation:"));
    vel_dev_display_ = new QLineEdit();
    vel_dev_display_->setText(tr("0.0"));
    vel_dev_layout->addWidget(vel_dev_label);
    vel_dev_layout->addWidget(vel_dev_display_);
    //Adding to layout.
    leftLayout->addLayout(dev_layout);
    leftLayout->addLayout(rel_layout);
    leftLayout->addLayout(vel_dev_layout);
}

void ControlWindow::setStopButton(){
    stop_button_ = new QPushButton();
    stop_button_->setIcon(QIcon(":/images/stop.png"));
    stop_button_->setIconSize(QSize(50, 50));
    QHBoxLayout *stop_layout = new QHBoxLayout();
    stop_layout->addWidget(stop_button_);
    rightLayout->addLayout(stop_layout);
}