#include "ControlWindow.hpp"

ControlWindow::ControlWindow(int argc, char **argv, QWidget *parent)
    : QWidget(parent),
      m_RosInterface(argc, argv, "/rovio/odometry")
{
    //Closing Button.
    p_quitButton = new QPushButton("&Quit");
    //Set up the Position Display - X.
    p_xLayout = new QHBoxLayout();
    p_xLabel = new QLabel();
    p_xLabel->setText(tr("X:"));
    p_xDisplay = new QLineEdit();
    p_xDisplay->setText(tr("0.0"));
    p_xLayout->addWidget(p_xLabel);
    p_xLayout->addWidget(p_xDisplay);
    //Set up the Position Display - Y.
    p_yLayout = new QHBoxLayout();
    p_yLabel = new QLabel();
    p_yLabel->setText(tr("Y:"));
    p_yDisplay = new QLineEdit();
    p_yDisplay->setText(tr("0.0"));
    p_yLayout->addWidget(p_yLabel);
    p_yLayout->addWidget(p_yDisplay);
    //Set up the Position Display - Z.
    p_zLayout = new QHBoxLayout();
    p_zLabel = new QLabel();
    p_zLabel->setText(tr("Z: "));
    p_zDisplay = new QLineEdit();
    p_zDisplay->setText(tr("0.0"));
    p_zLayout->addWidget(p_zLabel);
    p_zLayout->addWidget(p_zDisplay);
    //Creating display layout.
    leftLayout = new QVBoxLayout();
    leftLayout->addLayout(p_xLayout);
    leftLayout->addLayout(p_yLayout);
    leftLayout->addLayout(p_zLayout);
    leftLayout->addWidget(p_quitButton, 6);
    //Merging main layout.
    mainLayout = new QHBoxLayout();
    mainLayout->addLayout(leftLayout);
    setLayout(mainLayout);
    //Init window.
    show();
    setWindowTitle(tr("Control Window"));
    //Connecting actions.
    connect(p_quitButton,  &QPushButton::clicked, this, &ControlWindow::close);
    connect(&m_RosInterface, &RosInterface::newPose, this, &ControlWindow::updatePoseDisplay);
    //Init subscriber.
    m_RosInterface.init();
}

void ControlWindow::updatePoseDisplay(double x, double y, double z){
    QString xPose, yPose, zPose;
    xPose.setNum(x);
    yPose.setNum(y);
    zPose.setNum(z);
    p_xDisplay->setText(xPose);
    p_yDisplay->setText(yPose);
    p_zDisplay->setText(zPose);
}
