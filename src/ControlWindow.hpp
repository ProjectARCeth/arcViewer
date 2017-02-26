#ifndef CONTROL_WINDOW_H
#define CONTROL_WINDOW_H

#include <iostream>

#include <QIcon>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QString>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QTextStream>
#include <QtWidgets>
#include "RosInterface.hpp"

class ControlWindow : public QWidget
{
    Q_OBJECT

public:
    ControlWindow(int argc, char** argv, QWidget * parent = 0);
    Q_SLOT void notstop();
    Q_SLOT void updateVelDisplay(double x, double y, double z);
    Q_SLOT void updateDevDisplay(double deviation, double relative);
    Q_SLOT void updateVelDevDisplay(double deviation);

private:
    QHBoxLayout *mainLayout;
    QVBoxLayout *rightLayout;
    QVBoxLayout *leftLayout;
    //Velocity display.
    QLineEdit *x_vel_display_;
    QLineEdit *y_vel_display_;
    QLineEdit *z_vel_display_;
    //Path deviation display.
    QLineEdit *dev_display_;
    QLineEdit *rel_dev_display_;
    QLineEdit *vel_dev_display_;
    //Stop button.
    QPushButton *stop_button_;
    //RosInterface.
    RosInterface RosInterface_;
    //Set layout functions.
    void setVelocityDisplay();
    void setDeviationDisplay();
    void setStopButton();
};
#endif

