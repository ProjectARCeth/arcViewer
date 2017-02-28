#ifndef CONTROL_WINDOW_H
#define CONTROL_WINDOW_H

#include <iostream>

#include <QIcon>
#include <QLabel>
#include <QLineEdit>
#include <QPalette>
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
    Q_SLOT void popUpNotstop();
    Q_SLOT void shutdown();
    Q_SLOT void updateDevDisplay(double deviation, double relative);
    Q_SLOT void updateObstacleDisDisplay(double distance);
    Q_SLOT void updateVelDisplay(double x, double y, double z);
    Q_SLOT void updateVelDevDisplay(double deviation);

private:
    QHBoxLayout *mainLayout;
    QVBoxLayout *rightLayout;
    QVBoxLayout *leftLayout;
    //Velocity display.
    QLineEdit *abs_vel_display_;
    QLineEdit *x_vel_display_;
    QLineEdit *y_vel_display_;
    QLineEdit *z_vel_display_;
    //Path deviation display.
    QLineEdit *dev_display_;
    QLineEdit *rel_dev_display_;
    QLineEdit *vel_dev_display_;
    //Obstacle Distance.
    QLineEdit *obstacle_dis_display_;
    //Stop button.
    QPushButton *stop_button_;
    //Shutdown button.
    QPushButton *shutdown_button_;
    //Mode button and display.
    QPushButton *mode_button_;
    QLineEdit *mode_display_;
    //RosInterface.
    RosInterface RosInterface_;
    //Set layout functions.
    void buildInterface(bool mode);
    void deleteWidgets();
    void setDeviationDisplay();
    void setModeDisplay(bool mode);
    void setObstacleDistanceDisplay();
    void setShutdownButton();
    void setStopButton();
    void setVelocityDisplay();
};
#endif

