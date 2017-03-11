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
#include <vector>

#include "RosInterface.hpp"

class ControlWindow : public QWidget
{
    Q_OBJECT

public:
    ControlWindow(int argc, char** argv, QWidget * parent = 0);
    Q_SLOT void notstop();
    Q_SLOT void popUpNotstop();
    Q_SLOT void shutdown();
    Q_SLOT void updateDevDisplay(double deviation);
    Q_SLOT void updateObstacleDisDisplay(double distance);
    Q_SLOT void updatePurePursuitDisplay(std::vector <double> infos);
    Q_SLOT void updateSteeringDisplay(double angle);
    Q_SLOT void updateVelDisplay(double velocity);
    Q_SLOT void updateVelDevDisplay(double deviation);

private:
    QHBoxLayout *mainLayout;
    QVBoxLayout *rightLayout;
    QVBoxLayout *leftLayout;
    //Velocity display.
    QLineEdit *abs_vel_display_;
    //Path deviation display.
    QLineEdit *dev_display_;
    QLineEdit *vel_dev_display_;
    //Obstacle Distance.
    QLineEdit *obstacle_dis_display_;
    //Pure pursuit infos.
    QLineEdit *distance_start_display_;
    QLineEdit *distance_end_display_;
    QLineEdit *steering_reference_index_display_;
    QLineEdit *steering_should_display_;
    QLineEdit *radius_reference_index_display_;
    QLineEdit *radius_path_display_;
    QLineEdit *velocity_bound_physical_display_;
    QLineEdit *velocity_bound_teach_display_;
    QLineEdit *velocity_should_display_;
    QLineEdit *braking_distance_display_;
    //Steering information.
    QLineEdit *steering_ist_display_;
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
    void setModeDisplay(bool mode);
    void setShutdownButton();
    void setStopButton();
    void setUpDisplay(QLineEdit *display, std::string info, QVBoxLayout *bigLayout);
    void setVelocityDisplay();
};
#endif

