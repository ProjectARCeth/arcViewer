#ifndef CONTROL_WINDOW_H
#define CONTROL_WINDOW_H

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
    Q_SLOT void updatePoseDisplay(double x, double y, double z);

private:
    QHBoxLayout *mainLayout;
    QVBoxLayout *rightLayout;
    QVBoxLayout *leftLayout;
    QHBoxLayout *p_xLayout;
    QHBoxLayout *p_yLayout;
    QHBoxLayout *p_zLayout;
    QHBoxLayout *p_cameraLayout;

    QPushButton *p_quitButton;

    QLabel *p_xLabel;
    QLabel *p_yLabel;
    QLabel *p_zLabel;
    QLabel *p_imageLabel;

    QLineEdit *p_xDisplay;
    QLineEdit *p_yDisplay;
    QLineEdit *p_zDisplay;

    RosInterface m_RosInterface;
};
#endif

