/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QApplication>
#include "../include/ros_joystick/main_window.hpp"

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv) {

    /*********************
    ** Qt
    **********************/
    QApplication app(argc, argv);
    ros_joystick::MainWindow w(argc,argv);
    w.setWindowTitle("ROS Joystick");
    w.show();
    int result = app.exec();

    return result;
}
