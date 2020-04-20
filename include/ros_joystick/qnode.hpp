/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ros_joystick_QNODE_HPP_
#define ros_joystick_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#endif
#include <string>
#include <QThread>
#include <QStringList>
#include <stdlib.h>
#include <QMutex>
#include <iostream>
#include "assert.h"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ros_joystick {


/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
  Q_OBJECT
public:
    QNode(int argc, char** argv, const char * topic  = "/odom");
    virtual ~QNode();
    bool init();
    void run();

    /*********************
    ** Obtener variables
    ** posicion y velocidad
    **********************/
    double getXPos();
    double getXSpeed();
    double getASpeed();
    double getYPos();
    double getAPos();

    /*********************
    ** Methodo callback
    **********************/
    void poseCallback(const nav_msgs::Odometry & msg);

    /*********************
    ** Aplicar cambios
    ** velocidad y posicion
    **********************/
    void SetSpeed(double speed, double angle);
    void setPose(QList<double> to_set);


private:
    int init_argc;
    char** init_argv;
    const char * m_topic;

    /*********************
    ** Variables
    ** posicion y velocidad
    **********************/
    double m_speed;
    double m_angle;
    double m_xPos;
    double m_yPos;
    double m_aPos;

    /*********************
    ** Variables Rangos
    **********************/
    double m_maxRange;
    double m_minRange;

    /*********************
    ** Subscriber y
    ** Publisher
    **********************/
    ros::Subscriber pose_listener;
    ros::Publisher  sim_velocity;
};
} // namespace ros_joystick
#endif /* ros_joystick_QNODE_HPP_ */
