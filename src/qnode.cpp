/*****************************************************************************
** Includes
*****************************************************************************/

#include "../include/ros_joystick/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ros_joystick {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char **argv, const char * topic) :
    init_argc(argc),
    init_argv(argv),
    m_topic(topic)
    {}

QNode::~QNode()
{
  if (ros::isStarted())
  {
      ros::shutdown(); // we need this because we use ros::start()
      ros::waitForShutdown();
  }//end if

  wait();
}//end destructor

bool QNode::init()
{
  ros::init(init_argc, init_argv, "ros_joystick");
  if (!ros::master::check())
  {
      //Q_EMIT rosShutdown();
      return false;//do not start without ros.
  }//endif

  ros::start();
  ros::Time::init();
  ros::NodeHandle n;
  sim_velocity  = n.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
  pose_listener = n.subscribe(m_topic, 10, &QNode::poseCallback, this);
  start();
  return true;
}//set up the thread

void QNode::poseCallback(const nav_msgs::Odometry & msg)
{
  QMutex * pMutex = new QMutex();

  pMutex->lock();
  m_xPos = msg.pose.pose.position.x;
  m_yPos = msg.pose.pose.position.y;
  m_aPos = msg.pose.pose.orientation.w;
  pMutex->unlock();

  delete pMutex;
}//callback method to update the robot's position.

void QNode::run()
{
  ros::Rate loop_rate(100);
  QMutex * pMutex;
  while (ros::ok())
  {
      pMutex = new QMutex();

      geometry_msgs::Twist cmd_msg;
      pMutex->lock();
      cmd_msg.linear.x = m_speed;
      cmd_msg.angular.z = m_angle;
      pMutex->unlock();

      sim_velocity.publish(cmd_msg);
      ros::spinOnce();
      loop_rate.sleep();
      delete pMutex;
  } //loop run
  ROS_ERROR("ROS Master not found, closing app!");
}//ros:run

void QNode::SetSpeed(double speed, double angle)
{
  QMutex * pMutex = new QMutex();
  pMutex->lock();
  m_speed = speed;
  m_angle = angle;
  pMutex->unlock();

  ROS_INFO("Speed setpoint : (%.2f, %.2f)", m_speed, m_angle);

  delete pMutex;
}//set the speed of the robot.

double QNode::getXSpeed(){ return m_speed; }
double QNode::getASpeed(){ return m_angle; }

double QNode::getXPos(){ return m_xPos; }
double QNode::getYPos(){ return m_yPos; }
double QNode::getAPos(){ return m_aPos; }

} //namespace ros_joystick
