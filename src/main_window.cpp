/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/ros_joystick/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ros_joystick {

using namespace Qt;
/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
  : QMainWindow(parent),
    qnode(argc,argv)
{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.

  //First opens with no comm and it will refresh with timer
  ui.rosOK->setVisible(false);
  ui.rosBAD->setVisible(true);

  //Refresh position on screen every second
  QTimer *timer = new QTimer(this);
  connect(timer, SIGNAL(timeout()),
          this, SLOT(updatePoseDisplay()));
  timer->start(1000);

  //Init qnode
  qnode.init();
}

MainWindow::~MainWindow() {}

void ros_joystick::MainWindow::on_go_forward_clicked()
{
  if ( !qnode.init() )
  {
    showNoMasterMessage();
  } else
  {
    ROS_INFO("MOVE : FORWARD!");
    qnode.SetSpeed(set_Xspeed, 0);
  }
}

void ros_joystick::MainWindow::on_go_back_clicked()
{
  if ( !qnode.init() )
  {
    showNoMasterMessage();
  } else
  {
    ROS_INFO("MOVE : BACKWARD!");
    qnode.SetSpeed(-set_Xspeed, 0);
  }
}

void ros_joystick::MainWindow::on_go_left_clicked()
{
  if ( !qnode.init() )
  {
    showNoMasterMessage();
  } else
  {
    ROS_INFO("MOVE : LEFT!");
    qnode.SetSpeed(0, PI / set_Aspeed);
  }
}

void ros_joystick::MainWindow::on_go_right_clicked()
{
  if ( !qnode.init() )
  {
    showNoMasterMessage();
  } else
  {
    ROS_INFO("MOVE : RIGHT!");
    qnode.SetSpeed(0, PI / -set_Aspeed);
  }
}

void ros_joystick::MainWindow::on_pause_clicked()
{
  if ( !qnode.init() )
  {
    showNoMasterMessage();
  } else
  {
    ROS_INFO("STOP!");
    qnode.SetSpeed(0, 0);
  }

}

void ros_joystick::MainWindow::on_x_speed_valueChanged(int value)
{
  set_Xspeed = value/10.0;

  //Show current speed
  ui.show_x_speed->setText(QString::number(set_Xspeed,'f',2));
}
void ros_joystick::MainWindow::on_a_speed_valueChanged(int value)
{
  set_Aspeed = value;

  //Show current ANGLE
  ui.show_a_speed->setText(QString::number(set_Aspeed,'f',2));
}

void MainWindow::updatePoseDisplay()
{
  if ( !qnode.init() )
  {
    ROS_ERROR("ROS Master not found, connect one before using joystick!");
    ui.rosOK->setVisible(false);
    ui.rosBAD->setVisible(true);

  } else
  {
    ui.rosOK->setVisible(true);
    ui.rosBAD->setVisible(false);

    //Get position
    double x = qnode.getXPos();
    double y = qnode.getYPos();
    double a = qnode.getAPos();

    ROS_INFO("Current position : (%.2f,%.2f,%.2f)", x,y,a);

    //Show current posicion
    ui.x_label->setText(QString::number(x,'f',2));
    ui.y_label->setText(QString::number(y,'f',2));
    ui.o_label->setText(QString::number(a,'f',2));
  }
}

void MainWindow::showNoMasterMessage()
{
  QMessageBox msgBox;
  msgBox.setText("ROS Master not found!");
  msgBox.exec();
  close();
}

void MainWindow::closeEvent(QCloseEvent *event)
{
  QMainWindow::closeEvent(event);
}

}; //end namespace ros_joystick










