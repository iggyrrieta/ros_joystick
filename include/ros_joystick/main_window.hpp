#ifndef ros_joystick_MAIN_WINDOW_H
#define ros_joystick_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace ros_joystick {


#define PI 3.14159265359
/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

  /*********************
  ** CLOSE APP IF MASTER
  ** ROS NOT RUNNING
  **********************/
  void closeEvent(QCloseEvent *event); // Overloaded function
  void showNoMasterMessage();

public Q_SLOTS:
  /*********************
  ** REFRESH POSITION
  ** ON SCREEN
  **********************/
  void updatePoseDisplay();

private Q_SLOTS:
  /*********************
  ** PUSHBUTTONS EVENTS
  **********************/
  void on_go_forward_clicked();
  void on_go_back_clicked();
  void on_go_left_clicked();
  void on_go_right_clicked();
  void on_pause_clicked();

  /*********************
  ** SLIDERS
  **********************/
  void on_x_speed_valueChanged(int value);
  void on_a_speed_valueChanged(int value);

private:
	Ui::MainWindowDesign ui;

  /*********************
  ** SLIDERS VARIABLES
  **********************/
  double set_Xspeed;
  double set_Aspeed;

  /*********************
  ** QNODE REFERENCE
  **********************/
  QNode qnode;

};

}  // namespace ros_joystick

#endif // ros_joystick_MAIN_WINDOW_H
