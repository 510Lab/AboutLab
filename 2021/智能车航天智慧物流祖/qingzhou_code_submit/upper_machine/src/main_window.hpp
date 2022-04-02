#pragma once

#include "coord_input.hpp"
#include <QMainWindow>

class MainWindow : public QMainWindow {
	Q_OBJECT

  public:
	MainWindow(QWidget *parent = nullptr);

	CoordInputWidget *cod_waiting;
	CoordInputWidget *cod_load;
	CoordInputWidget *cod_unload;
	CoordInputWidget *cod_stop_line;
	CoordInputWidget *cod_exit;

	void cameraImageReceived(const QImage &img);
	void robotPoseReceived(double x, double y, double yaw);

  signals:
	void pointReceived(double x, double y);
	void cancel_nav_goal();
	void save_cods();

  private:
	QLabel *lbl_camera;
	QLineEdit *txt_robot_x;
	QLineEdit *txt_robot_y;
	QLineEdit *txt_robot_yaw;
	QPushButton *btn_cancel_goal;
	QPushButton *btn_save_cods;
};
