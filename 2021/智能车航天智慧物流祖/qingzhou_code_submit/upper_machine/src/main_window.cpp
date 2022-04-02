#include "main_window.hpp"
#include "Spoiler.hpp"
#include <QGroupBox>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <cmath>
#include <iomanip>
#include <qpushbutton.h>
#include <sstream>

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent) {

	QHBoxLayout *hbox_cods = new QHBoxLayout;
	cod_waiting = new CoordInputWidget("初始等待区");
	hbox_cods->addWidget(cod_waiting);
	cod_load = new CoordInputWidget("接货区");
	hbox_cods->addWidget(cod_load);
	cod_unload = new CoordInputWidget("卸货区");
	hbox_cods->addWidget(cod_unload);

	cod_stop_line = new CoordInputWidget("停止线", false);
	cod_exit = new CoordInputWidget("出口", false);

	QHBoxLayout *hbox_hidden_cods = new QHBoxLayout;
	hbox_hidden_cods->addWidget(cod_stop_line);
	hbox_hidden_cods->addWidget(cod_exit);
	Spoiler *spoiler_hidden_cods = new Spoiler("途经点");
	spoiler_hidden_cods->setContentLayout(*hbox_hidden_cods);

	lbl_camera = new QLabel;
	lbl_camera->setScaledContents(true);

	QHBoxLayout *hbox_robot_x = new QHBoxLayout;
	QLabel *lbl_robot_x = new QLabel;
	lbl_robot_x->setText("x:");
	hbox_robot_x->addWidget(lbl_robot_x);
	txt_robot_x = new QLineEdit;
	hbox_robot_x->addWidget(txt_robot_x);

	QHBoxLayout *hbox_robot_y = new QHBoxLayout;
	QLabel *lbl_robot_y = new QLabel;
	lbl_robot_y->setText("y:");
	hbox_robot_y->addWidget(lbl_robot_y);
	txt_robot_y = new QLineEdit;
	hbox_robot_y->addWidget(txt_robot_y);

	QHBoxLayout *hbox_robot_yaw = new QHBoxLayout;
	QLabel *lbl_robot_yaw = new QLabel;
	lbl_robot_yaw->setText("yaw:");
	hbox_robot_yaw->addWidget(lbl_robot_yaw);
	txt_robot_yaw = new QLineEdit;
	hbox_robot_yaw->addWidget(txt_robot_yaw);

	QVBoxLayout *vbox_robot_info = new QVBoxLayout;
	vbox_robot_info->addLayout(hbox_robot_x);
	vbox_robot_info->addLayout(hbox_robot_y);
	vbox_robot_info->addLayout(hbox_robot_yaw);
	QGroupBox *group_robot_info = new QGroupBox;
	group_robot_info->setTitle("机器人位姿");
	group_robot_info->setLayout(vbox_robot_info);

	btn_cancel_goal = new QPushButton;
	btn_cancel_goal->setText("取消导航目标");
	connect(btn_cancel_goal, &QPushButton::pressed, this,
	        &MainWindow::cancel_nav_goal);

	btn_save_cods = new QPushButton;
	btn_save_cods->setText("保存坐标");
	connect(btn_save_cods, &QPushButton::pressed, this, &MainWindow::save_cods);

	QVBoxLayout *vbox_right_side = new QVBoxLayout;
	vbox_right_side->addWidget(group_robot_info);
	vbox_right_side->addWidget(btn_cancel_goal);
	vbox_right_side->addWidget(btn_save_cods);

	QHBoxLayout *hbox_state = new QHBoxLayout;
	hbox_state->addWidget(lbl_camera);
	hbox_state->addLayout(vbox_right_side);

	QVBoxLayout *vbox = new QVBoxLayout;
	vbox->addLayout(hbox_state);
	vbox->addLayout(hbox_cods);
	vbox->addWidget(spoiler_hidden_cods);

	QWidget *central_widget = new QWidget;
	central_widget->setLayout(vbox);
	this->setCentralWidget(central_widget);
}

void MainWindow::cameraImageReceived(const QImage &img) {
	lbl_camera->setPixmap(QPixmap::fromImage(img));
	lbl_camera->setFixedSize(img.width(), img.height());
}

std::string to_string_fixed_precision(double val, int precision) {
	std::stringstream ss;
	ss << std::fixed << std::setprecision(precision) << val;
	return ss.str();
}

void MainWindow::robotPoseReceived(double x, double y, double yaw) {
	txt_robot_x->setText(to_string_fixed_precision(x, 2).c_str());
	txt_robot_y->setText(to_string_fixed_precision(y, 2).c_str());
	txt_robot_yaw->setText(
	    to_string_fixed_precision(yaw / M_PI * 180.0, 1).c_str());
}
