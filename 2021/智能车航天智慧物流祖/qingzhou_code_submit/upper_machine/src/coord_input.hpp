#pragma once

#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QWidget>

class CoordInputWidget : public QWidget {
	Q_OBJECT

  public:
	CoordInputWidget(const std::string &display_name,
	                 bool show_send_button = true, QWidget *parent = nullptr);

	bool isInSetting();
	void setInSetting(bool value);

	std::tuple<double, double> getPosition();
	void setPosition(double x, double y);
	void pointReceived(double x, double y);

  signals:
	void postionChanged(double x, double y);
	void sendGoal(double x, double y);

  private:
	QLineEdit *txt_x;
	QLineEdit *txt_y;
	QPushButton *btn_go;
	QPushButton *btn_set;

	bool in_setting = false;

	void toggleInSetting();
	void textChanged();
	void goPressed();
};
