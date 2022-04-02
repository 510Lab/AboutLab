#include "coord_input.hpp"
#include "util.hpp"
#include <QGroupBox>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <iostream>

CoordInputWidget::CoordInputWidget(const std::string &display_name,
                                   bool show_send_button, QWidget *parent)
    : QWidget(parent) {

	QHBoxLayout *hbox_x = new QHBoxLayout;
	QLabel *lbl_x = new QLabel;
	lbl_x->setText("x:");
	hbox_x->addWidget(lbl_x);
	txt_x = new QLineEdit;
	txt_x->setText("0.0");
	hbox_x->addWidget(txt_x);

	QHBoxLayout *hbox_y = new QHBoxLayout;
	QLabel *lbl_y = new QLabel;
	lbl_y->setText("y:");
	hbox_y->addWidget(lbl_y);
	txt_y = new QLineEdit;
	txt_y->setText("0.0");
	hbox_y->addWidget(txt_y);

	QHBoxLayout *hbox_ctrl = new QHBoxLayout;
	btn_go = new QPushButton;
	btn_go->setText("发送");
	btn_go->setVisible(show_send_button);
	hbox_ctrl->addWidget(btn_go);
	btn_set = new QPushButton;
	btn_set->setText("设置");
	hbox_ctrl->addWidget(btn_set);

	QVBoxLayout *vbox = new QVBoxLayout;
	vbox->addLayout(hbox_x);
	vbox->addLayout(hbox_y);
	vbox->addLayout(hbox_ctrl);

	QGroupBox *group = new QGroupBox;
	group->setTitle(display_name.c_str());
	group->setLayout(vbox);

	QHBoxLayout *root_layout = new QHBoxLayout;
	root_layout->addWidget(group);
	this->setLayout(root_layout);

	connect(btn_set, &QPushButton::pressed, this,
	        &CoordInputWidget::toggleInSetting);
	connect(txt_x, &QLineEdit::textChanged, this,
	        &CoordInputWidget::textChanged);
	connect(txt_y, &QLineEdit::textChanged, this,
	        &CoordInputWidget::textChanged);
	connect(btn_go, &QPushButton::pressed, this, &CoordInputWidget::goPressed);
}

bool CoordInputWidget::isInSetting() {
	return in_setting;
}

void CoordInputWidget::setInSetting(bool value) {
	in_setting = value;
	if (value) {
		btn_set->setText("取消设置");
	} else {
		btn_set->setText("设置");
	}
}

std::tuple<double, double> CoordInputWidget::getPosition() {
	auto x = parse_double(txt_x->text().toStdString());
	auto y = parse_double(txt_y->text().toStdString());

	if (!x.has_value()) {
		std::cerr << "Bad x coord: " << txt_x->text().toStdString() << "\n";
	}
	if (!y.has_value()) {
		std::cerr << "Bad y coord: " << txt_y->text().toStdString() << "\n";
	}

	return {x.value_or(0.0), y.value_or(0.0)};
}

void CoordInputWidget::setPosition(double x, double y) {
	txt_x->setText(std::to_string(x).c_str());
	txt_y->setText(std::to_string(y).c_str());
	emit postionChanged(x, y);
}

void CoordInputWidget::toggleInSetting() {
	setInSetting(!isInSetting());
}

void CoordInputWidget::textChanged() {
	auto [x, y] = getPosition();
	emit postionChanged(x, y);
}

void CoordInputWidget::goPressed() {
	auto [x, y] = getPosition();
	emit sendGoal(x, y);
}

void CoordInputWidget::pointReceived(double x, double y) {
	if (isInSetting()) {
		setInSetting(false);
		setPosition(x, y);
	}
}
