#include "QRosCallBackQueue.h"
#include "main_window.hpp"
#include "move_base_msgs/MoveBaseResult.h"
#include <QApplication>
#include <QTimer>
#include <actionlib/client/simple_action_client.h>
#include <cv_bridge/cv_bridge.h>
#include <fstream>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <image_transport/image_transport.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
    MoveBaseClient;

int main(int argc, char *argv[]) {
	QApplication app(argc, argv);
	QRosCallBackQueue::replaceGlobalQueue();
	ros::init(argc, argv, "upper_machine", ros::init_options::NoSigintHandler);

	ros::NodeHandle nh;
	MainWindow *main_window = new MainWindow;

	MoveBaseClient move_base("move_base", false);

	image_transport::ImageTransport image_transport(nh);
	image_transport::Subscriber sub = image_transport.subscribe(
	    "camera/image_preview", 1, [&](const sensor_msgs::ImageConstPtr &msg) {
		    cv_bridge::CvImageConstPtr cv_ptr =
		        cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
		    cv::Mat img = cv_ptr->image;
		    QImage qimg(img.data, img.cols, img.rows, img.step[0],
		                QImage::Format_RGB888);
		    main_window->cameraImageReceived(qimg);
	    });

	tf2_ros::Buffer tf_buffer;
	tf2_ros::TransformListener tf_listener(tf_buffer);
	geometry_msgs::Pose pose_in_base_link;
	pose_in_base_link.position.x = 0.0;
	pose_in_base_link.position.y = 0.0;
	pose_in_base_link.position.z = 0.0;
	pose_in_base_link.orientation.x = 0.0;
	pose_in_base_link.orientation.y = 0.0;
	pose_in_base_link.orientation.z = 0.0;
	pose_in_base_link.orientation.w = 1.0;
	auto update_robot_position = [&] {
		geometry_msgs::TransformStamped transform;
		try {
			transform = tf_buffer.lookupTransform(
			    "map", "base_link", ros::Time(0), ros::Duration(0));
		} catch (const std::exception &e) {
			ROS_WARN("Couldn't lookup robot pose: %s", e.what());
			return;
		}
		geometry_msgs::Pose pose_in_map;
		tf2::doTransform(pose_in_base_link, pose_in_map, transform);
		double x = pose_in_map.position.x;
		double y = pose_in_map.position.y;
		tf2::Quaternion q(pose_in_map.orientation.x, pose_in_map.orientation.y,
		                  pose_in_map.orientation.z, pose_in_map.orientation.w);
		tf2::Matrix3x3 m(q);
		double roll, pitch, yaw;
		m.getRPY(roll, pitch, yaw);
		main_window->robotPoseReceived(x, y, yaw);
	};
	QTimer timer;
	QObject::connect(&timer, &QTimer::timeout, update_robot_position);
	timer.setInterval(100);
	timer.start();

	ros::Subscriber sub_clicked_point =
	    nh.subscribe<geometry_msgs::PointStamped>(
	        "clicked_point", 10,
	        [&](const geometry_msgs::PointStamped::ConstPtr &msg) {
		        double x = msg->point.x;
		        double y = msg->point.y;
		        double z = msg->point.z;
		        std::cerr << "Received point: " << x << ", " << y << ", " << z
		                  << "\n";
		        emit main_window->pointReceived(x, y);
	        });

	auto publish_debug_point = [&](const std::string &topic,
	                               CoordInputWidget *widget) {
		ros::Publisher publisher =
		    nh.advertise<geometry_msgs::PointStamped>(topic, 1, true);

		auto publish_point = [publisher](double x, double y) {
			geometry_msgs::PointStamped msg;
			msg.header.frame_id = "map";
			msg.header.stamp = ros::Time::now();
			msg.point.x = x;
			msg.point.y = y;
			msg.point.z = 0.0;
			publisher.publish(msg);
		};

		QObject::connect(widget, &CoordInputWidget::postionChanged,
		                 publish_point);
		QObject::connect(main_window, &MainWindow::pointReceived, widget,
		                 &CoordInputWidget::pointReceived);

		auto [x, y] = widget->getPosition();
		publish_point(x, y);

		return publisher;
	};

	ros::Publisher pub_waiting_point =
	    publish_debug_point("waiting_point", main_window->cod_waiting);
	ros::Publisher pub_load_point =
	    publish_debug_point("load_point", main_window->cod_load);
	ros::Publisher pub_unload_point =
	    publish_debug_point("unload_point", main_window->cod_unload);
	ros::Publisher pub_stop_line_point =
	    publish_debug_point("stop_line_point", main_window->cod_stop_line);
	ros::Publisher pub_exit_point =
	    publish_debug_point("exit_point", main_window->cod_exit);

	QObject::connect(main_window, &MainWindow::cancel_nav_goal, [&] {
		if (!move_base.isServerConnected()) {
			ROS_WARN("move_base is not connected");
			return;
		}
		move_base.cancelAllGoals();
		ROS_INFO("All goals cancelled");
	});

	QObject::connect(main_window, &MainWindow::save_cods, [&] {
		std::ofstream out("coordinates.txt", std::ios::out | std::ios::trunc);
		if (out.fail()) {
			ROS_ERROR("Couldn't write coordinates.txt");
			return;
		}
		auto write_cod = [&out](CoordInputWidget *cod) {
			auto [x, y] = cod->getPosition();
			out << x << " " << y << "\n";
		};
		write_cod(main_window->cod_waiting);
		write_cod(main_window->cod_load);
		write_cod(main_window->cod_unload);
		write_cod(main_window->cod_stop_line);
		write_cod(main_window->cod_exit);
	});

	auto send_goal = [&](double x, double y,
	                     std::optional<std::function<void()>> callback =
	                         std::nullopt) {
		if (!move_base.isServerConnected()) {
			ROS_WARN("move_base is not connected");
			return;
		}

		move_base_msgs::MoveBaseGoal goal;
		goal.target_pose.header.frame_id = "map";
		goal.target_pose.header.stamp = ros::Time::now();
		goal.target_pose.pose.position.x = x;
		goal.target_pose.pose.position.y = y;
		goal.target_pose.pose.position.z = 0;
		goal.target_pose.pose.orientation.w = 1;
		goal.target_pose.pose.orientation.x = 0;
		goal.target_pose.pose.orientation.y = 0;
		goal.target_pose.pose.orientation.z = 0;
		move_base.sendGoal(
		    goal, [x, y, callback](
		              const actionlib::SimpleClientGoalState &state,
		              const move_base_msgs::MoveBaseResultConstPtr &result) {
			    ROS_INFO("Navigation to (%lf, %lf) done with %s", x, y,
			             state.getText().c_str());
			    if (callback.has_value() &&
			        (state == actionlib::SimpleClientGoalState::SUCCEEDED ||
			         state == actionlib::SimpleClientGoalState::ABORTED)) {
				    callback.value()();
			    }
		    });
		std::cerr << "Goal sent: " << x << ", " << y << "\n";
	};

	QObject::connect(
	    main_window->cod_waiting, &CoordInputWidget::sendGoal,
	    [&](double waiting_x, double waiting_y) {
		    auto [exit_x, exit_y] = main_window->cod_exit->getPosition();
		    send_goal(exit_x, exit_y, [&send_goal, waiting_x, waiting_y] {
			    send_goal(waiting_x, waiting_y);
		    });
	    });

	QObject::connect(main_window->cod_load, &CoordInputWidget::sendGoal,
	                 send_goal);

	QTimer traffic_light_timer;
	traffic_light_timer.setSingleShot(true);

	bool is_green_light = false;
	ros::Subscriber sub_green_light = nh.subscribe<std_msgs::Bool>(
	    "green_light", 10, [&](const std_msgs::Bool::ConstPtr &msg) {
		    is_green_light = msg->data;
		    if (is_green_light && traffic_light_timer.isActive()) {
			    ROS_INFO("Received green light");
			    traffic_light_timer.start(0);
		    }
	    });

	QObject::connect(&traffic_light_timer, &QTimer::timeout, [&] {
		auto [unload_x, unload_y] = main_window->cod_unload->getPosition();
		send_goal(unload_x, unload_y);
	});

	QObject::connect(
	    main_window->cod_unload, &CoordInputWidget::sendGoal,
	    [&](double unload_x, double unload_y) {
		    if (is_green_light) {
			    ROS_INFO("Green light, navigating to unload point");
			    send_goal(unload_x, unload_y);
		    } else {
			    ROS_INFO("Red light, navigating to stop line");
			    auto [stop_line_x, stop_line_y] =
			        main_window->cod_stop_line->getPosition();
			    send_goal(stop_line_x, stop_line_y, [&traffic_light_timer] {
				    traffic_light_timer.start(60 * 1000);
			    });
		    }
	    });

	{
		std::ifstream fin("coordinates.txt");
		if (fin.fail()) {
			ROS_WARN("Couldn't read coordinates.txt");
		} else {
			auto read_cod = [&fin](CoordInputWidget *cod) {
				double x, y;
				fin >> x >> y;
				cod->setPosition(x, y);
			};
			read_cod(main_window->cod_waiting);
			read_cod(main_window->cod_load);
			read_cod(main_window->cod_unload);
			read_cod(main_window->cod_stop_line);
			read_cod(main_window->cod_exit);
		}
	}

	main_window->show();
	app.exec();
	return 0;
}
