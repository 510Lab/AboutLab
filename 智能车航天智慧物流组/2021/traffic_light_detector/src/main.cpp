#include "detector.hpp"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <opencv2/opencv.hpp>

int main(int argc, char *argv[]) {
	ros::init(argc, argv, "traffic_light_detector");

	TrafficLightDetector detector;

	ros::NodeHandle nh;

	ros::Publisher green_light_pub =
	    nh.advertise<std_msgs::Bool>("green_light", 10);

	image_transport::ImageTransport image_transport(nh);
	image_transport::Subscriber sub = image_transport.subscribe(
	    "camera/image", 1, [&](const sensor_msgs::ImageConstPtr &msg) {
		    cv_bridge::CvImageConstPtr cv_ptr =
		        cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
		    cv::Mat img = cv_ptr->image;
		    bool is_green_light = detector.is_green_light(img);

		    std_msgs::Bool green_light_msg;
		    green_light_msg.data = is_green_light;
		    green_light_pub.publish(green_light_msg);
	    });

	ros::spin();
}
