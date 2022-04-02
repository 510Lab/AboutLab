#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream> // for converting the command line parameter to integer

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("camera/image", 1);
  image_transport::Publisher pub_preview = it.advertise("camera/image_preview", 1);

  int preview_resolution_width, preview_resolution_height, preview_framerate_div;
  ros::param::param("~preview_resolution_width", preview_resolution_width, 640);
  ros::param::param("~preview_resolution_height", preview_resolution_height, 480);
  ros::param::param("~preview_framerate_div", preview_framerate_div, 1);

  std::string video_source;
  if (!ros::param::get("~video_source", video_source)) {
    ROS_ERROR("~video_source is not specified");
    return -1;
  }
  int api_preference;
  ros::param::param("~api_preference", api_preference, (int)cv::CAP_ANY);
  cv::VideoCapture cap(video_source, api_preference);
  // Check if video device can be opened with the given index
  if(!cap.isOpened()) {
    ROS_ERROR("Couldn't open video source %s", video_source.c_str());
    return -1;
  }
  cv::Mat frame;
  sensor_msgs::ImagePtr msg;

  unsigned long long frame_count = 0;
  int publish_rate;
  ros::param::param("~publish_rate", publish_rate, 30);
  ros::Rate loop_rate(publish_rate);
  while (nh.ok()) {
    cap >> frame;

    if(frame.empty()) {
      ROS_ERROR("Recevied empty image, is the camera disconnected?");
      return -1;
    }

    msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
    pub.publish(msg);

    if (frame_count % preview_framerate_div == 0) {
      cv::Mat preview_img;
      cv::resize(frame, preview_img, {preview_resolution_width, preview_resolution_height});
      sensor_msgs::ImagePtr preview_msg;
      preview_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", preview_img).toImageMsg();
      pub_preview.publish(preview_msg);
    }
    frame_count++;

    ros::spinOnce();
    loop_rate.sleep();
  }
}
