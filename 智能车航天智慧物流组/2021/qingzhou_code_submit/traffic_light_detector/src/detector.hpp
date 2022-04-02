#pragma once

#include <opencv2/aruco.hpp>
#include <optional>

class TrafficLightDetector {
  public:
	TrafficLightDetector();
	bool is_green_light(const cv::Mat &image);

  private:
	int non_red_count = 0;
	int non_red_threshold = 3;
	cv::Ptr<cv::aruco::Dictionary> dictionary;

	std::optional<cv::Mat> roi_from_aruco_tag(const cv::Mat &image);
	bool check_red_light(const cv::Mat &image);
};
