#include "detector.hpp"

#include <iostream>
#include <limits>
#include <opencv2/imgproc.hpp>
#include <tuple>

TrafficLightDetector::TrafficLightDetector()
    : dictionary(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100)) {}

std::tuple<int, int, int, int>
calculate_min_max(std::initializer_list<std::vector<cv::Point2f>> points) {
	float xmin = std::numeric_limits<float>::infinity();
	float xmax = -std::numeric_limits<float>::infinity();
	float ymin = std::numeric_limits<float>::infinity();
	float ymax = -std::numeric_limits<float>::infinity();
	for (auto &c : points) {
		for (auto &p : c) {
			if (p.x < xmin)
				xmin = p.x;
			if (p.x > xmax)
				xmax = p.x;
			if (p.y < ymin)
				ymin = p.y;
			if (p.y > ymax)
				ymax = p.y;
		}
	}
	return {
	    static_cast<int>(std::round(xmin)), static_cast<int>(std::round(xmax)),
	    static_cast<int>(std::round(ymin)), static_cast<int>(std::round(ymax))};
}

std::optional<cv::Mat>
TrafficLightDetector::roi_from_aruco_tag(const cv::Mat &image) {
	std::vector<int> ids;
	std::vector<std::vector<cv::Point2f>> corners;
	cv::aruco::detectMarkers(image, dictionary, corners, ids);

	std::optional<std::vector<cv::Point2f>> tag0, tag1;
	for (size_t i = 0; i < ids.size(); i++) {
		if (ids[i] == 0) {
			tag0 = corners[i];
		} else if (ids[i] == 1) {
			tag1 = corners[i];
		}
	}

	if (!tag0.has_value() && !tag1.has_value()) {
		return std::nullopt;
	}

	int roi_x, roi_y, side_length;
	if (tag0.has_value() && tag1.has_value()) {
		auto [xmin, xmax, ymin, ymax] = calculate_min_max({*tag0, *tag1});
		side_length = (xmax - xmin) * 1.3;
		roi_x = xmin;
		roi_y = ymin - side_length;
	} else {
		auto &tag = tag0.has_value() ? *tag0 : *tag1;
		auto [xmin, xmax, ymin, ymax] = calculate_min_max({tag});
		side_length = (xmax - xmin) * 3;
		roi_x = (xmax + xmin - side_length) / 2;
		roi_y = ymin - side_length;
	}
	if (side_length <= 0) {
		std::cerr << "side_length = " << side_length << " is not positive\n";
		return std::nullopt;
	}
	int roi_width = side_length;
	int roi_height = side_length;
	if (roi_x < 0)
		roi_x = 0;
	if (roi_y < 0)
		roi_y = 0;
	if (roi_x + roi_width > image.cols)
		roi_width = image.cols - roi_x;
	if (roi_y + roi_height > image.rows)
		roi_height = image.rows - roi_y;
	cv::Mat roi = image(cv::Rect(roi_x, roi_y, roi_width, roi_height));
	return roi;
}

int bSums(const cv::Mat &src) {
	int counter = 0;
	cv::Mat_<uchar>::const_iterator it = src.begin<uchar>();
	cv::Mat_<uchar>::const_iterator itend = src.end<uchar>();
	for (; it != itend; ++it) {
		if ((*it) > 0)
			counter += 1;
	}
	return counter;
}

bool TrafficLightDetector::check_red_light(const cv::Mat &image) {
	auto roi = roi_from_aruco_tag(image);
	if (!roi.has_value()) {
		return false;
	}

	cv::Mat img_r, img_g;
	std::vector<cv::Mat> channels;
	cv::split(*roi, channels);
	img_r = channels.at(2) - channels.at(1);
	img_g = channels.at(1) - channels.at(2);
	cv::threshold(img_r, img_r, 160, 255, cv::THRESH_BINARY);
	cv::dilate(img_r, img_r, cv::Mat());
	cv::dilate(img_r, img_r, cv::Mat());
	cv::threshold(img_g, img_g, 160, 255, cv::THRESH_BINARY);
	cv::dilate(img_g, img_g, cv::Mat());
	cv::dilate(img_g, img_g, cv::Mat());

	int area_r = bSums(img_r);
	int area_g = bSums(img_g);
	if (area_r - area_g >= roi->cols) {
		return true;
	} else {
		return false;
	}
}

bool TrafficLightDetector::is_green_light(const cv::Mat &image) {
	if (check_red_light(image)) {
		non_red_count = 0;
		return false;
	} else {
		non_red_count++;
		return non_red_count >= non_red_threshold;
	}
}
