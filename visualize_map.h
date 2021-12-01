#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include "common_define.h"

class VisualizeMap
{
public:
	VisualizeMap();

	// ø… ”ªØ
	bool Visualize(const std::map<std::string, std::vector<Point>> visualize_points);

protected:
	cv::Mat img;

	double _min_x = double(INT_MAX), _max_x = double(INT_MIN);
	double _min_y = double(INT_MAX), _max_y = double(INT_MIN);

};

