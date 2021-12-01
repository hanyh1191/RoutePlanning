#include "visualize_map.h"

VisualizeMap::VisualizeMap()
{
	std::cout << "OpenCV_Version: " << CV_VERSION << std::endl;
}

bool VisualizeMap::Visualize(const std::map<std::string, std::vector<Point>> visualize_points)
{
    std::cout << "@@@@@@@@@@@@@@@@@@@@ " << std::endl;
    for (auto iter = visualize_points.begin(); iter != visualize_points.end(); iter++)
    {
        std::vector<Point> points = iter->second;
        std::cout << points.size() << std::endl;

        for (auto point : points)
        {
            //std::cout << point.x << " " << point.y << std::endl;
            if (point.x < _min_x)
                _min_x = point.x;
            if (point.y < _min_y)
                _min_y = point.y;
            if (point.x > _max_x)
                _max_x = point.x;
            if (point.y > _max_y)
                _max_y = point.y;
        }
        std::cout << _min_x << " " << _max_x << std::endl << _min_y << " " << _max_y << std::endl;
    }
    int height = int((_max_y - _min_y) * 1.2);
    int width = int((_max_x - _min_x) * 1.2);
    img = cv::Mat::zeros(cv::Size(width, height), CV_8UC3);
    img.setTo(255);              // 设置屏幕为白色	
    //std::cout << height << " " << width << " " << img.rows << " " << img.cols << std::endl;
    std::cout << "@@@@@@@@@@@@@@@@@@@@ " << std::endl;
    //
    std::vector<std::vector<int>> color_;
    for (int i = 0; i < 255; i += 50) {
        for (int j = 0; j < 255; j += 50) {
            for (int k = 0; k < 255; k += 50) {
                color_.push_back(std::vector<int>{i, j, k});
            }
        }
    }
    int index = 0;
    //
    std::string file_name;
    for (auto iter = visualize_points.begin(); iter != visualize_points.end(); iter++)
    {
        std::string name = iter->first;
        std::vector<Point> points = iter->second;
        for (int i = 0; i < points.size(); i++) 
        {
            cv::Point2d p(points[i].x - _min_x, height - (points[i].y - _min_y));
            if (name == "all_road_reference_points")
                if (i == 0)
                    cv::circle(img, p, 0.1, cv::Scalar(0, 0, 255), -1); // BGR, -1代表实心圆
                else
                    //cv::circle(img, p, 0.01, cv::Scalar(0, 0, 128), -1); // BGR, -1代表实心圆
                    break;
            else if (name == "all_lane_center_points")
                cv::circle(img, p, 0.01, cv::Scalar(0, 0, 0), -1); // BGR, -1代表实心圆
                //continue;
            else if (name == "test_points")
                cv::circle(img, p, 0.01, cv::Scalar(0, 255, 0), -1); // BGR, -1代表实心圆
            else if(name == "pre_points")
                cv::circle(img, p, 0.01, cv::Scalar(0, 255, 255), -1); // BGR, -1代表实心圆 黄色前驱
            else if(name == "suc_points")
                cv::circle(img, p, 0.01, cv::Scalar(255, 255, 0), -1); // BGR, -1代表实心圆 青色后继
            else {
                file_name = name.substr(1);
                if(name.substr(0, 1) == "1")
                    cv::circle(img, p, 0.01, cv::Scalar(255, 0, 0), -1); // BGR, -1代表实心圆 蓝色左侧车道
                else
                    cv::circle(img, p, 0.01, cv::Scalar(0, 255, 0), -1); // BGR, -1代表实心圆 绿色右侧车道
            }
        }
        index += 1;
    }
	
    //cv::Mat dst;
    cv::resize(img, img, cv::Size(0, 0), 2, 2, cv::INTER_CUBIC);
    //cv::putText(img, "R-reference start; B-left G-right Y-pre C-suc;", cv::Point(100, 100), cv::FONT_HERSHEY_DUPLEX, 1.0, (0, 0, 0), 2);
    //cv::imwrite("img/Borregasave/Borregasave_" + file_name + ".png", img);
	//cv::imshow("visualization", dst);
    cv::imshow("visualization", img);
	cv::waitKey(0);
	return true;
}


