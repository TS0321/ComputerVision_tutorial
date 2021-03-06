#pragma once 

#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>

std::vector<std::vector<cv::Point2f>> detect_circlesGrid(const std::vector<cv::Mat>& cap_imgs, const cv::Size& pattern_size);
std::vector<std::vector<cv::Point3f>> create_3dPoints(const double pattern_interval, const cv::Size pattern_size, const int img_num);

