#pragma once

#include <iostream>
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>

std::vector<cv::Mat> loadImage(const std::string& folder_name, const int& img_num);
