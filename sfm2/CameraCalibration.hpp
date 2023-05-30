#pragma once 

#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
struct CamParam
{
double fx;
double fy;
double cx;
double cy;
double k1;
double k2;
double p1;
double p2;
double k3;
};
std::vector<std::vector<cv::Point2f>> detect_circlesGrid(const std::vector<cv::Mat>& cap_imgs, const cv::Size& pattern_size);
std::vector<std::vector<cv::Point3f>> create_3dPoints(const double pattern_interval, const cv::Size pattern_size, const int img_num);
void writeCamParam(cv::Mat CameraMatrix, cv::Mat DistCoeffs, std::string filename);
void loadCamParam(std::string filename, CamParam &cparam);

