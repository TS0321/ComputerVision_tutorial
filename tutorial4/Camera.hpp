#pragma once

#include <iostream>
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

class Camera
{
public:
	Camera(CamParam cparam, int width, int height, int cam_id) : m_CamParam(cparam), m_width(width), m_height(height) 
	{
		m_cap.open(cam_id);
		m_cap.set(cv::CAP_PROP_FRAME_WIDTH, width);
		m_cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);
	};
	~Camera() {};
private:
	CamParam m_CamParam;
	const int m_width;
	const int m_height;
	cv::Mat m_capImg;
	cv::VideoCapture m_cap;

public:
	cv::Mat& get_capImg();
	CamParam& get_camParam();
	const int& get_width();
	const int& get_height();
	void capture();
};

void loadCamParam(std::string filename, CamParam& cparam);