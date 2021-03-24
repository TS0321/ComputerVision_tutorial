#include "Camera.hpp"

cv::Mat& Camera::get_capImg()
{
	return m_capImg;
}

CamParam& Camera::get_camParam()
{
	return m_CamParam;
}

const int& Camera::get_width()
{
	return m_width;
}

const int& Camera::get_height()
{
	return m_height;
}

void Camera::capture()
{
	m_cap.read(m_capImg);
}

void loadCamParam(std::string filename, CamParam& cparam)
{
	std::ifstream ifs(filename);
	double& fx = cparam.fx;
	double& fy = cparam.fy;
	double& cx = cparam.cx;
	double& cy = cparam.cy;
	double& k1 = cparam.k1;
	double& k2 = cparam.k2;
	double& p1 = cparam.p1;
	double& p2 = cparam.p2;
	double& k3 = cparam.k3;

	std::string buf;
	std::vector<double> params;
	while (std::getline(ifs, buf))
	{
		std::string fn;
		std::istringstream iss(buf);
		std::getline(iss, fn, ',');
		double num;
		iss >> num;
		params.push_back(num);
	}

	fx = params[0];
	fy = params[1];
	cx = params[2];
	cy = params[3];
	k1 = params[4];
	k2 = params[5];
	p1 = params[6];
	p2 = params[7];
	k3 = params[8];
}