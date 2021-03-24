#include "AR_GUI.hpp"

void AR_GUI::display(GLFWwindow* window)
{
	m_arCore.processFrame();
	CamParam& cparam = m_arCore.m_camera.get_camParam();
	const int w = m_arCore.m_camera.get_width();
	const int h = m_arCore.m_camera.get_height();
	const double fx = cparam.fx;
	const double fy = cparam.fy;
	const double cx = cparam.cx;
	const double cy = cparam.cy;

	cv::Mat& cap_img = m_arCore.m_camera.get_capImg();
	load2D(w, h);
	glDisable(GL_DEPTH_TEST);
	dispImg(cap_img, w, h, 3);


}