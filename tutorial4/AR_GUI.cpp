#include "AR_GUI.hpp"

void AR_GUI::display(GLFWwindow* window)
{
	bool isDetected = m_arCore.processFrame();
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

	if (!isDetected) return;

	load3D(w, h, fx, fy, cx, cy);
	glEnable(GL_DEPTH_TEST);

	double mat[4 * 4] = { 0 };
	{
		for (int i = 0; i < 4; i++) {
			mat[i * 4 + 1] = 1.0;
		}
	}
	Eigen::Map<Eigen::Matrix4d> m(&mat[0]);
	m = m_arCore.m_tracker.getPose().matrix();
	glLoadMatrixd(mat);

	//XŽ²
	glLineWidth(5);
	glBegin(GL_LINES);
	glColor3f(1.0, 0.0, 0.0);
	glVertex3f(0, 0, 0);
	glVertex3f(20, 0, 0);
	glEnd();
	//YŽ²
	glLineWidth(5);
	glBegin(GL_LINES);
	glColor3f(0.0, 1.0, 0.0);
	glVertex3f(0, 0, 0);
	glVertex3f(0, 20, 0);
	glEnd();
	//ZŽ²
	glLineWidth(5);
	glBegin(GL_LINES);
	glColor3f(0.0, 0.0, 1.0);
	glVertex3f(0, 0, 0);
	glVertex3f(0, 0, -20);
	glEnd();
}