#include <iostream>
#include "Camera.hpp"


int main(void)
{
	CamParam cparam;
	loadCamParam("../CameraParam.txt", cparam);
	Camera camera(cparam, 1280, 720, 0);

	camera.capture();
	cv::imshow("capimg", camera.get_capImg());
	cv::waitKey(0);

	return 0;
}