#include <iostream>
#include "Camera.hpp"


int main(void)
{
	CamParam cparam;
	loadCamParam("../CameraParam.txt", cparam);
	Camera camera(cparam, 1280, 720, 1);

	while (true) {
		camera.capture();
		cv::imshow("capimg", camera.get_capImg());
		cv::waitKey(1);
	}

	return 0;
}