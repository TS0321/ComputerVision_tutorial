#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include "Camera.hpp"
#include "Tracker.hpp"
#include "AR_Core.hpp"
#include "AR_GUI.hpp"
#include <stdio.h>
#include <chrono>

int main(void)
{
	CamParam cparam;
	loadCamParam("../CameraParam.txt", cparam);
	Camera camera(cparam, 1280, 720, 1);
	Tracker tracker;
	tracker.create_3dPoints(cv::Size(12, 8), 20);
	AR_Core arCore(camera, tracker);
	AR_GUI arGUI(arCore);
	arGUI.execute("arGUI", 1280, 720);


	return 0;
}