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
	tracker.set_camera(&camera);
	tracker.create_3dPoints(cv::Size(12, 8), 20);
	AR_Core arCore(camera, tracker);
	AR_GUI arGUI(arCore);
	arGUI.execute("arGUI", 1280, 720);

	//cv::Matx33d cameraMatrix;
	//cameraMatrix << cparam.fx, 0, cparam.cx,
	//	0, cparam.fy, cparam.cy,
	//	0, 0, 1;
	//cv::Mat distCoeffs;
	//cv::Mat rvec, tvec;
	//while (true) {
	//	camera.capture();
	//	if (tracker.detect())
	//	{
	//		tracker.estimatePose();
	//		Eigen::Isometry3d pose = tracker.getPose();
	//		Eigen::Matrix3f R = pose.rotation().matrix().cast<float>();
	//		cv::Mat rvec, tvec;
	//		cv::eigen2cv(R, rvec);
	//		cv::Rodrigues(rvec, rvec);
	//		Eigen::Vector3f t = pose.translation().matrix().cast<float>();
	//		cv::eigen2cv(t, tvec);
	//		cv::aruco::drawAxis(camera.get_capImg(), cameraMatrix, distCoeffs, rvec, tvec, 40);
	//	}
	//	cv::imshow("capimg", camera.get_capImg());
	//	cv::waitKey(1);

	//}

	return 0;
}