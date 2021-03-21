#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <fstream>
#include "../../loadimage.hpp"
#include "../../CameraCalibration.hpp"


int main(int argh, char* argv[])
{
	std::vector<cv::Mat> cap_imgs = loadImage("../../../Capdata/", 10);
	for (const auto& img : cap_imgs)
	{
		cv::imshow("capimg", img);
		cv::waitKey(0);
	}

	cv::Size pattern_size(12, 8);
	std::vector<std::vector<cv::Point2f>> centers(cap_imgs.size());
	centers = detect_circlesGrid(cap_imgs, pattern_size);

	//カメラキャリブレーション
	const double pattern_interval = 20; //20[mm]
	//3次元点の生成
	std::vector<std::vector<cv::Point3f>> objPoints;
	objPoints = create_3dPoints(pattern_interval, pattern_size, cap_imgs.size());

	const cv::Size img_size = cap_imgs[0].size();
	cv::Mat cameraMatrix;
	cv::Mat distCoeffs;
	cv::Mat rvecs;
	cv::Mat tvecs;

	cv::calibrateCamera(objPoints, centers, img_size, cameraMatrix, distCoeffs, rvecs, tvecs, 0, cv::TermCriteria());
	std::cout << "cameraMatrix : " << cameraMatrix << std::endl;
	std::cout << "distCoeffs : " << distCoeffs << std::endl;

	writeCamParam(cameraMatrix, distCoeffs, "../../../CameraParam.txt");
	//CamParam cparam;
	//loadCamParam("../../../CameraParam.txt", cparam);

	return 0;
}