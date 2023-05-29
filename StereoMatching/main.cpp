#include <iostream>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
int main(void)
{
	cv::Mat img0 = cv::imread("../image/shiba02.bmp", cv::IMREAD_GRAYSCALE);
	cv::Mat img1 = cv::imread("../image/shiba04.bmp", cv::IMREAD_GRAYSCALE);

	auto algorithm = cv::AKAZE::create();

	std::vector<cv::KeyPoint> keypoints0, keypoints1;
	cv::Mat descriptor0, descriptor1;
	cv::Mat mask;
	algorithm->detectAndCompute(img0, mask, keypoints0, descriptor0, false);
	algorithm->detectAndCompute(img1, mask, keypoints1, descriptor1, false);
	
	cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce");
	std::vector<cv::DMatch> match12, match21;
	matcher->match(descriptor0, descriptor1, match12);
	matcher->match(descriptor1, descriptor0, match21);

	std::vector<cv::DMatch> match;
	for (int i = 0; i < match12.size(); i++)
	{
		cv::DMatch forward = match12[i];
		cv::DMatch backward = match21[forward.trainIdx];
		if (backward.trainIdx == forward.queryIdx)
		{
			match.push_back(forward);
		}
	}

	std::vector<cv::Point2d> pts1;
	std::vector<cv::Point2d> pts2;

	for (int i = 0; i < match.size(); i++)
	{
		pts1.emplace_back(keypoints0[match[i].queryIdx].pt);
		pts2.emplace_back(keypoints1[match[i].trainIdx].pt);
	}

	//カメラパラメータ
	const double fx = 8.093086e+02;
	const double fy = 8.106449e+02;
	const double cx = 3.199442e+02;
	const double cy = 2.785334e+02;
	const double k1 = 1.818949e-02;
	const double k2 = -2.175777e-01;
	const double k3 = 6.684171e-01;
	const double p1 = 2.078000e-02;
	const double p2 = 1.822711e-03;

	cv::Mat cameraMatrix = 
		(cv::Mat_<double>(3, 3) << fx, 0, cx,
									0, fy, cy,
									0, 0, 1);
	cv::Mat distCoeffs = (cv::Mat_<double>(1, 5) << k1, k2, k3, p1, p2);

	//E行列の推定、R,ｔの位置姿勢推定
	cv::Mat mask0;
	std::vector<cv::Point2d> undist_pts1;
	std::vector<cv::Point2d> undist_pts2;
	cv::undistortPoints(pts1, undist_pts1, cameraMatrix, distCoeffs);
	cv::undistortPoints(pts2, undist_pts2, cameraMatrix, distCoeffs);
	cv::Mat mat = cv::Mat::eye(3, 3, CV_64F);
	double focal = (fx + fy) / 2.0;
	cv::Mat E = cv::findEssentialMat(pts1, pts2, cameraMatrix, cv::RANSAC, 0.999, 1.0, mask0);
	cv::Mat R, t;
	cv::recoverPose(E, pts1, pts2, cameraMatrix, R, t, mask0);


	Eigen::Matrix3d R_eig;
	R_eig << 0.999876, -0.0101091, -0.0121078,
		0.0100657, 0.999943, -0.00364251,
		0.012144, 0.00352019, 0.9992;
	Eigen::Vector3d t_vec(-0.99488, -0.04608, -0.08993);

	cv::eigen2cv(R_eig, R);
	cv::eigen2cv(t_vec, t);
	std::cout << "R : " << R << std::endl;
	std::cout << "t : " << t << std::endl;

	cv::Mat R1, R2;
	cv::Mat P1, P2;
	cv::Mat Q;
	cv::Size size = img0.size();
	std::cout << "size: " << size << std::endl;
	cv::stereoRectify(cameraMatrix, distCoeffs, cameraMatrix, distCoeffs, size, R, t, R1, R2, P1, P2, Q, cv::CALIB_ZERO_DISPARITY, 1, size);
	cv::Mat mapX1, mapY1;
	cv::Mat mapX2, mapY2;

	cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, R1, P1, size, CV_32FC1, mapX1, mapY1);
	cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, R2, P2, size, CV_32FC1, mapX2, mapY2);
	
	cv::Mat rectified0, rectified1;
	cv::remap(img0, rectified0, mapX1, mapY1, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
	cv::remap(img1, rectified1, mapX2, mapY2, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));

	cv::imshow("rectified0", rectified0);
	cv::imshow("rectified1", rectified1);
	cv::waitKey(0);

	return true;
}