#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include "../loadimage.hpp"
#include "../CameraCalibration.hpp"
#include "../CalcPose.hpp"
#include "../window.hpp"

class Test :public Window {
public:

	std::vector<cv::Mat> capimg;
	CamParam cparam;
	Eigen::Isometry3d pose;

	virtual void display(GLFWwindow* window)
	{
		const int w = 1280;
		const int h = 720;
		const double fx = cparam.fx;
		const double fy = cparam.fy;
		const double cx = cparam.cx;
		const double cy = cparam.cy;

		{
			srand(0);
			unsigned char* img = new unsigned char[w * h];
			for (int i = 0; i < w * h; i++) {
				img[i] = rand() % 256;
			}
			load2D(w, h);
			glDisable(GL_DEPTH_TEST);

			dispImg(capimg[0], w, h, 3);

			glPointSize(10);
			glBegin(GL_POINTS);
			glColor3d(1.0, 1.0, 1.0);

			glVertex2d(0, 0);
			glVertex2d(w - 1, h - 1);
			glEnd();
		}
		{
			load3D(w, h, fx, fy, cx, cy);
			glEnable(GL_DEPTH_TEST);

			double mat[4 * 4] = { 0 };
			{
				for (int i = 0; i < 4; i++) {
					mat[i * 4 + 1] = 1.0;
				}
			}
			Eigen::Map<Eigen::Matrix4d> m(&mat[0]);
			m = pose.matrix();
			glLoadMatrixd(mat);

			//X軸
			glLineWidth(5);
			glBegin(GL_LINES);
			glColor3f(1.0, 0.0, 0.0);
			glVertex3f(0, 0, 0);
			glVertex3f(20, 0, 0);
			glEnd();
			//Y軸
			glLineWidth(5);
			glBegin(GL_LINES);
			glColor3f(0.0, 1.0, 0.0);
			glVertex3f(0, 0, 0);
			glVertex3f(0, 20, 0);
			glEnd();
			//Z軸
			glLineWidth(5);
			glBegin(GL_LINES);
			glColor3f(0.0, 0.0, 1.0);
			glVertex3f(0, 0, 0);
			glVertex3f(0, 0, -20);
			glEnd();

			
		}
	}
};

static const int img_num = 1;

int main()
{
	//撮影データの読み込み
	std::vector<cv::Mat> cap_imgs;
	std::string folder_name = "../Capdata";
	cap_imgs = loadImage(folder_name, img_num);
	
	CamParam cparam;
	loadCamParam("../CameraParam.txt", cparam);

	//ドットパターンの検出
	cv::Size pattern_size(12, 8);
	std::vector<std::vector<cv::Point2f>> centers(cap_imgs.size());
	centers = detect_circlesGrid(cap_imgs, pattern_size);

	//カメラキャリブレーション
	const double pattern_interval = 20; //20[mm]
	//3次元点の生成
	std::vector<std::vector<cv::Point3f>> objPoints;
	objPoints = create_3dPoints(pattern_interval, pattern_size, cap_imgs.size());

	const double fx = cparam.fx;
	const double fy = cparam.fy;
	const double cx = cparam.cx;
	const double cy = cparam.cy;
	
	Eigen::Isometry3d pose_0;
	pose_0.setIdentity();
	for (int i = 0; i < cap_imgs.size(); i++)
	{
		cv::Mat rvec;
		cv::Mat tvec;
		Eigen::Isometry3d pose;
		pose.setIdentity();
		std::vector<Eigen::Vector2d> imgPoints;
		std::vector<Eigen::Vector3d> tmp_objPoints;
		for (int j = 0; j < centers[i].size(); j++)
		{
			double u = double(centers[i][j].x);
			double v = double(centers[i][j].y);

			double x = 0;
			double y = 0;
			normalize(fx, fy, cx, cy, u, v, x, y);
			imgPoints.push_back(Eigen::Vector2d(x, y));

			double X = double(objPoints[i][j].x);
			double Y = double(objPoints[i][j].y);
			double Z = double(objPoints[i][j].z);
			tmp_objPoints.push_back(Eigen::Vector3d(X, Y, Z));
		}


		//calcPose_Plane(tmp_objPoints, imgPoints, cameraMatrix_eigen, pose);
		calcPose(tmp_objPoints, imgPoints, pose);
		Eigen::Matrix3f R = pose.rotation().matrix().cast<float>();
		cv::eigen2cv(R, rvec);
		cv::Rodrigues(rvec, rvec);
		Eigen::Vector3f t = pose.translation().matrix().cast<float>();
		cv::eigen2cv(t, tvec);
		//cv::solvePnP(objPoints[i], centers[i], cameraMatrix, distCoeffs, rvec, tvec, cv::SOLVEPNP_ITERATIVE);

		//std::cout << "eigen_R : " << R << std::endl;
		//std::cout << "eigen_t : " << t << std::endl;
		cv::Rodrigues(rvec, rvec);
		//std::cout << "cv_R : " << rvec << std::endl;
		//std::cout << "cv_t : " << tvec << std::endl;

		//cv::imshow("cap_img", cap_imgs[i]);
		//cv::waitKey(0);
		if (i == 0) pose_0 = pose;
	}
	std::cout << pose_0.matrix() << std::endl;

	Test window;
	window.capimg = cap_imgs;
	window.cparam = cparam;
	window.pose = pose_0;
	window.execute("test", 1280, 720);


	return EXIT_SUCCESS;
}