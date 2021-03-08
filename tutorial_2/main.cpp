#include <iostream>
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include "../loadimage.hpp"
#include "../CameraCalibration.hpp"
#include "../CalcPose.hpp"
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>

static const int img_num = 10;

int main(void)
{
	//�B�e�f�[�^�̓ǂݍ���
	std::vector<cv::Mat> cap_imgs;
	std::string folder_name = "../Capdata";
	cap_imgs = loadImage(folder_name, img_num);
	std::cout << cap_imgs.size() << std::endl;

	//�h�b�g�p�^�[���̌��o
	cv::Size pattern_size(12, 8);
	std::vector<std::vector<cv::Point2f>> centers(cap_imgs.size());
	centers = detect_circlesGrid(cap_imgs, pattern_size);

	//�J�����L�����u���[�V����
	const double pattern_interval = 20; //20[mm]
	//3�����_�̐���
	std::vector<std::vector<cv::Point3f>> objPoints;
	objPoints = create_3dPoints(pattern_interval, pattern_size, cap_imgs.size());

	const cv::Size img_size = cap_imgs[0].size();
	cv::Mat cameraMatrix;
	cv::Mat distCoeffs;
	cv::Mat rvecs;
	cv::Mat tvecs;

	cv::calibrateCamera(objPoints, centers, img_size, cameraMatrix, distCoeffs, rvecs, tvecs, 0, cv::TermCriteria());

	std::cout << cameraMatrix << std::endl;
	const double fx = cameraMatrix.at<double>(0, 0);
	const double fy = cameraMatrix.at<double>(1, 1);
	const double cx = cameraMatrix.at<double>(0, 2);
	const double cy = cameraMatrix.at<double>(1, 2);

	Eigen::Matrix3d cameraMatrix_eigen;
	cv::cv2eigen(cameraMatrix, cameraMatrix_eigen);

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
		cv::solvePnP(objPoints[i], centers[i], cameraMatrix, distCoeffs, rvec, tvec, cv::SOLVEPNP_ITERATIVE);
		
		cv::aruco::drawAxis(cap_imgs[i], cameraMatrix, distCoeffs, rvec, tvec, 40);
		std::cout << "eigen_R : " << R << std::endl;
		std::cout << "eigen_t : " << t << std::endl;
		cv::Rodrigues(rvec, rvec);
		std::cout << "cv_R : " << rvec << std::endl;
		std::cout << "cv_t : " << tvec << std::endl;
		
		cv::imshow("cap_img", cap_imgs[i]);
		cv::waitKey(0);
	}

	std::cout << "start capture !" << std::endl;
	cv::VideoCapture cap(1);//�f�o�C�X�̃I�[�v��
//cap.open(0);//�������ł��ǂ��D
//cap.set(cv::CAP_PROP_FOCUS, 500);
	const int camera_width = 1280;
	const int camera_height = 720;

	cap.set(cv::CAP_PROP_FRAME_WIDTH, camera_width);
	cap.set(cv::CAP_PROP_FRAME_HEIGHT, camera_height);

	if (!cap.isOpened())//�J�����f�o�C�X������ɃI�[�v���������m�F�D
	{
		//�ǂݍ��݂Ɏ��s�����Ƃ��̏���
		return EXIT_FAILURE;
	}

	cv::Mat frame; //�擾�����t���[��
	int img_num = 0;
	while (cap.read(frame))//�������[�v
	{
		//
		//�擾�����t���[���摜�ɑ΂��āC�N���[�X�P�[���ϊ���2�l���Ȃǂ̏������������ށD
		//
		std::vector<cv::Point2f> centers;
		bool isFound = cv::findCirclesGrid(frame, pattern_size, centers);
		cv::Mat rvec;
		cv::Mat tvec;
		cv::Mat R;


		if (isFound) {
			cv::solvePnP(objPoints[0], centers, cameraMatrix, distCoeffs, rvec, tvec, cv::SOLVEPNP_ITERATIVE);
			cv::aruco::drawAxis(frame, cameraMatrix, distCoeffs, rvec, tvec, 40);
		}

		cv::imshow("win", frame);//�摜��\���D
		const int key = cv::waitKey(1);
		if (key == 'q'/*113*/)//q�{�^���������ꂽ�Ƃ�
		{
			break;//while���[�v���甲����D
		}
	}
	cv::destroyAllWindows();


	return EXIT_SUCCESS;
}