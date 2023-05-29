#include <iostream>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <unordered_map>
#include <opencv2/core/eigen.hpp>
#include "window.hpp"
struct Keyframe
{
	cv::Mat image;
	std::vector<cv::KeyPoint> keyPoints;
	cv::Mat descriptor;
	std::unordered_map<int, int> worldPointsIDs;	//(観測点のID、三次元点のID）
	Eigen::Isometry3d tfKeyframeFromWorld;
};

class SfmGUI : public Window
{
public:
	SfmGUI() {};
	~SfmGUI() {};

	virtual void display(GLFWwindow* window)
	{
		const int w = 640;
		const int h = 480;
		const double fx = cameraMatrix.at<double>(0, 0);
		const double fy = cameraMatrix.at<double>(1, 1);
		const double cx = cameraMatrix.at<double>(0, 2);
		const double cy = cameraMatrix.at<double>(1, 2);

		{
			load3D(w, h, fx, fy, cx, cy, 1.0, 100);
			double mat[4 * 4] = { 0 };
			{
				for (int i = 0; i < 4; i++) {
					mat[i * 4 + i] = 1.0;
				}
			}

			if (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS) view_id = (view_id < 6) ? view_id + 1 : view_id;
			if (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS) view_id = (view_id > 0) ? view_id - 1 : view_id;

			std::vector<Keyframe>::iterator kf_itr = keyframes->begin() + view_id;
			Eigen::Map<Eigen::Matrix4d> mvMat(&mat[0]);
			mvMat = kf_itr->tfKeyframeFromWorld.matrix() * mvMat;
			cv::imshow("kf", kf_itr->image);
			cv::waitKey(1);

			glLoadMatrixd(mat);
			glPointSize(3);
			glBegin(GL_POINTS);
			glColor3f(0.0, 1.0, 0.0);
			for (auto& [id, point] : worldPoints)
			{
				glVertex3d(point.x(), point.y(), point.z());
			}
			//for (int i = 0; i < map->mapPoints.size(); i++) {
			//	glVertex3f(map->mapPoints[i].pos.x(), map->mapPoints[i].pos.y(), map->mapPoints[i].pos.z());
			//}
			glEnd();

		}
	}

	void setCameraParam(const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs)
	{
		this->cameraMatrix = cameraMatrix;
		this->distCoeffs = distCoeffs;
	}

	void setKeyframes(std::vector<Keyframe>& keyframes) {
		this->keyframes = &keyframes;
	}
	void setWorldPoints(std::unordered_map<int, Eigen::Vector3d> &map) {
		this->worldPoints = map;
	}
private:
	std::vector<Keyframe>* keyframes;
	std::unordered_map<int, Eigen::Vector3d> worldPoints;
	int view_id = 0;
	cv::Mat cameraMatrix;
	cv::Mat distCoeffs;
};

int main(void)
{
	//画像の読み込み
	std::vector<cv::Mat> capImgs;
	for (int i = 0; i < 7; i++)
	{
		std::string imgPath = "../image/shiba0" + std::to_string(i) + ".bmp";
		cv::Mat img = cv::imread(imgPath);
		//cv::imshow("img", img);
		//cv::waitKey(0);
		capImgs.push_back(img);
	}


	//カメラパラメータ
	const double fx = 8.093086e+02;
	const double fy = 8.106449e+02;
	const double cx = 3.199442e+02;
	const double cy = 2.785334e+02;
	const double k1 = 0;
	const double k2 = 0;
	const double k3 = 0;
	const double p1 = 0;
	const double p2 = 0;

	cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << fx, 0, cx,
		0, fy, cy,
		0, 0, 1);
	cv::Mat distCoeffs = (cv::Mat_<double>(1, 5) << k1, k2, k3, p1, p2);
	std::vector<Keyframe> keyframes;
	std::unordered_map<int, Eigen::Vector3d> worldPoints; //(三次元点のID、三次元点）
	//最初の二枚でSFM
	//特徴点マッチング
	{
		cv::Mat& img0 = capImgs[0];
		cv::Mat& img1 = capImgs[1];

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

		std::vector<cv::Point2d> undist_pts1;
		std::vector<cv::Point2d> undist_pts2;
		cv::undistortPoints(pts1, undist_pts1, cameraMatrix, distCoeffs);
		cv::undistortPoints(pts2, undist_pts2, cameraMatrix, distCoeffs);

		cv::Mat identityMat = cv::Mat::eye(3, 3, CV_64F);
		const double focal = (fx + fy) / 2;
		cv::Mat mask0;
		cv::Mat E = cv::findEssentialMat(undist_pts1, undist_pts2, identityMat, cv::RANSAC, 0.999, 1.0 / focal, mask0);
		cv::Mat R, t;
		cv::recoverPose(E, undist_pts1, undist_pts2, identityMat, R, t, mask0);
		Eigen::Matrix3d R_eigen;
		Eigen::Vector3d t_eigen;
		cv::cv2eigen(R, R_eigen);
		cv::cv2eigen(t, t_eigen);
		std::cout << "R : " << R << std::endl;
		std::cout << "t : " << t << std::endl;
		Keyframe keyframe0;
		keyframe0.image = capImgs[0];
		keyframe0.keyPoints = keypoints0;
		keyframe0.descriptor = descriptor0;
		keyframe0.tfKeyframeFromWorld = Eigen::Isometry3d::Identity();
		Keyframe keyframe1;
		keyframe1.image = capImgs[1];
		keyframe1.keyPoints = keypoints1;
		keyframe1.descriptor = descriptor1;
		Eigen::Matrix4d transMat = Eigen::Matrix4d::Identity();
		transMat << R_eigen, t_eigen;
		keyframe1.tfKeyframeFromWorld.matrix() = transMat;
		//keyframe1.tfKeyframeFromWorld.prerotate(R_eigen);
		//keyframe1.tfKeyframeFromWorld.pretranslate(t_eigen);
		keyframes.push_back(keyframe0);
		keyframes.push_back(keyframe1);
		//三角測量
		Eigen::Matrix<double, 3, 4> projMat1;
		projMat1 << keyframes[0].tfKeyframeFromWorld.rotation(), keyframes[0].tfKeyframeFromWorld.translation();
		cv::Mat cvProjMat1;
		cv::eigen2cv(projMat1, cvProjMat1);
		std::cout << "projMat1 : " << projMat1 << std::endl;
		Eigen::Matrix<double, 3, 4> projMat2;
		projMat2 << keyframes[1].tfKeyframeFromWorld.rotation(), keyframes[1].tfKeyframeFromWorld.translation();
		cv::Mat cvProjMat2;
		cv::eigen2cv(projMat2, cvProjMat2);
		std::cout << "projMat2 : " << cvProjMat2 << std::endl;

		cv::Mat point4D;
		cv::triangulatePoints(cvProjMat1, cvProjMat2, undist_pts1, undist_pts2, point4D);

		for (int i = 0; i < point4D.cols; i++)
		{
			//outlierは登録しない
			if (mask0.at<bool>(i) == 0)
			{
				continue;
			}

			Eigen::Vector4d pos(0, 0, 0, 1);
			pos.x() = point4D.at<double>(0, i) / point4D.at<double>(3, i);
			pos.y() = point4D.at<double>(1, i) / point4D.at<double>(3, i);
			pos.z() = point4D.at<double>(2, i) / point4D.at<double>(3, i);

			pos = keyframes[1].tfKeyframeFromWorld.inverse() * pos;
			std::cout << "pos : " << pos << std::endl;
			int pointId = worldPoints.size();
			worldPoints.insert(std::make_pair(pointId, pos.head<3>()));

			keyframes[0].worldPointsIDs.insert(std::make_pair(match[i].queryIdx, pointId));
			keyframes[1].worldPointsIDs.insert(std::make_pair(match[i].trainIdx, pointId));
		}

	}
	std::cout << worldPoints.size() << std::endl;

	SfmGUI gui;
	gui.setCameraParam(cameraMatrix, distCoeffs);
	gui.setKeyframes(keyframes);
	gui.setWorldPoints(worldPoints);
	gui.execute("window", 640, 480);
	return true;
}