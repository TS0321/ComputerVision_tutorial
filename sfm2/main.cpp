#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <unordered_map>
#include "window.hpp"


struct Keyframe
{
	cv::Mat image;
	std::vector<cv::KeyPoint> keyPoints;
	cv::Mat descriptor;
	std::unordered_map<int, int> worldPointsIDs;
	Eigen::Isometry3d tfKeyframeFromWorld;
};

class OpenGLWindow : public Window
{
public:
	std::string m_dataPath = std::string("../image/");
	cv::Mat m_cameraMatrix = cv::Mat::eye(3, 3, CV_64FC1);
	cv::Mat m_distCoeffs = cv::Mat::zeros(1, 5, CV_64FC1);

	bool m_bInit = false;

	std::unordered_map<int, Eigen::Vector3d> worldPoints;
	std::unordered_map<int, Eigen::Vector3d> colors;
	std::vector<Keyframe> keyframes;
	int newPointId = 0;
public:
	virtual void init()
	{
		std::ifstream ifs;
		ifs.open(m_dataPath + "calibration.txt");
		if (!ifs.is_open())
		{
			std::cout << "calibration file not found" << std::endl;
			return;
		}

		std::string strLine;
		std::vector<double> params;
		while (std::getline(ifs, strLine))
		{
			try {
				params.push_back(std::stod(strLine));
			}
			catch (...)
			{
				std::cout << "invalid camera parameter." << std::endl;
				return;
			}
		}
		if (params.size() != 9)
		{
			std::cout << "invalid camera parameter" << std::endl;
			return;
		}

		double fx = params[0];
		double fy = params[1];
		double cx = params[2];
		double cy = params[3];
		double k1 = params[4];
		double k2 = params[5];
		double p1 = params[6];
		double p2 = params[7];
		double k3 = params[8];

		m_cameraMatrix = (cv::Mat_<double>(3, 3, CV_64FC1) << fx, 0, cx,
			0, fy, cy,
			0, 0, 1);
		m_distCoeffs = (cv::Mat_<double>(1, 5, CV_64FC1) << k1, k2, p1, p2, k3);

		m_bInit = true;
	}

	void renderAxis(Eigen::Isometry3d& pose, double length = 1)
	{
		Eigen::Vector3d origin = pose * Eigen::Vector3d(0, 0, 0);
		Eigen::Vector3d xAxis = pose * Eigen::Vector3d(length, 0, 0);
		Eigen::Vector3d yAxis = pose * Eigen::Vector3d(0, length, 0);
		Eigen::Vector3d zAxis = pose * Eigen::Vector3d(0, 0, length);

		glLineWidth(5.0);

		glColor3ub(255, 0, 0);
		glBegin(GL_LINES);
		glVertex3d(origin.x(), origin.y(), origin.z());
		glVertex3d(xAxis.x(), xAxis.y(), xAxis.z());
		glEnd();

		glColor3ub(0, 255, 0);
		glBegin(GL_LINES);
		glVertex3d(origin.x(), origin.y(), origin.z());
		glVertex3d(yAxis.x(), yAxis.y(), yAxis.z());
		glEnd();

		glColor3ub(0, 0, 255);
		glBegin(GL_LINES);
		glVertex3d(origin.x(), origin.y(), origin.z());
		glVertex3d(zAxis.x(), zAxis.y(), zAxis.z());
		glEnd();
	}

	void render(
		const std::unordered_map<int, Eigen::Vector3d>& points3d,
		const std::unordered_map<int, Eigen::Vector3d>& colors
	)
	{
		glClearColor(0, 0, 0, 1);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		glViewport(0, 0, m_windowWidth, m_windowHeight);

		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		glPerspectiveCam(300, m_windowWidth, m_windowHeight, 1.0, 1000.);

		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		static int count = 0;
		count++;
		glTranslatef(0.f, -0.f, -10.f);
		glRotatef(180, 1, 0, 0);
		glRotatef(0.2 * count, 0, 1, 0);

		glEnable(GL_DEPTH_TEST);
		for (auto& kf : keyframes)
		{
			renderAxis(kf.tfKeyframeFromWorld);
		}
		glPointSize(3);
		glBegin(GL_POINTS);
		for (const auto& [id, point] : points3d)
		{
			const Eigen::Vector3d color = colors.at(id);
			glColor3ub(color.x(), color.y(), color.z());
			glVertex3d(point.x(), point.y(), point.z());
		}
		glEnd();

		glPopMatrix();
		glLineWidth(1.0);

		glDisable(GL_DEPTH_TEST);
	}

	void triangulation(Keyframe& keyframe0, Keyframe& keyframe1, const std::vector<cv::Point2d>& undist_pts1, const std::vector<cv::Point2d>& undist_pts2, const std::vector<cv::DMatch>& match, const cv::Mat& inlier_mask,
		std::unordered_map<int, Eigen::Vector3d>& worldPoints, std::unordered_map<int, Eigen::Vector3d>& vertexColor)
	{
		Eigen::Matrix<double, 3, 4> projMat1;
		projMat1 << keyframe0.tfKeyframeFromWorld.rotation().matrix(), keyframe0.tfKeyframeFromWorld.translation().matrix();
		cv::Mat cvProjMat1;
		cv::eigen2cv(projMat1, cvProjMat1);
		Eigen::Matrix<double, 3, 4> projMat2;
		projMat2 << keyframe1.tfKeyframeFromWorld.rotation().matrix(), keyframe1.tfKeyframeFromWorld.translation().matrix();
		cv::Mat cvProjMat2;
		cv::eigen2cv(projMat2, cvProjMat2);

		cv::Mat point4D;
		cv::triangulatePoints(cvProjMat1, cvProjMat2, undist_pts1, undist_pts2, point4D);
		cv::Mat img0 = keyframe0.image.clone();
		const std::vector<cv::KeyPoint>& keypoints0 = keyframe0.keyPoints;

		for (int i = 0; i < point4D.cols; i++)
		{
			if (!inlier_mask.empty() && inlier_mask.at<bool>(i) == 0)
			{
				continue;
			}

			Eigen::Vector4d pos(0, 0, 0, 1);
			pos.x() = point4D.at<double>(0, i) / point4D.at<double>(3, i);
			pos.y() = point4D.at<double>(1, i) / point4D.at<double>(3, i);
			pos.z() = point4D.at<double>(2, i) / point4D.at<double>(3, i);
			if (pos.z() < 0)
			{
				continue;
			}
			int pointId = newPointId++;
			worldPoints.insert(std::make_pair(pointId, pos.head<3>()));
			Eigen::Vector3d color(img0.at<cv::Vec3b>(keypoints0[match[i].queryIdx].pt)[2], img0.at<cv::Vec3b>(keypoints0[match[i].queryIdx].pt)[1], img0.at<cv::Vec3b>(keypoints0[match[i].queryIdx].pt)[0]);
			vertexColor.insert(std::make_pair(pointId, color));
			keyframe0.worldPointsIDs.insert(std::make_pair(match[i].queryIdx, pointId));
			keyframe1.worldPointsIDs.insert(std::make_pair(match[i].trainIdx, pointId));
		}
	}

	void initFromFirstView(std::vector<Keyframe>& keyframes, std::unordered_map<int, Eigen::Vector3d>& worldPoints, std::unordered_map<int, Eigen::Vector3d>& vertexColor)
	{
		cv::Mat& srcImg = keyframes[0].image;
		cv::Mat& dstImg = keyframes[1].image;

		//特徴点マッチング
		std::vector<cv::DMatch> match;
		std::vector<cv::KeyPoint> srcKeypoints;
		std::vector<cv::KeyPoint> dstKeypoints;
		cv::Mat srcDescriptor;
		cv::Mat dstDescriptor;
		auto algorithm = cv::AKAZE::create();
		cv::Mat mask;
		algorithm->detectAndCompute(srcImg, mask, srcKeypoints, srcDescriptor, false);
		algorithm->detectAndCompute(dstImg, mask, dstKeypoints, dstDescriptor, false);

		cv::BFMatcher matcher(cv::NORM_L2, true);
		matcher.match(srcDescriptor, dstDescriptor, match);

		cv::Mat match_result;
		cv::drawMatches(srcImg, srcKeypoints, dstImg, dstKeypoints, match, match_result);
		cv::imshow("match_result", match_result);
		cv::waitKey(0);

		std::vector<cv::Point2d> srcPoints;
		std::vector<cv::Point2d> dstPoints;

		for (int i = 0; i < match.size(); i++)
		{
			srcPoints.emplace_back(srcKeypoints[match[i].queryIdx].pt);
			dstPoints.emplace_back(dstKeypoints[match[i].trainIdx].pt);
		}

		std::vector<cv::Point2d> undist_srcPoints;
		std::vector<cv::Point2d> undist_dstPoints;
		cv::undistortPoints(srcPoints, undist_srcPoints, m_cameraMatrix, m_distCoeffs);
		cv::undistortPoints(dstPoints, undist_dstPoints, m_cameraMatrix, m_distCoeffs);

		cv::Mat R, t;
		cv::Mat inlier_mask;
		//E行列からポーズ推定
		const double focal = (m_cameraMatrix.at<double>(0, 0) + m_cameraMatrix.at<double>(1, 1)) / 2;
		cv::Mat mat = cv::Mat::eye(3, 3, CV_64F);
		cv::Mat E = cv::findEssentialMat(undist_srcPoints, undist_dstPoints, mat, cv::RANSAC, 0.999, 1.0 / focal, inlier_mask);
		cv::recoverPose(E, undist_srcPoints, undist_dstPoints, mat, R, t, inlier_mask);

		Eigen::Matrix3d R_eigen;
		Eigen::Vector3d t_eigen;
		cv::cv2eigen(R, R_eigen);
		cv::cv2eigen(t, t_eigen);

		//キーフレームに特徴点、記述子、位置姿勢を登録
		keyframes[0].keyPoints = srcKeypoints;
		keyframes[0].descriptor = srcDescriptor;
		keyframes[0].tfKeyframeFromWorld = Eigen::Isometry3d::Identity();
		keyframes[1].keyPoints = dstKeypoints;
		keyframes[1].descriptor = dstDescriptor;
		Eigen::Matrix4d transMat = Eigen::Matrix4d::Identity();
		transMat.block<3, 3>(0, 0) = R_eigen;
		transMat.block<3, 1>(0, 3) = t_eigen;
		keyframes[1].tfKeyframeFromWorld.matrix() = transMat;

		//三角測量
		triangulation(keyframes[0], keyframes[1], undist_srcPoints, undist_dstPoints, match, inlier_mask, worldPoints, vertexColor);
	}

	void estimatePoseAndReconstrunction(Keyframe& prevKeyframe, Keyframe& currentKeyframe, std::unordered_map<int, Eigen::Vector3d>& worldPoints, std::unordered_map<int, Eigen::Vector3d>& colors)
	{
		cv::Mat& prevImg = prevKeyframe.image;
		cv::Mat& currentImg = currentKeyframe.image;

		//特徴点マッチング
		auto algorithm = cv::AKAZE::create();

		std::vector<cv::KeyPoint> srcKeypoints = prevKeyframe.keyPoints;
		std::vector<cv::KeyPoint> dstKeypoints;
		cv::Mat srcDescriptor = prevKeyframe.descriptor;
		cv::Mat dstDescriptor;
		cv::Mat mask;
		algorithm->detectAndCompute(currentImg, mask, dstKeypoints, dstDescriptor, false);

		cv::BFMatcher matcher(cv::NORM_L2, true);
		std::vector<cv::DMatch> match;
		matcher.match(srcDescriptor, dstDescriptor, match);
		cv::Mat match_result;
		cv::drawMatches(prevImg, srcKeypoints, currentImg, dstKeypoints, match, match_result);
		cv::imshow("match_result", match_result);
		cv::waitKey(0);

		currentKeyframe.keyPoints = dstKeypoints;
		currentKeyframe.descriptor = dstDescriptor;

		//位置姿勢推定
		std::vector<std::pair<int, int>> ptsIds;
		std::vector<cv::Point3d> pts3D;
		std::vector<cv::Point2d> pts2D;
		for (int i = 0; i < match.size(); i++)
		{
			//三次元点が登録されてなければスキップ
			if (prevKeyframe.worldPointsIDs.count(match[i].queryIdx) == 0)
			{
				continue;
			}

			const int pointId = prevKeyframe.worldPointsIDs[match[i].queryIdx];
			const Eigen::Vector3d& worldPoint = worldPoints[pointId];
			cv::Point3d pt3d(worldPoint.x(), worldPoint.y(), worldPoint.z());
			pts3D.push_back(pt3d);
			ptsIds.push_back(std::make_pair(match[i].queryIdx, match[i].trainIdx));
			const cv::Point2d kp = cv::Point2d(currentKeyframe.keyPoints[match[i].trainIdx].pt);
			pts2D.push_back(kp);
			currentKeyframe.worldPointsIDs.insert(std::make_pair(match[i].trainIdx, pointId));
		}

		Eigen::Matrix3d R_eig = prevKeyframe.tfKeyframeFromWorld.rotation().matrix();
		Eigen::Vector3d t_eig = prevKeyframe.tfKeyframeFromWorld.translation().matrix();
		cv::Mat R_cv, t_cv;
		cv::Mat rvec_cv;
		cv::eigen2cv(R_eig, R_cv);
		cv::eigen2cv(t_eig, t_cv);
		cv::Rodrigues(R_cv, rvec_cv);
		cv::Mat inliers;
		std::cout << "before : " << worldPoints.size() << std::endl;
		cv::solvePnPRansac(pts3D, pts2D, m_cameraMatrix, m_distCoeffs, rvec_cv, t_cv, true, 1000, 3.0, 0.999, inliers, cv::SOLVEPNP_ITERATIVE);
		int cnt = 0;
		for (int i = 0; i < ptsIds.size(); i++)
		{
			bool remove_flag = true;
			for (int j = 0; j < inliers.rows; j++)
			{
				if (i == inliers.at<int>(j))
				{
					remove_flag = false;
					break;
				}
			}
			if (remove_flag)
			{
				cnt++;
				const int pointId = prevKeyframe.worldPointsIDs.at(ptsIds[i].first);
				worldPoints.erase(pointId);
				prevKeyframe.worldPointsIDs.erase(ptsIds[i].first);
				currentKeyframe.worldPointsIDs.erase(ptsIds[i].second);
			}
		}
		std::cout << "after : " << worldPoints.size() << std::endl;

		cv::Rodrigues(rvec_cv, R_cv);
		cv::cv2eigen(R_cv, R_eig);
		cv::cv2eigen(t_cv, t_eig);
		Eigen::Matrix4d transMat = Eigen::Matrix4d::Identity();
		transMat.block<3, 3>(0, 0) = R_eig;
		transMat.block<3, 1>(0, 3) = t_eig;
		currentKeyframe.tfKeyframeFromWorld.matrix() = transMat;

		//三次元点の追加
		std::vector<cv::KeyPoint>& prevKeyPoints = prevKeyframe.keyPoints;
		std::vector<cv::KeyPoint>& currentKeyPoints = currentKeyframe.keyPoints;

		std::vector<cv::Point2d> prevPts;
		std::vector<cv::Point2d> currentPts;

		for (int i = 0; i < match.size(); i++)
		{
			//すでに三次元点がある場合はスキップ
			if (prevKeyframe.worldPointsIDs.count(match[i].queryIdx) != 0)
			{
				continue;
			}

			prevPts.push_back(cv::Point2d(prevKeyPoints[match[i].queryIdx].pt));
			currentPts.push_back(cv::Point2d(currentKeyPoints[match[i].trainIdx].pt));
		}

		std::vector<cv::Point2d> undist_prevPts;
		std::vector<cv::Point2d> undist_currentPts;

		cv::undistortPoints(prevPts, undist_prevPts, m_cameraMatrix, m_distCoeffs);
		cv::undistortPoints(currentPts, undist_currentPts, m_cameraMatrix, m_distCoeffs);

		triangulation(prevKeyframe, currentKeyframe, undist_prevPts, undist_currentPts, match, cv::Mat(), worldPoints, colors);
		std::cout << "after2 : " << worldPoints.size() << std::endl;

	}

	void display()
	{
		static bool bFirst = true;
		if (bFirst == true)
		{
			bFirst = false;

			std::vector<cv::Mat> capImgs;
			for (int i = 0; i < 7; i++)
			{
				std::string imgPath = m_dataPath + "shiba0" + std::to_string(i) + ".bmp";
				cv::Mat img = cv::imread(imgPath);
				cv::imshow("img", img);
				cv::waitKey(0);
				capImgs.push_back(img);
			}

			for (int i = 0; i < 7; i++)
			{
				Keyframe keyframe;
				keyframe.image = capImgs[i];
				keyframes.push_back(keyframe);
			}


			initFromFirstView(keyframes, worldPoints, colors);

			for (int i = 1; i < 6; i++)
			{
				estimatePoseAndReconstrunction(keyframes[i], keyframes[i + 1], worldPoints, colors);
			}
		}
		render(worldPoints, colors);
	}
};

int main()
{
	OpenGLWindow gui;
	gui.init();
	if (gui.m_bInit == false)
	{
		return -1;
	}

	gui.execute("OpenGLWindow", 640, 480);
	return 0;
}