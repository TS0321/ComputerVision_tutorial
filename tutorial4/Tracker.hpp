#pragma once

#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include "Camera.hpp"

class Tracker
{
public:
	Tracker() {};
	~Tracker() {};

private:
	Camera* camera;
	Eigen::Isometry3d m_currentPose;
	std::vector<Eigen::Vector3d> m_objPoints;
	std::vector<Eigen::Vector2d> m_imgPoints;
	cv::Size pattern_size;

public:
	void set_camera(Camera* camera);
	bool detect();
	void create_3dPoints(const cv::Size pattern_size, const double pattern_interval);
	bool estimatePose();
	bool calcPose(const std::vector<Eigen::Vector3d>& objPoints, const std::vector<Eigen::Vector2d>& imgPoints, Eigen::Isometry3d& current_pose);
	bool optimizePose(const std::vector<Eigen::Vector3d>& objPoints, const std::vector<Eigen::Vector2d>& imgPoints, Eigen::Isometry3d& pose, const int iteration_num, const double error_thresh);
	Eigen::Isometry3d& getPose();
};

void normalize(const double fx, const double fy, const double cx, const double cy, const double u, const double v, double& x, double& y);
