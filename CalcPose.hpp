#pragma once

#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

bool calcPose_Plane(const std::vector<Eigen::Vector3d>& objPoints, const std::vector<Eigen::Vector2d>& imgPoints, const Eigen::Matrix3d cameraMatrix, Eigen::Isometry3d& current_pose);
bool calcPose(const std::vector<Eigen::Vector3d>& objPoints, const std::vector<Eigen::Vector2d>& imgPoints, Eigen::Isometry3d& current_pose);
bool optimizePose(const std::vector<Eigen::Vector3d>& objPoints, const std::vector<Eigen::Vector2d>& imgPoints, Eigen::Isometry3d& pose, const int iteration_num = 10, const double error_thresh = 0.001);
void normalize(const double fx, const double fy, const double cx, const double cy, const double u, const double v, double& x, double& y);