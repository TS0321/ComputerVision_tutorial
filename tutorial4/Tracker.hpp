#pragma once

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
};