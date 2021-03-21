#pragma once

#include <iostream>
#include "AR_GUI.hpp"
#include "Tracker.hpp"
#include "Camera.hpp"

class AR_Sample
{
public:
	AR_Sample(Camera camera) : m_camera(camera) {};
	~AR_Sample() {};

private:
	Tracker m_tracker;
	AR_GUI m_arGUI;
	Camera m_camera;
};