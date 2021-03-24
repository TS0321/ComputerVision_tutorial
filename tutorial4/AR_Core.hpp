#pragma once

#include <iostream>
#include "Tracker.hpp"
#include "Camera.hpp"

class AR_Core
{
public:
	AR_Core(Camera camera) : m_camera(camera) {};
	AR_Core(Camera camera, Tracker tracker) : m_camera(camera), m_tracker(tracker) {};
	~AR_Core() {};
	void init() {
		m_tracker.set_camera(&m_camera);
	}
	bool processFrame();

public:
	Tracker m_tracker;
	Camera m_camera;
};