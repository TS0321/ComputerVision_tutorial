#pragma once

#include "Camera.hpp"
#include "Tracker.hpp"
#include "../window.hpp"

class AR_GUI
{
public:
	AR_GUI(Window window) : m_window(window) {};
	~AR_GUI() {};

private:
	Camera* camera;
	Tracker* tracker;
	Window m_window;
};