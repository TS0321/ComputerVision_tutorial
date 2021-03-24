#pragma once

#include "Camera.hpp"
#include "Tracker.hpp"
#include "../window.hpp"

class AR_GUI : public Window
{
public:
	AR_GUI(Window window) : m_window(window) {};
	~AR_GUI() {};
	virtual void display(GLFWwindow* window);

private:
	Camera* camera;
	Tracker* tracker;
	Window m_window;
};