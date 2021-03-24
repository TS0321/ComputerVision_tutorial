#pragma once

#include "AR_Core.hpp"
#include "../window.hpp"

class AR_GUI : public Window
{
public:
	AR_GUI(AR_Core arCore): m_arCore(arCore){};
	~AR_GUI() {};
	virtual void display(GLFWwindow* window);

private:
	AR_Core m_arCore;
};