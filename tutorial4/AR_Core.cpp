#include "AR_Core.hpp"

bool AR_Core::processFrame()
{
	m_camera.capture();
	if (!m_tracker.detect())
	{
		return false;
	}

	std::cout << "detected!" << std::endl;
	m_tracker.estimatePose();
	
	return true;
}