#include "AR_Core.hpp"

void AR_Core::processFrame()
{
	m_camera.capture();
	if (m_tracker.detect())
	{
		m_tracker.estimatePose();
	}
}