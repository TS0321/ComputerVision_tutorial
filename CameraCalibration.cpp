#include "CameraCalibration.hpp"

std::vector<std::vector<cv::Point2f>> detect_circlesGrid(const std::vector<cv::Mat>& cap_imgs, const cv::Size& pattern_size)
{
	std::vector<std::vector<cv::Point2f>> centers(cap_imgs.size());
	for (int i = 0; i < cap_imgs.size(); i++)
	{
		bool isFound = cv::findCirclesGrid(cap_imgs[i], pattern_size, centers[i]);

		if (!isFound)
		{
			std::cerr << "Can't detect!" << std::endl;
			//cv::imshow("cap_img", cap_imgs[i]);
			//cv::waitKey(0);
		}
	}
	return centers;
}

std::vector<std::vector<cv::Point3f>> create_3dPoints(const double pattern_interval, const cv::Size pattern_size, const int img_num)
{
	std::vector<std::vector<cv::Point3f>> objPoints;
	for (int i = 0; i < img_num; i++) {
		std::vector<cv::Point3f> tmp;
		for (int y = 0; y < pattern_size.height; y++)
		{
			for (int x = 0; x < pattern_size.width; x++)
			{
				cv::Point3f tmp_point = cv::Point3f(x * pattern_interval, y * pattern_interval, 0);
				//std::cout << tmp_point << std::endl;
				tmp.push_back(tmp_point);

			}
		}
		objPoints.push_back(tmp);
	}

	return objPoints;
}