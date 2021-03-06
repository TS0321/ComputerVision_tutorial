#include "loadimage.hpp"

std::vector<cv::Mat> loadImage(const std::string& folder_name, const int& img_num) {
	std::vector<cv::Mat> imgs(img_num);
	for (int i = 0; i < img_num; i++)
	{
		cv::Mat img = cv::imread(folder_name + "/img" + std::to_string(i) + ".png", cv::IMREAD_COLOR);
		imgs.push_back(img);
	}

	return imgs;
}
