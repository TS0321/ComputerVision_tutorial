#include "CameraCalibration.hpp"
#include <fstream>
#include <sstream>
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

void writeCamParam(cv::Mat cameraMatrix, cv::Mat distCoeffs, std::string filename)
{
	std::ofstream ofs(filename);
	ofs << "fx, " << cameraMatrix.at<double>(0, 0) << std::endl;
	ofs << "fy, " << cameraMatrix.at<double>(1, 1) << std::endl;
	ofs << "cx, " << cameraMatrix.at<double>(0, 2) << std::endl;
	ofs << "cy, " << cameraMatrix.at<double>(1, 2) << std::endl;
	ofs << "k1, " << distCoeffs.at<double>(0) << std::endl;
	ofs << "k2, " << distCoeffs.at<double>(1) << std::endl;
	ofs << "p1, " << distCoeffs.at<double>(2) << std::endl;
	ofs << "p2, " << distCoeffs.at<double>(3) << std::endl;
	ofs << "k3, " << distCoeffs.at<double>(4) << std::endl;
	ofs.close();
}

void loadCamParam(std::string filename, CamParam &cparam)
{
	std::ifstream ifs(filename);
	double& fx = cparam.fx;
	double& fy = cparam.fy;
	double& cx = cparam.cx;
	double& cy = cparam.cy;
	double& k1 = cparam.k1;
	double& k2 = cparam.k2;
	double& p1 = cparam.p1;
	double& p2 = cparam.p2;
	double& k3 = cparam.k3;

	std::string buf;
	std::vector<double> params;
	while (std::getline(ifs, buf))
	{
		std::string fn;
		std::istringstream iss(buf);
		std::getline(iss, fn, ',');
		double num;
		iss >> num;
		params.push_back(num);
	}

	fx = params[0];
	fy = params[1];
	cx = params[2];
	cy = params[3];
	k1 = params[4];
	k2 = params[5];
	p1 = params[6];
	p2 = params[7];
	k3 = params[8];
}

