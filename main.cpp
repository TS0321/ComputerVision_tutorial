#include <iostream>
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

static const int img_num = 10;

int main(void)
{
	//撮影データの読み込み
	std::vector<cv::Mat> cap_imgs;
	std::string folder_name = "../Capdata";
	for (int i = 0; i < img_num; i++)
	{
		cv::Mat cap_img = cv::imread(folder_name + "/img" + std::to_string(i) + ".png", cv::IMREAD_COLOR);
		//cv::imshow("cap_img", cap_img);
		//cv::waitKey(0);
		cap_imgs.push_back(cap_img);
	}

	//ドットパターンの検出
	cv::Size pattern_size(12, 8);
	std::vector<std::vector<cv::Point2f>> centers(cap_imgs.size());
	std::cout << "detect dot pattern! " << std::endl;
	for (int i = 0; i < cap_imgs.size(); i++)
	{
		bool isFound = cv::findCirclesGrid(cap_imgs[i], pattern_size, centers[i]);
		cv::drawChessboardCorners(cap_imgs[i], pattern_size, cv::Mat(centers[i]), isFound);
		cv::imshow("cap_img", cap_imgs[i]);
		cv::waitKey(0);
	}

	//カメラキャリブレーション
	const double pattern_interval = 20; //20[mm]
	//3次元点の生成
	std::vector<std::vector<cv::Point3f>> objPoints;
	for (int i = 0; i < cap_imgs.size(); i++) {
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
	const cv::Size img_size = cap_imgs[0].size();
	cv::Mat cameraMatrix;
	cv::Mat distCoeffs;
	cv::Mat rvecs;
	cv::Mat tvecs;

	cv::calibrateCamera(objPoints, centers, img_size, cameraMatrix, distCoeffs, rvecs, tvecs, 0, cv::TermCriteria());

	//std::cout << cameraMatrix << std::endl;

	for (int i = 0; i < cap_imgs.size(); i++)
	{
		cv::Mat rvec;
		cv::Mat tvec;
		cv::solvePnP(objPoints[i], centers[i], cameraMatrix, distCoeffs, rvec, tvec, cv::SOLVEPNP_ITERATIVE);
		cv::aruco::drawAxis(cap_imgs[i], cameraMatrix, distCoeffs, rvec, tvec, 40);
		cv::imshow("cap_img", cap_imgs[i]);
		cv::waitKey(0);
	}

	std::cout << "start capture !" << std::endl;
	cv::VideoCapture cap(1);//デバイスのオープン
//cap.open(0);//こっちでも良い．
//cap.set(cv::CAP_PROP_FOCUS, 500);
	const int camera_width = 1280;
	const int camera_height = 720;

	cap.set(cv::CAP_PROP_FRAME_WIDTH, camera_width);
	cap.set(cv::CAP_PROP_FRAME_HEIGHT, camera_height);

	if (!cap.isOpened())//カメラデバイスが正常にオープンしたか確認．
	{
		//読み込みに失敗したときの処理
		return -1;
	}

	cv::Mat frame; //取得したフレーム
	int img_num = 0;
	while (cap.read(frame))//無限ループ
	{
		//
		//取得したフレーム画像に対して，クレースケール変換や2値化などの処理を書き込む．
		//
		std::vector<cv::Point2f> centers;
		bool isFound = cv::findCirclesGrid(frame, pattern_size, centers);
		cv::Mat rvec;
		cv::Mat tvec;
		if (isFound) {
			cv::solvePnP(objPoints[0], centers, cameraMatrix, distCoeffs, rvec, tvec, cv::SOLVEPNP_ITERATIVE);
			cv::aruco::drawAxis(frame, cameraMatrix, distCoeffs, rvec, tvec, 40);
		}
		
		cv::imshow("win", frame);//画像を表示．
		const int key = cv::waitKey(1);
		if (key == 'q'/*113*/)//qボタンが押されたとき
		{
			break;//whileループから抜ける．
		}
	}
	cv::destroyAllWindows();


	return 0;
}