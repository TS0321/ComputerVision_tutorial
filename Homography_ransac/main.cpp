#include <iostream>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <random>

cv::Mat DLT_Homography(std::vector<cv::Point> srcPoints, std::vector<cv::Point> dstPoints)
{
	if (srcPoints.size() != dstPoints.size())
	{
		return cv::Mat();
	}

	if (srcPoints.size() < 4)
	{
		return cv::Mat();
	}

	Eigen::MatrixXd A = Eigen::MatrixXd::Zero(2 * srcPoints.size(), 9);
	for (int i = 0; i < srcPoints.size(); i++)
	{
		double xa = srcPoints[i].x;
		double ya = srcPoints[i].y;
		double xb = dstPoints[i].x;
		double yb = dstPoints[i].y;
		A(2 * i, 0) = xa;
		A(2 * i, 1) = ya;
		A(2 * i, 2) = 1.0;
		A(2 * i, 6) = -xb * xa;
		A(2 * i, 7) = -xb * ya;
		A(2 * i, 8) = -xb;

		A(2 * i + 1, 3) = xa;
		A(2 * i + 1, 4) = ya;
		A(2 * i + 1, 5) = 1.0;
		A(2 * i + 1, 6) = -yb * xa;
		A(2 * i + 1, 7) = -yb * ya;
		A(2 * i + 1, 8) = -yb;
	}

	const Eigen::MatrixXd AtA = A.transpose() * A;
	Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigenSolver(AtA);
	Eigen::VectorXd eigenVector = eigenSolver.eigenvectors().col(0);
	cv::Mat homography(3, 3, CV_64F);
	for (int y = 0; y < 3; y++)
	{
		for (int x = 0; x < 3; x++)
		{
			homography.at<double>(y, x) = eigenVector(y * 3 + x) / eigenVector(8);
		}
	}

	return homography;
}

cv::Mat findHomographyWithRansac(std::vector<cv::Point>& srcPoints, std::vector<cv::Point>& dstPoints)
{
	cv::Mat est_Homography;

	int numPoints = srcPoints.size();
	std::random_device seed_gen;
	std::mt19937 engine(seed_gen());
	std::uniform_int_distribution<> uniDist(0, numPoints - 1);
	float pixThreshold = 3.0;
	int maxNumInlier = 0;
	std::vector<int> maxInliers;
	int stoppingCriterion = 10000;
	for (int i = 0; i < stoppingCriterion; i++)
	{
		std::vector<int> usePtIdx;
		//4点選び出してホモグラフィ行列を推定
		while (1)
		{
			int idx = uniDist(engine);
			bool bUsed = false;
			for (const auto& ptIdx : usePtIdx)
			{
				if (idx == ptIdx)
				{
					bUsed = true;
					break;
				}
			}

			if (bUsed)
			{
				bUsed = false;
				continue;
			}

			usePtIdx.push_back(idx);
			if (usePtIdx.size() == 4)
			{
				break;
			}
		}
		//ホモグラフィ変換の入力と出力の点を生成
		std::vector<cv::Point> srcPts_ransac;
		std::vector<cv::Point> dstPts_ransac;
		for (const auto& ptIdx : usePtIdx)
		{
			srcPts_ransac.push_back(srcPoints[ptIdx]);
			dstPts_ransac.push_back(dstPoints[ptIdx]);
		}

		//ホモグラフィ行列の推定
		cv::Mat mask;
		cv::Mat cvHomography = cv::findHomography(srcPts_ransac, dstPts_ransac, mask, 0);
		//cv::Mat cvHomography = DLT_Homography(srcPts_ransac, dstPts_ransac);
		if (cvHomography.empty())
		{
			continue;
		}
		Eigen::Matrix3d homography;
		homography << cvHomography.at<double>(0, 0), cvHomography.at<double>(0, 1), cvHomography.at<double>(0, 2),
			cvHomography.at<double>(1, 0), cvHomography.at<double>(1, 1), cvHomography.at<double>(1, 2),
			cvHomography.at<double>(2, 0), cvHomography.at<double>(2, 1), cvHomography.at<double>(2, 2);
		
		int numInliers = 0;
		std::vector<int> inliers;
		for (int idx = 0; idx < srcPoints.size(); idx++)
		{
			Eigen::Vector3d src(srcPoints[idx].x, srcPoints[idx].y, 1);
			Eigen::Vector3d dst(dstPoints[idx].x, dstPoints[idx].y, 1);
			Eigen::Vector3d est_dst = homography * src;
			est_dst /= est_dst.z();

			if ((dst - est_dst).head<2>().norm() < pixThreshold)
			{
				numInliers++;
				inliers.push_back(idx);
			}
		}

		if (numInliers > maxNumInlier)
		{
			maxNumInlier = numInliers;
			est_Homography = cvHomography;
			maxInliers = inliers;
			float eps = float(maxNumInlier) / numPoints;
			stoppingCriterion = floor(log(1 - 0.999) / log(1 - pow(eps, 2)) + 1);
		}
	}

	std::cout << "inlierNum : " << maxNumInlier << std::endl;
	std::vector<cv::Point> src_inliers;
	std::vector<cv::Point> dst_inliers;
	for (int i = 0; i < maxInliers.size(); i++)
	{
		src_inliers.push_back(srcPoints[maxInliers[i]]);
		dst_inliers.push_back(dstPoints[maxInliers[i]]);
	}
	cv::Mat mask;
	est_Homography = cv::findHomography(src_inliers, dst_inliers, mask, 0);
	//est_Homography = DLT_Homography(src_inliers, dst_inliers);
	return est_Homography;

}


int main(void)
{
	cv::Mat img0 = cv::imread("../image/shiba04.bmp", cv::IMREAD_GRAYSCALE);
	cv::Mat img1 = cv::imread("../image/shiba01.bmp", cv::IMREAD_GRAYSCALE);

	cv::imshow("img", img1);
	cv::waitKey(0);
	auto algorithm = cv::AKAZE::create();

	std::vector<cv::KeyPoint> keypoints0, keypoints1;
	cv::Mat descriptor0, descriptor1;
	cv::Mat mask;
	algorithm->detectAndCompute(img0, mask, keypoints0, descriptor0, false);
	algorithm->detectAndCompute(img1, mask, keypoints1, descriptor1, false);
	
	cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce");
	std::vector<cv::DMatch> match12, match21;
	matcher->match(descriptor0, descriptor1, match12);
	matcher->match(descriptor1, descriptor0, match21);

	std::vector<cv::DMatch> match;
	for (int i = 0; i < match12.size(); i++)
	{
		cv::DMatch forward = match12[i];
		cv::DMatch backward = match21[forward.trainIdx];
		if (backward.trainIdx == forward.queryIdx)
		{
			match.push_back(forward);
		}
	}

	std::vector<cv::Point> pts1;
	std::vector<cv::Point> pts2;

	for (int i = 0; i < match.size(); i++)
	{
		pts1.emplace_back(keypoints0[match[i].queryIdx].pt);
		pts2.emplace_back(keypoints1[match[i].trainIdx].pt);
	}
	cv::Mat mask0;
	std::cout << "b " << std::endl;

	cv::Mat homography_cv = cv::findHomography(pts1, pts2, mask0, cv::RANSAC, 3);
	cv::Mat homography = findHomographyWithRansac(pts1, pts2);

	std::cout << "a " << std::endl;
	Eigen::Matrix3d h;
	h << homography.at<double>(0, 0), homography.at<double>(0, 1), homography.at<double>(0, 2),
		homography.at<double>(1, 0), homography.at<double>(1, 1), homography.at<double>(1, 2),
		homography.at<double>(2, 0), homography.at<double>(2, 1), homography.at<double>(2, 2);
	std::cout << h << std::endl;
	std::cout << homography_cv << std::endl;
	Eigen::Vector3d p1(0, 0, 1);
	Eigen::Vector3d p2(640, 0, 1);
	Eigen::Vector3d p3(640, 480, 1);
	Eigen::Vector3d p4(0, 480, 1);
	p1 = h * p1;
	p2 = h * p2;
	p3 = h * p3;
	p4 = h * p4;
	p1 /= p1.z();
	p2 /= p2.z();
	p3 /= p3.z();
	p4 /= p4.z();

	cv::Mat dstimg = img0.clone();
	cv::warpPerspective(img0, dstimg, homography, dstimg.size());
	cv::imshow("dstimg", dstimg);
	cv::waitKey(0);

	std::cout << "p1 : " << p1.transpose() << std::endl;
	std::cout << "p2 : " << p2.transpose() << std::endl;
	std::cout << "p3 : " << p3.transpose() << std::endl;
	std::cout << "p4 : " << p4.transpose() << std::endl;
	cv::line(img1, cv::Point(p1.x(), p1.y()), cv::Point(p2.x(), p2.y()), cv::Scalar(255));
	cv::line(img1, cv::Point(p2.x(), p2.y()), cv::Point(p3.x(), p3.y()), cv::Scalar(255));
	cv::line(img1, cv::Point(p3.x(), p3.y()), cv::Point(p4.x(), p4.y()), cv::Scalar(255));
	cv::line(img1, cv::Point(p4.x(), p4.y()), cv::Point(p1.x(), p1.y()), cv::Scalar(255));

	cv::imshow("img1", img1);
	cv::waitKey(0);

	//cv::Size imageSize(640, 480);
	//const double fx = 8.093086e+02;
	//const double fy = 8.106449e+02;
	//const double cx = 3.199442e+02;
	//const double cy = 2.785334e+02;
	//const double k1 = 1.818949e-02;
	//const double k2 = -2.175777e-01;
	//const double k3 = 6.684171e-01;
	//const double p1 = 2.078000e-02;
	//const double p2 = 1.822711e-03;

	//const double focal = (fx + fy) / 2.0;
	//const cv::Point2d principalPoint(cx, cy);


	//cv::Mat E, R, t, mask0;
	//E = cv::findEssentialMat(pts1, pts2, focal, principalPoint, cv::RANSAC, 0.999, 1, mask0);
	//cv::Mat R1, R2;
	//cv::decomposeEssentialMat(E, R1, R2, t);
	//std::cout << "R1 : " << R1 << std::endl;
	//std::cout << "R2 : " << R2 << std::endl;
	//std::cout << "t : " << t << std::endl;

	return true;
}