#include "Tracker.hpp"

void Tracker::set_camera(Camera* camera)
{
	this->camera = camera;
}

bool Tracker::calcPose(const std::vector<Eigen::Vector3d>& objPoints, const std::vector<Eigen::Vector2d>& imgPoints, Eigen::Isometry3d& current_pose)
{
	//3次元点と２次元点の数が異なるときを省く
	if (objPoints.size() != imgPoints.size())
	{
		return false;
	}

	const int points_num = imgPoints.size();
	Eigen::MatrixXd A(2 * points_num, 12);

	for (int i = 0; i < points_num; i++)
	{
		const double X = objPoints[i].x();
		const double Y = objPoints[i].y();
		const double Z = objPoints[i].z();
		const double x = imgPoints[i].x();
		const double y = imgPoints[i].y();

		Eigen::Matrix<double, 2, 12> a;
		a << X, Y, Z, 1, 0, 0, 0, 0, -x * X, -x * Y, -x * Z, -x,
			0, 0, 0, 0, X, Y, Z, 1, -y * X, -y * Y, -y * Z, -y;

		A.row(2 * i) = a.row(0);
		A.row(2 * i + 1) = a.row(1);
	}

	Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
	int min_sigularIndex = 0;	//最小特異値のインデックス
	/*最小特異ベクトルの抽出*/
	Eigen::VectorXd sigular_valueVec = svd.singularValues();
	for (int i = 0; i < 12; i++)
	{
		if (sigular_valueVec(i) == 0) {
			min_sigularIndex = i - 1;
			break;
		}
		min_sigularIndex = i;
	}
	Eigen::Matrix<double, 12, 1> min_eigenVec = svd.matrixV().col(min_sigularIndex);	//最小特異ベクトル

	Eigen::Matrix<double, 3, 4> P;	//透視投影行列
	P.setIdentity();
	P << min_eigenVec(0), min_eigenVec(1), min_eigenVec(2), min_eigenVec(3),
		min_eigenVec(4), min_eigenVec(5), min_eigenVec(6), min_eigenVec(7),
		min_eigenVec(8), min_eigenVec(9), min_eigenVec(10), min_eigenVec(11);

	/*回転行列R部分を正規化*/
	double scale = std::sqrt(P(0, 0) * P(0, 0) + P(0, 1) * P(0, 1) + P(0, 2) * P(0, 2)) *
		std::sqrt(P(1, 0) * P(1, 0) + P(1, 1) * P(1, 1) + P(1, 2) * P(1, 2)) *
		std::sqrt(P(2, 0) * P(2, 0) + P(2, 1) * P(2, 1) + P(2, 2) * P(2, 2));

	scale = std::pow(scale, 1 / 3.0);

	P = P / scale;
	Eigen::Vector4d world_point(objPoints[0].x(), objPoints[0].y(), objPoints[0].z(), 1);
	Eigen::Vector3d projected_point = P * world_point;
	/*カメラ座標のZが負の場合は符号を逆転*/
	if (projected_point.z() < 0)
	{
		P = -P;
	}

	/*回転行列を求める*/
	Eigen::Matrix<double, 3, 3> R;
	R.setIdentity();
	R = P.leftCols<3>();

	Eigen::JacobiSVD<Eigen::Matrix<double, 3, 3>> svd_R(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
	Eigen::Matrix<double, 3, 3> sigma;
	sigma.setIdentity();
	sigma(2, 2) = (svd_R.matrixU() * svd_R.matrixV().transpose()).determinant();
	R = svd_R.matrixU() * sigma * svd_R.matrixV().transpose();

	/*並進ベクトルを求める*/
	Eigen::Matrix<double, 3, 1> t;
	t.setZero();
	t = P.rightCols<1>();

	/*位置姿勢行列*/
	Eigen::Isometry3d pose;
	pose.setIdentity();
	pose.prerotate(R);
	pose.pretranslate(t);

	if (optimizePose(objPoints, imgPoints, pose, 5, 0.001))
	{
		current_pose = pose;
	}

	return true;
}
bool Tracker::optimizePose(const std::vector<Eigen::Vector3d>& objPoints, const std::vector<Eigen::Vector2d>& imgPoints, Eigen::Isometry3d& pose, const int iteration_num, const double error_thresh)
{
	if (objPoints.size() != imgPoints.size())
	{
		return false;
	}

	/*非線形最適化*/
	Eigen::Matrix<double, 6, 6> JtJ;
	Eigen::Matrix<double, 6, 1> JtE;


	//6DoF姿勢パラメータ[ωx, ωy, ωz, tx, ty, tz]
	Eigen::Matrix<double, 6, 1> Q;
	Q.setZero();
	Eigen::AngleAxisd rot(pose.rotation().matrix());
	Q << Eigen::Vector3d(rot.axis() * rot.angle()), pose.translation().matrix();

	const size_t points_num = objPoints.size();
	int cnt = 0;
	while (1) {
		JtJ.setZero();
		JtE.setZero();
		if (cnt++ > iteration_num)
		{
			break;
		}

		for (int i = 0; i < points_num; i++)
		{
			Eigen::Vector3d rotVec(Q(0), Q(1), Q(2));
			Eigen::AngleAxisd rot(rotVec.norm(), rotVec.normalized());
			const Eigen::Matrix<double, 3, 3> R_Mat = rot.toRotationMatrix();
			const Eigen::Vector3d transVec(Q(3), Q(4), Q(5));

			Eigen::Matrix<double, 2, 3> Ja;
			Eigen::Matrix<double, 3, 6> Jb;
			Ja.setZero();
			Jb.setZero();
			const Eigen::Vector3d objPoint = objPoints[i];
			Eigen::Vector3d cam_Point = R_Mat * objPoint + transVec;
			const double X = cam_Point.x();
			const double Y = cam_Point.y();
			const double Z = cam_Point.z();

			Eigen::Vector3d R_objPoint;
			R_objPoint.setZero();
			R_objPoint = R_Mat * objPoint;
			const double& Xr = R_objPoint(0);
			const double& Yr = R_objPoint(1);
			const double& Zr = R_objPoint(2);

			//ヤコビアンの計算
			Ja(0, 0) = 1 / Z;
			Ja(0, 2) = -X / (Z * Z);
			Ja(1, 1) = 1 / Z;
			Ja(1, 2) = -Y / (Z * Z);

			Jb(0, 0) = 0;
			Jb(0, 1) = Zr;
			Jb(0, 2) = -Yr;
			Jb(0, 3) = 1;
			Jb(1, 0) = -Zr;
			Jb(1, 2) = Xr;
			Jb(1, 4) = 1;
			Jb(2, 0) = Yr;
			Jb(2, 1) = -Xr;
			Jb(2, 5) = 1;


			Eigen::Matrix<double, 2, 6> J;
			J = Ja * Jb;
			JtJ += J.transpose() * J;

			Eigen::Vector2d projected_point;
			projected_point.setZero();
			projected_point(0) = X / Z;
			projected_point(1) = Y / Z;

			Eigen::Vector2d E;
			E.setZero();
			//再投影誤差の計算
			E = imgPoints[i] - projected_point;
			E = -E;

			JtE += J.transpose() * E;
		}

		Eigen::FullPivLU< Eigen::Matrix<double, 6, 6>> lu(JtJ);
		Eigen::Matrix<double, 6, 1> delta_Q;
		delta_Q = lu.solve(JtE);
		if (delta_Q.norm() < error_thresh) break;
		Q -= delta_Q;
	}

	Eigen::Vector3d rotVec(Q(0), Q(1), Q(2));

	pose.setIdentity();
	pose.prerotate(Eigen::AngleAxisd(rotVec.norm(), rotVec.normalized()));
	pose.pretranslate(Eigen::Vector3d(Q(3), Q(4), Q(5)));

	return true;
}

bool Tracker::detect()
{
	cv::Mat cap_img = camera->get_capImg();
	std::vector<cv::Point2f> centers;
	bool isFound = cv::findCirclesGrid(cap_img, pattern_size, centers);

	if (!isFound) return false;
	m_imgPoints.resize(centers.size());
	CamParam cparam = camera->get_camParam();
	const double fx = cparam.fx;
	const double fy = cparam.fy;
	const double cx = cparam.cx;
	const double cy = cparam.cy;
	for (int i = 0; i < centers.size(); i++)
	{
		const double u = centers[i].x;
		const double v = centers[i].y;
		double x = 0.0;
		double y = 0.0;
		normalize(fx, fy, cx, cy, u, v, x, y);
		Eigen::Vector2d imgpoint(x, y);
		m_imgPoints[i] = imgpoint;
	}

	if (m_objPoints.size() != m_imgPoints.size()) return false;

	return true;
}

void Tracker::create_3dPoints(const cv::Size pattern_size, const double pattern_interval)
{
	this->pattern_size = pattern_size;
	for (int y = 0; y < pattern_size.height; y++)
	{
		for (int x = 0; x < pattern_size.width; x++)
		{
			Eigen::Vector3d tmp_point(x * pattern_interval, y * pattern_interval, 0);
			m_objPoints.push_back(tmp_point);
		}
	}
}

bool Tracker::estimatePose()
{
	return calcPose(m_objPoints, m_imgPoints, m_currentPose);
}

Eigen::Isometry3d& Tracker::getPose()
{
	return m_currentPose;
}

void normalize(const double fx, const double fy, const double cx, const double cy, const double u, const double v, double& x, double& y)
{
	x = (u - cx) / fx;
	y = (v - cy) / fy;
}