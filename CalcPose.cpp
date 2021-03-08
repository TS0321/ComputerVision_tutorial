#include "CalcPose.hpp"

bool calcPose_Plane(const std::vector<Eigen::Vector3d>& objPoints, const std::vector<Eigen::Vector2d>& imgPoints, const Eigen::Matrix3d cameraMatrix, Eigen::Isometry3d& current_pose)
{
	if (objPoints.size() != imgPoints.size())
	{
		return false;
	}

	const int points_num = imgPoints.size();
	Eigen::Matrix<double, Eigen::Dynamic, 9> A(2 * points_num, 9);
	for (int i = 0; i < points_num; i++)
	{
		const double& X = objPoints[i].x();
		const double& Y = objPoints[i].y();
		const double& x = imgPoints[i].x();
		const double& y = imgPoints[i].y();

		Eigen::Matrix<double, 2, 9> a;
		a.setZero();
		a << X, Y, 1, 0, 0, 0, -x * X, -x * Y, -x,
			0, 0, 0, X, Y, 1, -y * X, -y * Y, -y;
		A.row(2 * i) = a.row(0);
		A.row(2 * i + 1) = a.row(1);
	}

	Eigen::Matrix<double, 9, 9> AtA;
	AtA = A.transpose() * A;

	Eigen::JacobiSVD<Eigen::Matrix<double, 9, 9>> svd(AtA, Eigen::ComputeFullU | Eigen::ComputeFullV);
	int min_sigularIndex = 0;
	Eigen::VectorXd sigular_valueVec = svd.singularValues();
	for (int i = 0; i < 9; i++)
	{
		if (sigular_valueVec(i) == 0) {
			min_sigularIndex = i - 1;
			break;
		}
		min_sigularIndex = i;
	}
	Eigen::Matrix<double, 9, 1> min_eigenVec = svd.matrixV().col(min_sigularIndex);

	Eigen::Matrix<double, 3, 3> H;
	H << min_eigenVec(0), min_eigenVec(1), min_eigenVec(2),
		min_eigenVec(3), min_eigenVec(4), min_eigenVec(5),
		min_eigenVec(6), min_eigenVec(7), min_eigenVec(8);

	Eigen::Matrix3d cameraMatrix_Inv = cameraMatrix.inverse();
	const double lambda1 = 1 / (cameraMatrix_Inv * H.col(0)).norm();
	const double lambda2 = 1 / (cameraMatrix_Inv * H.col(1)).norm();

	Eigen::Vector3d r1 = lambda1 * cameraMatrix_Inv * H.col(0);
	Eigen::Vector3d r2 = lambda2 * cameraMatrix_Inv * H.col(1);
	Eigen::Vector3d r3 = r1.cross(r2);
	Eigen::Vector3d t = lambda1 * cameraMatrix_Inv * H.col(2);
	Eigen::Matrix3d R;
	R << r1, r2, r3;
	current_pose.prerotate(R);
	current_pose.pretranslate(t);
}

bool calcPose(const std::vector<Eigen::Vector3d>& objPoints, const std::vector<Eigen::Vector2d>& imgPoints, Eigen::Isometry3d& current_pose)
{
	if (objPoints.size() != imgPoints.size()) 
	{
		return false;
	}
	const int points_num = imgPoints.size();
	Eigen::Matrix<double, Eigen::Dynamic, 12> A(2 * points_num, 12);
	Eigen::Vector3d object_point_average;
	Eigen::Vector2d detected_point_average;
	object_point_average.setZero();
	detected_point_average.setZero();


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

	object_point_average /= points_num;
	detected_point_average /= points_num;


	//std::cout << "A : " <<  A << std::endl;
	Eigen::Matrix<double, 12, 12> AtA;
	AtA = A.transpose() * A;
	//std::cout << "AtA : " <<  AtA << std::endl;
	Eigen::JacobiSVD<Eigen::MatrixXd> svd(AtA, Eigen::ComputeFullU | Eigen::ComputeFullV);
	int min_sigularIndex = 0;
	Eigen::VectorXd sigular_valueVec = svd.singularValues();
	for (int i = 0; i < 12; i++)
	{
		if(sigular_valueVec(i) == 0){
			min_sigularIndex = i - 1;
			break;
		}
		min_sigularIndex = i;
	}
	Eigen::Matrix<double, 12, 1> min_eigenVec = svd.matrixV().col(min_sigularIndex);



	Eigen::Matrix<double, 3, 4> P;
	P.setIdentity();
	P << min_eigenVec(0), min_eigenVec(1), min_eigenVec(2), min_eigenVec(3),
		min_eigenVec(4), min_eigenVec(5), min_eigenVec(6), min_eigenVec(7),
		min_eigenVec(8), min_eigenVec(9), min_eigenVec(10), min_eigenVec(11);
	//P /= min_eigenVec(11);
	std::cout << "P : " << P << std::endl;
	Eigen::Matrix<double, 3, 4> P_default;
	P_default = P;
	double scale = std::sqrt(P(0, 0) * P(0, 0) + P(0, 1) * P(0, 1) + P(0, 2) * P(0, 2)) * 
		std::sqrt(P(1, 0) * P(1, 0) + P(1, 1) * P(1, 1) + P(1, 2) * P(1, 2)) * 
		std::sqrt(P(2, 0) * P(2, 0) + P(2, 1) * P(2, 1) + P(2, 2) * P(2, 2));

	scale = std::pow(scale, 1 / 3.0);

	P = P / scale;
	Eigen::Vector4d world_point(objPoints[0].x(), objPoints[0].y(), objPoints[0].z(), 1);
	Eigen::Vector3d projected_point = P * world_point;
	if (projected_point(2) < 0)
	{
		P = -P;
		P_default = -P_default;
	}

	Eigen::Vector3d projected_point_ave;
	projected_point_ave.setZero();
	projected_point.setZero();
	Eigen::Vector3d objPoint_ave;
	objPoint_ave.setZero();
	for (int i = 0; i < points_num; i++)
	{
		Eigen::Vector4d worldPoint(objPoints[i].x(), objPoints[i].y(), objPoints[i].z(), 1);
		projected_point_ave += P * worldPoint;
		objPoint_ave += objPoints[i];
	}
	projected_point_ave /= points_num;
	objPoint_ave /= points_num;
	projected_point_ave = P * world_point;
	objPoint_ave = objPoints[0];


	Eigen::Matrix<double, 3, 3> R;
	R.setIdentity();
	R << P(0, 0), P(0, 1), P(0, 2),
		P(1, 0), P(1, 1), P(1, 2),
		P(2, 0), P(2, 1), P(2, 2);

	Eigen::JacobiSVD<Eigen::Matrix<double, 3, 3>> svd_R(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
	Eigen::Matrix<double, 3, 3> sigma;
	sigma.setIdentity();
	sigma(2, 2) = (svd_R.matrixU() * svd_R.matrixV().transpose()).determinant();
	R = svd_R.matrixU() * sigma * svd_R.matrixV().transpose();

	Eigen::Matrix<double, 3, 1> t;
	t.setZero();
	/*t(0) = P(0, 3);
	t(1) = P(1, 3);
	t(2) = P(2, 3);
	*/
	t = projected_point_ave - R * objPoint_ave;
	std::cout << "R : " << R << std::endl;
	std::cout << "t : " << t << std::endl;

	Eigen::Isometry3d pose;
	Eigen::Matrix<double, 4, 4> transMat;
	transMat.setIdentity();
	transMat << R, t;
	pose.matrix() = transMat;

	//Eigen::Matrix<double, 6, 6> JtJ;
	//Eigen::Matrix<double, 6, 1> JtE;

	//int cnt = 0;
	//while (1) {
	//	JtJ.setZero();
	//	JtE.setZero();
	//	if (cnt++ > 10)
	//	{
	//		break;
	//	}

	//	for (int i = 0; i < points_num; i++)
	//	{
	//		const Eigen::Matrix<double, 3, 3> R_Mat = pose.rotation().matrix();
	//		const Eigen::Vector3d transVec = pose.translation().matrix();

	//		Eigen::Matrix<double, 2, 3> Ja;
	//		Eigen::Matrix<double, 3, 6> Jb;
	//		Ja.setZero();
	//		Jb.setZero();
	//		const double& X = objPoints[i].x();
	//		const double& Y = objPoints[i].y();
	//		const double& Z = objPoints[i].z();
	//		Eigen::Vector3d objPoint(X, Y, Z);
	//		Eigen::Vector3d cam_Point;
	//		cam_Point = R_Mat * objPoint + transVec;
	//		const double X_ = cam_Point.x();
	//		const double Y_ = cam_Point.y();
	//		const double Z_ = cam_Point.z();
	//		//std::cout << "camPoint : " << cam_Point << std::endl;
	//		Eigen::Vector3d R_objPoint;
	//		R_objPoint.setZero();
	//		R_objPoint = R_Mat * objPoint;
	//		const double& Xr = R_objPoint(0);
	//		const double& Yr = R_objPoint(1);
	//		const double& Zr = R_objPoint(2);

	//		Ja(0, 0) = 1 / Z_;
	//		Ja(0, 2) = -X_ / (Z_ * Z_);
	//		Ja(1, 1) = 1 / Z_;
	//		Ja(1, 2) = -Y_ / (Z_ * Z_);

	//		Jb(0, 0) = 0;
	//		Jb(0, 1) = Zr;
	//		Jb(0, 2) = -Yr;
	//		Jb(0, 3) = 1;
	//		Jb(1, 0) = -Zr;
	//		Jb(1, 2) = Xr;
	//		Jb(1, 4) = 1;
	//		Jb(2, 0) = Yr;
	//		Jb(2, 1) = -Xr;
	//		Jb(2, 5) = 1;


	//		Eigen::Matrix<double, 2, 6> J;
	//		J = Ja * Jb;
	//		JtJ += J.transpose() * J;

	//		Eigen::AngleAxisd rot(R_Mat);
	//		//Eigen::Vector3d P_cam;
	//		//P_cam = rot.matrix() * objPoint + t;
	//		Eigen::Vector2d projected_point;
	//		projected_point.setZero();
	//		projected_point(0) = X_ / Z_;
	//		projected_point(1) = Y_ / Z_;

	//		Eigen::Vector2d E;
	//		E.setZero();
	//		E = imgPoints[i] - projected_point;
	//		E = -E;

	//		JtE += J.transpose() * E;
	//	}


	//	Eigen::FullPivLU< Eigen::Matrix<double, 6, 6>> lu(JtJ);
	//	//std::cout << "JtJ : " << JtJ << std::endl;
	//	Eigen::Matrix<double, 6, 1> x;
	//	x = lu.solve(JtE);
	//	//std::cout << "delta : " << x.norm() << std::endl;
	//	if (x.norm() < 0.0001) break;
	//	Eigen::AngleAxisd rot(pose.rotation().matrix());

	//	//Eigen::Isometry3d delta_pose;
	//	//delta_pose.setIdentity();
	//	Eigen::Vector3d axis(x(0), x(1), x(2));
	//	Eigen::AngleAxisd delta_rot(axis.norm(), axis.normalized());
	//	Eigen::Vector3d delta_trans(x(3), x(4), x(5));
	//	pose.prerotate(delta_rot);
	//	pose.pretranslate(delta_trans);

	//}

	current_pose = pose;



	return true;
}

void normalize(const double fx, const double fy, const double cx, const double cy, const double u, const double v, double& x, double& y)
{
	x = (u - cx) / fx;
	y = (v - cy) / fy;
}