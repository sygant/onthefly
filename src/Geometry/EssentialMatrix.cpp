#include "EssentialMatrix.h"
using namespace std;

void DecomposeEssentialMatrix(const Eigen::Matrix3d& E, Eigen::Matrix3d& R1, Eigen::Matrix3d& R2, Eigen::Vector3d& t)
{
	// Step 1. 计算E的SVD
	Eigen::JacobiSVD<Eigen::Matrix3d> svd(E, Eigen::ComputeFullU | Eigen::ComputeFullV);

	// Step 2. 从SVD结果中提取出U和V矩阵
	Eigen::Matrix3d U = svd.matrixU();
	Eigen::Matrix3d V = svd.matrixV().transpose();

	// Step 3. 确保旋转矩阵的正确性. 如果U或V的行列式的值为负数, 那么就乘以-1
	if (U.determinant() < 0) 
	{
		U *= -1;
	}
	if (V.determinant() < 0) 
	{
		V *= -1;
	}

	// Step 4. 构造预定义的W矩阵. 用来计算可能的旋转矩阵R1和R2
	Eigen::Matrix3d W;
	W << 0, 1, 0, -1, 0, 0, 0, 0, 1;

	// Step 5. 使用U, V和W来计算所有可能的旋转矩阵
	R1 = U * W * V;
	R2 = U * W.transpose() * V;

	// Step 6. 平移向量t是U的最后一列, 还要归一化.
	t = U.col(2).normalized();
}
void EssentialMatrixToPose(const Eigen::Matrix3d& E, const vector<Eigen::Vector2d>& points1, const vector<Eigen::Vector2d>& points2, Eigen::Matrix3d& R, Eigen::Vector3d& t, vector<Eigen::Vector3d>& points3D)
{
	CHECK(points1.size() == points2.size());
	Eigen::Matrix3d R1;
	Eigen::Matrix3d R2;
	DecomposeEssentialMatrix(E, R1, R2, t);

	// 生成所有可能的投影矩阵组合
	const array<Eigen::Matrix3d, 4> R_cmbs{ {R1, R2, R1, R2} };
	const array<Eigen::Vector3d, 4> t_cmbs{ {t, t, -t, -t} };

	points3D.clear();
	for (size_t i = 0; i < R_cmbs.size(); i++) 
	{
		vector<Eigen::Vector3d> points3D_cmb;
		CheckCheirality(R_cmbs[i], t_cmbs[i], points1, points2, points3D_cmb);
		if (points3D_cmb.size() >= points3D.size()) 
		{
			R = R_cmbs[i];
			t = t_cmbs[i];
			points3D = points3D_cmb;
		}
	}
}
Eigen::Matrix3d PoseToEssentialMatrix(const CRigid3D& camera1ToCamera2)
{
	return CrossProductMatrix(camera1ToCamera2.translation.normalized()) * camera1ToCamera2.rotation.toRotationMatrix();
}
void FindOptimalImageObservations(const Eigen::Matrix3d& E, const Eigen::Vector2d& point1, const Eigen::Vector2d& point2, Eigen::Vector2d& optimalPoint1, Eigen::Vector2d& optimalPoint2)
{
	const Eigen::Vector3d& point1h = point1.homogeneous();
	const Eigen::Vector3d& point2h = point2.homogeneous();

	Eigen::Matrix<double, 2, 3> S;
	S << 1, 0, 0, 0, 1, 0;

	// 核线
	Eigen::Vector2d n1 = S * E * point2h;
	Eigen::Vector2d n2 = S * E.transpose() * point1h;

	const Eigen::Matrix2d E_tilde = E.block<2, 2>(0, 0);

	const double a = n1.transpose() * E_tilde * n2;
	const double b = (n1.squaredNorm() + n2.squaredNorm()) / 2.0;
	const double c = point1h.transpose() * E * point2h;
	const double d = sqrt(b * b - a * c);
	double lambda = c / (b + d);

	n1 -= E_tilde * lambda * n1;
	n2 -= E_tilde.transpose() * lambda * n2;

	lambda *= (2.0 * d) / (n1.squaredNorm() + n2.squaredNorm());

	optimalPoint1 = (point1h - S.transpose() * lambda * n1).hnormalized();
	optimalPoint2 = (point2h - S.transpose() * lambda * n2).hnormalized();
}
Eigen::Vector3d EssentialMatrixToEpipole(const Eigen::Matrix3d& E, bool isLeftImage)
{
	Eigen::Vector3d e;
	if (isLeftImage)
	{
		Eigen::JacobiSVD<Eigen::Matrix3d> svd(E, Eigen::ComputeFullV);
		e = svd.matrixV().block<3, 1>(0, 2);
	}
	else
	{
		Eigen::JacobiSVD<Eigen::Matrix3d> svd(E.transpose(), Eigen::ComputeFullV);
		e = svd.matrixV().block<3, 1>(0, 2);
	}
	return e;
}
Eigen::Matrix3d InvertEssentialMatrix(const Eigen::Matrix3d& E)
{
	return E.transpose();
}