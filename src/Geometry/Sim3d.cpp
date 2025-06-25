#include "Sim3d.h"

using namespace std;

CSim3D::CSim3D() noexcept
{
	scale = 1;
	rotation = Eigen::Quaterniond::Identity();
	translation = Eigen::Vector3d::Zero();
}
CSim3D::CSim3D(double scale, const Eigen::Quaterniond& rotation, const Eigen::Vector3d& translation) noexcept
{
	this->scale = scale;
	this->rotation = rotation;
	this->translation = translation;
}
CSim3D CSim3D::Inverse() const
{
	CSim3D inverse;
	inverse.scale = 1.0 / scale;
	inverse.rotation = rotation.inverse();
	inverse.translation = (inverse.rotation * translation) / -scale;
	return inverse;
}
void CSim3D::SaveToFile(const string path) const
{
	ofstream ofs(path, ios::trunc);
	CHECK(ofs.good());
	ofs.precision(17);
	ofs << scale << " " << rotation.w() << " " << rotation.x() << " "
		<< rotation.y() << " " << rotation.z() << " " << translation.x() << " "
		<< translation.y() << " " << translation.z() << endl;
}
void CSim3D::ReadFromFile(const string path)
{
	ifstream ifs(path);
	CHECK(ifs.good());
	ifs >> scale >> rotation.w() >> rotation.x() >> rotation.y() >> rotation.z() >> translation(0) >> translation(1) >> translation(2);
}
Eigen::Matrix3x4d CSim3D::ToMatrix() const noexcept
{
	Eigen::Matrix3x4d matrix;
	matrix.leftCols<3>() = scale * rotation.toRotationMatrix();
	matrix.col(3) = translation;
	return matrix;
}
bool CSim3D::Estimate(const vector<Eigen::Vector3d>& srcPoints, const vector<Eigen::Vector3d>& targetPoints)
{
	CSimilarityTransformEstimator SimilarityTransformEstimator;
	const vector<Eigen::Matrix<double, 3, 4>> result = SimilarityTransformEstimator.Estimate(srcPoints, targetPoints);
	if (result.empty())
	{
		return false;
	}
	CHECK(result.size() == 1);
	scale = result[0].col(0).norm();
	rotation = Eigen::Quaterniond(result[0].leftCols<3>() / scale).normalized();
	translation = result[0].rightCols<1>();
	return true;
}
