#pragma once

#include <vector>
#include <unordered_map>
#include <unordered_set>

#include "Point3D.h"
#include "Database.h"

class CModel final
{
public:
	CModel(size_t modelID, CDatabase* database);

	inline size_t GetModelID() const
	{
		return modelID;
	}
	inline void SetModelID(size_t modelID)
	{
		this->modelID = modelID;
	}

	inline bool IsImageRegistered(size_t imageID) const
	{
		if (regImageIDs.find(imageID) == regImageIDs.end())
		{
			return false;
		}
		CHECK(database && database->IsImageExists(imageID));
		CHECK(database->GetImage(imageID).IsRegistered(modelID));
		return true;
	}
	inline size_t GetRegImagesNum() const noexcept
	{
		return regImageIDs.size();
	}
	
	inline size_t GetPoints3DNum() const noexcept
	{
		return points3D.size();
	}
	std::unordered_set<size_t> GetAllPoints3D() const;
	inline const CPoint3D& GetPoint3D(size_t point3DID) const
	{
		auto it = points3D.find(point3DID);
		CHECK(it != points3D.end());
		return it->second;
	}
	inline CPoint3D& GetPoint3D(size_t point3DID)
	{
		auto it = points3D.find(point3DID);
		CHECK(it != points3D.end());
		return it->second;
	}
	inline bool IsExistsPoint3D(size_t point3DID) const
	{
		return points3D.find(point3DID) != points3D.end();
	}
	size_t AddPoint3D(const Eigen::Vector3d& XYZ, const CTrack& track, const Eigen::Vector3ub& color);

	// 为已存在的3D点添加新的观测
	void AddObservation(size_t point3DID, const CTrackElement& trackElement);

	// 合并两个3D点并且返回新的3D点ID. 合并的原则: 根据两个3D点的Track长度进行加权平均
	size_t MergePoints3D(size_t point3D1ID, size_t point3D2ID);

	// 删除一个3D点, 以及在观测到的影像中所有它的关联
	void DeletePoint3D(size_t point3DID);

	// 删除某个影像的某个2D点与其已有的3D点之间的观测, 如果本来该3D点只能被2张影像观测到, 那么该3D点也会被删除
	void DeleteObservation(size_t imageID, size_t point2DID);

	// 注册影像
	void RegisterImage(size_t imageID);

	// 取消注册影像
	void DeRegisterImage(size_t imageID);

	// 通过缩放和平移对场景进行归一化, 以免在平差之后出现退化的可视化效果, 同时提高算法的数值稳定性
	// 平移: 使得相机中心或点位置的均值位于坐标系的原点
	// 缩放: 使得最小和最大的相机中心位于extent上, p0和p1表示被考虑的相机中心的最小和最大百分位数
	void Normalize(double extent = 10.0, double p0 = 0.1, double p1 = 0.9, bool isUseImage = true);

	// 计算3D点的质心
	Eigen::Vector3d ComputeCentroid(double p0 = 0.1, double p1 = 0.9) const;

	// 计算所有3D点的最小边界
	std::pair<Eigen::Vector3d, Eigen::Vector3d> ComputeBoundingBox(double p0 = 0.0, double p1 = 1.0) const;

	// 对所有影像和3D点应用三维相似性变换
	void Transform(const CSim3D& oldWorldToNewWorld);

	// 找出在当前模型和传入模型中都被注册的影像ID
	std::vector<size_t> FindCommonRegisteredImages(const CModel& otherModel) const;

	// 过滤掉重投影误差过大, 负深度或者交会角过小的3D点
	size_t FilterPoints3D(double maxReprojectionError, double minTriAngle, const std::unordered_set<size_t>& pointsToBeFiltered);
	size_t FilterPoints3DInImages(double maxReprojectionError, double minTriAngle, const std::unordered_set<size_t>& imageIDs);
	size_t FilterAllPoints3D(double maxReprojectionError, double minTriAngle);

	// 过滤掉负深度的观测
	size_t FilterObservationsWithNegativeDepth();

	// 过滤掉没有观测或者相机参数错误的影像
	std::vector<size_t> FilterImages(double minFocalLengthRatio, double maxFocalLengthRatio, double maxExtraParam);

	size_t ComputeNumObservations() const;
	double ComputeMeanTrackLength() const;
	double ComputeMeanObservationsPerRegImage() const;
	double ComputeMeanReprojectionError() const;

	void UpdatePoint3DErrors();

	bool ExtractColorsForImage(size_t imageID, const std::string& imagePath);

private:
	size_t modelID;
	CDatabase* const database;
	std::unordered_set<size_t> regImageIDs;
	std::unordered_map<size_t, CPoint3D> points3D;
	size_t nextPoint3DID;

	void SetObservationAsTriangulated(size_t imageID, size_t point2DID, bool isContinuedPoint3D);
	void ResetTriObservations(size_t imageID, size_t point2DID, bool isDeletedPoint3D);

	size_t FilterPoints3DWithSmallTriangulationAngle(double minTriAngle, const std::unordered_set<size_t>& points3DID);
	size_t FilterPoints3DWithLargeReprojectionError(double maxReprojectionError, const std::unordered_set<size_t>& points3DID);

	std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d> ComputeBoundsAndCentroid(double p0, double p1, bool isUseImages) const;
};


















