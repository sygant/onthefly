#pragma once
#define EIGEN_USE_MKL_ALL
#include <algorithm>
#include <vector>

#include <tbb/tbb.h>
#include <Eigen/Core>

#include "Point2D.h"
#include "Point3D.h"
#include "Camera.h"
#include "../Base/Options.h"
#include "../Base/Math.h"
#include "../Geometry/Rigid3D.h"
#include "../Geometry/Triangulation.h"
#include "../Geometry/EssentialMatrix.h"
#include "../Geometry/HomographyMatrix.h"

// 双视几何关系类型
enum CTwoViewGeometryType :size_t
{
	CUndefined = 0,              // 双视几何关系尚未定义
	CDegenerate = 1,             // 退化的双视几何关系, 无法推断出稳健的几何模型. 原因: 双视图之间重叠区域不够, 匹配点(内点)不足
	CCalibrated = 2,             // 已校准的双视几何关系, 使用本质矩阵描述, 相机内参已知, 可以推断出准确的双视几何关系
	CUncalibrated = 3,           // 未校准的双视几何关系, 使用基础矩阵描述. 原因: 相机内参未知或不准确
	CPlanar = 4,                 // 平面双视几何关系, 存在平移关系, 使用单应矩阵
	CPanoramic = 5,              // 无平移关系的纯旋转双视几何关系, 使用单应矩阵
	CPlanarOrPanoramic = 6,      // 平面或纯旋转双视几何关系, 使用单应矩阵
	CWatermark = 7,              // 影像边缘的纯二维平移. 一般是图像边缘的水印或者标签
	CMultiple = 8,               // 以上多种双视几何类型的混合. 例如: 场景中有些物体是CCalibrated类型, 也有一些物体(例如墙壁)是CPlanar类型
	CTwoViewGeometryTypeNum = 9, // 共有9种双视几何类型
};

class CTwoViewGeometry final
{
public:
	size_t type = CTwoViewGeometryType::CUndefined; // 双视几何关系类型
	Eigen::Matrix3d E = Eigen::Matrix3d::Zero();    // 本质矩阵
	Eigen::Matrix3d F = Eigen::Matrix3d::Zero();    // 基础矩阵
	Eigen::Matrix3d H = Eigen::Matrix3d::Zero();    // 单应矩阵
	CRigid3D image1ToImage2;                        // 影像1到影像2的相对位姿(刚体变换)
	CSIFTMatches inlierMatches;                     // 内点
	double meanTriAngle = -1;                       // 三角化交会角的中位数

	// 反转双视几何关系来匹配交换的影像对(将当前双视几何关系调整为: 从影像2到影像1的相对位姿). 这个函数会改变成员变量.
	void Invert();

	// 相对定向. 估计双视几何的相对位姿
	bool EstimateRelativePose(const CCamera& camera1, const std::vector<Eigen::Vector2d>& points1, const CCamera& camera2, const std::vector<Eigen::Vector2d>& points2);

	// 根据相机中是否存在先验焦距, 从校准或未校准的影像对中估计双视几何
	void Estimate(const CCamera& camera1, const std::vector<Eigen::Vector2d>& points1, const CCamera& camera2, const std::vector<Eigen::Vector2d>& points2, const CSIFTMatches& matches, const CTwoViewGeometryOptions& options);

	// 从校准的影像对中估计双视几何
	void EstimateCalibrated(const CCamera& camera1, const std::vector<Eigen::Vector2d>& points1, const CCamera& camera2, const std::vector<Eigen::Vector2d>& points2, const CSIFTMatches& matches, const CTwoViewGeometryOptions& options);

	// 从未校准的影像对中估计双视几何
	void EstimateUncalibrated(const CCamera& camera1, const std::vector<Eigen::Vector2d>& points1, const CCamera& camera2, const std::vector<Eigen::Vector2d>& points2, const CSIFTMatches& matches, const CTwoViewGeometryOptions& options);

	// 从校准的影像对中估计单应矩阵
	void EstimateCalibratedHomography(const CCamera& camera1, const std::vector<Eigen::Vector2d>& points1, const CCamera& camera2, const std::vector<Eigen::Vector2d>& points2, const CSIFTMatches& matches, const CTwoViewGeometryOptions& options);
	
	// 估计多配置双视几何. 从匹配中移除先前的内点集来递归估计多个配置, 直到找不到足够的内点为止. 如果能估计出多个配置, 那么内点匹配将被连接起来, 配置类型将为CMultiple
	// 适用于具有复杂场景(多个刚性移动物体)以及大畸变的影像
	void EstimateMultipleType(const CCamera& camera1, const std::vector<Eigen::Vector2d>& points1, const CCamera& camera2, const std::vector<Eigen::Vector2d>& points2, const CSIFTMatches& matches, const CTwoViewGeometryOptions& options);

	// 检测内点匹配是否由水印引起. 水印在图像的边缘区域引起纯平移
	bool DetectWaterMark(const CCamera& camera1, const std::vector<Eigen::Vector2d>& points1, const CCamera& camera2, const std::vector<Eigen::Vector2d>& points2, size_t numInliers, const std::vector<char>& inlierMask, const CTwoViewGeometryOptions& options);
};