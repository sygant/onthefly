#pragma once
#include <string>
#include <stdexcept>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <algorithm>
#include <fstream>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <any>
#include <Windows.h>
#include <math.h>
#include "Math.h"
#include "Base.h"

#include <boost/stacktrace.hpp>
#include <boost/format.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/iostreams/device/mapped_file.hpp>

struct CBaseOptions
{
	std::string imageDir = "";
	virtual void Check() const = 0;
};

// SIFT描述子的归一化方式
enum class CSIFTNormalizationType
{
	L1_ROOT, // 先进行L1归一化, 然后求元素的平方根. 这种归一化通常优于标准的L2归一化. 参考文献: "Three things everyone should know to improve object retrieval", Relja Arandjelovic and Andrew Zisserman, CVPR 2012.
	L2  // 传统的L2标准化，即使每个描述子向量的L2范数为1
};
// SIFT特征提取器的选项参数
struct CSIFTExtractionOptions final : public CBaseOptions
{
	// 是否使用GPU
	bool isUseGPU = true;

	// 最大影像尺寸, 如果影像超过这个值, 影像会被缩小
	size_t maxImageSize = 3200;

	// 最多能检测到的特征点数量, 如果检测到的特征点数量超过这个值, 会保留那些尺度更大的特征点
	size_t maxNumFeatures = 8192;

	// 金字塔的总层数
	size_t numOctaves = 4;

	// 每个金字塔层级里的子层个数
	size_t octaveResolution = 3;

	// 用于特征点检测的峰值阈值
	double peakThreshold = 0.02 / octaveResolution;

	// 用于特征点检测的边缘阈值
	double edgeThreshold = 10.0;

	// 是否要对SIFT特征进行仿射形状的估计. 以定向椭圆而不是定向圆盘的形式估算SIFT特征的仿射形状
	bool isEstimateAffineShape = false;

	// 如果isEstimateAffineShape为false的话, 设定每个关键点最多可以有多少个方向
	size_t maxNumOrientations = 2;

	// 固定关键点的方向为0
	bool isUpRight = false;

	// 实现Domain-Size Pooling的相关参数, 计算的是检测的尺度周围多个尺度的平均SIFT描述符
	// 参考文献: "Domain-Size Pooling in Local Descriptors and Network Architectures", J. Dong and S. Soatto, CVPR 2015
	// 参考文献: "Comparative Evaluation of Hand-Crafted and Learned Local Features", Schönberger, Hardmeier, Sattler, Pollefeys, CVPR 2016.
	bool isDomainSizePooling = false;
	double dspMinScale = 1.0 / 6.0;
	double dspMaxScale = 3.0;
	size_t dspNumScales = 10;

	CSIFTNormalizationType normalizationType = CSIFTNormalizationType::L1_ROOT; // SIFT描述子的归一化方式

	inline void Check() const override
	{
		CHECK(peakThreshold > 0);
		CHECK(edgeThreshold > 0);
		CHECK(dspMinScale > 0);
		CHECK(dspMaxScale >= dspMinScale);
	}
};

// SIFT特征匹配器的选项参数
struct CSIFTMatchingOptions final : public CBaseOptions
{
	// 是否使用GPU
	bool isUseGPU = true;

	// 最大距离比率: 第一和第二最佳匹配之间的最大距离比. 0.8
	double maxRatio = 0.8;

	// 与最佳匹配之间的最大距离. 值越大, 匹配数越多. 512
	double maxDistance = 128;

	// 是否启用交叉检测. 当启用时, 一个匹配对只有在两个方向(A->B和B->A)都是最佳匹配时, 才算一个有效匹配
	bool doCrossCheck = true;

	// 最大匹配数
	size_t maxNumMatches = 8192;

	// 几何验证时的最大对极误差(单位: 像素). 3
	double maxError = 1;

	// 几何验证的置信度阈值
	double confidence = 0.999;

	// RANSAC迭代的最小和最大次数
	size_t minNumTrials = 100;
	size_t maxNumTrials = 10000;

	// 预先假设的最小内点比率, 用于确定RANSAC的最大迭代次数
	double minInlierRatio = 0.25;

	// 影像对被视作有效的双视几何需要的最小内点数
	size_t minNumInliers = 15;

	// 是否尝试估计多种双视几何模型
	bool isEstimateMultiModels = false;

	// 是否执行引导匹配
	bool isPerformGuidedMatching = false;

	// 是否强制使用单应矩阵来应对平面场景
	bool isPlanarScene = false;

	// 是否估算影像之间的相对位姿并且保存到数据库
	bool isComputeRelativePose = false;

	// 是否使用FLANN算法加速匹配
	bool isUseFLANN = true;

	inline void Check() const override
	{
		CHECK(maxRatio > 0);
		CHECK(maxDistance > 0);
		CHECK(maxError > 0);
		CHECK(maxNumTrials > 0);
		CHECK(maxNumTrials >= minNumTrials);
		CHECK(minInlierRatio >= 0 && minInlierRatio <= 1);
	}
};

// RANSAC算法的选项参数
struct CRANSACOptions final : public CBaseOptions
{
	// 如果样本的误差的平方不超过maxError, 那么就认为是内点
	double maxError = 4.0;	

	// 预先设定的最小内点比例, 用来计算RANSAC的最大迭代次数. 只会在小于maxNumTrials时适用
	double minInlierRatio = 0.25;

	// 置信度阈值. 如果当前找到一个不含外点的概率达到了这个阈值, 那么终止迭代
	double confidence = 0.999;

	// 基于指定置信值动态计算最大迭代次数的乘数. 基于指定置信度值动态计算的最大迭代次数的num_trials_multiplier的倍数
	double maxIterNumTrialsMultiplier = 3.0;

	// 从随机子集估计模型的最小/最大随机试验次数. RANSAC算法随机选择样本集合进行模型估计的最小/最大次数
	size_t minNumTrials = 100;
	size_t maxNumTrials = 10000;

	inline void Check() const override
	{
		CHECK(maxError > 0);
		CHECK(minInlierRatio >= 0 && minInlierRatio <= 1);
		CHECK(confidence >= 0 && confidence <= 1);
		CHECK(minNumTrials <= maxNumTrials);
	}
};

// 双视几何选项参数
struct CTwoViewGeometryOptions final :public CBaseOptions
{
	// 有效双视几何所需要的最小内点数量
	size_t minInliersNum = 15;

	// 通过估计本质矩阵E和基础矩阵F来判断相机是否校正正确
	// 如果E产生的内点数量接近或者超过阈值(=minE_F_InliersRatio*F产生的内点数), 那么就认为相机校正是正确的
	double minE_F_InliersRatio = 0.95;

	// 用于检测退化场景(平面或者纯旋转). 如果单应矩阵H产生的内点数比例接近对极几何的内点数比例(由maxH_InliersRatio控制), 那么就认为是退化的场景
	double maxH_InliersRatio = 0.8;
		
	// 用于检测是否存在水印. 如果"纯平移而且集中在边缘区域的"内点与总内点的比例超过waterMarkMinInlierRatio, 那么就认为存在水印
	double watermarkMinInlierRatio = 0.7;
	
	// 定义图像边界区域的宽度为影像对角线长度的watermarkBorderSize倍
	double watermarkBorderSize = 0.1;

	// 是否启用水印检测. 在图像边界区域(由watermarkBorderSize指定)计算满足纯平移运动条件的内点比率, 如果超过watermarkMinInlierRatio, 就认为存在水印
	bool isDetectWatermark = false;

	// 在多模型估计中是否忽略水印模型
	bool isIgnoreWatermarkInMultiple = true;

	// 是否只估计单应矩阵H(纯平面或者纯旋转场景)
	bool isForceUse_H = false;

	// 是否要计算相对位姿, 会有更多的计算量
	bool isComputeRelativePose = false;

	CRANSACOptions RANSACOptions; // 双视几何估计过程中使用的RANSAC算法的选项参数

	inline void Check() const override
	{
		CHECK(minInliersNum >= 0);
		CHECK(minE_F_InliersRatio >= 0 && minE_F_InliersRatio <= 1);
		CHECK(maxH_InliersRatio >= 0 && maxH_InliersRatio <= 1);
		CHECK(watermarkMinInlierRatio >= 0 && watermarkMinInlierRatio <= 1);
		CHECK(watermarkBorderSize >= 0 && watermarkBorderSize <= 1);
		RANSACOptions.Check();
	}
};

enum class CTriangulationResidualType
{
	CAngularError,
	CReprojectionError
};
// 三角测量选项参数
struct CEstimateTriangulationOptions final :public CBaseOptions
{
	double minTriAngle = 1.5 * M_PI / 180.0;
	CTriangulationResidualType residualType = CTriangulationResidualType::CAngularError;
	CRANSACOptions ransacOptions;
	inline void Check() const override
	{
		CHECK(minTriAngle >= 0);
		ransacOptions.Check();
	}
};


struct COptions final : public CBaseOptions
{
	CSIFTExtractionOptions SIFTExtractionOptions;
	CSIFTMatchingOptions SIFTMatchingOptions;
	CRANSACOptions RANSACOptions;
	CTwoViewGeometryOptions twoViewGeometryOptions;
	CEstimateTriangulationOptions estimateTriangulationOptions;


	inline void Check() const override
	{
		SIFTExtractionOptions.Check();
		SIFTMatchingOptions.Check();
		RANSACOptions.Check();
		twoViewGeometryOptions.Check();
		estimateTriangulationOptions.Check();
	}
};















