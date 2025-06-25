#pragma once
#include "Common.h"
#include "../Base/types.h"
#include <Eigen/Geometry>
#include <glog/logging.h>
#include <flann/flann.hpp>
#include "../../thirdparty/SIFTGPU/SiftGPU.h"
#include <Python.h>

// SIFT特征匹配器的选项参数
struct CSIFTMatchingOptions final
{
	// 是否使用GPU
	bool isUseGPU = true;

	// 最大距离比率: 第一和第二最佳匹配之间的最大距离比. 0.8
	double maxRatio = 0.8;

	// 与最佳匹配之间的最大距离. 值越大, 匹配数越多.
	double maxDistance = 0.7;

	// 是否启用交叉检测. 当启用时, 一个匹配对只有在两个方向(A->B和B->A)都是最佳匹配时, 才算一个有效匹配
	bool doCrossCheck = true;

	// 最大匹配数
	size_t maxNumMatches = 16384;

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

	inline void CheckOptions() const
	{
		CHECK(maxRatio > 0);
		CHECK(maxDistance > 0);
		CHECK(maxError > 0);
		CHECK(maxNumTrials > 0);
		CHECK(maxNumTrials >= minNumTrials);
		CHECK(minInlierRatio >= 0 && minInlierRatio <= 1);
	}
};

class CSIFTCPUMatcher final
{
public:
	explicit CSIFTCPUMatcher(const CSIFTMatchingOptions& options);
	FeatureMatches Match(const FeatureDescriptors& descriptors1, const FeatureDescriptors& descriptors2);

private:
	CSIFTMatchingOptions options;
	using FlannIndexType = flann::Index<flann::L2<uint8_t>>;

	FlannIndexType BuildFlannIndex(const FeatureDescriptors& descriptors);
};

class CSIFTGPUMatcher final
{
public:
	explicit CSIFTGPUMatcher(const CSIFTMatchingOptions& options);
	FeatureMatches Match(const FeatureDescriptors& descriptors1, const FeatureDescriptors& descriptors2);

	static size_t lastDescriptors1Index, lastDescriptors2Index;
	static bool isUploadDescriptors1, isUploadDescriptors2;
private:
	CSIFTMatchingOptions options;
	SiftMatchGPU siftMatchGPU;
};

class CPythonMatcher final {
public:
	explicit CPythonMatcher();
	FeatureMatches Match(const std::size_t imageID1 , const std::size_t imageID2);
	bool SimilarStructureFilter(const std::string imageID1, const std::string imageID2,const std::size_t imagePairId);
};

class Database;
class CGlobalFeatureRetriever final
{
public:
	explicit CGlobalFeatureRetriever(Database* database, size_t topN);
	std::vector<image_t> Retrieve(image_t imageID);

private:
	Database* database = nullptr;
	size_t topN = 30;
};

