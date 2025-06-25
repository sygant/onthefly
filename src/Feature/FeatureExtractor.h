#pragma once
#include "Bitmap.h"
#include "Common.h"
#include "../Base/types.h"
#include "../../thirdparty/VLFeat/sift.h"
#include "../../thirdparty/SIFTGPU/SiftGPU.h"

//#undef slots
//#include <cmath>
//#include <Python.h>
//#include "../Lib/site-packages/numpy/core/include/numpy/arrayobject.h"
//#define slots Q_SLOTS
#include "PyLoader.h"

#include <memory>
#include <string>

// SIFT描述子的归一化方式
enum class CSIFTNormalizationType
{
	L1_ROOT, // 先进行L1归一化, 然后求元素的平方根. 这种归一化通常优于标准的L2归一化. 参考文献: "Three things everyone should know to improve object retrieval", Relja Arandjelovic and Andrew Zisserman, CVPR 2012.
	L2  // 传统的L2标准化，即使每个描述子向量的L2范数为1
};
// SIFT特征提取器的选项参数
struct CSIFTExtractionOptions final
{
	// 是否使用GPU
	bool isUseGPU = true;

	// 最大影像尺寸, 如果影像超过这个值, 影像会被缩小
	size_t maxImageSize = 3200;

	// 最多能检测到的特征点数量, 如果检测到的特征点数量超过这个值, 会保留那些尺度更大的特征点
	size_t maxNumFeatures = 4096;

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
};



class CSIFTCPUExtractor final
{
public:
	using VlSiftType = std::unique_ptr<VlSiftFilt, void (*)(VlSiftFilt*)>;
	explicit CSIFTCPUExtractor(const CSIFTExtractionOptions& options);
	bool Extract(const Bitmap& bitmap, FeatureKeypoints& keypoints, FeatureDescriptors& descriptors);

private:
	CSIFTExtractionOptions options;
	VlSiftType sift;
};


class CSIFTGPUExtractor final
{
public:
	explicit CSIFTGPUExtractor(const CSIFTExtractionOptions& options);
	bool Extract(const Bitmap& bitmap, FeatureKeypoints& keypoints, FeatureDescriptors& descriptors);

private:
	CSIFTExtractionOptions options;
	SiftGPU siftGPU;
};


//struct CGlobalFeatureExtractonOptions
//{
//	std::string pthPath = "D:/CS/study/On_the_fly_SfM/src/Feature/GlobalFeature/vgg16-397923af.pth";
//	std::string HDF5Path = "D:/CS/study/On_the_fly_SfM/src/Feature/GlobalFeature/VGG16_64_desc_cen.hdf5";
//	std::string checkPointPath = "D:/CS/study/On_the_fly_SfM/src/Feature/GlobalFeature/VGG16_NetVlad_NoSplit.pth.tar";
//	std::string PCAModelPath = "D:/CS/study/On_the_fly_SfM/src/Feature/GlobalFeature/PCA_dims32768to2048.model";
//};

class CGlobalFeatureExtractor final
{
public:
	explicit CGlobalFeatureExtractor(const CGlobalFeatureExtractonOptions& options);
	~CGlobalFeatureExtractor();
	bool Extract(const std::string& imagePath, GlobalFeature& globalFeature);

private:
	const size_t globalFeatureDim = 2048;
	CGlobalFeatureExtractonOptions options;
	bool isInitializeError = false;

	void Initialize();
};


class CPythonExtractor final {
	// TODO: 提取参数设置
public:
	explicit CPythonExtractor();
	PyObject* Extract(const std::string& imagePath , FeatureKeypoints& keypoints , FeatureDescriptors& descriptors);
	void SaveFeatures(const std::string& imageName, PyObject* feat);
};

















