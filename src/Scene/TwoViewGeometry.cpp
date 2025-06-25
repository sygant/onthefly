#include "TwoViewGeometry.h"

using namespace std;

CSIFTMatches ExtractInlierMatches(const CSIFTMatches& matches, size_t numInliers, const vector<char>& inlierMask)
{
	CSIFTMatches inlierMatches(numInliers);
	size_t currentIndex = 0;
	for (size_t i = 0; i < matches.size(); i++)
	{
		if (inlierMask[i])
		{
			inlierMatches[currentIndex] = matches[i];
			currentIndex++;
		}
	}
	return inlierMatches;
}
CSIFTMatches ExtractOutlierMatches(const CSIFTMatches& matches, const CSIFTMatches& inlierMatches)
{
	CHECK(matches.size() >= inlierMatches.size());
	unordered_set<pair<size_t, size_t>, MatchPairHash, MatchPairEqual> inlierMatchesMap;
	inlierMatchesMap.reserve(inlierMatches.size());
	for (const CSIFTMatch& match : inlierMatches)
	{
		inlierMatchesMap.insert(make_pair(match.point2DIndex1, match.point2DIndex2));
	}
	CSIFTMatches outlierMatches;
	outlierMatches.reserve(matches.size() - inlierMatches.size());
	for (const CSIFTMatch& match : matches)
	{
		if (inlierMatchesMap.find(make_pair(match.point2DIndex1, match.point2DIndex2)) == inlierMatchesMap.end())
		{
			outlierMatches.push_back(match);
		}
	}
	return outlierMatches;
}
inline bool IsImagePointInBoundingBox(const Eigen::Vector2d& point, const double minx, const double maxx, const double miny, const double maxy)
{
	return point.x() >= minx && point.x() <= maxx && point.y() >= miny && point.y() <= maxy;
}

void CTwoViewGeometry::Invert()
{
	F.transposeInPlace();
	E.transposeInPlace();
	H = H.inverse().eval();
	image1ToImage2 = image1ToImage2.Inverse();
	for (size_t i = 0; i < inlierMatches.size(); i++)
	{
		swap(inlierMatches[i].point2DIndex1, inlierMatches[i].point2DIndex2);
	}
}
bool CTwoViewGeometry::EstimateRelativePose(const CCamera& camera1, const vector<Eigen::Vector2d>& points1, const CCamera& camera2, const vector<Eigen::Vector2d>& points2)
{
	// Step 1. 检查有效的几何配置
	const size_t type_Num = static_cast<size_t>(type);
	if (type_Num < 2 || type_Num > 6) // 只有CCalibrated, CUncalibrated, CPlanar, CPanoramic, CPlanarOrPanoramic这类型的双视几何才能做相对定向
	{
		return false;
	}

	// Step 2. 提取归一化内点, 并将这些点从影像坐标转到归一化的相机坐标系
	const size_t numInlierMatches = inlierMatches.size();
	vector<Eigen::Vector2d> normalizedInlierPoints1(numInlierMatches), normalizedInlierPoints2(numInlierMatches);
	for (size_t i = 0; i < numInlierMatches; i++)
	{
		const CSIFTMatch& match = inlierMatches[i];
		normalizedInlierPoints1[i] = camera1.ImageToCamera(points1[match.point2DIndex1]);
		normalizedInlierPoints2[i] = camera2.ImageToCamera(points2[match.point2DIndex2]);
	}

	// Step 3. 计算相对位姿
	Eigen::Matrix3d R_Image1ToImage2;
	vector<Eigen::Vector3d> points3D;
	if (type == CTwoViewGeometryType::CCalibrated || type == CTwoViewGeometryType::CUncalibrated)
	{
		EssentialMatrixToPose(E, normalizedInlierPoints1, normalizedInlierPoints2, R_Image1ToImage2, image1ToImage2.translation, points3D);
	}
	else
	{
		Eigen::Vector3d normal;
		HomographyMatrixToPose(H, camera1.GetCalibrationMatrix(), camera2.GetCalibrationMatrix(), normalizedInlierPoints1, normalizedInlierPoints2, R_Image1ToImage2, image1ToImage2.translation, normal, points3D);
	}

	// Step 4. 设置旋转和平移
	image1ToImage2.rotation = Eigen::Quaterniond(R_Image1ToImage2);

	// Step 5. 计算三角化交会角
	if (points3D.empty())
	{
		meanTriAngle = 0;
	}
	else
	{
		meanTriAngle = GetMedian(CalculateTriangulationAngles(Eigen::Vector3d::Zero(), -R_Image1ToImage2.transpose() * image1ToImage2.translation, points3D));
	}

	// Step 6. 设置双视几何关系类型
	if (type == CTwoViewGeometryType::CPlanarOrPanoramic)
	{
		if (abs(image1ToImage2.translation.norm() < 1e-6))
		{
			type = CTwoViewGeometryType::CPanoramic;
			meanTriAngle = 0;
		}
		else
		{
			type = CTwoViewGeometryType::CPlanar;
		}
	}
	return true;
}
void CTwoViewGeometry::Estimate(const CCamera& camera1, const vector<Eigen::Vector2d>& points1, const CCamera& camera2, const vector<Eigen::Vector2d>& points2, const CSIFTMatches& matches, const CTwoViewGeometryOptions& options)
{
	if (options.isForceUse_H)
	{
		EstimateCalibratedHomography(camera1, points1, camera2, points2, matches, options);
	}
	else if (camera1.IsFocalLengthPrior() && camera2.IsFocalLengthPrior())
	{
		EstimateCalibrated(camera1, points1, camera2, points2, matches, options);
	}
	else
	{
		EstimateUncalibrated(camera1, points1, camera2, points2, matches, options);
	}
}
void CTwoViewGeometry::EstimateCalibrated(const CCamera& camera1, const vector<Eigen::Vector2d>& points1, const CCamera& camera2, const vector<Eigen::Vector2d>& points2, const CSIFTMatches& matches, const CTwoViewGeometryOptions& options)
{
	options.Check();
	if (matches.size() < options.minInliersNum) 
	{
		type = CTwoViewGeometryType::CDegenerate;
		return;
	}

	// 提取同名点
	vector<Eigen::Vector2d> matchedPoints1(matches.size());
	vector<Eigen::Vector2d> matchedPoints2(matches.size());
	vector<Eigen::Vector2d> matchedPoints1Normalized(matches.size());
	vector<Eigen::Vector2d> matchedPoints2Normalized(matches.size());

	for (size_t i = 0; i < matches.size(); i++)
	{
		const size_t index1 = matches[i].point2DIndex1;
		const size_t index2 = matches[i].point2DIndex2;
		matchedPoints1[i] = points1[index1];
		matchedPoints2[i] = points2[index2];
		matchedPoints1Normalized[i] = camera1.ImageToCamera(points1[index1]); // 做畸变校正
		matchedPoints2Normalized[i] = camera2.ImageToCamera(points2[index2]);
	}

	CEssentialMatrixEstimate_5PointsRANSACReport E_report;
	CFundamentalMatrixEstimate_7PointsRANSACReport F_report;
	CHomographyMatrixEstimateRANSACReport H_report;

	// 使用带有LoRANSAC的五点法估计本质矩阵. 传入的2D点要做畸变校正(从畸变的影像坐标系坐标转到理想的相机坐标系坐标)
	std::thread threadE([&]()
		{
			CRANSACOptions ransacOptions_E = options.RANSACOptions;
			ransacOptions_E.maxError = (camera1.ImageToCameraThreshold(ransacOptions_E.maxError) + camera2.ImageToCameraThreshold(ransacOptions_E.maxError)) / 2.0;
			CEssentialMatrixEstimator_5Points E_Estimator_5Points;
			E_report = E_Estimator_5Points.EstimateLoRANSAC(matchedPoints1Normalized, matchedPoints2Normalized, ransacOptions_E);
		});

	// 使用带有LoRANSAC的七点法估计基础矩阵, 其中, 局部估计器使用八点法的基础矩阵估计方法. 传入的2D点无需做畸变校正
	std::thread threadF([&]()
		{
			CFundamentalMatrixEstimator_7Points F_Estimator_7Points;
			F_report = F_Estimator_7Points.EstimateLoRANSAC(matchedPoints1, matchedPoints2, options.RANSACOptions);
		});

	// 使用带有LoRANSAC的DLT(直接线性变换)方法估计单应矩阵. 传入的2D点无需做畸变校正
	std::thread threadH([&]()
		{
			CHomographyMatrixEstimator HomographyMatrixEstimator;
			H_report = HomographyMatrixEstimator.EstimateLoRANSAC(matchedPoints1, matchedPoints2, options.RANSACOptions);
		});

	threadE.join();
	threadF.join();
	threadH.join();
	E = E_report.model;
	F = F_report.model;
	H = H_report.model;

	bool isAnySuccess = (E_report.isSuccess || F_report.isSuccess || H_report.isSuccess);
	bool isAnyEnoughNumInliers = (E_report.support.numInliers >= options.minInliersNum ||
		F_report.support.numInliers >= options.minInliersNum || H_report.support.numInliers >= options.minInliersNum);
	if (!isAnySuccess || !isAnyEnoughNumInliers)
	{
		type = CTwoViewGeometryType::CDegenerate;
		return;
	}

	// 确定E, F, H模型之间的内点比例
	const double E_F_InlierRatio = E_report.support.numInliers * 1.0 / F_report.support.numInliers;
	const double H_F_InlierRatio = H_report.support.numInliers * 1.0 / F_report.support.numInliers;
	const double H_E_InlierRatio = H_report.support.numInliers * 1.0 / E_report.support.numInliers;

	// 选择最好的模型
	const vector<char>* bestInlierMask = nullptr;
	size_t numInliers = 0;
	if (E_report.isSuccess && E_F_InlierRatio > options.minE_F_InliersRatio && E_report.support.numInliers >= options.minInliersNum)
	{
		bool isE_Better = (E_report.support.numInliers >= F_report.support.numInliers);
		bestInlierMask = (isE_Better ? &E_report.inlierMask : &F_report.inlierMask);

		numInliers = max(E_report.support.numInliers, F_report.support.numInliers);

		if (H_E_InlierRatio > options.maxH_InliersRatio)
		{
			type = CTwoViewGeometryType::CPlanarOrPanoramic;
			if (H_report.support.numInliers > numInliers)
			{
				numInliers = H_report.support.numInliers;
				bestInlierMask = &H_report.inlierMask;
			}
		}
		else
		{
			type = CTwoViewGeometryType::CCalibrated;
		}
	}
	else if (F_report.isSuccess && F_report.support.numInliers >= options.minInliersNum)
	{
		numInliers = F_report.support.numInliers;
		bestInlierMask = &F_report.inlierMask;

		if (H_F_InlierRatio > options.maxH_InliersRatio)
		{
			type = CTwoViewGeometryType::CPlanarOrPanoramic;
			if (H_report.support.numInliers > numInliers)
			{
				numInliers = H_report.support.numInliers;
				bestInlierMask = &H_report.inlierMask;
			}
		}
		else
		{
			type = CTwoViewGeometryType::CUncalibrated;
		}
	}
	else if (H_report.isSuccess && H_report.support.numInliers >= options.minInliersNum)
	{
		numInliers = H_report.support.numInliers;
		bestInlierMask = &H_report.inlierMask;
		type = CTwoViewGeometryType::CPlanarOrPanoramic;
	}
	else
	{
		type = CTwoViewGeometryType::CDegenerate;
		return;
	}

	inlierMatches = ExtractInlierMatches(matches, numInliers, *bestInlierMask);

	if (options.isDetectWatermark && DetectWaterMark(camera1, matchedPoints1, camera2, matchedPoints2, numInliers, *bestInlierMask, options))
	{
		type = CTwoViewGeometryType::CWatermark;
	}

	if (options.isComputeRelativePose)
	{
		EstimateRelativePose(camera1, points1, camera2, points2);
	}
}
void CTwoViewGeometry::EstimateUncalibrated(const CCamera& camera1, const vector<Eigen::Vector2d>& points1, const CCamera& camera2, const vector<Eigen::Vector2d>& points2, const CSIFTMatches& matches, const CTwoViewGeometryOptions& options)
{
	options.Check();
	if (matches.size() < options.minInliersNum)
	{
		type = CTwoViewGeometryType::CDegenerate;
		return;
	}

	vector<Eigen::Vector2d> matchedPoints1(matches.size());
	vector<Eigen::Vector2d> matchedPoints2(matches.size());
	for (size_t i = 0; i < matches.size(); i++)
	{
		matchedPoints1[i] = points1[matches[i].point2DIndex1];
		matchedPoints2[i] = points2[matches[i].point2DIndex2];
	}

	CFundamentalMatrixEstimate_7PointsRANSACReport F_report;
	CHomographyMatrixEstimateRANSACReport H_report;

	// 使用带有LoRANSAC的七点法估计基础矩阵, 其中, 局部估计器使用八点法的基础矩阵估计方法. 传入的2D点无需做畸变校正
	std::thread threadF([&]()
		{
			CFundamentalMatrixEstimator_7Points FundamentalMatrixEstimator_7Points;
			F_report = FundamentalMatrixEstimator_7Points.EstimateLoRANSAC(matchedPoints1, matchedPoints2, options.RANSACOptions);
		});

	// 使用带有LoRANSAC的DLT(直接线性变换)方法估计单应矩阵. 传入的2D点无需做畸变校正
	std::thread threadH([&]()
		{
			CHomographyMatrixEstimator HomographyMatrixEstimator;
			H_report = HomographyMatrixEstimator.EstimateLoRANSAC(matchedPoints1, matchedPoints2, options.RANSACOptions);
		});
	threadF.join();
	threadH.join();
	F = F_report.model;
	H = H_report.model;
	
	bool isAnySuccess = (F_report.isSuccess || H_report.isSuccess);
	bool isAnyEnoughNumInliers = (F_report.support.numInliers >= options.minInliersNum || H_report.support.numInliers >= options.minInliersNum);
	if (!isAnySuccess || !isAnyEnoughNumInliers)
	{
		type = CTwoViewGeometryType::CDegenerate;
		return;
	}

	// 确定F, H模型之间的内点比例
	const double H_F_InlierRatio = H_report.support.numInliers * 1.0 / F_report.support.numInliers;

	// 选择最好的模型
	const vector<char>* bestInlierMask = &F_report.inlierMask;
	size_t numInliers = F_report.support.numInliers;
	if (H_F_InlierRatio > options.maxH_InliersRatio)
	{
		type = CTwoViewGeometryType::CPlanarOrPanoramic;
		if (H_report.support.numInliers >= F_report.support.numInliers)
		{
			numInliers = H_report.support.numInliers;
			bestInlierMask = &H_report.inlierMask;
		}
	}
	else
	{
		type = CTwoViewGeometryType::CUncalibrated;
	}

	inlierMatches = ExtractInlierMatches(matches, numInliers, *bestInlierMask);

	if (options.isDetectWatermark && DetectWaterMark(camera1, matchedPoints1, camera2, matchedPoints2, numInliers, *bestInlierMask, options))
	{
		type = CTwoViewGeometryType::CWatermark;
	}
	if (options.isComputeRelativePose)
	{
		EstimateRelativePose(camera1, points1, camera2, points2);
	}
}
void CTwoViewGeometry::EstimateCalibratedHomography(const CCamera& camera1, const vector<Eigen::Vector2d>& points1, const CCamera& camera2, const vector<Eigen::Vector2d>& points2, const CSIFTMatches& matches, const CTwoViewGeometryOptions& options)
{
	options.Check();
	if (matches.size() < options.minInliersNum)
	{
		type = CTwoViewGeometryType::CDegenerate;
		return;
	}

	vector<Eigen::Vector2d> matchedPoints1(matches.size());
	vector<Eigen::Vector2d> matchedPoints2(matches.size());
	for (size_t i = 0; i < matches.size(); i++)
	{
		matchedPoints1[i] = points1[matches[i].point2DIndex1];
		matchedPoints2[i] = points2[matches[i].point2DIndex2];
	}

	// 使用带有LoRANSAC的DLT(直接线性变换)方法估计单应矩阵. 传入的2D点无需做畸变校正
	CHomographyMatrixEstimator H_Estimator;          // LoRANSAC全局估计器
	CHomographyMatrixEstimator H_Estimator_Local;    // LoRANSAC局部估计器
	CLORANSAC H_LoRANSAC(options.RANSACOptions, &H_Estimator, &H_Estimator_Local);
	const CRANSACReport H_report = H_LoRANSAC.Estimate<Eigen::Vector2d, Eigen::Vector2d, Eigen::Matrix3d>(matchedPoints1, matchedPoints2);
	H = H_report.model;

	if (!H_report.isSuccess || H_report.support.numInliers < options.minInliersNum)
	{
		type = CTwoViewGeometryType::CDegenerate;
		return;
	}
	else
	{
		type = CTwoViewGeometryType::CPlanarOrPanoramic;
	}
	inlierMatches = ExtractInlierMatches(matches, H_report.support.numInliers, H_report.inlierMask);

	if (options.isDetectWatermark && DetectWaterMark(camera1, matchedPoints1, camera2, matchedPoints2, H_report.support.numInliers, H_report.inlierMask, options))
	{
		type = CTwoViewGeometryType::CWatermark;
	}
	if (options.isComputeRelativePose)
	{
		EstimateRelativePose(camera1, points1, camera2, points2);
	}
}
void CTwoViewGeometry::EstimateMultipleType(const CCamera& camera1, const vector<Eigen::Vector2d>& points1, const CCamera& camera2, const vector<Eigen::Vector2d>& points2, const CSIFTMatches& matches, const CTwoViewGeometryOptions& options)
{
	CSIFTMatches remainingMatches = matches;
	vector<CTwoViewGeometry> geometries;
	while (true)
	{
		CTwoViewGeometry currentGeometry;
		currentGeometry.Estimate(camera1, points1, camera2, points2, remainingMatches, options);
		if (currentGeometry.type == CTwoViewGeometryType::CDegenerate)
		{
			break;
		}
		if (!options.isIgnoreWatermarkInMultiple || currentGeometry.type != CTwoViewGeometryType::CWatermark)
		{
			geometries.push_back(currentGeometry);
		}
		remainingMatches = ExtractOutlierMatches(remainingMatches, currentGeometry.inlierMatches);
	}
	if (geometries.empty())
	{
		type = CTwoViewGeometryType::CDegenerate;
	}
	else if (geometries.size() == 1)
	{
		*this = geometries[0];
	}
	else
	{
		type = CTwoViewGeometryType::CMultiple;
		for (const CTwoViewGeometry& currentGeometry : geometries)
		{
			inlierMatches.insert(inlierMatches.end(), currentGeometry.inlierMatches.begin(), currentGeometry.inlierMatches.end());
		}
	}
}
bool CTwoViewGeometry::DetectWaterMark(const CCamera& camera1, const vector<Eigen::Vector2d>& points1, const CCamera& camera2, const vector<Eigen::Vector2d>& points2, size_t numInliers, const vector<char>& inlierMask, const CTwoViewGeometryOptions& options)
{
	options.Check();

	// 检查边缘区域内的内点, 并提取这些内点匹配
	const double diagonal1 = sqrt(camera1.GetWidth() * camera1.GetWidth() + camera1.GetHeight() * camera1.GetHeight());
	const double diagonal2 = sqrt(camera2.GetWidth() * camera2.GetWidth() + camera2.GetHeight() * camera2.GetHeight());
	const double minx1 = options.watermarkBorderSize * diagonal1;
	const double miny1 = minx1;
	const double maxx1 = camera1.GetWidth() - minx1;
	const double maxy1 = camera1.GetHeight() - miny1;
	const double minx2 = options.watermarkBorderSize * diagonal2;
	const double miny2 = minx2;
	const double maxx2 = camera2.GetWidth() - minx2;
	const double maxy2 = camera2.GetHeight() - miny2;

	vector<Eigen::Vector2d> inlierPoints1(numInliers);
	vector<Eigen::Vector2d> inlierPoints2(numInliers);

	size_t numMatchesInBorder = 0;
	size_t currentIndex = 0;
	for (size_t i = 0; i < inlierMask.size(); i++)
	{
		if (inlierMask[i])
		{
			const Eigen::Vector2d& point1 = points1[i];
			const Eigen::Vector2d& point2 = points2[i];

			inlierPoints1[currentIndex] = point1;
			inlierPoints2[currentIndex] = point2;
			currentIndex++;

			if (!IsImagePointInBoundingBox(point1, minx1, maxx1, miny1, maxy1) && !IsImagePointInBoundingBox(point2, minx2, maxx2, miny2, maxy2))
			{
				numMatchesInBorder++;
			}
		}
	}
	const double matchesInBorderRatio = numMatchesInBorder * 1.0 / numInliers;

	if (matchesInBorderRatio < options.watermarkMinInlierRatio) 
	{
		return false;
	}

	CRANSACOptions ransacOptions = options.RANSACOptions;
	ransacOptions.minInlierRatio = options.watermarkMinInlierRatio;
	CTranslationTransformEstimator translationTransformEstimator;
	CTranslationTransformEstimator translationTransformEstimator_Local;
	CLORANSAC loransac(ransacOptions, &translationTransformEstimator, &translationTransformEstimator_Local);
	const CRANSACReport report = loransac.Estimate<Eigen::Vector2d, Eigen::Vector2d, Eigen::Vector2d>(inlierPoints1, inlierPoints2);

	const double inlierRatio = report.support.numInliers * 1.0 / numInliers;

	return inlierRatio >= options.watermarkMinInlierRatio;
}