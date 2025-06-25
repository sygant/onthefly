#pragma once
#include <vector>
#include <array>
#include <any>


#include "../Scene/Camera.h"
#include "../Scene/Projection.h"
#include "../Base/Base.h"
#include "../Base/Options.h"
#include "Common.h"

// 求解器. 抽象类
class CEstimator
{
public:
	const size_t minNumSamples; // 估计模型所需的理论最少样本数
	virtual std::vector<std::any> Estimate(const std::vector<std::any>& X, const std::vector<std::any>& Y) = 0;
	virtual void Residuals(const std::vector<std::any>& X, const std::vector<std::any>& Y, const std::any& model, std::vector<double>& residuals) = 0;

protected:
	explicit CEstimator(size_t minNumSamples);
	size_t GetNumTrials(size_t minNumSamples, size_t numInliers, size_t numSamples, double confidence, double numTrialsMultiplier) const;
};


struct CP3PEstimateRANSACReport
{
	bool isSuccess = false;          // RANSAC估计是否成功
	size_t numTrials = 0;            // RANSAC迭代的次数
	CSupport support;                // 估计模型的支持度
	std::vector<char> inlierMask;    // 掩码, 是内点的样本就为true
	Eigen::Matrix3x4d model;         // RANSAC估计出的模型
};
// P3P问题求解器. 参考文献: X.S. Gao, X.-R. Hou, J. Tang, H.-F. Chang. Complete Solution Classification for the Perspective-Three-Point Problem.
class CP3PEstimator final :public CEstimator
{
public:
	CP3PEstimator();
	
	// 从3对2D-3D点对应关系中估算P3P问题的解
	//
	// @param points2D   归一化的2D图像点. 实际类型应该是vector<Eigen::Vector2d>, vector的大小应该为3
	// @param points3D   3D点. 实际类型应该是vector<Eigen::Vector3d>, vector的大小应该为3
	//
	// @return           可能的相机姿态. 实际类型应该是vector<Eigen::Matrix3x4d>
	std::vector<std::any> Estimate(const std::vector<std::any>& points2D, const std::vector<std::any>& points3D) override;
	std::vector<Eigen::Matrix3x4d> Estimate(const std::vector<Eigen::Vector2d>& points2D, const std::vector<Eigen::Vector3d>& points3D) const;
	CP3PEstimateRANSACReport EstimateRANSAC(const std::vector<Eigen::Vector2d>& points2D, const std::vector<Eigen::Vector3d>& points3D, const CRANSACOptions& options, CSupportMeasurer* supportMeasurer = nullptr, CSampler* sampler = nullptr) const;
	CP3PEstimateRANSACReport EstimateLoRANSAC(const std::vector<Eigen::Vector2d>& points2D, const std::vector<Eigen::Vector3d>& points3D, const CRANSACOptions& options, CSupportMeasurer* supportMeasurer = nullptr, CSampler* sampler = nullptr) const;

	// 给定一组2D-3D点对应关系和一个投影矩阵, 计算平方重投影误差
	// 
	// @param points2D       归一化的2D图像点. 实际类型应该是vector<Eigen::Vector2d>
	// @param points3D       3D点. 实际类型应该是vector<Eigen::Vector3d>
	// @param projectMatrix  3×4的投影矩阵. 实际类型应该是Eigen::Matrix3x4d
	// @param residuals      输出: 平方重投影误差向量
	void Residuals(const std::vector<std::any>& points2D, const std::vector<std::any>& points3D, const std::any& projectMatrix, std::vector<double>& residuals) override;
	void Residuals(const std::vector<Eigen::Vector2d>& points2D, const std::vector<Eigen::Vector3d>& points3D, const Eigen::Matrix3x4d& projectMatrix, std::vector<double>& residuals) const;
};

// 使用EPnP方法求解PnP问题, 至少需要4个2D-3D对应关系. 参考文献: Lepetit, Vincent, Francesc Moreno-Noguer, and Pascal Fua. "Epnp: An accurate o (n) solution to the pnp problem."
// 该实现基于他们原始的开源发布, 但已移植到Eigen并包含对原始代码的多项改进
class CEPnPEstimator final : public CEstimator
{
public:
	CEPnPEstimator();

	// 从多对2D-3D点对应关系中估算P3P问题的解
	//
	// @param points2D   归一化的2D图像点. 实际类型应该是vector<Eigen::Vector2d>, vector的大小应该为3
	// @param points3D   3D点. 实际类型应该是vector<Eigen::Vector3d>, vector的大小应该为3
	//
	// @return           可能的相机姿态. 实际类型应该是vector<Eigen::Matrix3x4d>, vector的大小应该为0(估计不成功)或1
	std::vector<std::any> Estimate(const std::vector<std::any>& points2D, const std::vector<std::any>& points3D) override;
	std::vector<Eigen::Matrix3x4d> Estimate(const std::vector<Eigen::Vector2d>& points2D, const std::vector<Eigen::Vector3d>& points3D);

	// 给定一组2D-3D点对应关系和一个投影矩阵, 计算平方重投影误差
	// 
	// @param points2D       归一化的2D图像点. 实际类型应该是vector<Eigen::Vector2d>
	// @param points3D       3D点. 实际类型应该是vector<Eigen::Vector3d>
	// @param projectMatrix  3×4的投影矩阵. 实际类型应该是Eigen::Matrix3x4d
	// @param residuals      输出: 平方重投影误差向量
	void Residuals(const std::vector<std::any>& points2D, const std::vector<std::any>& points3D, const std::any& projectMatrix, std::vector<double>& residuals) override;
	void Residuals(const std::vector<Eigen::Vector2d>& points2D, const std::vector<Eigen::Vector3d>& points3D, const Eigen::Matrix3x4d& projectMatrix, std::vector<double>& residuals);

private:
	const std::vector<Eigen::Vector2d>* points2D = nullptr;
	const std::vector<Eigen::Vector3d>* points3D = nullptr;
	std::vector<Eigen::Vector3d> pcs;
	std::vector<Eigen::Vector4d> alphas;
	std::array<Eigen::Vector3d, 4> cws;
	std::array<Eigen::Vector3d, 4> ccs;

	bool ComputePose(const std::vector<Eigen::Vector2d>& points2D, const std::vector<Eigen::Vector3d>& points3D, Eigen::Matrix3x4d& projectMatrix);
	void ChooseControlPoints();
	bool ComputeBarycentricCoordinates();
	Eigen::Matrix<double, Eigen::Dynamic, 12> ComputeM();
	Eigen::Matrix<double, 6, 10> ComputeL6x10(const Eigen::Matrix<double, 12, 12>& Ut);
	Eigen::Matrix<double, 6, 1> ComputeRho();

	// betas10        = [B11 B12 B22 B13 B23 B33 B14 B24 B34 B44]
    // betasApprox_1  = [B11 B12     B13         B14            ]
	void FindBetasApprox1(const Eigen::Matrix<double, 6, 10>& L_6x10, const Eigen::Matrix<double, 6, 1>& rho, Eigen::Vector4d& betas);

	// betas10        = [B11 B12 B22 B13 B23 B33 B14 B24 B34 B44]
    // betasApprox_2  = [B11 B12 B22                            ]
	void FindBetasApprox2(const Eigen::Matrix<double, 6, 10>& L_6x10, const Eigen::Matrix<double, 6, 1>& rho, Eigen::Vector4d& betas);

	// betas10        = [B11 B12 B22 B13 B23 B33 B14 B24 B34 B44]
    // betasApprox_3  = [B11 B12 B22 B13 B23                    ]
	void FindBetasApprox3(const Eigen::Matrix<double, 6, 10>& L_6x10, const Eigen::Matrix<double, 6, 1>& rho, Eigen::Vector4d& betas);
	void RunGaussNewton(const Eigen::Matrix<double, 6, 10>& L_6x10, const Eigen::Matrix<double, 6, 1>& rho, Eigen::Vector4d& betas);
	double ComputeRT(const Eigen::Matrix<double, 12, 12>& Ut, const Eigen::Vector4d& betas, Eigen::Matrix3d& R, Eigen::Vector3d& t);
	void ComputeCcs(const Eigen::Vector4d& betas, const Eigen::Matrix<double, 12, 12>& Ut);
	void ComputePcs();
	void SolveForSign();
	void EstimateRT(Eigen::Matrix3d& R, Eigen::Vector3d& t);
	double ComputeTotalReprojectionError(const Eigen::Matrix3d& R, const Eigen::Vector3d& t);
};

struct CEssentialMatrixEstimate_5PointsRANSACReport
{
	bool isSuccess = false;          // RANSAC估计是否成功
	size_t numTrials = 0;            // RANSAC迭代的次数
	CSupport support;                // 估计模型的支持度
	std::vector<char> inlierMask;    // 掩码, 是内点的样本就为true
	Eigen::Matrix3d model;           // RANSAC估计出的模型
};
// 使用五点法从归一化2D点对估计本质矩阵E. 参考文献: D. Nister, An efficient solution to the five-point relative pose problem, IEEE-T-PAMI, 26(6), 2004.
class CEssentialMatrixEstimator_5Points final :public CEstimator
{
public:
	CEssentialMatrixEstimator_5Points();

	// 从至少5组对应点中估算10个可能的本质矩阵
	//
	// @param points1   2D点. 实际类型应该是vector<Eigen::Vector2d>
	// @param points2   2D点. 实际类型应该是vector<Eigen::Vector2d>
	//
	// @return          所有可能的3×3本质矩阵. 实际类型应该是vector<Eigen::Matrix3d>, vector的大小应该为10
	std::vector<std::any> Estimate(const std::vector<std::any>& points1, const std::vector<std::any>& points2) override;
	std::vector<Eigen::Matrix3d> Estimate(const std::vector<Eigen::Vector2d>& points1, const std::vector<Eigen::Vector2d>& points2) const;
	CEssentialMatrixEstimate_5PointsRANSACReport EstimateLoRANSAC(const std::vector<Eigen::Vector2d>& points1, const std::vector<Eigen::Vector2d>& points2, const CRANSACOptions& options, CSupportMeasurer* supportMeasurer = nullptr, CSampler* sampler = nullptr) const;

	// 给定一组对应点和本质矩阵, 计算平方的Sampson误差
	// 
	// @param points1       第一组对应点. 实际类型应该是vector<Eigen::Vector2d>
	// @param points2       第二组对应点. 实际类型应该是vector<Eigen::Vector2d>
	// @param E             3×3的本质矩阵. 实际类型应该是Eigen::Matrix3d
	// @param residuals     输出: 平方的Sampson误差
	void Residuals(const std::vector<std::any>& points1, const std::vector<std::any>& points2, const std::any& E, std::vector<double>& residuals) override;
	void Residuals(const std::vector<Eigen::Vector2d>& points1, const std::vector<Eigen::Vector2d>& points2, const Eigen::Matrix3d& E, std::vector<double>& residuals) const;

private:
	void CalculateA(const Eigen::Matrix<double, 9, 4>& E, Eigen::Matrix<double, 10, 20>& A) const;
	void CalculateCoeffs(const Eigen::Matrix<double, 13, 3>& B, Eigen::Matrix<double, 11, 1>& coeffs) const;

};

// 使用八点法从归一化2D点对估计本质矩阵E. 参考文献: Hartley and Zisserman, Multiple View Geometry, algorithm 11.1, page 282.
class CEssentialMatrixEstimator_8Points final :public CEstimator
{
public:
	CEssentialMatrixEstimator_8Points();

	// 从至少8组对应点中估算本质矩阵
	//
	// @param points1   2D点. 实际类型应该是vector<Eigen::Vector2d>
	// @param points2   2D点. 实际类型应该是vector<Eigen::Vector2d>
	//
	// @return          3×3本质矩阵. 实际类型应该是vector<Eigen::Matrix3d>, vector的大小应该为1
	std::vector<std::any> Estimate(const std::vector<std::any>& points1, const std::vector<std::any>& points2) override;
	std::vector<Eigen::Matrix3d> Estimate(const std::vector<Eigen::Vector2d>& points1, const std::vector<Eigen::Vector2d>& points2) const;

	// 给定一组对应点和本质矩阵, 计算平方的Sampson误差
	// 
	// @param points1       第一组对应点. 实际类型应该是vector<Eigen::Vector2d>
	// @param points2       第二组对应点. 实际类型应该是vector<Eigen::Vector2d>
	// @param E             3×3的本质矩阵. 实际类型应该是Eigen::Matrix3d
	// @param residuals     输出: 平方的Sampson误差
	void Residuals(const std::vector<std::any>& points1, const std::vector<std::any>& points2, const std::any& E, std::vector<double>& residuals) override;
	void Residuals(const std::vector<Eigen::Vector2d>& points1, const std::vector<Eigen::Vector2d>& points2, const Eigen::Matrix3d& E, std::vector<double>& residuals) const;
};

struct CFundamentalMatrixEstimate_7PointsRANSACReport
{
	bool isSuccess = false;          // RANSAC估计是否成功
	size_t numTrials = 0;            // RANSAC迭代的次数
	CSupport support;                // 估计模型的支持度
	std::vector<char> inlierMask;    // 掩码, 是内点的样本就为true
	Eigen::Matrix3d model;           // RANSAC估计出的模型
};
// 使用七点法估计基础矩阵F. 参考文献: Zhengyou Zhang and T. Kanade, Determining the Epipolar Geometry and its Uncertainty: A Review, International Journal of Computer Vision, 1998.
class CFundamentalMatrixEstimator_7Points final :public CEstimator
{
public:
	CFundamentalMatrixEstimator_7Points();

	// 从7对点中估计1个或3个可能的基础矩阵F
	//
	// @param points1   2D点. 实际类型应该是vector<Eigen::Vector2d>, vector的大小应该为7
	// @param points2   2D点. 实际类型应该是vector<Eigen::Vector2d>, vector的大小应该为7
	//
	// @return          3×3基础矩阵. 实际类型应该是vector<Eigen::Matrix3d>, vector的大小不超过3
	std::vector<std::any> Estimate(const std::vector<std::any>& points1, const std::vector<std::any>& points2) override;
	std::vector<Eigen::Matrix3d> Estimate(const std::vector<Eigen::Vector2d>& points1, const std::vector<Eigen::Vector2d>& points2) const;
	CFundamentalMatrixEstimate_7PointsRANSACReport EstimateLoRANSAC(const std::vector<Eigen::Vector2d>& points1, const std::vector<Eigen::Vector2d>& points2, const CRANSACOptions& options, CSupportMeasurer* supportMeasurer = nullptr, CSampler* sampler = nullptr) const;

	// 给定一组对应点和基础矩阵, 计算平方的Sampson误差
	// 
	// @param points1       第一组对应点. 实际类型应该是vector<Eigen::Vector2d>
	// @param points2       第二组对应点. 实际类型应该是vector<Eigen::Vector2d>
	// @param F             3×3的基础矩阵. 实际类型应该是Eigen::Matrix3d
	// @param residuals     输出: 平方的Sampson误差
	void Residuals(const std::vector<std::any>& points1, const std::vector<std::any>& points2, const std::any& F, std::vector<double>& residuals) override;
	void Residuals(const std::vector<Eigen::Vector2d>& points1, const std::vector<Eigen::Vector2d>& points2, const Eigen::Matrix3d& F, std::vector<double>& residuals) const;
};

// 使用八点法估计基础矩阵F. 参考文献: Hartley and Zisserman, Multiple View Geometry, algorithm 11.1, page 282.
class CFundamentalMatrixEstimator_8Points final :public CEstimator
{
public:
	CFundamentalMatrixEstimator_8Points();

	// 从至少8对点中估计基础矩阵F
	//
	// @param points1   2D点. 实际类型应该是vector<Eigen::Vector2d>
	// @param points2   2D点. 实际类型应该是vector<Eigen::Vector2d>
	//
	// @return          3×3基础矩阵. 实际类型应该是vector<Eigen::Matrix3d>, vector的大小为1
	std::vector<std::any> Estimate(const std::vector<std::any>& points1, const std::vector<std::any>& points2) override;
	std::vector<Eigen::Matrix3d> Estimate(const std::vector<Eigen::Vector2d>& points1, const std::vector<Eigen::Vector2d>& points2) const;

	// 给定一组对应点和基础矩阵, 计算平方的Sampson误差
	// 
	// @param points1       第一组对应点. 实际类型应该是vector<Eigen::Vector2d>
	// @param points2       第二组对应点. 实际类型应该是vector<Eigen::Vector2d>
	// @param F             3×3的基础矩阵. 实际类型应该是Eigen::Matrix3d
	// @param residuals     输出: 平方的Sampson误差
	void Residuals(const std::vector<std::any>& points1, const std::vector<std::any>& points2, const std::any& F, std::vector<double>& residuals) override;
	void Residuals(const std::vector<Eigen::Vector2d>& points1, const std::vector<Eigen::Vector2d>& points2, const Eigen::Matrix3d& F, std::vector<double>& residuals) const;
};

struct CHomographyMatrixEstimateRANSACReport
{
	bool isSuccess = false;          // RANSAC估计是否成功
	size_t numTrials = 0;            // RANSAC迭代的次数
	CSupport support;                // 估计模型的支持度
	std::vector<char> inlierMask;    // 掩码, 是内点的样本就为true
	Eigen::Matrix3d model;           // RANSAC估计出的模型
};
// 使用DLT(直接线性变换)算法估计单应矩阵H, 至少需要4对点来进行最小二乘估计
class CHomographyMatrixEstimator final :public CEstimator
{
public:
	CHomographyMatrixEstimator();

	// 从至少4对点中估计单应矩阵H
	//
	// @param points1   2D点. 实际类型应该是vector<Eigen::Vector2d>
	// @param points2   2D点. 实际类型应该是vector<Eigen::Vector2d>
	//
	// @return          3×3齐次变换矩阵. 实际类型应该是vector<Eigen::Matrix3d>, vector的大小为1
	std::vector<std::any> Estimate(const std::vector<std::any>& points1, const std::vector<std::any>& points2) override;
	std::vector<Eigen::Matrix3d> Estimate(const std::vector<Eigen::Vector2d>& points1, const std::vector<Eigen::Vector2d>& points2) const;
	CHomographyMatrixEstimateRANSACReport EstimateLoRANSAC(const std::vector<Eigen::Vector2d>& points1, const std::vector<Eigen::Vector2d>& points2, const CRANSACOptions& options, CSupportMeasurer* supportMeasurer = nullptr, CSampler* sampler = nullptr) const;

	// 给定一组对应点和单应矩阵, 计算变换误差的平方
	// 
	// @param points1       第一组对应点. 实际类型应该是vector<Eigen::Vector2d>
	// @param points2       第二组对应点. 实际类型应该是vector<Eigen::Vector2d>
	// @param H             3×3的投影矩阵. 实际类型应该是Eigen::Matrix3d
	// @param residuals     输出: 变换误差的平方
	void Residuals(const std::vector<std::any>& points1, const std::vector<std::any>& points2, const std::any& H, std::vector<double>& residuals) override;
	void Residuals(const std::vector<Eigen::Vector2d>& points1, const std::vector<Eigen::Vector2d>& points2, const Eigen::Matrix3d& H, std::vector<double>& residuals) const;
};

// 估计仿射变换, 至少需要3对2D点
class CAffineTransformEstimator final : public CEstimator
{
public:
	CAffineTransformEstimator();

	// 从至少3组对应点中估计仿射变换
	//
	// @param points1   2D点. 实际类型应该是vector<Eigen::Vector2d>
	// @param points2   2D点. 实际类型应该是vector<Eigen::Vector2d>
	//
	// @return          估计的仿射变换矩阵. 实际类型应该是vector<Eigen::Matrix<double, 2, 3>>, vector的大小应该为1
	std::vector<std::any> Estimate(const std::vector<std::any>& points1, const std::vector<std::any>& points2) override;
	std::vector<Eigen::Matrix<double, 2, 3>> Estimate(const std::vector<Eigen::Vector2d>& points1, const std::vector<Eigen::Vector2d>& points2) const;

	// 计算每个点的变换误差的平方
	// 
	// @param points1       2D点. 实际类型应该是vector<Eigen::Vector2d>
	// @param points2       2D点. 实际类型应该是vector<Eigen::Vector2d>
	// @param A             仿射变换矩阵. 实际类型应该是Eigen::Matrix<double, 2, 3>
	// @param residuals     输出: 每个点的变换误差的平方
	void Residuals(const std::vector<std::any>& points1, const std::vector<std::any>& points2, const std::any& A, std::vector<double>& residuals) override;
	void Residuals(const std::vector<Eigen::Vector2d>& points1, const std::vector<Eigen::Vector2d>& points2, const Eigen::Matrix<double, 2, 3>& A, std::vector<double>& residuals) const;
};

// 根据源和目标坐标系统的对应点, 估计三维相似性变换
// 参考文献: S. Umeyama. Least-Squares Estimation of Transformation Parameters Between Two Point Patterns. IEEE Transactions on Pattern Analysis and Machine Intelligence, Volume 13 Issue 4, Page 376-380, 1991.
class CSimilarityTransformEstimator final : public CEstimator
{
public:
	// isEstimateScale表示"是否需要估计尺度"
	explicit CSimilarityTransformEstimator(bool isEstimateScale = true);

	// 估计三维相似性变换矩阵
	//
	// @param src   3D点. 实际类型应该是vector<Eigen::Vector3d>
	// @param dst   3D点. 实际类型应该是vector<Eigen::Vector3d>
	//
	// @return      估计的相似性变换矩阵. 实际类型应该是vector<Eigen::Matrix<double, 3, 4>>, vector的大小应该为1
	std::vector<std::any> Estimate(const std::vector<std::any>& src, const std::vector<std::any>& dst) override;
	std::vector<Eigen::Matrix<double, 3, 4>> Estimate(const std::vector<Eigen::Vector3d>& src, const std::vector<Eigen::Vector3d>& dst) const;

	// 计算每个点的变换残差(从源坐标系变换到目标坐标系时的平方变换误差)
	// 
	// @param src           3D点. 实际类型应该是vector<Eigen::Vector3d>
	// @param dst           3D点. 实际类型应该是vector<Eigen::Vector3d>
	// @param matrix        仿射变换矩阵. 实际类型应该是Eigen::Matrix<double, 3, 4>
	// @param residuals     输出: 每个点的变换残差
	void Residuals(const std::vector<std::any>& src, const std::vector<std::any>& dst, const std::any& matrix, std::vector<double>& residuals) override;
	void Residuals(const std::vector<Eigen::Vector3d>& src, const std::vector<Eigen::Vector3d>& dst, const Eigen::Matrix<double, 3, 4>& matrix, std::vector<double>& residuals) const;

private:
	bool isEstimateScale;
};

// 估计二维平移变换, 至少需要1对2D点
class CTranslationTransformEstimator final : public CEstimator
{
public:
	CTranslationTransformEstimator();

	// 估计二维平移变换
	//
	// @param points1   2D点. 实际类型应该是vector<Eigen::Vector2d>
	// @param points2   2D点. 实际类型应该是vector<Eigen::Vector2d>
	//
	// @return          平移向量. 实际类型应该是vector<Eigen::Vector2d>, vector的大小应该为1
	std::vector<std::any> Estimate(const std::vector<std::any>& points1, const std::vector<std::any>& points2) override;
	std::vector<Eigen::Vector2d> Estimate(const std::vector<Eigen::Vector2d>& points1, const std::vector<Eigen::Vector2d>& points2) const;

	// 计算平移误差的平方
	// 
	// @param points1       2D点. 实际类型应该是vector<Eigen::Vector2d>
	// @param points2       2D点. 实际类型应该是vector<Eigen::Vector2d>
	// @param translation   平移向量. 实际类型应该是Eigen::Vector2d
	// @param residuals     输出: 每一对点的残差
	void Residuals(const std::vector<std::any>& points1, const std::vector<std::any>& points2, const std::any& translation, std::vector<double>& residuals) override;
	void Residuals(const std::vector<Eigen::Vector2d>& points1, const std::vector<Eigen::Vector2d>& points2, const Eigen::Vector2d& translation, std::vector<double>& residuals) const;
};


struct CTriangulationPoint
{
	Eigen::Vector2d point;            // 影像测量值, 以像素为单位. 只需要在计算重投影误差(CReprojectionError)时设置
	Eigen::Vector2d pointNormalized;  // 归一化影像测量值. 始终都要设置

	CTriangulationPoint() = default;
	CTriangulationPoint(const Eigen::Vector2d& point, const Eigen::Vector2d& pointNormalized)
	{
		this->point = point;
		this->pointNormalized = pointNormalized;
	}
};
struct CTriangulationPose
{
	Eigen::Matrix3x4d projectionMatrix; // 观测影像的投影矩阵
	Eigen::Vector3d projectionCenter;   // 观测影像的投影中心
	const CCamera* camera = nullptr;    // 观测影像的相机

	CTriangulationPose() = default;
	CTriangulationPose(const Eigen::Matrix3x4d& projectionMatrix, const Eigen::Vector3d& pose, const CCamera* camera)
	{
		this->projectionMatrix = projectionMatrix;
		projectionCenter = pose;
		this->camera = camera;
	}
};
// 三角测量问题求解器, 从多个观测值估计3D点, 观测值由影像测量值以及相应的相机位姿及内参组成
// 必须满足2个约束条件: 1. 交会角必须足够大. 2. 所有观测都必须满足正景深约束(cheirality constraint)
class CTriangulationEstimator final : public CEstimator
{
public:
	CTriangulationEstimator(double minTriAngle, CTriangulationResidualType residualType);

	// 估计二维平移变换
	//
	// @param points   影像观测值. 实际类型应该是vector<CTriangulationPoint>
	// @param poses    相机位姿. 实际类型应该是vector<CTriangulationPose>
	//
	// @return         3D点的估计值. 实际类型应该是vector<Eigen::Vector3d>, 如果成功则vector的大小应该为1, 失败则vector的大小为0
	std::vector<std::any> Estimate(const std::vector<std::any>& points, const std::vector<std::any>& poses) override;
	std::vector<Eigen::Vector3d> Estimate(const std::vector<CTriangulationPoint>& points, const std::vector<CTriangulationPose>& poses) const;

	// 以平方重投影误差或角度误差的形式计算残差
	// 
	// @param points      影像观测值. 实际类型应该是vector<CTriangulationPoint>
	// @param poses       相机位姿. 实际类型应该是vector<CTriangulationPose>
	// @param XYZ         3D点. 实际类型应该是Eigen::Vector3d
	// @param residuals   输出: 每次观测的残差
	void Residuals(const std::vector<std::any>& points, const std::vector<std::any>& poses, const std::any& XYZ, std::vector<double>& residuals) override;
	void Residuals(const std::vector<CTriangulationPoint>& points, const std::vector<CTriangulationPose>& poses, const Eigen::Vector3d& XYZ, std::vector<double>& residuals) const;

private:
	CTriangulationResidualType residualType = CTriangulationResidualType::CReprojectionError;
	double minTriAngle = 0.0;
};





