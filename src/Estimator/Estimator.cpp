#include "../Base/Base.h"
#include "../Base/Math.h"

#include "Estimator.h"
#include "../Geometry/Triangulation.h"

using namespace std;

CEstimator::CEstimator(size_t minNumSamples) :minNumSamples(minNumSamples)
{
	CHECK(minNumSamples > 0);
}
size_t CEstimator::GetNumTrials(size_t minNumSamples, size_t numInliers, size_t numSamples, double confidence, double numTrialsMultiplier) const
{
    const double nom = 1 - confidence;
    if (nom <= 0)
    {
        return numeric_limits<size_t>::max();
    }
    const double inlierRatio = numInliers * 1.0 / numSamples;
    const double denom = 1 - pow(inlierRatio, minNumSamples);
    if (denom <= 0)
    {
        return 1;
    }
    if (abs(denom - 1) < 1e-6) // 防止下面除以0
    {
        return numeric_limits<size_t>::max();
    }

    return static_cast<size_t>(ceil(log(nom) / log(denom) * numTrialsMultiplier));
}


CP3PEstimator::CP3PEstimator():CEstimator(3)
{

}
vector<any> CP3PEstimator::Estimate(const vector<any>& points2D_Any, const vector<any>& points3D_Any)
{
	CHECK(points2D_Any.size() == 3 && points3D_Any.size() == 3);
	CHECK(points2D_Any[0].type() == typeid(Eigen::Vector2d) && points3D_Any[0].type() == typeid(Eigen::Vector3d));

    return TypeVec2AnyVec(Estimate(AnyVec2TypeVec<Eigen::Vector2d>(points2D_Any), AnyVec2TypeVec<Eigen::Vector3d>(points3D_Any)));
}
vector<Eigen::Matrix3x4d> CP3PEstimator::Estimate(const vector<Eigen::Vector2d>& points2D, const vector<Eigen::Vector3d>& points3D) const
{
    CHECK(points2D.size() == minNumSamples && points3D.size() == minNumSamples);

    // Step 1. 坐标预处理
    Eigen::Matrix3d points3DWorld;
    points3DWorld.col(0) = points3D[0];
    points3DWorld.col(1) = points3D[1];
    points3DWorld.col(2) = points3D[2];

    const Eigen::Vector3d u = points2D[0].homogeneous().normalized(); //把二维图像点转为同质坐标, 并进行单位化
    const Eigen::Vector3d v = points2D[1].homogeneous().normalized();
    const Eigen::Vector3d w = points2D[2].homogeneous().normalized();

    // Step 2. 计算2D点之间间和3D点之间的夹角和距离
    const double cosUV = u.transpose() * v;
    const double cosUW = u.transpose() * w;
    const double cosVW = v.transpose() * w;

    const double dist2_AB = (points3D[0] - points3D[1]).squaredNorm();
    const double dist2_AC = (points3D[0] - points3D[2]).squaredNorm();
    const double dist2_BC = (points3D[1] - points3D[2]).squaredNorm();

    const double dist_AB = sqrt(dist2_AB);

    // Step 3. 多项式系数计算. 建立一个四次多项式方程, 其中包含5个系数: a4*x^4 + a3*x^3 + a2*x^2 + a1*x + a0 = 0
    const double a = dist2_BC / dist2_AB;
    const double b = dist2_AC / dist2_AB;

    const double a2 = a * a;
    const double b2 = b * b;
    const double p = 2 * cosVW;
    const double q = 2 * cosUW;
    const double r = 2 * cosUV;
    const double p2 = p * p;
    const double p3 = p2 * p;
    const double q2 = q * q;
    const double r2 = r * r;
    const double r3 = r2 * r;
    const double r4 = r3 * r;
    const double r5 = r4 * r;

    Eigen::Matrix<double, 5, 1> coeffs;
    coeffs(0) = -2 * b + b2 + a2 + 1 + a * b * (2 - r2) - 2 * a;
    coeffs(1) = -2 * q * a2 - r * p * b2 + 4 * q * a + (2 * q + p * r) * b +
        (r2 * q - 2 * q + r * p) * a * b - 2 * q;
    coeffs(2) = (2 + q2) * a2 + (p2 + r2 - 2) * b2 - (4 + 2 * q2) * a -
        (p * q * r + p2) * b - (p * q * r + r2) * a * b + q2 + 2;
    coeffs(3) = -2 * q * a2 - r * p * b2 + 4 * q * a +
        (p * r + q * p2 - 2 * q) * b + (r * p + 2 * q) * a * b - 2 * q;
    coeffs(4) = a2 + b2 - 2 * a + (2 - p2) * b - 2 * a * b + 1;

    // Step 4. 求解多项式
    Eigen::VectorXd rootsReal;
    Eigen::VectorXd rootsImag;
#ifdef FindPolynomialRoots_Fast
    if (!FindPolynomialRootsDurandKerner(coeffs, rootsReal, rootsImag))
    {
        return {};
    }
#else
    if (!FindPolynomialRootsCompanionMatrix(coeffs, rootsReal, rootsImag))
    {
        return {};
    }
#endif

    // Step 5. 模型生成. 使用找到的多项式根来计算相机的位置和方向
    vector<Eigen::Matrix3x4d> models; // models存储找到的最有可能的相机模型(位置和方向)
    models.reserve(rootsReal.size());

    for (Eigen::VectorXd::Index i = 0; i < rootsReal.size(); i++)
    {
        if (abs(rootsImag(i)) > 1e-10)  // 如果是复数根(虚数不为0), 则跳过
        {
            continue;
        }

        const double x = rootsReal(i); // 如果根是负数, 也跳过
        if (x < 0)
        {
            continue;
        }

        const double x2 = x * x;
        const double x3 = x2 * x;

        // 求解y值: b1*y+b0=0
        const double bb1 = (p2 - p * q * r + r2) * a + (p2 - r2) * b - p2 + p * q * r - r2;
        const double b1 = b * bb1 * bb1;
        const double b0 =
            ((1 - a - b) * x2 + (a - 1) * q * x - a + b + 1) *
            (r3 * (a2 + b2 - 2 * a - 2 * b + (2 - r2) * a * b + 1) * x3 +
                r2 *
                (p + p * a2 - 2 * r * q * a * b + 2 * r * q * b - 2 * r * q -
                    2 * p * a - 2 * p * b + p * r2 * b + 4 * r * q * a +
                    q * r3 * a * b - 2 * r * q * a2 + 2 * p * a * b + p * b2 -
                    r2 * p * b2) *
                x2 +
                (r5 * (b2 - a * b) - r4 * p * q * b +
                    r3 * (q2 - 4 * a - 2 * q2 * a + q2 * a2 + 2 * a2 - 2 * b2 + 2) +
                    r2 * (4 * p * q * a - 2 * p * q * a * b + 2 * p * q * b - 2 * p * q -
                        2 * p * q * a2) +
                    r * (p2 * b2 - 2 * p2 * b + 2 * p2 * a * b - 2 * p2 * a + p2 +
                        p2 * a2)) *
                x +
                (2 * p * r2 - 2 * r3 * q + p3 - 2 * p2 * q * r + p * q2 * r2) * a2 +
                (p3 - 2 * p * r2) * b2 +
                (4 * q * r3 - 4 * p * r2 - 2 * p3 + 4 * p2 * q * r - 2 * p * q2 * r2) *
                a +
                (-2 * q * r3 + p * r4 + 2 * p2 * q * r - 2 * p3) * b +
                (2 * p3 + 2 * q * r3 - 2 * p2 * q * r) * a * b + p * q2 * r2 -
                2 * p2 * q * r + 2 * p * r2 + p3 - 2 * r3 * q);


        const double y = b0 / b1;
        const double y2 = y * y;

        const double nu = x2 + y2 - 2 * x * y * cosUV;

        const double dist_PC = dist_AB / sqrt(nu);
        const double dist_PB = y * dist_PC;
        const double dist_PA = x * dist_PC;

        Eigen::Matrix3d points3DCamera;
        points3DCamera.col(0) = u * dist_PA;  // A'
        points3DCamera.col(1) = v * dist_PB;  // B'
        points3DCamera.col(2) = w * dist_PC;  // C'

        // 找到从世界坐标系到相机坐标系的变换
        const Eigen::Matrix4d transform = Eigen::umeyama(points3DWorld, points3DCamera, false);
        models.push_back(transform.topLeftCorner<3, 4>());
    }
    return models;
}
CP3PEstimateRANSACReport CP3PEstimator::EstimateRANSAC(const std::vector<Eigen::Vector2d>& X, const std::vector<Eigen::Vector3d>& Y, const CRANSACOptions& options, CSupportMeasurer* supportMeasurer, CSampler* sampler) const
{
    // Step 1. 初始化. 检查输入向量大小, 初始化结果, 检查特殊情况
    CHECK(X.size() == Y.size());

    const size_t numSamples = X.size();
    CP3PEstimateRANSACReport report;
    if (numSamples < minNumSamples) // 如果当前样本数太少, 不足以用来估计模型, 则直接返回
    {
        return report;
    }

    bool isSupportMeasurerNull = false, isSamplerNull = false;
    if (!supportMeasurer)
    {
        isSupportMeasurerNull = true;
        supportMeasurer = new CInlierSupportMeasurer();
    }
    if (!sampler)
    {
        isSamplerNull = true;
        sampler = new CRandomSampler(minNumSamples);
    }

    // Step 2. RANSAC主循环
    bool isAbort = false;
    const double maxResidual = options.maxError * options.maxError; // 允许的最大残差(只有当残差不超过maxResidual, 才会被标记为内点)

    std::vector<double> residuals(numSamples); // 模型与所有数据点的残差
    std::vector<Eigen::Vector2d> X_rand(minNumSamples);
    std::vector<Eigen::Vector3d> Y_rand(minNumSamples);
    sampler->Initialize(numSamples); // 初始化采样器

    CSupport bestSupport; // 当前得到的最好的支持度
    Eigen::Matrix3x4d bestModel;  // 当前得到的最好模型

    size_t maxNumTrials = std::min(options.maxNumTrials, sampler->GetMaxNumSamples()); // 确定最大迭代次数
    size_t dynamicMaxNumTrials = maxNumTrials;
    for (report.numTrials = 0; report.numTrials < maxNumTrials; report.numTrials++)
    {
        if (isAbort)
        {
            report.numTrials++;
            break;
        }

        sampler->GetSampleXY(X, Y, X_rand, Y_rand); // 从X和Y中以采样器内部的采样规则采样
        const std::vector<Eigen::Matrix3x4d> sampledModels = Estimate(X_rand, Y_rand); // 使用随机样本来估计模型

        for (const Eigen::Matrix3x4d& sampledModel : sampledModels)
        {
            Residuals(X, Y, sampledModel, residuals); // 对于每一个估计出的模型, 计算其与所有数据点的残差
            CHECK(residuals.size() == numSamples);

            const CSupport support = supportMeasurer->Evaluate(residuals, maxResidual); // 评估这个模型的支持度
            if (supportMeasurer->Compare(support, bestSupport)) // 如果新的支持度比当前最好的支持度更好, 就更新最好的支持度和最好的模型
            {
                bestSupport = support;
                bestModel = sampledModel;

                // 根据最好的支持度动态地更新最大迭代次数
                dynamicMaxNumTrials = GetNumTrials(minNumSamples, bestSupport.numInliers, numSamples, options.confidence, options.maxIterNumTrialsMultiplier);
            }

            // 如果达到动态更新后的最大迭代次数, 并且迭代次数超过了最小迭代次数，就提前终止算法
            if (report.numTrials >= dynamicMaxNumTrials && report.numTrials >= options.minNumTrials)
            {
                isAbort = true;
                break;
            }
        }
    }

    // Step 3. 结果收集与返回
    report.support = bestSupport;
    report.model = bestModel;
    if (report.support.numInliers < minNumSamples) // 如果找到的最佳模型的内点数少于最小样本数, 那么说明失败了
    {
        if (isSamplerNull)
        {
            delete sampler;
        }
        if (isSupportMeasurerNull)
        {
            delete supportMeasurer;
        }
        return report;
    }
    Residuals(X, Y, report.model, residuals);
    CHECK(residuals.size() == numSamples);

    report.inlierMask.resize(numSamples);
    for (size_t i = 0; i < residuals.size(); i++) // 判断每个样本是否为内点
    {
        report.inlierMask[i] = (residuals[i] <= maxResidual);
    }
    report.isSuccess = true;

    if (isSamplerNull)
    {
        delete sampler;
    }
    if (isSupportMeasurerNull)
    {
        delete supportMeasurer;
    }
    return report;
}
void CP3PEstimator::Residuals(const vector<any>& points2D_Any, const vector<any>& points3D_Any, const any& projectMatrix_Any, vector<double>& residuals)
{
    CHECK(points2D_Any.size() == points3D_Any.size() && !points2D_Any.empty());
    CHECK(points2D_Any[0].type() == typeid(Eigen::Vector2d) && points3D_Any[0].type() == typeid(Eigen::Vector3d) && projectMatrix_Any.type() == typeid(Eigen::Matrix3x4d));

    Residuals(AnyVec2TypeVec<Eigen::Vector2d>(points2D_Any), AnyVec2TypeVec<Eigen::Vector3d>(points3D_Any), any_cast<Eigen::Matrix3x4d>(projectMatrix_Any), residuals);
}
void CP3PEstimator::Residuals(const vector<Eigen::Vector2d>& points2D, const vector<Eigen::Vector3d>& points3D, const Eigen::Matrix3x4d& projectMatrix, vector<double>& residuals) const
{
    CHECK(points2D.size() == points3D.size() && !points2D.empty());

    ComputeSquaredReprojectionError(points2D, points3D, projectMatrix, residuals);
}

CEPnPEstimator::CEPnPEstimator() :CEstimator(4)
{

}
vector<any> CEPnPEstimator::Estimate(const vector<any>& points2D_Any, const vector<any>& points3D_Any)
{
    CHECK(points2D_Any.size() == points3D_Any.size());
    CHECK(points2D_Any.size() >= minNumSamples);
    CHECK(points2D_Any[0].type() == typeid(Eigen::Vector2d) && points3D_Any[0].type() == typeid(Eigen::Vector3d));

    return TypeVec2AnyVec(Estimate(AnyVec2TypeVec<Eigen::Vector2d>(points2D_Any), AnyVec2TypeVec<Eigen::Vector3d>(points3D_Any)));
}
vector<Eigen::Matrix3x4d> CEPnPEstimator::Estimate(const vector<Eigen::Vector2d>& points2D, const vector<Eigen::Vector3d>& points3D)
{
    CHECK(points2D.size() == points3D.size());
    CHECK(points2D.size() >= minNumSamples);

    Eigen::Matrix3x4d projectMatrix;
    if (!ComputePose(points2D, points3D, projectMatrix))
    {
        return {};
    }
    return { projectMatrix };
}
void CEPnPEstimator::Residuals(const vector<any>& points2D_Any, const vector<any>& points3D_Any, const any& projectMatrix_Any, vector<double>& residuals)
{
    CHECK(points2D_Any.size() == points3D_Any.size() && !points2D_Any.empty());
    CHECK(points2D_Any[0].type() == typeid(Eigen::Vector2d) && points3D_Any[0].type() == typeid(Eigen::Vector3d) && projectMatrix_Any.type() == typeid(Eigen::Matrix3x4d));

    Residuals(AnyVec2TypeVec<Eigen::Vector2d>(points2D_Any), AnyVec2TypeVec<Eigen::Vector3d>(points3D_Any), any_cast<Eigen::Matrix3x4d>(projectMatrix_Any), residuals);
}
void CEPnPEstimator::Residuals(const vector<Eigen::Vector2d>& points2D, const vector<Eigen::Vector3d>& points3D, const Eigen::Matrix3x4d& projectMatrix, vector<double>& residuals)
{
    CHECK(points2D.size() == points3D.size() && !points2D.empty());
    ComputeSquaredReprojectionError(points2D, points3D, projectMatrix, residuals);
}
bool CEPnPEstimator::ComputePose(const vector<Eigen::Vector2d>& points2D, const vector<Eigen::Vector3d>& points3D, Eigen::Matrix3x4d& projectMatrix)
{
    this->points2D = &points2D;
    this->points3D = &points3D;
    ChooseControlPoints();
    if (!ComputeBarycentricCoordinates()) 
    {
        return false;
    }

    const Eigen::Matrix<double, Eigen::Dynamic, 12> M = ComputeM();
    const Eigen::Matrix<double, 12, 12> MtM = M.transpose() * M;

    Eigen::JacobiSVD<Eigen::Matrix<double, 12, 12>> svd(MtM, Eigen::ComputeFullV | Eigen::ComputeFullU);
    const Eigen::Matrix<double, 12, 12> Ut = svd.matrixU().transpose();

    const Eigen::Matrix<double, 6, 10> L6x10 = ComputeL6x10(Ut);
    const Eigen::Matrix<double, 6, 1> rho = ComputeRho();

    Eigen::Vector4d betas[4];
    array<double, 4> reprojectErrors;
    array<Eigen::Matrix3d, 4> Rs;
    array<Eigen::Vector3d, 4> ts;

    FindBetasApprox1(L6x10, rho, betas[1]);
    RunGaussNewton(L6x10, rho, betas[1]);
    reprojectErrors[1] = ComputeRT(Ut, betas[1], Rs[1], ts[1]);

    FindBetasApprox2(L6x10, rho, betas[2]);
    RunGaussNewton(L6x10, rho, betas[2]);
    reprojectErrors[2] = ComputeRT(Ut, betas[2], Rs[2], ts[2]);

    FindBetasApprox3(L6x10, rho, betas[3]);
    RunGaussNewton(L6x10, rho, betas[3]);
    reprojectErrors[3] = ComputeRT(Ut, betas[3], Rs[3], ts[3]);

    int bestIndex = 1;
    if (reprojectErrors[2] < reprojectErrors[1]) 
    {
        bestIndex = 2;
    }
    if (reprojectErrors[3] < reprojectErrors[bestIndex]) 
    {
        bestIndex = 3;
    }
    projectMatrix.leftCols<3>() = Rs[bestIndex];
    projectMatrix.rightCols<1>() = ts[bestIndex];
    return true;
}
void CEPnPEstimator::ChooseControlPoints()
{
    cws[0].setZero();
    for (size_t i = 0; i < points3D->size(); i++) 
    {
        cws[0] += (*points3D)[i];
    }
    cws[0] /= points3D->size();

    Eigen::Matrix<double, Eigen::Dynamic, 3> PW0(points3D->size(), 3);
    for (size_t i = 0; i < points3D->size(); i++) 
    {
        PW0.row(i) = (*points3D)[i] - cws[0];
    }

    const Eigen::Matrix3d PW0tPW0 = PW0.transpose() * PW0;
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(PW0tPW0, Eigen::ComputeFullV | Eigen::ComputeFullU);
    const Eigen::Vector3d& D = svd.singularValues();
    const Eigen::Matrix3d Ut = svd.matrixU().transpose();

    for (int i = 1; i < 4; i++) 
    {
        const double k = sqrt(D(i - 1) / points3D->size());
        cws[i] = cws[0] + k * Ut.row(i - 1).transpose();
    }
}
bool CEPnPEstimator::ComputeBarycentricCoordinates()
{
    Eigen::Matrix3d CC;
    for (int i = 0; i < 3; i++) 
    {
        for (int j = 1; j < 4; j++) 
        {
            CC(i, j - 1) = cws[j][i] - cws[0][i];
        }
    }

    if (CC.colPivHouseholderQr().rank() < 3) 
    {
        return false;
    }

    const Eigen::Matrix3d CC_inv = CC.inverse();

    alphas.resize(points2D->size());
    for (size_t i = 0; i < points3D->size(); i++) 
    {
        for (int j = 0; j < 3; j++) 
        {
            alphas[i][1 + j] = CC_inv(j, 0) * ((*points3D)[i][0] - cws[0][0]) + CC_inv(j, 1) * ((*points3D)[i][1] - cws[0][1]) + CC_inv(j, 2) * ((*points3D)[i][2] - cws[0][2]);
        }
        alphas[i][0] = 1.0 - alphas[i][1] - alphas[i][2] - alphas[i][3];
    }

    return true;
}
Eigen::Matrix<double, Eigen::Dynamic, 12> CEPnPEstimator::ComputeM() 
{
    Eigen::Matrix<double, Eigen::Dynamic, 12> M(2 * points2D->size(), 12);
    for (size_t i = 0; i < points3D->size(); i++) 
    {
        for (size_t j = 0; j < 4; j++) 
        {
            M(2 * i, 3 * j) = alphas[i][j];
            M(2 * i, 3 * j + 1) = 0.0;
            M(2 * i, 3 * j + 2) = -alphas[i][j] * (*points2D)[i].x();

            M(2 * i + 1, 3 * j) = 0.0;
            M(2 * i + 1, 3 * j + 1) = alphas[i][j];
            M(2 * i + 1, 3 * j + 2) = -alphas[i][j] * (*points2D)[i].y();
        }
    }
    return M;
}
Eigen::Matrix<double, 6, 10> CEPnPEstimator::ComputeL6x10(const Eigen::Matrix<double, 12, 12>& Ut)
{
    Eigen::Matrix<double, 6, 10> L6x10;

    array<array<Eigen::Vector3d, 6>, 4> dv;
    for (int i = 0; i < 4; i++) 
    {
        int a = 0, b = 1;
        for (int j = 0; j < 6; j++) 
        {
            dv[i][j][0] = Ut(11 - i, 3 * a) - Ut(11 - i, 3 * b);
            dv[i][j][1] = Ut(11 - i, 3 * a + 1) - Ut(11 - i, 3 * b + 1);
            dv[i][j][2] = Ut(11 - i, 3 * a + 2) - Ut(11 - i, 3 * b + 2);

            b++;
            if (b > 3) 
            {
                a++;
                b = a + 1;
            }
        }
    }

    for (int i = 0; i < 6; i++) 
    {
        L6x10(i, 0) = dv[0][i].transpose() * dv[0][i];
        L6x10(i, 1) = 2.0 * dv[0][i].transpose() * dv[1][i];
        L6x10(i, 2) = dv[1][i].transpose() * dv[1][i];
        L6x10(i, 3) = 2.0 * dv[0][i].transpose() * dv[2][i];
        L6x10(i, 4) = 2.0 * dv[1][i].transpose() * dv[2][i];
        L6x10(i, 5) = dv[2][i].transpose() * dv[2][i];
        L6x10(i, 6) = 2.0 * dv[0][i].transpose() * dv[3][i];
        L6x10(i, 7) = 2.0 * dv[1][i].transpose() * dv[3][i];
        L6x10(i, 8) = 2.0 * dv[2][i].transpose() * dv[3][i];
        L6x10(i, 9) = dv[3][i].transpose() * dv[3][i];
    }

    return L6x10;
}
Eigen::Matrix<double, 6, 1> CEPnPEstimator::ComputeRho()
{
    Eigen::Matrix<double, 6, 1> rho;
    rho[0] = (cws[0] - cws[1]).squaredNorm();
    rho[1] = (cws[0] - cws[2]).squaredNorm();
    rho[2] = (cws[0] - cws[3]).squaredNorm();
    rho[3] = (cws[1] - cws[2]).squaredNorm();
    rho[4] = (cws[1] - cws[3]).squaredNorm();
    rho[5] = (cws[2] - cws[3]).squaredNorm();
    return rho;
}
void CEPnPEstimator::FindBetasApprox1(const Eigen::Matrix<double, 6, 10>& L6x10, const Eigen::Matrix<double, 6, 1>& rho, Eigen::Vector4d& betas)
{
    Eigen::Matrix<double, 6, 4> L_6x4;
    for (int i = 0; i < 6; i++) 
    {
        L_6x4(i, 0) = L6x10(i, 0);
        L_6x4(i, 1) = L6x10(i, 1);
        L_6x4(i, 2) = L6x10(i, 3);
        L_6x4(i, 3) = L6x10(i, 6);
    }

    Eigen::JacobiSVD<Eigen::Matrix<double, 6, 4>> svd(L_6x4, Eigen::ComputeFullV | Eigen::ComputeFullU);
    const Eigen::Matrix<double, 4, 1> b4 = svd.solve(rho);

    if (b4[0] < 0) 
    {
        betas[0] = sqrt(-b4[0]);
        betas[1] = -b4[1] / betas[0];
        betas[2] = -b4[2] / betas[0];
        betas[3] = -b4[3] / betas[0];
    }
    else 
    {
        betas[0] = sqrt(b4[0]);
        betas[1] = b4[1] / betas[0];
        betas[2] = b4[2] / betas[0];
        betas[3] = b4[3] / betas[0];
    }
}
void CEPnPEstimator::FindBetasApprox2(const Eigen::Matrix<double, 6, 10>& L6x10, const Eigen::Matrix<double, 6, 1>& rho, Eigen::Vector4d& betas)
{
    Eigen::Matrix<double, 6, 3> L_6x3(6, 3);

    for (int i = 0; i < 6; i++) 
    {
        L_6x3(i, 0) = L6x10(i, 0);
        L_6x3(i, 1) = L6x10(i, 1);
        L_6x3(i, 2) = L6x10(i, 2);
    }

    Eigen::JacobiSVD<Eigen::Matrix<double, 6, 3>> svd(L_6x3, Eigen::ComputeFullV | Eigen::ComputeFullU);
    const Eigen::Matrix<double, 3, 1> b3 = svd.solve(rho);

    if (b3[0] < 0)
    {
        betas[0] = sqrt(-b3[0]);
        betas[1] = (b3[2] < 0) ? sqrt(-b3[2]) : 0.0;
    }
    else
    {
        betas[0] = sqrt(b3[0]);
        betas[1] = (b3[2] > 0) ? sqrt(b3[2]) : 0.0;
    }

    if (b3[1] < 0)
    {
        betas[0] = -betas[0];
    }

    betas[2] = 0.0;
    betas[3] = 0.0;
}
void CEPnPEstimator::FindBetasApprox3(const Eigen::Matrix<double, 6, 10>& L6x10, const Eigen::Matrix<double, 6, 1>& rho, Eigen::Vector4d& betas)
{
    Eigen::JacobiSVD<Eigen::Matrix<double, 6, 5>> svd(L6x10.leftCols<5>(), Eigen::ComputeFullV | Eigen::ComputeFullU);
    const Eigen::Matrix<double, 5, 1> b5 = svd.solve(rho);

    if (b5[0] < 0)
    {
        betas[0] = sqrt(-b5[0]);
        betas[1] = (b5[2] < 0) ? sqrt(-b5[2]) : 0.0;
    }
    else
    {
        betas[0] = sqrt(b5[0]);
        betas[1] = (b5[2] > 0) ? sqrt(b5[2]) : 0.0;
    }
    if (b5[1] < 0)
    {
        betas[0] = -betas[0];
    }
    betas[2] = b5[3] / betas[0];
    betas[3] = 0.0;
}
void CEPnPEstimator::RunGaussNewton(const Eigen::Matrix<double, 6, 10>& L6x10, const Eigen::Matrix<double, 6, 1>& rho, Eigen::Vector4d& betas)
{
    Eigen::Matrix<double, 6, 4> A;
    Eigen::Matrix<double, 6, 1> b;

    for (int k = 0; k < 5; k++)
    {
        for (int i = 0; i < 6; i++)
        {
            A(i, 0) = 2 * L6x10(i, 0) * betas[0] + L6x10(i, 1) * betas[1] + L6x10(i, 3) * betas[2] + L6x10(i, 6) * betas[3];
            A(i, 1) = L6x10(i, 1) * betas[0] + 2 * L6x10(i, 2) * betas[1] + L6x10(i, 4) * betas[2] + L6x10(i, 7) * betas[3];
            A(i, 2) = L6x10(i, 3) * betas[0] + L6x10(i, 4) * betas[1] + 2 * L6x10(i, 5) * betas[2] + L6x10(i, 8) * betas[3];
            A(i, 3) = L6x10(i, 6) * betas[0] + L6x10(i, 7) * betas[1] + L6x10(i, 8) * betas[2] + 2 * L6x10(i, 9) * betas[3];

            b(i) = rho[i] - (L6x10(i, 0) * betas[0] * betas[0] +
                L6x10(i, 1) * betas[0] * betas[1] +
                L6x10(i, 2) * betas[1] * betas[1] +
                L6x10(i, 3) * betas[0] * betas[2] +
                L6x10(i, 4) * betas[1] * betas[2] +
                L6x10(i, 5) * betas[2] * betas[2] +
                L6x10(i, 6) * betas[0] * betas[3] +
                L6x10(i, 7) * betas[1] * betas[3] +
                L6x10(i, 8) * betas[2] * betas[3] +
                L6x10(i, 9) * betas[3] * betas[3]);
        }

        const Eigen::Vector4d x = A.colPivHouseholderQr().solve(b);
        betas += x;
    }
}
double CEPnPEstimator::ComputeRT(const Eigen::Matrix<double, 12, 12>& Ut, const Eigen::Vector4d& betas, Eigen::Matrix3d& R, Eigen::Vector3d& t)
{
    ComputeCcs(betas, Ut);
    ComputePcs();
    SolveForSign();
    EstimateRT(R, t);
    return ComputeTotalReprojectionError(R, t);
}
void CEPnPEstimator::ComputeCcs(const Eigen::Vector4d& betas, const Eigen::Matrix<double, 12, 12>& Ut)
{
    for (int i = 0; i < 4; i++)
    {
        ccs[i][0] = ccs[i][1] = ccs[i][2] = 0.0;
    }

    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            for (int k = 0; k < 3; k++)
            {
                ccs[j][k] += betas[i] * Ut(11 - i, 3 * j + k);
            }
        }
    }
}
void CEPnPEstimator::ComputePcs()
{
    pcs.resize(points2D->size());
    for (size_t i = 0; i < points3D->size(); i++)
    {
        for (int j = 0; j < 3; j++)
        {
            pcs[i][j] = alphas[i][0] * ccs[0][j] + alphas[i][1] * ccs[1][j] + alphas[i][2] * ccs[2][j] + alphas[i][3] * ccs[3][j];
        }
    }
}
void CEPnPEstimator::SolveForSign() 
{
    if (pcs[0][2] < 0.0)
    {
        for (int i = 0; i < 4; i++)
        {
            ccs[i] = -ccs[i];
        }
        for (size_t i = 0; i < points3D->size(); i++)
        {
            pcs[i] = -pcs[i];
        }
    }
}
void CEPnPEstimator::EstimateRT(Eigen::Matrix3d& R, Eigen::Vector3d& t)
{
    Eigen::Vector3d pc0 = Eigen::Vector3d::Zero();
    Eigen::Vector3d pw0 = Eigen::Vector3d::Zero();

    for (size_t i = 0; i < points3D->size(); i++)
    {
        pc0 += pcs[i];
        pw0 += (*points3D)[i];
    }
    pc0 /= points3D->size();
    pw0 /= points3D->size();

    Eigen::Matrix3d abt = Eigen::Matrix3d::Zero();
    for (size_t i = 0; i < points3D->size(); i++)
    {
        for (int j = 0; j < 3; j++)
        {
            abt(j, 0) += (pcs[i][j] - pc0[j]) * ((*points3D)[i][0] - pw0[0]);
            abt(j, 1) += (pcs[i][j] - pc0[j]) * ((*points3D)[i][1] - pw0[1]);
            abt(j, 2) += (pcs[i][j] - pc0[j]) * ((*points3D)[i][2] - pw0[2]);
        }
    }

    Eigen::JacobiSVD<Eigen::Matrix3d> svd(abt, Eigen::ComputeFullV | Eigen::ComputeFullU);
    const Eigen::Matrix3d& abt_U = svd.matrixU();
    const Eigen::Matrix3d& abt_V = svd.matrixV();

    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            R(i, j) = abt_U.row(i) * abt_V.row(j).transpose();
        }
    }

    if (R.determinant() < 0)
    {
        Eigen::Matrix3d Abt_v_prime = abt_V;
        Abt_v_prime.col(2) = -abt_V.col(2);
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                R(i, j) = abt_U.row(i) * Abt_v_prime.row(j).transpose();
            }
        }
    }

    t = pc0 - R * pw0;
}
double CEPnPEstimator::ComputeTotalReprojectionError(const Eigen::Matrix3d& R, const Eigen::Vector3d& t)
{
    Eigen::Matrix3x4d projectMatrix;
    projectMatrix.leftCols<3>() = R;
    projectMatrix.rightCols<1>() = t;

    vector<double> residuals;
    ComputeSquaredReprojectionError(*points2D, *points3D, projectMatrix, residuals);

    double reprojectError = 0.0;
    for (const double residual : residuals)
    {
        reprojectError += sqrt(residual);
    }

    return reprojectError;
}

CEssentialMatrixEstimator_5Points::CEssentialMatrixEstimator_5Points() :CEstimator(5)
{

}
vector<any> CEssentialMatrixEstimator_5Points::Estimate(const vector<any>& points1_Any, const vector<any>& points2_Any)
{
    CHECK(points1_Any.size() == points2_Any.size());
    CHECK(points1_Any.size() >= minNumSamples);
    CHECK(points1_Any[0].type() == typeid(Eigen::Vector2d) && points2_Any[0].type() == typeid(Eigen::Vector2d));

    return TypeVec2AnyVec(Estimate(AnyVec2TypeVec<Eigen::Vector2d>(points1_Any), AnyVec2TypeVec<Eigen::Vector2d>(points2_Any)));
}
vector<Eigen::Matrix3d> CEssentialMatrixEstimator_5Points::Estimate(const vector<Eigen::Vector2d>& points1, const vector<Eigen::Vector2d>& points2) const
{
    CHECK(points1.size() == points2.size());
    CHECK(points1.size() >= minNumSamples);

    // Step 1. 构建矩阵Q. 该矩阵用于从points1和points2对应点中提取几何关系
    Eigen::Matrix<double, Eigen::Dynamic, 9> Q(points1.size(), 9);
    for (size_t i = 0; i < points1.size(); i++)
    {
        const double x1_0 = points1[i](0);
        const double x1_1 = points1[i](1);
        const double x2_0 = points2[i](0);
        const double x2_1 = points2[i](1);
        Q(i, 0) = x1_0 * x2_0;
        Q(i, 1) = x1_1 * x2_0;
        Q(i, 2) = x2_0;
        Q(i, 3) = x1_0 * x2_1;
        Q(i, 4) = x1_1 * x2_1;
        Q(i, 5) = x2_1;
        Q(i, 6) = x1_0;
        Q(i, 7) = x1_1;
        Q(i, 8) = 1;
    }

    // Step 2. 通过奇异值分解来提取矩阵Q的零空间: 获取Q的右奇异向量, 并从中选择与最小奇异值相对应的4个向量
    const Eigen::JacobiSVD<Eigen::Matrix<double, Eigen::Dynamic, 9>> svd(Q, Eigen::ComputeFullV);
    const Eigen::Matrix<double, 9, 4> E = svd.matrixV().block<9, 4>(0, 5);

    // Step 3. 使用Gauss-Jordan消元法处理矩阵A, 并且使用部分主干(partial pivoting)以提高数值稳定性
    Eigen::Matrix<double, 10, 20> A;
    CalculateA(E, A);
    Eigen::Matrix<double, 10, 10> AA = A.block<10, 10>(0, 0).partialPivLu().solve(A.block<10, 10>(0, 10));

    // Step 4. 将一个3x3的多项式矩阵B的行列式展开为一个10次多项式
    Eigen::Matrix<double, 13, 3> B;
    for (size_t i = 0; i < 3; i++)
    {
        B(0, i) = 0;
        B(4, i) = 0;
        B(8, i) = 0;
        B.block<3, 1>(1, i) = AA.block<1, 3>(i * 2 + 4, 0);
        B.block<3, 1>(5, i) = AA.block<1, 3>(i * 2 + 4, 3);
        B.block<4, 1>(9, i) = AA.block<1, 4>(i * 2 + 4, 6);
        B.block<3, 1>(0, i) -= AA.block<1, 3>(i * 2 + 5, 0);
        B.block<3, 1>(4, i) -= AA.block<1, 3>(i * 2 + 5, 3);
        B.block<4, 1>(8, i) -= AA.block<1, 4>(i * 2 + 5, 6);
    }

    // Step 5. 求解10次多项式的根
    Eigen::Matrix<double, 11, 1> coeffs;
    CalculateCoeffs(B, coeffs);
    Eigen::VectorXd rootsReal;
    Eigen::VectorXd rootsImag;
#ifdef FindPolynomialRoots_Fast
    if (!FindPolynomialRootsDurandKerner(coeffs, rootsReal, rootsImag))
    {
        return {};
    }
#else
    if (!FindPolynomialRootsCompanionMatrix(coeffs, rootsReal, rootsImag))
    {
        return {};
    }
#endif

    // Step 6. 构建模型
    vector<Eigen::Matrix3d> models;
    models.reserve(rootsReal.size());
    for (Eigen::VectorXd::Index i = 0; i < rootsImag.size(); i++)
    {
        if (abs(rootsImag(i)) > 1e-10)
        {
            continue;
        }

        const double z1 = rootsReal(i);
        const double z2 = z1 * z1;
        const double z3 = z2 * z1;
        const double z4 = z3 * z1;

        Eigen::Matrix3d Bz;
        for (size_t j = 0; j < 3; j++)
        {
            Bz(j, 0) = B(0, j) * z3 + B(1, j) * z2 + B(2, j) * z1 + B(3, j);
            Bz(j, 1) = B(4, j) * z3 + B(5, j) * z2 + B(6, j) * z1 + B(7, j);
            Bz(j, 2) = B(8, j) * z4 + B(9, j) * z3 + B(10, j) * z2 + B(11, j) * z1 + B(12, j);
        }

        const Eigen::JacobiSVD<Eigen::Matrix3d> svd(Bz, Eigen::ComputeFullV);
        const Eigen::Vector3d X = svd.matrixV().block<3, 1>(0, 2);
        if (abs(X(2)) < 1e-10)
        {
            continue;
        }

        Eigen::MatrixXd essentialVec = E.col(0) * (X(0) / X(2)) + E.col(1) * (X(1) / X(2)) + E.col(2) * z1 + E.col(3);
        essentialVec /= essentialVec.norm();

        const Eigen::Matrix3d essentialMatrix = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(essentialVec.data());
        models.push_back(essentialMatrix);
    }
    return models;
}
CEssentialMatrixEstimate_5PointsRANSACReport CEssentialMatrixEstimator_5Points::EstimateLoRANSAC(const std::vector<Eigen::Vector2d>& X, const std::vector<Eigen::Vector2d>& Y, const CRANSACOptions& options, CSupportMeasurer* supportMeasurer, CSampler* sampler) const
{
    // Step 1. 初始化. 检查输入向量大小, 初始化结果, 检查特殊情况
    CHECK(X.size() == Y.size());

    const size_t numSamples = X.size();
    CEssentialMatrixEstimate_5PointsRANSACReport report;
    if (numSamples < minNumSamples) // 如果当前样本数太少, 不足以用来估计模型, 则直接返回
    {
        return report;
    }

    bool isSupportMeasurerNull = false, isSamplerNull = false;
    if (!supportMeasurer)
    {
        isSupportMeasurerNull = true;
        supportMeasurer = new CInlierSupportMeasurer();
    }
    if (!sampler)
    {
        isSamplerNull = true;
        sampler = new CRandomSampler(minNumSamples);
    }

    // Step 2. RANSAC主循环
    bool isAbort = false;
    bool bestModelIsLocal = false;
    const double maxResidual = options.maxError * options.maxError; // 允许的最大残差(只有当残差不超过maxResidual, 才会被标记为内点)

    std::vector<double> residuals; // 模型与所有数据点的残差
    std::vector<double> bestLocalResiduals; // 模型与所有数据点的残差
    std::vector<Eigen::Vector2d> XInlier;
    std::vector<Eigen::Vector2d> YInlier;
    std::vector<Eigen::Vector2d> X_rand(minNumSamples);
    std::vector<Eigen::Vector2d> Y_rand(minNumSamples);
    sampler->Initialize(numSamples); // 初始化采样器

    CSupport bestSupport; // 当前得到的最好的支持度
    Eigen::Matrix3d bestModel;  // 当前得到的最好模型

    size_t maxNumTrials = std::min(options.maxNumTrials, sampler->GetMaxNumSamples()); // 确定最大迭代次数
    size_t dynamicMaxNumTrials = maxNumTrials;
    for (report.numTrials = 0; report.numTrials < maxNumTrials; report.numTrials++)
    {
        if (isAbort)
        {
            report.numTrials++;
            break;
        }

        sampler->GetSampleXY(X, Y, X_rand, Y_rand); // 从X和Y中以采样器内部的采样规则采样
        const std::vector<Eigen::Matrix3d> sampledModels = Estimate(X_rand, Y_rand); // 使用随机样本来估计模型
        for (const Eigen::Matrix3d& sampledModel : sampledModels)
        {
            Residuals(X, Y, sampledModel, residuals); // 对于每一个估计出的模型, 计算其与所有数据点的残差
            CHECK(residuals.size() == numSamples);

            const CSupport support = supportMeasurer->Evaluate(residuals, maxResidual); // 评估这个模型的支持度

            if (supportMeasurer->Compare(support, bestSupport)) // 如果新的支持度比当前最好的支持度更好, 就做局部优化
            {
                bestSupport = support;
                bestModel = sampledModel;
                bestModelIsLocal = false;

                // 根据内点来局部估计更优模型
                if (support.numInliers > minNumSamples && support.numInliers >= minNumSamples)
                {
                    // 迭代式局部优化来扩大内点集
                    const size_t maxLocalTrials = 10;
                    for (size_t localNumTrials = 0; localNumTrials < maxLocalTrials; localNumTrials++)
                    {
                        XInlier.clear();
                        YInlier.clear();
                        XInlier.reserve(numSamples);
                        YInlier.reserve(numSamples);
                        for (size_t i = 0; i < residuals.size(); i++)
                        {
                            if (residuals[i] <= maxResidual)
                            {
                                XInlier.push_back(X[i]);
                                YInlier.push_back(Y[i]);
                            }
                        }
                        const std::vector<Eigen::Matrix3d> localModels = Estimate(XInlier, YInlier);
                        const size_t preBestNumInliers = bestSupport.numInliers;
                        for (const Eigen::Matrix3d& localModel : localModels)
                        {
                            Residuals(X, Y, localModel, residuals);
                            CHECK(residuals.size() == numSamples);
                            const CSupport localSupport = supportMeasurer->Evaluate(residuals, maxResidual);

                            // 检查局部优化模型是否更优
                            if (supportMeasurer->Compare(localSupport, bestSupport))
                            {
                                bestSupport = localSupport;
                                bestModel = localModel;
                                bestModelIsLocal = true;
                                std::swap(residuals, bestLocalResiduals); // 交换残差
                            }
                        }
                        // 只有当内点集变多了从而有机会进一步优化时, 才继续迭代
                        if (bestSupport.numInliers <= preBestNumInliers)
                        {
                            break;
                        }

                        //把残差再交换回来, 这样就可以在下一次局部优化的迭代中提取出最佳的内点集
                        std::swap(residuals, bestLocalResiduals);
                    }
                }
                dynamicMaxNumTrials = GetNumTrials(minNumSamples, bestSupport.numInliers, numSamples, options.confidence, options.maxIterNumTrialsMultiplier);
            }
            if (report.numTrials >= dynamicMaxNumTrials && report.numTrials >= options.minNumTrials)
            {
                isAbort = true;
                break;
            }
        }
    }

    // Step 3. 结果收集与返回
    report.support = bestSupport;
    report.model = bestModel;
    if (report.support.numInliers < minNumSamples) // 如果找到的最佳模型的内点数少于最小样本数, 那么说明失败了
    {
        if (isSamplerNull)
        {
            delete sampler;
        }
        if (isSupportMeasurerNull)
        {
            delete supportMeasurer;
        }
        return report;
    }

    // 这将对最佳模型的残差进行两次计算, 但避免了对每个评估模型都复制和填充内点掩码, 这种方法其实更快
    Residuals(X, Y, report.model, residuals);
    CHECK(residuals.size() == numSamples);
    report.inlierMask.resize(numSamples);

    for (size_t i = 0; i < residuals.size(); i++) // 判断每个样本是否为内点
    {
        report.inlierMask[i] = (residuals[i] <= maxResidual);
    }
    report.isSuccess = true;

    if (isSamplerNull)
    {
        delete sampler;
    }
    if (isSupportMeasurerNull)
    {
        delete supportMeasurer;
    }
    return report;
}
void CEssentialMatrixEstimator_5Points::Residuals(const vector<any>& points1_Any, const vector<any>& points2_Any, const any& E_any, vector<double>& residuals)
{
    CHECK(points1_Any.size() == points2_Any.size() && !points1_Any.empty());
    CHECK(points1_Any[0].type() == typeid(Eigen::Vector2d) && points2_Any[0].type() == typeid(Eigen::Vector2d) && E_any.type() == typeid(Eigen::Matrix3d));

    Residuals(AnyVec2TypeVec<Eigen::Vector2d>(points1_Any), AnyVec2TypeVec<Eigen::Vector2d>(points2_Any), any_cast<Eigen::Matrix3d>(E_any), residuals);
}
void CEssentialMatrixEstimator_5Points::Residuals(const vector<Eigen::Vector2d>& points1, const vector<Eigen::Vector2d>& points2, const Eigen::Matrix3d& E, vector<double>& residuals) const
{
    CHECK(points1.size() == points2.size() && !points1.empty());
    ComputeSquaredSampsonError(points1, points2, E, residuals);
}
void CEssentialMatrixEstimator_5Points::CalculateA(const Eigen::Matrix<double, 9, 4>& E, Eigen::Matrix<double, 10, 20>& A) const
{
    double* a = A.data();
    const double* e = E.data();
    double e2[36];
    double e3[36];
    for (size_t i = 0; i < 36; i++) 
    {
        e2[i] = e[i] * e[i];
        e3[i] = e2[i] * e[i];
    }
    a[190] = e[33] * e[28] * e[32] - e[33] * e[31] * e[29] +
        e[30] * e[34] * e[29] - e[30] * e[28] * e[35] -
        e[27] * e[32] * e[34] + e[27] * e[31] * e[35];
    a[7] = 0.5 * e[6] * e2[8] - 0.5 * e[6] * e2[5] + 0.5 * e3[6] +
        0.5 * e[6] * e2[7] - 0.5 * e[6] * e2[4] + e[0] * e[2] * e[8] +
        e[3] * e[4] * e[7] + e[3] * e[5] * e[8] + e[0] * e[1] * e[7] -
        0.5 * e[6] * e2[1] - 0.5 * e[6] * e2[2] + 0.5 * e2[0] * e[6] +
        0.5 * e2[3] * e[6];
    a[120] = e[30] * e[34] * e[2] + e[33] * e[1] * e[32] - e[3] * e[28] * e[35] +
        e[0] * e[31] * e[35] + e[3] * e[34] * e[29] - e[30] * e[1] * e[35] +
        e[27] * e[31] * e[8] - e[27] * e[32] * e[7] - e[30] * e[28] * e[8] -
        e[33] * e[31] * e[2] - e[0] * e[32] * e[34] + e[6] * e[28] * e[32] -
        e[33] * e[4] * e[29] + e[33] * e[28] * e[5] + e[30] * e[7] * e[29] +
        e[27] * e[4] * e[35] - e[27] * e[5] * e[34] - e[6] * e[31] * e[29];
    a[77] =
        e[9] * e[27] * e[15] + e[9] * e[29] * e[17] + e[9] * e[11] * e[35] +
        e[9] * e[28] * e[16] + e[9] * e[10] * e[34] + e[27] * e[11] * e[17] +
        e[27] * e[10] * e[16] + e[12] * e[30] * e[15] + e[12] * e[32] * e[17] +
        e[12] * e[14] * e[35] + e[12] * e[31] * e[16] + e[12] * e[13] * e[34] +
        e[30] * e[14] * e[17] + e[30] * e[13] * e[16] + e[15] * e[35] * e[17] +
        e[15] * e[34] * e[16] - e[15] * e[28] * e[10] - e[15] * e[31] * e[13] -
        e[15] * e[32] * e[14] - e[15] * e[29] * e[11] + 0.5 * e2[9] * e[33] +
        0.5 * e[33] * e2[16] - 0.5 * e[33] * e2[11] + 0.5 * e[33] * e2[12] +
        1.5 * e[33] * e2[15] + 0.5 * e[33] * e2[17] - 0.5 * e[33] * e2[10] -
        0.5 * e[33] * e2[14] - 0.5 * e[33] * e2[13];
    a[180] =
        -e[33] * e[22] * e[29] - e[33] * e[31] * e[20] - e[27] * e[32] * e[25] +
        e[27] * e[22] * e[35] - e[27] * e[23] * e[34] + e[27] * e[31] * e[26] +
        e[33] * e[28] * e[23] - e[21] * e[28] * e[35] + e[30] * e[25] * e[29] +
        e[24] * e[28] * e[32] - e[24] * e[31] * e[29] + e[18] * e[31] * e[35] -
        e[30] * e[28] * e[26] - e[30] * e[19] * e[35] + e[21] * e[34] * e[29] +
        e[33] * e[19] * e[32] - e[18] * e[32] * e[34] + e[30] * e[34] * e[20];
    a[87] = e[18] * e[2] * e[17] + e[3] * e[21] * e[15] + e[3] * e[12] * e[24] +
        e[3] * e[23] * e[17] + e[3] * e[14] * e[26] + e[3] * e[22] * e[16] +
        e[3] * e[13] * e[25] + 3. * e[6] * e[24] * e[15] +
        e[6] * e[26] * e[17] + e[6] * e[25] * e[16] + e[0] * e[20] * e[17] +
        e[0] * e[11] * e[26] + e[0] * e[19] * e[16] + e[0] * e[10] * e[25] +
        e[15] * e[26] * e[8] - e[15] * e[20] * e[2] - e[15] * e[19] * e[1] -
        e[15] * e[22] * e[4] + e[15] * e[25] * e[7] - e[15] * e[23] * e[5] +
        e[12] * e[21] * e[6] + e[12] * e[22] * e[7] + e[12] * e[4] * e[25] +
        e[12] * e[23] * e[8] + e[12] * e[5] * e[26] - e[24] * e[11] * e[2] -
        e[24] * e[10] * e[1] - e[24] * e[13] * e[4] + e[24] * e[16] * e[7] -
        e[24] * e[14] * e[5] + e[24] * e[17] * e[8] + e[21] * e[13] * e[7] +
        e[21] * e[4] * e[16] + e[21] * e[14] * e[8] + e[21] * e[5] * e[17] -
        e[6] * e[23] * e[14] - e[6] * e[20] * e[11] - e[6] * e[19] * e[10] -
        e[6] * e[22] * e[13] + e[9] * e[18] * e[6] + e[9] * e[0] * e[24] +
        e[9] * e[19] * e[7] + e[9] * e[1] * e[25] + e[9] * e[20] * e[8] +
        e[9] * e[2] * e[26] + e[18] * e[0] * e[15] + e[18] * e[10] * e[7] +
        e[18] * e[1] * e[16] + e[18] * e[11] * e[8];
    a[150] =
        e[33] * e[10] * e[32] + e[33] * e[28] * e[14] - e[33] * e[13] * e[29] -
        e[33] * e[31] * e[11] + e[9] * e[31] * e[35] - e[9] * e[32] * e[34] +
        e[27] * e[13] * e[35] - e[27] * e[32] * e[16] + e[27] * e[31] * e[17] -
        e[27] * e[14] * e[34] + e[12] * e[34] * e[29] - e[12] * e[28] * e[35] +
        e[30] * e[34] * e[11] + e[30] * e[16] * e[29] - e[30] * e[10] * e[35] -
        e[30] * e[28] * e[17] + e[15] * e[28] * e[32] - e[15] * e[31] * e[29];
    a[57] = e[0] * e[27] * e[6] + e[0] * e[28] * e[7] + e[0] * e[1] * e[34] +
        e[0] * e[29] * e[8] + e[0] * e[2] * e[35] + e[6] * e[34] * e[7] -
        e[6] * e[32] * e[5] + e[6] * e[30] * e[3] + e[6] * e[35] * e[8] -
        e[6] * e[29] * e[2] - e[6] * e[28] * e[1] - e[6] * e[31] * e[4] +
        e[27] * e[1] * e[7] + e[27] * e[2] * e[8] + e[3] * e[31] * e[7] +
        e[3] * e[4] * e[34] + e[3] * e[32] * e[8] + e[3] * e[5] * e[35] +
        e[30] * e[4] * e[7] + e[30] * e[5] * e[8] + 0.5 * e2[0] * e[33] +
        1.5 * e[33] * e2[6] - 0.5 * e[33] * e2[4] - 0.5 * e[33] * e2[5] -
        0.5 * e[33] * e2[1] + 0.5 * e[33] * e2[7] + 0.5 * e[33] * e2[3] -
        0.5 * e[33] * e2[2] + 0.5 * e[33] * e2[8];
    a[80] = -e[0] * e[23] * e[16] + e[9] * e[4] * e[26] + e[9] * e[22] * e[8] -
        e[9] * e[5] * e[25] - e[9] * e[23] * e[7] + e[18] * e[4] * e[17] +
        e[18] * e[13] * e[8] - e[18] * e[5] * e[16] - e[18] * e[14] * e[7] +
        e[3] * e[16] * e[20] + e[3] * e[25] * e[11] - e[3] * e[10] * e[26] -
        e[3] * e[19] * e[17] + e[12] * e[7] * e[20] + e[12] * e[25] * e[2] -
        e[12] * e[1] * e[26] - e[12] * e[19] * e[8] + e[21] * e[7] * e[11] +
        e[21] * e[16] * e[2] - e[21] * e[1] * e[17] - e[21] * e[10] * e[8] +
        e[6] * e[10] * e[23] + e[6] * e[19] * e[14] - e[6] * e[13] * e[20] -
        e[6] * e[22] * e[11] + e[15] * e[1] * e[23] + e[15] * e[19] * e[5] -
        e[15] * e[4] * e[20] - e[15] * e[22] * e[2] + e[24] * e[1] * e[14] +
        e[24] * e[10] * e[5] - e[24] * e[4] * e[11] - e[24] * e[13] * e[2] +
        e[0] * e[13] * e[26] + e[0] * e[22] * e[17] - e[0] * e[14] * e[25];
    a[167] = e[18] * e[19] * e[25] + 0.5 * e3[24] - 0.5 * e[24] * e2[23] +
        e[18] * e[20] * e[26] + e[21] * e[22] * e[25] +
        e[21] * e[23] * e[26] - 0.5 * e[24] * e2[19] + 0.5 * e2[21] * e[24] +
        0.5 * e[24] * e2[26] - 0.5 * e[24] * e2[20] + 0.5 * e2[18] * e[24] -
        0.5 * e[24] * e2[22] + 0.5 * e[24] * e2[25];
    a[50] = -e[3] * e[1] * e[35] - e[0] * e[32] * e[7] + e[27] * e[4] * e[8] +
        e[33] * e[1] * e[5] - e[33] * e[4] * e[2] + e[0] * e[4] * e[35] +
        e[3] * e[34] * e[2] - e[30] * e[1] * e[8] + e[30] * e[7] * e[2] -
        e[6] * e[4] * e[29] + e[3] * e[7] * e[29] + e[6] * e[1] * e[32] -
        e[0] * e[5] * e[34] - e[3] * e[28] * e[8] + e[0] * e[31] * e[8] +
        e[6] * e[28] * e[5] - e[6] * e[31] * e[2] - e[27] * e[5] * e[7];
    a[97] = e[33] * e[16] * e[7] - e[33] * e[14] * e[5] + e[33] * e[17] * e[8] +
        e[30] * e[13] * e[7] + e[30] * e[4] * e[16] + e[30] * e[14] * e[8] +
        e[30] * e[5] * e[17] + e[6] * e[27] * e[9] - e[6] * e[28] * e[10] -
        e[6] * e[31] * e[13] - e[6] * e[32] * e[14] - e[6] * e[29] * e[11] +
        e[9] * e[28] * e[7] + e[9] * e[1] * e[34] + e[9] * e[29] * e[8] +
        e[9] * e[2] * e[35] + e[27] * e[10] * e[7] + e[27] * e[1] * e[16] +
        e[27] * e[11] * e[8] + e[27] * e[2] * e[17] + e[3] * e[30] * e[15] +
        e[3] * e[12] * e[33] + e[3] * e[32] * e[17] + e[3] * e[14] * e[35] +
        e[3] * e[31] * e[16] + e[3] * e[13] * e[34] +
        3. * e[6] * e[33] * e[15] + e[6] * e[35] * e[17] +
        e[6] * e[34] * e[16] + e[0] * e[27] * e[15] + e[0] * e[9] * e[33] +
        e[0] * e[29] * e[17] + e[0] * e[11] * e[35] + e[0] * e[28] * e[16] +
        e[0] * e[10] * e[34] + e[15] * e[34] * e[7] - e[15] * e[32] * e[5] +
        e[15] * e[35] * e[8] - e[15] * e[29] * e[2] - e[15] * e[28] * e[1] -
        e[15] * e[31] * e[4] + e[12] * e[30] * e[6] + e[12] * e[31] * e[7] +
        e[12] * e[4] * e[34] + e[12] * e[32] * e[8] + e[12] * e[5] * e[35] -
        e[33] * e[11] * e[2] - e[33] * e[10] * e[1] - e[33] * e[13] * e[4];
    a[0] = e[6] * e[1] * e[5] - e[6] * e[4] * e[2] + e[3] * e[7] * e[2] +
        e[0] * e[4] * e[8] - e[0] * e[5] * e[7] - e[3] * e[1] * e[8];
    a[17] = 0.5 * e3[15] + e[9] * e[10] * e[16] - 0.5 * e[15] * e2[11] +
        e[9] * e[11] * e[17] + 0.5 * e2[12] * e[15] + 0.5 * e[15] * e2[16] +
        0.5 * e[15] * e2[17] - 0.5 * e[15] * e2[13] + 0.5 * e2[9] * e[15] +
        e[12] * e[14] * e[17] - 0.5 * e[15] * e2[10] - 0.5 * e[15] * e2[14] +
        e[12] * e[13] * e[16];
    a[70] =
        e[15] * e[28] * e[14] - e[15] * e[13] * e[29] - e[15] * e[31] * e[11] +
        e[33] * e[10] * e[14] - e[33] * e[13] * e[11] + e[9] * e[13] * e[35] -
        e[9] * e[32] * e[16] + e[9] * e[31] * e[17] - e[9] * e[14] * e[34] +
        e[27] * e[13] * e[17] - e[27] * e[14] * e[16] + e[12] * e[34] * e[11] +
        e[12] * e[16] * e[29] - e[12] * e[10] * e[35] - e[12] * e[28] * e[17] +
        e[30] * e[16] * e[11] - e[30] * e[10] * e[17] + e[15] * e[10] * e[32];
    a[177] =
        e[18] * e[27] * e[24] + e[18] * e[28] * e[25] + e[18] * e[19] * e[34] +
        e[18] * e[29] * e[26] + e[18] * e[20] * e[35] + e[27] * e[19] * e[25] +
        e[27] * e[20] * e[26] + e[21] * e[30] * e[24] + e[21] * e[31] * e[25] +
        e[21] * e[22] * e[34] + e[21] * e[32] * e[26] + e[21] * e[23] * e[35] +
        e[30] * e[22] * e[25] + e[30] * e[23] * e[26] + e[24] * e[34] * e[25] +
        e[24] * e[35] * e[26] - e[24] * e[29] * e[20] - e[24] * e[31] * e[22] -
        e[24] * e[32] * e[23] - e[24] * e[28] * e[19] + 1.5 * e[33] * e2[24] +
        0.5 * e[33] * e2[25] + 0.5 * e[33] * e2[26] - 0.5 * e[33] * e2[23] -
        0.5 * e[33] * e2[19] - 0.5 * e[33] * e2[20] - 0.5 * e[33] * e2[22] +
        0.5 * e2[18] * e[33] + 0.5 * e2[21] * e[33];
    a[170] =
        e[21] * e[25] * e[29] - e[27] * e[23] * e[25] + e[24] * e[19] * e[32] -
        e[21] * e[28] * e[26] - e[21] * e[19] * e[35] + e[18] * e[31] * e[26] -
        e[30] * e[19] * e[26] - e[24] * e[31] * e[20] + e[24] * e[28] * e[23] +
        e[27] * e[22] * e[26] + e[30] * e[25] * e[20] - e[33] * e[22] * e[20] +
        e[33] * e[19] * e[23] + e[21] * e[34] * e[20] - e[18] * e[23] * e[34] -
        e[24] * e[22] * e[29] - e[18] * e[32] * e[25] + e[18] * e[22] * e[35];
    a[37] = e[12] * e[14] * e[8] + e[12] * e[5] * e[17] + e[15] * e[16] * e[7] +
        e[15] * e[17] * e[8] + e[0] * e[11] * e[17] + e[0] * e[9] * e[15] +
        e[0] * e[10] * e[16] + e[3] * e[14] * e[17] + e[3] * e[13] * e[16] +
        e[9] * e[10] * e[7] + e[9] * e[1] * e[16] + e[9] * e[11] * e[8] +
        e[9] * e[2] * e[17] - e[15] * e[11] * e[2] - e[15] * e[10] * e[1] -
        e[15] * e[13] * e[4] - e[15] * e[14] * e[5] + e[12] * e[3] * e[15] +
        e[12] * e[13] * e[7] + e[12] * e[4] * e[16] + 0.5 * e2[12] * e[6] +
        1.5 * e2[15] * e[6] + 0.5 * e[6] * e2[17] + 0.5 * e[6] * e2[16] +
        0.5 * e[6] * e2[9] - 0.5 * e[6] * e2[11] - 0.5 * e[6] * e2[10] -
        0.5 * e[6] * e2[14] - 0.5 * e[6] * e2[13];
    a[10] = -e[9] * e[14] * e[16] - e[12] * e[10] * e[17] + e[9] * e[13] * e[17] -
        e[15] * e[13] * e[11] + e[15] * e[10] * e[14] + e[12] * e[16] * e[11];
    a[67] = e[21] * e[14] * e[17] + e[21] * e[13] * e[16] +
        e[15] * e[26] * e[17] + e[15] * e[25] * e[16] -
        e[15] * e[23] * e[14] - e[15] * e[20] * e[11] -
        e[15] * e[19] * e[10] - e[15] * e[22] * e[13] + e[9] * e[20] * e[17] +
        e[9] * e[11] * e[26] + e[9] * e[19] * e[16] + e[9] * e[10] * e[25] +
        0.5 * e2[12] * e[24] + 1.5 * e[24] * e2[15] + 0.5 * e[24] * e2[17] +
        0.5 * e[24] * e2[16] + 0.5 * e2[9] * e[24] - 0.5 * e[24] * e2[11] -
        0.5 * e[24] * e2[10] - 0.5 * e[24] * e2[14] - 0.5 * e[24] * e2[13] +
        e[18] * e[11] * e[17] + e[18] * e[9] * e[15] + e[18] * e[10] * e[16] +
        e[12] * e[21] * e[15] + e[12] * e[23] * e[17] +
        e[12] * e[14] * e[26] + e[12] * e[22] * e[16] + e[12] * e[13] * e[25];
    a[90] = -e[9] * e[5] * e[34] + e[9] * e[31] * e[8] - e[9] * e[32] * e[7] +
        e[27] * e[4] * e[17] + e[27] * e[13] * e[8] - e[27] * e[5] * e[16] -
        e[27] * e[14] * e[7] + e[0] * e[13] * e[35] - e[0] * e[32] * e[16] +
        e[0] * e[31] * e[17] - e[0] * e[14] * e[34] + e[9] * e[4] * e[35] +
        e[6] * e[10] * e[32] + e[6] * e[28] * e[14] - e[6] * e[13] * e[29] -
        e[6] * e[31] * e[11] + e[15] * e[1] * e[32] + e[3] * e[34] * e[11] +
        e[3] * e[16] * e[29] - e[3] * e[10] * e[35] - e[3] * e[28] * e[17] -
        e[12] * e[1] * e[35] + e[12] * e[7] * e[29] + e[12] * e[34] * e[2] -
        e[12] * e[28] * e[8] + e[15] * e[28] * e[5] - e[15] * e[4] * e[29] -
        e[15] * e[31] * e[2] + e[33] * e[1] * e[14] + e[33] * e[10] * e[5] -
        e[33] * e[4] * e[11] - e[33] * e[13] * e[2] + e[30] * e[7] * e[11] +
        e[30] * e[16] * e[2] - e[30] * e[1] * e[17] - e[30] * e[10] * e[8];
    a[117] = e[21] * e[31] * e[7] + e[21] * e[4] * e[34] + e[21] * e[32] * e[8] +
        e[21] * e[5] * e[35] + e[30] * e[22] * e[7] + e[30] * e[4] * e[25] +
        e[30] * e[23] * e[8] + e[30] * e[5] * e[26] +
        3. * e[24] * e[33] * e[6] + e[24] * e[34] * e[7] +
        e[24] * e[35] * e[8] + e[33] * e[25] * e[7] + e[33] * e[26] * e[8] +
        e[0] * e[27] * e[24] + e[0] * e[18] * e[33] + e[0] * e[28] * e[25] +
        e[0] * e[19] * e[34] + e[0] * e[29] * e[26] + e[0] * e[20] * e[35] +
        e[18] * e[27] * e[6] + e[18] * e[28] * e[7] + e[18] * e[1] * e[34] +
        e[18] * e[29] * e[8] + e[18] * e[2] * e[35] + e[27] * e[19] * e[7] +
        e[27] * e[1] * e[25] + e[27] * e[20] * e[8] + e[27] * e[2] * e[26] +
        e[3] * e[30] * e[24] + e[3] * e[21] * e[33] + e[3] * e[31] * e[25] +
        e[3] * e[22] * e[34] + e[3] * e[32] * e[26] + e[3] * e[23] * e[35] +
        e[6] * e[30] * e[21] - e[6] * e[29] * e[20] + e[6] * e[35] * e[26] -
        e[6] * e[31] * e[22] - e[6] * e[32] * e[23] - e[6] * e[28] * e[19] +
        e[6] * e[34] * e[25] - e[24] * e[32] * e[5] - e[24] * e[29] * e[2] -
        e[24] * e[28] * e[1] - e[24] * e[31] * e[4] - e[33] * e[20] * e[2] -
        e[33] * e[19] * e[1] - e[33] * e[22] * e[4] - e[33] * e[23] * e[5];
    a[160] = e[21] * e[25] * e[20] - e[21] * e[19] * e[26] +
        e[18] * e[22] * e[26] - e[18] * e[23] * e[25] -
        e[24] * e[22] * e[20] + e[24] * e[19] * e[23];
    a[47] = e[3] * e[4] * e[25] + e[3] * e[23] * e[8] + e[3] * e[5] * e[26] +
        e[21] * e[4] * e[7] + e[21] * e[5] * e[8] + e[6] * e[25] * e[7] +
        e[6] * e[26] * e[8] + e[0] * e[19] * e[7] + e[0] * e[1] * e[25] +
        e[0] * e[20] * e[8] + e[0] * e[2] * e[26] - e[6] * e[20] * e[2] -
        e[6] * e[19] * e[1] - e[6] * e[22] * e[4] - e[6] * e[23] * e[5] +
        e[18] * e[1] * e[7] + e[18] * e[0] * e[6] + e[18] * e[2] * e[8] +
        e[3] * e[21] * e[6] + e[3] * e[22] * e[7] - 0.5 * e[24] * e2[4] +
        0.5 * e[24] * e2[0] + 1.5 * e[24] * e2[6] - 0.5 * e[24] * e2[5] -
        0.5 * e[24] * e2[1] + 0.5 * e[24] * e2[7] + 0.5 * e[24] * e2[3] -
        0.5 * e[24] * e2[2] + 0.5 * e[24] * e2[8];
    a[110] = e[6] * e[28] * e[23] - e[6] * e[22] * e[29] - e[6] * e[31] * e[20] -
        e[3] * e[19] * e[35] + e[3] * e[34] * e[20] + e[3] * e[25] * e[29] -
        e[21] * e[1] * e[35] + e[21] * e[7] * e[29] + e[21] * e[34] * e[2] +
        e[24] * e[1] * e[32] + e[24] * e[28] * e[5] - e[24] * e[4] * e[29] -
        e[24] * e[31] * e[2] + e[33] * e[1] * e[23] + e[33] * e[19] * e[5] -
        e[33] * e[4] * e[20] - e[33] * e[22] * e[2] - e[21] * e[28] * e[8] +
        e[30] * e[7] * e[20] + e[30] * e[25] * e[2] - e[30] * e[1] * e[26] +
        e[18] * e[4] * e[35] - e[18] * e[5] * e[34] + e[18] * e[31] * e[8] -
        e[18] * e[32] * e[7] + e[27] * e[4] * e[26] + e[27] * e[22] * e[8] -
        e[27] * e[5] * e[25] - e[27] * e[23] * e[7] - e[3] * e[28] * e[26] -
        e[0] * e[32] * e[25] + e[0] * e[22] * e[35] - e[0] * e[23] * e[34] +
        e[0] * e[31] * e[26] - e[30] * e[19] * e[8] + e[6] * e[19] * e[32];
    a[107] = 0.5 * e2[18] * e[6] + 0.5 * e2[21] * e[6] + 1.5 * e2[24] * e[6] +
        0.5 * e[6] * e2[26] - 0.5 * e[6] * e2[23] - 0.5 * e[6] * e2[19] -
        0.5 * e[6] * e2[20] - 0.5 * e[6] * e2[22] + 0.5 * e[6] * e2[25] +
        e[21] * e[3] * e[24] + e[18] * e[20] * e[8] + e[21] * e[4] * e[25] +
        e[18] * e[19] * e[7] + e[18] * e[1] * e[25] + e[21] * e[22] * e[7] +
        e[21] * e[23] * e[8] + e[18] * e[0] * e[24] + e[18] * e[2] * e[26] +
        e[21] * e[5] * e[26] + e[24] * e[26] * e[8] - e[24] * e[20] * e[2] -
        e[24] * e[19] * e[1] - e[24] * e[22] * e[4] + e[24] * e[25] * e[7] -
        e[24] * e[23] * e[5] + e[0] * e[19] * e[25] + e[0] * e[20] * e[26] +
        e[3] * e[22] * e[25] + e[3] * e[23] * e[26];
    a[40] = e[18] * e[4] * e[8] + e[3] * e[7] * e[20] + e[3] * e[25] * e[2] -
        e[3] * e[1] * e[26] - e[18] * e[5] * e[7] + e[6] * e[1] * e[23] +
        e[6] * e[19] * e[5] - e[6] * e[4] * e[20] - e[6] * e[22] * e[2] +
        e[21] * e[7] * e[2] - e[21] * e[1] * e[8] + e[24] * e[1] * e[5] -
        e[24] * e[4] * e[2] - e[3] * e[19] * e[8] + e[0] * e[4] * e[26] +
        e[0] * e[22] * e[8] - e[0] * e[5] * e[25] - e[0] * e[23] * e[7];
    a[27] = e[9] * e[1] * e[7] + e[9] * e[0] * e[6] + e[9] * e[2] * e[8] +
        e[3] * e[12] * e[6] + e[3] * e[13] * e[7] + e[3] * e[4] * e[16] +
        e[3] * e[14] * e[8] + e[3] * e[5] * e[17] + e[12] * e[4] * e[7] +
        e[12] * e[5] * e[8] + e[6] * e[16] * e[7] + e[6] * e[17] * e[8] -
        e[6] * e[11] * e[2] - e[6] * e[10] * e[1] - e[6] * e[13] * e[4] -
        e[6] * e[14] * e[5] + e[0] * e[10] * e[7] + e[0] * e[1] * e[16] +
        e[0] * e[11] * e[8] + e[0] * e[2] * e[17] + 0.5 * e2[3] * e[15] +
        1.5 * e[15] * e2[6] + 0.5 * e[15] * e2[7] + 0.5 * e[15] * e2[8] +
        0.5 * e2[0] * e[15] - 0.5 * e[15] * e2[4] - 0.5 * e[15] * e2[5] -
        0.5 * e[15] * e2[1] - 0.5 * e[15] * e2[2];
    a[30] = -e[15] * e[13] * e[2] - e[6] * e[13] * e[11] - e[15] * e[4] * e[11] +
        e[12] * e[16] * e[2] - e[3] * e[10] * e[17] + e[3] * e[16] * e[11] +
        e[0] * e[13] * e[17] - e[0] * e[14] * e[16] + e[15] * e[1] * e[14] -
        e[12] * e[10] * e[8] + e[9] * e[4] * e[17] + e[9] * e[13] * e[8] -
        e[9] * e[5] * e[16] - e[9] * e[14] * e[7] + e[15] * e[10] * e[5] +
        e[12] * e[7] * e[11] + e[6] * e[10] * e[14] - e[12] * e[1] * e[17];
    a[147] =
        e[12] * e[30] * e[24] + e[12] * e[21] * e[33] + e[12] * e[31] * e[25] +
        e[12] * e[22] * e[34] + e[12] * e[32] * e[26] + e[12] * e[23] * e[35] +
        e[9] * e[27] * e[24] + e[9] * e[18] * e[33] + e[9] * e[28] * e[25] +
        e[9] * e[19] * e[34] + e[9] * e[29] * e[26] + e[9] * e[20] * e[35] +
        e[21] * e[30] * e[15] + e[21] * e[32] * e[17] + e[21] * e[14] * e[35] +
        e[21] * e[31] * e[16] + e[21] * e[13] * e[34] + e[30] * e[23] * e[17] +
        e[30] * e[14] * e[26] + e[30] * e[22] * e[16] + e[30] * e[13] * e[25] +
        e[15] * e[27] * e[18] + 3. * e[15] * e[33] * e[24] -
        e[15] * e[29] * e[20] + e[15] * e[35] * e[26] - e[15] * e[31] * e[22] -
        e[15] * e[32] * e[23] - e[15] * e[28] * e[19] + e[15] * e[34] * e[25] +
        e[18] * e[29] * e[17] + e[18] * e[11] * e[35] + e[18] * e[28] * e[16] +
        e[18] * e[10] * e[34] + e[27] * e[20] * e[17] + e[27] * e[11] * e[26] +
        e[27] * e[19] * e[16] + e[27] * e[10] * e[25] - e[24] * e[28] * e[10] -
        e[24] * e[31] * e[13] - e[24] * e[32] * e[14] + e[24] * e[34] * e[16] +
        e[24] * e[35] * e[17] - e[24] * e[29] * e[11] - e[33] * e[23] * e[14] +
        e[33] * e[25] * e[16] + e[33] * e[26] * e[17] - e[33] * e[20] * e[11] -
        e[33] * e[19] * e[10] - e[33] * e[22] * e[13];
    a[60] = e[18] * e[13] * e[17] + e[9] * e[13] * e[26] + e[9] * e[22] * e[17] -
        e[9] * e[14] * e[25] - e[18] * e[14] * e[16] - e[15] * e[13] * e[20] -
        e[15] * e[22] * e[11] + e[12] * e[16] * e[20] +
        e[12] * e[25] * e[11] - e[12] * e[10] * e[26] -
        e[12] * e[19] * e[17] + e[21] * e[16] * e[11] -
        e[21] * e[10] * e[17] - e[9] * e[23] * e[16] + e[24] * e[10] * e[14] -
        e[24] * e[13] * e[11] + e[15] * e[10] * e[23] + e[15] * e[19] * e[14];
    a[137] =
        e[21] * e[12] * e[24] + e[21] * e[23] * e[17] + e[21] * e[14] * e[26] +
        e[21] * e[22] * e[16] + e[21] * e[13] * e[25] + e[24] * e[26] * e[17] +
        e[24] * e[25] * e[16] + e[9] * e[19] * e[25] + e[9] * e[18] * e[24] +
        e[9] * e[20] * e[26] + e[12] * e[22] * e[25] + e[12] * e[23] * e[26] +
        e[18] * e[20] * e[17] + e[18] * e[11] * e[26] + e[18] * e[19] * e[16] +
        e[18] * e[10] * e[25] - e[24] * e[23] * e[14] - e[24] * e[20] * e[11] -
        e[24] * e[19] * e[10] - e[24] * e[22] * e[13] + 0.5 * e2[21] * e[15] +
        1.5 * e2[24] * e[15] + 0.5 * e[15] * e2[25] + 0.5 * e[15] * e2[26] +
        0.5 * e[15] * e2[18] - 0.5 * e[15] * e2[23] - 0.5 * e[15] * e2[19] -
        0.5 * e[15] * e2[20] - 0.5 * e[15] * e2[22];
    a[20] = e[6] * e[1] * e[14] + e[15] * e[1] * e[5] - e[0] * e[5] * e[16] -
        e[0] * e[14] * e[7] + e[0] * e[13] * e[8] - e[15] * e[4] * e[2] +
        e[12] * e[7] * e[2] + e[6] * e[10] * e[5] + e[3] * e[7] * e[11] -
        e[6] * e[4] * e[11] + e[3] * e[16] * e[2] - e[6] * e[13] * e[2] -
        e[3] * e[1] * e[17] - e[9] * e[5] * e[7] - e[3] * e[10] * e[8] -
        e[12] * e[1] * e[8] + e[0] * e[4] * e[17] + e[9] * e[4] * e[8];
    a[16] = -0.5 * e[14] * e2[16] - 0.5 * e[14] * e2[10] - 0.5 * e[14] * e2[9] +
        e[11] * e[9] * e[12] + 0.5 * e3[14] + e[17] * e[13] * e[16] +
        0.5 * e[14] * e2[12] + e[11] * e[10] * e[13] - 0.5 * e[14] * e2[15] +
        0.5 * e[14] * e2[17] + e[17] * e[12] * e[15] + 0.5 * e2[11] * e[14] +
        0.5 * e[14] * e2[13];
    a[100] = -e[21] * e[19] * e[8] + e[18] * e[4] * e[26] - e[18] * e[5] * e[25] -
        e[18] * e[23] * e[7] + e[21] * e[25] * e[2] - e[21] * e[1] * e[26] +
        e[6] * e[19] * e[23] + e[18] * e[22] * e[8] - e[0] * e[23] * e[25] -
        e[6] * e[22] * e[20] + e[24] * e[1] * e[23] + e[24] * e[19] * e[5] -
        e[24] * e[4] * e[20] - e[24] * e[22] * e[2] + e[3] * e[25] * e[20] -
        e[3] * e[19] * e[26] + e[0] * e[22] * e[26] + e[21] * e[7] * e[20];
    a[176] =
        0.5 * e2[20] * e[32] + 1.5 * e[32] * e2[23] + 0.5 * e[32] * e2[22] +
        0.5 * e[32] * e2[21] + 0.5 * e[32] * e2[26] - 0.5 * e[32] * e2[18] -
        0.5 * e[32] * e2[19] - 0.5 * e[32] * e2[24] - 0.5 * e[32] * e2[25] +
        e[20] * e[27] * e[21] + e[20] * e[18] * e[30] + e[20] * e[28] * e[22] +
        e[20] * e[19] * e[31] + e[20] * e[29] * e[23] + e[29] * e[19] * e[22] +
        e[29] * e[18] * e[21] + e[23] * e[30] * e[21] + e[23] * e[31] * e[22] +
        e[26] * e[30] * e[24] + e[26] * e[21] * e[33] + e[26] * e[31] * e[25] +
        e[26] * e[22] * e[34] + e[26] * e[23] * e[35] + e[35] * e[22] * e[25] +
        e[35] * e[21] * e[24] - e[23] * e[27] * e[18] - e[23] * e[33] * e[24] -
        e[23] * e[28] * e[19] - e[23] * e[34] * e[25];
    a[130] =
        -e[9] * e[23] * e[25] - e[21] * e[10] * e[26] - e[21] * e[19] * e[17] -
        e[18] * e[23] * e[16] + e[18] * e[13] * e[26] + e[12] * e[25] * e[20] -
        e[12] * e[19] * e[26] - e[15] * e[22] * e[20] + e[21] * e[16] * e[20] +
        e[21] * e[25] * e[11] + e[24] * e[10] * e[23] + e[24] * e[19] * e[14] -
        e[24] * e[13] * e[20] - e[24] * e[22] * e[11] + e[18] * e[22] * e[17] -
        e[18] * e[14] * e[25] + e[9] * e[22] * e[26] + e[15] * e[19] * e[23];
    a[166] = 0.5 * e[23] * e2[21] + e[20] * e[19] * e[22] +
        e[20] * e[18] * e[21] + 0.5 * e3[23] + e[26] * e[22] * e[25] +
        0.5 * e[23] * e2[26] - 0.5 * e[23] * e2[18] + 0.5 * e[23] * e2[22] -
        0.5 * e[23] * e2[19] + e[26] * e[21] * e[24] + 0.5 * e2[20] * e[23] -
        0.5 * e[23] * e2[24] - 0.5 * e[23] * e2[25];
    a[140] =
        e[18] * e[13] * e[35] - e[18] * e[32] * e[16] + e[18] * e[31] * e[17] -
        e[18] * e[14] * e[34] + e[27] * e[13] * e[26] + e[27] * e[22] * e[17] -
        e[27] * e[14] * e[25] - e[27] * e[23] * e[16] - e[9] * e[32] * e[25] +
        e[9] * e[22] * e[35] - e[9] * e[23] * e[34] + e[9] * e[31] * e[26] +
        e[15] * e[19] * e[32] + e[15] * e[28] * e[23] - e[15] * e[22] * e[29] -
        e[15] * e[31] * e[20] + e[24] * e[10] * e[32] + e[24] * e[28] * e[14] -
        e[24] * e[13] * e[29] - e[24] * e[31] * e[11] + e[33] * e[10] * e[23] +
        e[33] * e[19] * e[14] - e[33] * e[13] * e[20] - e[33] * e[22] * e[11] +
        e[21] * e[16] * e[29] - e[21] * e[10] * e[35] - e[21] * e[28] * e[17] +
        e[30] * e[16] * e[20] + e[30] * e[25] * e[11] - e[30] * e[10] * e[26] -
        e[30] * e[19] * e[17] - e[12] * e[28] * e[26] - e[12] * e[19] * e[35] +
        e[12] * e[34] * e[20] + e[12] * e[25] * e[29] + e[21] * e[34] * e[11];
    a[96] = -e[32] * e[10] * e[1] + e[32] * e[13] * e[4] - e[32] * e[16] * e[7] -
        e[32] * e[15] * e[6] - e[32] * e[9] * e[0] + e[32] * e[12] * e[3] +
        e[17] * e[30] * e[6] + e[17] * e[3] * e[33] + e[17] * e[31] * e[7] +
        e[17] * e[4] * e[34] + e[17] * e[5] * e[35] - e[5] * e[27] * e[9] -
        e[5] * e[28] * e[10] - e[5] * e[33] * e[15] - e[5] * e[34] * e[16] +
        e[5] * e[29] * e[11] + e[35] * e[12] * e[6] + e[35] * e[3] * e[15] +
        e[35] * e[13] * e[7] + e[35] * e[4] * e[16] + e[11] * e[27] * e[3] +
        e[11] * e[0] * e[30] + e[11] * e[28] * e[4] + e[11] * e[1] * e[31] +
        e[29] * e[9] * e[3] + e[29] * e[0] * e[12] + e[29] * e[10] * e[4] +
        e[29] * e[1] * e[13] + e[5] * e[30] * e[12] +
        3. * e[5] * e[32] * e[14] + e[5] * e[31] * e[13] +
        e[8] * e[30] * e[15] + e[8] * e[12] * e[33] + e[8] * e[32] * e[17] +
        e[8] * e[14] * e[35] + e[8] * e[31] * e[16] + e[8] * e[13] * e[34] +
        e[2] * e[27] * e[12] + e[2] * e[9] * e[30] + e[2] * e[29] * e[14] +
        e[2] * e[11] * e[32] + e[2] * e[28] * e[13] + e[2] * e[10] * e[31] -
        e[14] * e[27] * e[0] - e[14] * e[34] * e[7] - e[14] * e[33] * e[6] +
        e[14] * e[30] * e[3] - e[14] * e[28] * e[1] + e[14] * e[31] * e[4];
    a[181] =
        0.5 * e[18] * e2[29] + 0.5 * e[18] * e2[28] + 0.5 * e[18] * e2[30] +
        0.5 * e[18] * e2[33] - 0.5 * e[18] * e2[32] - 0.5 * e[18] * e2[31] -
        0.5 * e[18] * e2[34] - 0.5 * e[18] * e2[35] + 1.5 * e[18] * e2[27] +
        e[27] * e[28] * e[19] + e[27] * e[29] * e[20] + e[21] * e[27] * e[30] +
        e[21] * e[29] * e[32] + e[21] * e[28] * e[31] + e[30] * e[28] * e[22] +
        e[30] * e[19] * e[31] + e[30] * e[29] * e[23] + e[30] * e[20] * e[32] +
        e[24] * e[27] * e[33] + e[24] * e[29] * e[35] + e[24] * e[28] * e[34] +
        e[33] * e[28] * e[25] + e[33] * e[19] * e[34] + e[33] * e[29] * e[26] +
        e[33] * e[20] * e[35] - e[27] * e[35] * e[26] - e[27] * e[31] * e[22] -
        e[27] * e[32] * e[23] - e[27] * e[34] * e[25];
    a[46] = e[20] * e[1] * e[4] + e[20] * e[0] * e[3] + e[20] * e[2] * e[5] +
        e[5] * e[21] * e[3] + e[5] * e[22] * e[4] + e[8] * e[21] * e[6] +
        e[8] * e[3] * e[24] + e[8] * e[22] * e[7] + e[8] * e[4] * e[25] +
        e[8] * e[5] * e[26] + e[26] * e[4] * e[7] + e[26] * e[3] * e[6] +
        e[2] * e[18] * e[3] + e[2] * e[0] * e[21] + e[2] * e[19] * e[4] +
        e[2] * e[1] * e[22] - e[5] * e[19] * e[1] - e[5] * e[18] * e[0] -
        e[5] * e[25] * e[7] - e[5] * e[24] * e[6] + 0.5 * e[23] * e2[4] -
        0.5 * e[23] * e2[0] - 0.5 * e[23] * e2[6] + 1.5 * e[23] * e2[5] -
        0.5 * e[23] * e2[1] - 0.5 * e[23] * e2[7] + 0.5 * e[23] * e2[3] +
        0.5 * e[23] * e2[2] + 0.5 * e[23] * e2[8];
    a[151] =
        1.5 * e[9] * e2[27] + 0.5 * e[9] * e2[29] + 0.5 * e[9] * e2[28] -
        0.5 * e[9] * e2[32] - 0.5 * e[9] * e2[31] + 0.5 * e[9] * e2[33] +
        0.5 * e[9] * e2[30] - 0.5 * e[9] * e2[34] - 0.5 * e[9] * e2[35] +
        e[33] * e[27] * e[15] + e[33] * e[29] * e[17] + e[33] * e[11] * e[35] +
        e[33] * e[28] * e[16] + e[33] * e[10] * e[34] + e[27] * e[29] * e[11] +
        e[27] * e[28] * e[10] + e[27] * e[30] * e[12] - e[27] * e[31] * e[13] -
        e[27] * e[32] * e[14] - e[27] * e[34] * e[16] - e[27] * e[35] * e[17] +
        e[30] * e[29] * e[14] + e[30] * e[11] * e[32] + e[30] * e[28] * e[13] +
        e[30] * e[10] * e[31] + e[12] * e[29] * e[32] + e[12] * e[28] * e[31] +
        e[15] * e[29] * e[35] + e[15] * e[28] * e[34];
    a[116] = -e[32] * e[24] * e[6] + e[8] * e[30] * e[24] + e[8] * e[21] * e[33] +
        e[8] * e[31] * e[25] + e[8] * e[22] * e[34] + e[26] * e[30] * e[6] +
        e[26] * e[3] * e[33] + e[26] * e[31] * e[7] + e[26] * e[4] * e[34] +
        e[26] * e[32] * e[8] + e[26] * e[5] * e[35] + e[35] * e[21] * e[6] +
        e[35] * e[3] * e[24] + e[35] * e[22] * e[7] + e[35] * e[4] * e[25] +
        e[35] * e[23] * e[8] + e[2] * e[27] * e[21] + e[2] * e[18] * e[30] +
        e[2] * e[28] * e[22] + e[2] * e[19] * e[31] + e[2] * e[29] * e[23] +
        e[2] * e[20] * e[32] + e[20] * e[27] * e[3] + e[20] * e[0] * e[30] +
        e[20] * e[28] * e[4] + e[20] * e[1] * e[31] + e[20] * e[29] * e[5] +
        e[29] * e[18] * e[3] + e[29] * e[0] * e[21] + e[29] * e[19] * e[4] +
        e[29] * e[1] * e[22] + e[5] * e[30] * e[21] + e[5] * e[31] * e[22] +
        3. * e[5] * e[32] * e[23] - e[5] * e[27] * e[18] -
        e[5] * e[33] * e[24] - e[5] * e[28] * e[19] - e[5] * e[34] * e[25] -
        e[23] * e[27] * e[0] - e[23] * e[34] * e[7] - e[23] * e[33] * e[6] +
        e[23] * e[30] * e[3] - e[23] * e[28] * e[1] + e[23] * e[31] * e[4] +
        e[32] * e[21] * e[3] - e[32] * e[19] * e[1] + e[32] * e[22] * e[4] -
        e[32] * e[18] * e[0] - e[32] * e[25] * e[7];
    a[191] = 0.5 * e[27] * e2[33] - 0.5 * e[27] * e2[32] - 0.5 * e[27] * e2[31] -
        0.5 * e[27] * e2[34] - 0.5 * e[27] * e2[35] + e[33] * e[29] * e[35] +
        0.5 * e[27] * e2[29] + e[30] * e[29] * e[32] +
        e[30] * e[28] * e[31] + e[33] * e[28] * e[34] +
        0.5 * e[27] * e2[28] + 0.5 * e[27] * e2[30] + 0.5 * e3[27];
    a[66] =
        e[14] * e[21] * e[12] + e[14] * e[22] * e[13] + e[17] * e[21] * e[15] +
        e[17] * e[12] * e[24] + e[17] * e[14] * e[26] + e[17] * e[22] * e[16] +
        e[17] * e[13] * e[25] + e[26] * e[12] * e[15] + e[26] * e[13] * e[16] -
        e[14] * e[24] * e[15] - e[14] * e[25] * e[16] - e[14] * e[18] * e[9] -
        e[14] * e[19] * e[10] + e[11] * e[18] * e[12] + e[11] * e[9] * e[21] +
        e[11] * e[19] * e[13] + e[11] * e[10] * e[22] + e[20] * e[11] * e[14] +
        e[20] * e[9] * e[12] + e[20] * e[10] * e[13] + 1.5 * e[23] * e2[14] +
        0.5 * e[23] * e2[12] + 0.5 * e[23] * e2[13] + 0.5 * e[23] * e2[17] +
        0.5 * e2[11] * e[23] - 0.5 * e[23] * e2[16] - 0.5 * e[23] * e2[9] -
        0.5 * e[23] * e2[15] - 0.5 * e[23] * e2[10];
    a[121] = 1.5 * e[0] * e2[27] + 0.5 * e[0] * e2[29] + 0.5 * e[0] * e2[28] +
        0.5 * e[0] * e2[30] - 0.5 * e[0] * e2[32] - 0.5 * e[0] * e2[31] +
        0.5 * e[0] * e2[33] - 0.5 * e[0] * e2[34] - 0.5 * e[0] * e2[35] -
        e[27] * e[31] * e[4] + e[3] * e[27] * e[30] + e[3] * e[29] * e[32] +
        e[3] * e[28] * e[31] + e[30] * e[28] * e[4] + e[30] * e[1] * e[31] +
        e[30] * e[29] * e[5] + e[30] * e[2] * e[32] + e[6] * e[27] * e[33] +
        e[6] * e[29] * e[35] + e[6] * e[28] * e[34] + e[27] * e[28] * e[1] +
        e[27] * e[29] * e[2] + e[33] * e[28] * e[7] + e[33] * e[1] * e[34] +
        e[33] * e[29] * e[8] + e[33] * e[2] * e[35] - e[27] * e[34] * e[7] -
        e[27] * e[32] * e[5] - e[27] * e[35] * e[8];
    a[36] = e[14] * e[12] * e[3] + e[14] * e[13] * e[4] + e[17] * e[12] * e[6] +
        e[17] * e[3] * e[15] + e[17] * e[13] * e[7] + e[17] * e[4] * e[16] +
        e[17] * e[14] * e[8] + e[8] * e[12] * e[15] + e[8] * e[13] * e[16] +
        e[2] * e[11] * e[14] + e[2] * e[9] * e[12] + e[2] * e[10] * e[13] +
        e[11] * e[9] * e[3] + e[11] * e[0] * e[12] + e[11] * e[10] * e[4] +
        e[11] * e[1] * e[13] - e[14] * e[10] * e[1] - e[14] * e[16] * e[7] -
        e[14] * e[15] * e[6] - e[14] * e[9] * e[0] - 0.5 * e[5] * e2[16] -
        0.5 * e[5] * e2[9] + 0.5 * e[5] * e2[11] + 0.5 * e[5] * e2[12] -
        0.5 * e[5] * e2[15] - 0.5 * e[5] * e2[10] + 0.5 * e[5] * e2[13] +
        1.5 * e2[14] * e[5] + 0.5 * e[5] * e2[17];
    a[71] = 1.5 * e[27] * e2[9] - 0.5 * e[27] * e2[16] + 0.5 * e[27] * e2[11] +
        0.5 * e[27] * e2[12] + 0.5 * e[27] * e2[15] - 0.5 * e[27] * e2[17] +
        0.5 * e[27] * e2[10] - 0.5 * e[27] * e2[14] - 0.5 * e[27] * e2[13] +
        e[12] * e[10] * e[31] + e[30] * e[11] * e[14] +
        e[30] * e[10] * e[13] + e[15] * e[9] * e[33] + e[15] * e[29] * e[17] +
        e[15] * e[11] * e[35] + e[15] * e[28] * e[16] +
        e[15] * e[10] * e[34] + e[33] * e[11] * e[17] +
        e[33] * e[10] * e[16] - e[9] * e[31] * e[13] - e[9] * e[32] * e[14] -
        e[9] * e[34] * e[16] - e[9] * e[35] * e[17] + e[9] * e[29] * e[11] +
        e[9] * e[28] * e[10] + e[12] * e[9] * e[30] + e[12] * e[29] * e[14] +
        e[12] * e[11] * e[32] + e[12] * e[28] * e[13];
    a[146] =
        e[29] * e[18] * e[12] + e[29] * e[9] * e[21] + e[29] * e[19] * e[13] +
        e[29] * e[10] * e[22] + e[17] * e[30] * e[24] + e[17] * e[21] * e[33] +
        e[17] * e[31] * e[25] + e[17] * e[22] * e[34] + e[17] * e[32] * e[26] +
        e[17] * e[23] * e[35] - e[23] * e[27] * e[9] - e[23] * e[28] * e[10] -
        e[23] * e[33] * e[15] - e[23] * e[34] * e[16] - e[32] * e[24] * e[15] -
        e[32] * e[25] * e[16] - e[32] * e[18] * e[9] - e[32] * e[19] * e[10] +
        e[26] * e[30] * e[15] + e[26] * e[12] * e[33] + e[26] * e[31] * e[16] +
        e[26] * e[13] * e[34] + e[35] * e[21] * e[15] + e[35] * e[12] * e[24] +
        e[35] * e[22] * e[16] + e[35] * e[13] * e[25] + e[14] * e[30] * e[21] +
        e[14] * e[31] * e[22] + 3. * e[14] * e[32] * e[23] +
        e[11] * e[27] * e[21] + e[11] * e[18] * e[30] + e[11] * e[28] * e[22] +
        e[11] * e[19] * e[31] + e[11] * e[29] * e[23] + e[11] * e[20] * e[32] +
        e[23] * e[30] * e[12] + e[23] * e[31] * e[13] + e[32] * e[21] * e[12] +
        e[32] * e[22] * e[13] - e[14] * e[27] * e[18] - e[14] * e[33] * e[24] +
        e[14] * e[29] * e[20] + e[14] * e[35] * e[26] - e[14] * e[28] * e[19] -
        e[14] * e[34] * e[25] + e[20] * e[27] * e[12] + e[20] * e[9] * e[30] +
        e[20] * e[28] * e[13] + e[20] * e[10] * e[31];
    a[1] = 0.5 * e[0] * e2[1] + 0.5 * e[0] * e2[2] + e[6] * e[2] * e[8] +
        e[6] * e[1] * e[7] + 0.5 * e[0] * e2[3] + e[3] * e[1] * e[4] +
        0.5 * e[0] * e2[6] + e[3] * e[2] * e[5] - 0.5 * e[0] * e2[5] -
        0.5 * e[0] * e2[8] + 0.5 * e3[0] - 0.5 * e[0] * e2[7] -
        0.5 * e[0] * e2[4];
    a[136] =
        1.5 * e2[23] * e[14] + 0.5 * e[14] * e2[26] - 0.5 * e[14] * e2[18] -
        0.5 * e[14] * e2[19] + 0.5 * e[14] * e2[20] + 0.5 * e[14] * e2[22] -
        0.5 * e[14] * e2[24] + 0.5 * e[14] * e2[21] - 0.5 * e[14] * e2[25] +
        e[23] * e[21] * e[12] + e[23] * e[22] * e[13] + e[26] * e[21] * e[15] +
        e[26] * e[12] * e[24] + e[26] * e[23] * e[17] + e[26] * e[22] * e[16] +
        e[26] * e[13] * e[25] + e[17] * e[22] * e[25] + e[17] * e[21] * e[24] +
        e[11] * e[19] * e[22] + e[11] * e[18] * e[21] + e[11] * e[20] * e[23] +
        e[20] * e[18] * e[12] + e[20] * e[9] * e[21] + e[20] * e[19] * e[13] +
        e[20] * e[10] * e[22] - e[23] * e[24] * e[15] - e[23] * e[25] * e[16] -
        e[23] * e[18] * e[9] - e[23] * e[19] * e[10];
    a[51] = 1.5 * e[27] * e2[0] - 0.5 * e[27] * e2[4] + 0.5 * e[27] * e2[6] -
        0.5 * e[27] * e2[5] + 0.5 * e[27] * e2[1] - 0.5 * e[27] * e2[7] +
        0.5 * e[27] * e2[3] + 0.5 * e[27] * e2[2] - 0.5 * e[27] * e2[8] +
        e[0] * e[33] * e[6] + e[0] * e[30] * e[3] - e[0] * e[35] * e[8] -
        e[0] * e[31] * e[4] + e[3] * e[28] * e[4] + e[3] * e[1] * e[31] +
        e[3] * e[29] * e[5] + e[3] * e[2] * e[32] + e[30] * e[1] * e[4] +
        e[30] * e[2] * e[5] + e[6] * e[28] * e[7] + e[6] * e[1] * e[34] +
        e[6] * e[29] * e[8] + e[6] * e[2] * e[35] + e[33] * e[1] * e[7] +
        e[33] * e[2] * e[8] + e[0] * e[28] * e[1] + e[0] * e[29] * e[2] -
        e[0] * e[34] * e[7] - e[0] * e[32] * e[5];
    a[106] = e[8] * e[22] * e[25] + e[8] * e[21] * e[24] + e[20] * e[18] * e[3] +
        e[20] * e[0] * e[21] + e[20] * e[19] * e[4] + e[20] * e[1] * e[22] +
        e[20] * e[2] * e[23] + e[23] * e[21] * e[3] + e[23] * e[22] * e[4] +
        e[23] * e[26] * e[8] - e[23] * e[19] * e[1] - e[23] * e[18] * e[0] -
        e[23] * e[25] * e[7] - e[23] * e[24] * e[6] + e[2] * e[19] * e[22] +
        e[2] * e[18] * e[21] + e[26] * e[21] * e[6] + e[26] * e[3] * e[24] +
        e[26] * e[22] * e[7] + e[26] * e[4] * e[25] + 0.5 * e2[20] * e[5] +
        1.5 * e2[23] * e[5] + 0.5 * e[5] * e2[22] + 0.5 * e[5] * e2[21] +
        0.5 * e[5] * e2[26] - 0.5 * e[5] * e2[18] - 0.5 * e[5] * e2[19] -
        0.5 * e[5] * e2[24] - 0.5 * e[5] * e2[25];
    a[81] = e[24] * e[11] * e[8] + e[24] * e[2] * e[17] +
        3. * e[9] * e[18] * e[0] + e[9] * e[19] * e[1] + e[9] * e[20] * e[2] +
        e[18] * e[10] * e[1] + e[18] * e[11] * e[2] + e[3] * e[18] * e[12] +
        e[3] * e[9] * e[21] + e[3] * e[20] * e[14] + e[3] * e[11] * e[23] +
        e[3] * e[19] * e[13] + e[3] * e[10] * e[22] + e[6] * e[18] * e[15] +
        e[6] * e[9] * e[24] + e[6] * e[20] * e[17] + e[6] * e[11] * e[26] +
        e[6] * e[19] * e[16] + e[6] * e[10] * e[25] + e[0] * e[20] * e[11] +
        e[0] * e[19] * e[10] - e[9] * e[26] * e[8] - e[9] * e[22] * e[4] -
        e[9] * e[25] * e[7] - e[9] * e[23] * e[5] + e[12] * e[0] * e[21] +
        e[12] * e[19] * e[4] + e[12] * e[1] * e[22] + e[12] * e[20] * e[5] +
        e[12] * e[2] * e[23] - e[18] * e[13] * e[4] - e[18] * e[16] * e[7] -
        e[18] * e[14] * e[5] - e[18] * e[17] * e[8] + e[21] * e[10] * e[4] +
        e[21] * e[1] * e[13] + e[21] * e[11] * e[5] + e[21] * e[2] * e[14] +
        e[15] * e[0] * e[24] + e[15] * e[19] * e[7] + e[15] * e[1] * e[25] +
        e[15] * e[20] * e[8] + e[15] * e[2] * e[26] - e[0] * e[23] * e[14] -
        e[0] * e[25] * e[16] - e[0] * e[26] * e[17] - e[0] * e[22] * e[13] +
        e[24] * e[10] * e[7] + e[24] * e[1] * e[16];
    a[26] = e[11] * e[1] * e[4] + e[11] * e[0] * e[3] + e[11] * e[2] * e[5] +
        e[5] * e[12] * e[3] + e[5] * e[13] * e[4] + e[8] * e[12] * e[6] +
        e[8] * e[3] * e[15] + e[8] * e[13] * e[7] + e[8] * e[4] * e[16] +
        e[8] * e[5] * e[17] + e[17] * e[4] * e[7] + e[17] * e[3] * e[6] -
        e[5] * e[10] * e[1] - e[5] * e[16] * e[7] - e[5] * e[15] * e[6] -
        e[5] * e[9] * e[0] + e[2] * e[9] * e[3] + e[2] * e[0] * e[12] +
        e[2] * e[10] * e[4] + e[2] * e[1] * e[13] + 0.5 * e2[2] * e[14] -
        0.5 * e[14] * e2[0] - 0.5 * e[14] * e2[6] - 0.5 * e[14] * e2[1] -
        0.5 * e[14] * e2[7] + 1.5 * e[14] * e2[5] + 0.5 * e[14] * e2[4] +
        0.5 * e[14] * e2[3] + 0.5 * e[14] * e2[8];
    a[91] = e[3] * e[27] * e[12] + e[3] * e[9] * e[30] + e[3] * e[29] * e[14] +
        e[3] * e[11] * e[32] + e[3] * e[28] * e[13] + e[3] * e[10] * e[31] +
        e[6] * e[27] * e[15] + e[6] * e[9] * e[33] + e[6] * e[29] * e[17] +
        e[6] * e[11] * e[35] + e[6] * e[28] * e[16] + e[6] * e[10] * e[34] +
        3. * e[0] * e[27] * e[9] + e[0] * e[29] * e[11] +
        e[0] * e[28] * e[10] - e[9] * e[34] * e[7] - e[9] * e[32] * e[5] -
        e[9] * e[35] * e[8] + e[9] * e[29] * e[2] + e[9] * e[28] * e[1] -
        e[9] * e[31] * e[4] + e[12] * e[0] * e[30] + e[12] * e[28] * e[4] +
        e[12] * e[1] * e[31] + e[12] * e[29] * e[5] + e[12] * e[2] * e[32] +
        e[27] * e[11] * e[2] + e[27] * e[10] * e[1] - e[27] * e[13] * e[4] -
        e[27] * e[16] * e[7] - e[27] * e[14] * e[5] - e[27] * e[17] * e[8] +
        e[30] * e[10] * e[4] + e[30] * e[1] * e[13] + e[30] * e[11] * e[5] +
        e[30] * e[2] * e[14] + e[15] * e[0] * e[33] + e[15] * e[28] * e[7] +
        e[15] * e[1] * e[34] + e[15] * e[29] * e[8] + e[15] * e[2] * e[35] -
        e[0] * e[31] * e[13] - e[0] * e[32] * e[14] - e[0] * e[34] * e[16] -
        e[0] * e[35] * e[17] + e[33] * e[10] * e[7] + e[33] * e[1] * e[16] +
        e[33] * e[11] * e[8] + e[33] * e[2] * e[17];
    a[127] = 0.5 * e2[30] * e[6] + 0.5 * e[6] * e2[27] - 0.5 * e[6] * e2[32] -
        0.5 * e[6] * e2[28] - 0.5 * e[6] * e2[29] - 0.5 * e[6] * e2[31] +
        1.5 * e[6] * e2[33] + 0.5 * e[6] * e2[34] + 0.5 * e[6] * e2[35] +
        e[0] * e[27] * e[33] + e[0] * e[29] * e[35] + e[0] * e[28] * e[34] +
        e[3] * e[30] * e[33] + e[3] * e[32] * e[35] + e[3] * e[31] * e[34] +
        e[30] * e[31] * e[7] + e[30] * e[4] * e[34] + e[30] * e[32] * e[8] +
        e[30] * e[5] * e[35] + e[27] * e[28] * e[7] + e[27] * e[1] * e[34] +
        e[27] * e[29] * e[8] + e[27] * e[2] * e[35] + e[33] * e[34] * e[7] +
        e[33] * e[35] * e[8] - e[33] * e[32] * e[5] - e[33] * e[29] * e[2] -
        e[33] * e[28] * e[1] - e[33] * e[31] * e[4];
    a[161] = e[24] * e[20] * e[26] + e[21] * e[19] * e[22] -
        0.5 * e[18] * e2[22] - 0.5 * e[18] * e2[25] + 0.5 * e3[18] +
        0.5 * e[18] * e2[21] + e[21] * e[20] * e[23] + 0.5 * e[18] * e2[20] +
        0.5 * e[18] * e2[19] + 0.5 * e[18] * e2[24] + e[24] * e[19] * e[25] -
        0.5 * e[18] * e2[23] - 0.5 * e[18] * e2[26];
    a[197] = 0.5 * e[33] * e2[35] + 0.5 * e3[33] + 0.5 * e2[27] * e[33] +
        0.5 * e2[30] * e[33] - 0.5 * e[33] * e2[29] + 0.5 * e[33] * e2[34] -
        0.5 * e[33] * e2[32] - 0.5 * e[33] * e2[28] + e[30] * e[32] * e[35] -
        0.5 * e[33] * e2[31] + e[27] * e[29] * e[35] +
        e[27] * e[28] * e[34] + e[30] * e[31] * e[34];
    a[171] =
        1.5 * e[27] * e2[18] + 0.5 * e[27] * e2[19] + 0.5 * e[27] * e2[20] +
        0.5 * e[27] * e2[21] + 0.5 * e[27] * e2[24] - 0.5 * e[27] * e2[26] -
        0.5 * e[27] * e2[23] - 0.5 * e[27] * e2[22] - 0.5 * e[27] * e2[25] +
        e[33] * e[20] * e[26] - e[18] * e[35] * e[26] - e[18] * e[31] * e[22] -
        e[18] * e[32] * e[23] - e[18] * e[34] * e[25] + e[18] * e[28] * e[19] +
        e[18] * e[29] * e[20] + e[21] * e[18] * e[30] + e[21] * e[28] * e[22] +
        e[21] * e[19] * e[31] + e[21] * e[29] * e[23] + e[21] * e[20] * e[32] +
        e[30] * e[19] * e[22] + e[30] * e[20] * e[23] + e[24] * e[18] * e[33] +
        e[24] * e[28] * e[25] + e[24] * e[19] * e[34] + e[24] * e[29] * e[26] +
        e[24] * e[20] * e[35] + e[33] * e[19] * e[25];
    a[157] =
        e[9] * e[27] * e[33] + e[9] * e[29] * e[35] + e[9] * e[28] * e[34] +
        e[33] * e[35] * e[17] + e[33] * e[34] * e[16] + e[27] * e[29] * e[17] +
        e[27] * e[11] * e[35] + e[27] * e[28] * e[16] + e[27] * e[10] * e[34] +
        e[33] * e[30] * e[12] - e[33] * e[28] * e[10] - e[33] * e[31] * e[13] -
        e[33] * e[32] * e[14] - e[33] * e[29] * e[11] + e[30] * e[32] * e[17] +
        e[30] * e[14] * e[35] + e[30] * e[31] * e[16] + e[30] * e[13] * e[34] +
        e[12] * e[32] * e[35] + e[12] * e[31] * e[34] + 0.5 * e[15] * e2[27] -
        0.5 * e[15] * e2[32] - 0.5 * e[15] * e2[28] - 0.5 * e[15] * e2[29] -
        0.5 * e[15] * e2[31] + 1.5 * e[15] * e2[33] + 0.5 * e[15] * e2[30] +
        0.5 * e[15] * e2[34] + 0.5 * e[15] * e2[35];
    a[11] = 0.5 * e[9] * e2[12] - 0.5 * e[9] * e2[16] + 0.5 * e[9] * e2[10] -
        0.5 * e[9] * e2[17] - 0.5 * e[9] * e2[13] + e[15] * e[10] * e[16] +
        e[12] * e[11] * e[14] + 0.5 * e[9] * e2[11] + 0.5 * e[9] * e2[15] -
        0.5 * e[9] * e2[14] + e[15] * e[11] * e[17] + 0.5 * e3[9] +
        e[12] * e[10] * e[13];
    a[187] =
        e[18] * e[27] * e[33] + e[18] * e[29] * e[35] + e[18] * e[28] * e[34] +
        e[27] * e[28] * e[25] + e[27] * e[19] * e[34] + e[27] * e[29] * e[26] +
        e[27] * e[20] * e[35] + e[21] * e[30] * e[33] + e[21] * e[32] * e[35] +
        e[21] * e[31] * e[34] + e[30] * e[31] * e[25] + e[30] * e[22] * e[34] +
        e[30] * e[32] * e[26] + e[30] * e[23] * e[35] + e[33] * e[34] * e[25] +
        e[33] * e[35] * e[26] - e[33] * e[29] * e[20] - e[33] * e[31] * e[22] -
        e[33] * e[32] * e[23] - e[33] * e[28] * e[19] + 0.5 * e2[27] * e[24] +
        0.5 * e2[30] * e[24] + 1.5 * e[24] * e2[33] + 0.5 * e[24] * e2[35] +
        0.5 * e[24] * e2[34] - 0.5 * e[24] * e2[32] - 0.5 * e[24] * e2[28] -
        0.5 * e[24] * e2[29] - 0.5 * e[24] * e2[31];
    a[131] =
        0.5 * e[9] * e2[21] + 0.5 * e[9] * e2[24] + 0.5 * e[9] * e2[19] +
        1.5 * e[9] * e2[18] + 0.5 * e[9] * e2[20] - 0.5 * e[9] * e2[26] -
        0.5 * e[9] * e2[23] - 0.5 * e[9] * e2[22] - 0.5 * e[9] * e2[25] +
        e[21] * e[18] * e[12] + e[21] * e[20] * e[14] + e[21] * e[11] * e[23] +
        e[21] * e[19] * e[13] + e[21] * e[10] * e[22] + e[24] * e[18] * e[15] +
        e[24] * e[20] * e[17] + e[24] * e[11] * e[26] + e[24] * e[19] * e[16] +
        e[24] * e[10] * e[25] + e[15] * e[19] * e[25] + e[15] * e[20] * e[26] +
        e[12] * e[19] * e[22] + e[12] * e[20] * e[23] + e[18] * e[20] * e[11] +
        e[18] * e[19] * e[10] - e[18] * e[23] * e[14] - e[18] * e[25] * e[16] -
        e[18] * e[26] * e[17] - e[18] * e[22] * e[13];
    a[189] =
        0.5 * e2[29] * e[26] + 0.5 * e2[32] * e[26] + 0.5 * e[26] * e2[33] +
        1.5 * e[26] * e2[35] + 0.5 * e[26] * e2[34] - 0.5 * e[26] * e2[27] -
        0.5 * e[26] * e2[28] - 0.5 * e[26] * e2[31] - 0.5 * e[26] * e2[30] +
        e[20] * e[27] * e[33] + e[20] * e[29] * e[35] + e[20] * e[28] * e[34] +
        e[29] * e[27] * e[24] + e[29] * e[18] * e[33] + e[29] * e[28] * e[25] +
        e[29] * e[19] * e[34] + e[23] * e[30] * e[33] + e[23] * e[32] * e[35] +
        e[23] * e[31] * e[34] + e[32] * e[30] * e[24] + e[32] * e[21] * e[33] +
        e[32] * e[31] * e[25] + e[32] * e[22] * e[34] + e[35] * e[33] * e[24] +
        e[35] * e[34] * e[25] - e[35] * e[27] * e[18] - e[35] * e[30] * e[21] -
        e[35] * e[31] * e[22] - e[35] * e[28] * e[19];
    a[141] =
        e[12] * e[19] * e[31] + e[12] * e[29] * e[23] + e[12] * e[20] * e[32] +
        3. * e[9] * e[27] * e[18] + e[9] * e[28] * e[19] + e[9] * e[29] * e[20] +
        e[21] * e[9] * e[30] + e[21] * e[29] * e[14] + e[21] * e[11] * e[32] +
        e[21] * e[28] * e[13] + e[21] * e[10] * e[31] + e[30] * e[20] * e[14] +
        e[30] * e[11] * e[23] + e[30] * e[19] * e[13] + e[30] * e[10] * e[22] +
        e[9] * e[33] * e[24] - e[9] * e[35] * e[26] - e[9] * e[31] * e[22] -
        e[9] * e[32] * e[23] - e[9] * e[34] * e[25] + e[18] * e[29] * e[11] +
        e[18] * e[28] * e[10] + e[27] * e[20] * e[11] + e[27] * e[19] * e[10] +
        e[15] * e[27] * e[24] + e[15] * e[18] * e[33] + e[15] * e[28] * e[25] +
        e[15] * e[19] * e[34] + e[15] * e[29] * e[26] + e[15] * e[20] * e[35] -
        e[18] * e[31] * e[13] - e[18] * e[32] * e[14] - e[18] * e[34] * e[16] -
        e[18] * e[35] * e[17] - e[27] * e[23] * e[14] - e[27] * e[25] * e[16] -
        e[27] * e[26] * e[17] - e[27] * e[22] * e[13] + e[24] * e[29] * e[17] +
        e[24] * e[11] * e[35] + e[24] * e[28] * e[16] + e[24] * e[10] * e[34] +
        e[33] * e[20] * e[17] + e[33] * e[11] * e[26] + e[33] * e[19] * e[16] +
        e[33] * e[10] * e[25] + e[12] * e[27] * e[21] + e[12] * e[18] * e[30] +
        e[12] * e[28] * e[22];
    a[159] =
        -0.5 * e[17] * e2[27] + 0.5 * e[17] * e2[32] - 0.5 * e[17] * e2[28] +
        0.5 * e[17] * e2[29] - 0.5 * e[17] * e2[31] + 0.5 * e[17] * e2[33] -
        0.5 * e[17] * e2[30] + 0.5 * e[17] * e2[34] + 1.5 * e[17] * e2[35] +
        e[32] * e[30] * e[15] + e[32] * e[12] * e[33] + e[32] * e[31] * e[16] +
        e[32] * e[13] * e[34] + e[14] * e[30] * e[33] + e[14] * e[31] * e[34] +
        e[11] * e[27] * e[33] + e[11] * e[29] * e[35] + e[11] * e[28] * e[34] +
        e[35] * e[33] * e[15] + e[35] * e[34] * e[16] + e[29] * e[27] * e[15] +
        e[29] * e[9] * e[33] + e[29] * e[28] * e[16] + e[29] * e[10] * e[34] -
        e[35] * e[27] * e[9] - e[35] * e[30] * e[12] - e[35] * e[28] * e[10] -
        e[35] * e[31] * e[13] + e[35] * e[32] * e[14];
    a[21] = 0.5 * e[9] * e2[1] + 1.5 * e[9] * e2[0] + 0.5 * e[9] * e2[2] +
        0.5 * e[9] * e2[3] + 0.5 * e[9] * e2[6] - 0.5 * e[9] * e2[4] -
        0.5 * e[9] * e2[5] - 0.5 * e[9] * e2[7] - 0.5 * e[9] * e2[8] +
        e[6] * e[0] * e[15] + e[6] * e[10] * e[7] + e[6] * e[1] * e[16] +
        e[6] * e[11] * e[8] + e[6] * e[2] * e[17] + e[15] * e[1] * e[7] +
        e[15] * e[2] * e[8] + e[0] * e[11] * e[2] + e[0] * e[10] * e[1] -
        e[0] * e[13] * e[4] - e[0] * e[16] * e[7] - e[0] * e[14] * e[5] -
        e[0] * e[17] * e[8] + e[3] * e[0] * e[12] + e[3] * e[10] * e[4] +
        e[3] * e[1] * e[13] + e[3] * e[11] * e[5] + e[3] * e[2] * e[14] +
        e[12] * e[1] * e[4] + e[12] * e[2] * e[5];
    a[199] = 0.5 * e[35] * e2[33] + 0.5 * e[35] * e2[34] - 0.5 * e[35] * e2[27] -
        0.5 * e[35] * e2[28] - 0.5 * e[35] * e2[31] - 0.5 * e[35] * e2[30] +
        e[32] * e[31] * e[34] + 0.5 * e2[29] * e[35] + 0.5 * e2[32] * e[35] +
        e[29] * e[28] * e[34] + e[32] * e[30] * e[33] + 0.5 * e3[35] +
        e[29] * e[27] * e[33];
    a[101] = 0.5 * e[0] * e2[19] + 0.5 * e[0] * e2[20] + 0.5 * e[0] * e2[24] -
        0.5 * e[0] * e2[26] - 0.5 * e[0] * e2[23] - 0.5 * e[0] * e2[22] -
        0.5 * e[0] * e2[25] + 1.5 * e2[18] * e[0] + 0.5 * e[0] * e2[21] +
        e[18] * e[19] * e[1] + e[18] * e[20] * e[2] + e[21] * e[18] * e[3] +
        e[21] * e[19] * e[4] + e[21] * e[1] * e[22] + e[21] * e[20] * e[5] +
        e[21] * e[2] * e[23] - e[18] * e[26] * e[8] - e[18] * e[22] * e[4] -
        e[18] * e[25] * e[7] - e[18] * e[23] * e[5] + e[18] * e[24] * e[6] +
        e[3] * e[19] * e[22] + e[3] * e[20] * e[23] + e[24] * e[19] * e[7] +
        e[24] * e[1] * e[25] + e[24] * e[20] * e[8] + e[24] * e[2] * e[26] +
        e[6] * e[19] * e[25] + e[6] * e[20] * e[26];
    a[129] = 0.5 * e2[32] * e[8] - 0.5 * e[8] * e2[27] - 0.5 * e[8] * e2[28] +
        0.5 * e[8] * e2[29] - 0.5 * e[8] * e2[31] + 0.5 * e[8] * e2[33] -
        0.5 * e[8] * e2[30] + 0.5 * e[8] * e2[34] + 1.5 * e[8] * e2[35] +
        e[2] * e[27] * e[33] + e[2] * e[29] * e[35] + e[2] * e[28] * e[34] +
        e[5] * e[30] * e[33] + e[5] * e[32] * e[35] + e[5] * e[31] * e[34] +
        e[32] * e[30] * e[6] + e[32] * e[3] * e[33] + e[32] * e[31] * e[7] +
        e[32] * e[4] * e[34] + e[29] * e[27] * e[6] + e[29] * e[0] * e[33] +
        e[29] * e[28] * e[7] + e[29] * e[1] * e[34] + e[35] * e[33] * e[6] +
        e[35] * e[34] * e[7] - e[35] * e[27] * e[0] - e[35] * e[30] * e[3] -
        e[35] * e[28] * e[1] - e[35] * e[31] * e[4];
    a[41] = -0.5 * e[18] * e2[4] + 1.5 * e[18] * e2[0] + 0.5 * e[18] * e2[6] -
        0.5 * e[18] * e2[5] + 0.5 * e[18] * e2[1] - 0.5 * e[18] * e2[7] +
        0.5 * e[18] * e2[3] + 0.5 * e[18] * e2[2] - 0.5 * e[18] * e2[8] +
        e[3] * e[0] * e[21] + e[3] * e[19] * e[4] + e[3] * e[1] * e[22] +
        e[3] * e[20] * e[5] + e[3] * e[2] * e[23] + e[21] * e[1] * e[4] +
        e[21] * e[2] * e[5] + e[6] * e[0] * e[24] + e[6] * e[19] * e[7] +
        e[6] * e[1] * e[25] + e[6] * e[20] * e[8] + e[6] * e[2] * e[26] +
        e[24] * e[1] * e[7] + e[24] * e[2] * e[8] + e[0] * e[19] * e[1] +
        e[0] * e[20] * e[2] - e[0] * e[26] * e[8] - e[0] * e[22] * e[4] -
        e[0] * e[25] * e[7] - e[0] * e[23] * e[5];
    a[28] = e[10] * e[1] * e[7] + e[10] * e[0] * e[6] + e[10] * e[2] * e[8] +
        e[4] * e[12] * e[6] + e[4] * e[3] * e[15] + e[4] * e[13] * e[7] +
        e[4] * e[14] * e[8] + e[4] * e[5] * e[17] + e[13] * e[3] * e[6] +
        e[13] * e[5] * e[8] + e[7] * e[15] * e[6] + e[7] * e[17] * e[8] -
        e[7] * e[11] * e[2] - e[7] * e[9] * e[0] - e[7] * e[14] * e[5] -
        e[7] * e[12] * e[3] + e[1] * e[9] * e[6] + e[1] * e[0] * e[15] +
        e[1] * e[11] * e[8] + e[1] * e[2] * e[17] + 1.5 * e[16] * e2[7] +
        0.5 * e[16] * e2[6] + 0.5 * e[16] * e2[8] + 0.5 * e2[1] * e[16] -
        0.5 * e[16] * e2[0] - 0.5 * e[16] * e2[5] - 0.5 * e[16] * e2[3] -
        0.5 * e[16] * e2[2] + 0.5 * e2[4] * e[16];
    a[111] = e[0] * e[30] * e[21] - e[0] * e[35] * e[26] - e[0] * e[31] * e[22] -
        e[0] * e[32] * e[23] - e[0] * e[34] * e[25] - e[18] * e[34] * e[7] -
        e[18] * e[32] * e[5] - e[18] * e[35] * e[8] - e[18] * e[31] * e[4] -
        e[27] * e[26] * e[8] - e[27] * e[22] * e[4] - e[27] * e[25] * e[7] -
        e[27] * e[23] * e[5] + e[6] * e[28] * e[25] + e[6] * e[19] * e[34] +
        e[6] * e[29] * e[26] + e[6] * e[20] * e[35] + e[21] * e[28] * e[4] +
        e[21] * e[1] * e[31] + e[21] * e[29] * e[5] + e[21] * e[2] * e[32] +
        e[30] * e[19] * e[4] + e[30] * e[1] * e[22] + e[30] * e[20] * e[5] +
        e[30] * e[2] * e[23] + e[24] * e[27] * e[6] + e[24] * e[0] * e[33] +
        e[24] * e[28] * e[7] + e[24] * e[1] * e[34] + e[24] * e[29] * e[8] +
        e[24] * e[2] * e[35] + e[33] * e[18] * e[6] + e[33] * e[19] * e[7] +
        e[33] * e[1] * e[25] + e[33] * e[20] * e[8] + e[33] * e[2] * e[26] +
        3. * e[0] * e[27] * e[18] + e[0] * e[28] * e[19] +
        e[0] * e[29] * e[20] + e[18] * e[28] * e[1] + e[18] * e[29] * e[2] +
        e[27] * e[19] * e[1] + e[27] * e[20] * e[2] + e[3] * e[27] * e[21] +
        e[3] * e[18] * e[30] + e[3] * e[28] * e[22] + e[3] * e[19] * e[31] +
        e[3] * e[29] * e[23] + e[3] * e[20] * e[32];
    a[108] = e[19] * e[18] * e[6] + e[19] * e[0] * e[24] + e[19] * e[1] * e[25] +
        e[19] * e[20] * e[8] + e[19] * e[2] * e[26] + e[22] * e[21] * e[6] +
        e[22] * e[3] * e[24] + e[22] * e[4] * e[25] + e[22] * e[23] * e[8] +
        e[22] * e[5] * e[26] - e[25] * e[21] * e[3] + e[25] * e[26] * e[8] -
        e[25] * e[20] * e[2] - e[25] * e[18] * e[0] - e[25] * e[23] * e[5] +
        e[25] * e[24] * e[6] + e[1] * e[18] * e[24] + e[1] * e[20] * e[26] +
        e[4] * e[21] * e[24] + e[4] * e[23] * e[26] + 0.5 * e2[19] * e[7] +
        0.5 * e2[22] * e[7] + 1.5 * e2[25] * e[7] + 0.5 * e[7] * e2[26] -
        0.5 * e[7] * e2[18] - 0.5 * e[7] * e2[23] - 0.5 * e[7] * e2[20] +
        0.5 * e[7] * e2[24] - 0.5 * e[7] * e2[21];
    a[61] = 0.5 * e[18] * e2[11] + 1.5 * e[18] * e2[9] + 0.5 * e[18] * e2[10] +
        0.5 * e[18] * e2[12] + 0.5 * e[18] * e2[15] - 0.5 * e[18] * e2[16] -
        0.5 * e[18] * e2[17] - 0.5 * e[18] * e2[14] - 0.5 * e[18] * e2[13] +
        e[12] * e[9] * e[21] + e[12] * e[20] * e[14] + e[12] * e[11] * e[23] +
        e[12] * e[19] * e[13] + e[12] * e[10] * e[22] +
        e[21] * e[11] * e[14] + e[21] * e[10] * e[13] + e[15] * e[9] * e[24] +
        e[15] * e[20] * e[17] + e[15] * e[11] * e[26] +
        e[15] * e[19] * e[16] + e[15] * e[10] * e[25] +
        e[24] * e[11] * e[17] + e[24] * e[10] * e[16] - e[9] * e[23] * e[14] -
        e[9] * e[25] * e[16] - e[9] * e[26] * e[17] + e[9] * e[20] * e[11] +
        e[9] * e[19] * e[10] - e[9] * e[22] * e[13];
    a[138] =
        e[13] * e[21] * e[24] + e[13] * e[23] * e[26] + e[19] * e[18] * e[15] +
        e[19] * e[9] * e[24] + e[19] * e[20] * e[17] + e[19] * e[11] * e[26] -
        e[25] * e[23] * e[14] - e[25] * e[20] * e[11] - e[25] * e[18] * e[9] -
        e[25] * e[21] * e[12] + e[22] * e[21] * e[15] + e[22] * e[12] * e[24] +
        e[22] * e[23] * e[17] + e[22] * e[14] * e[26] + e[22] * e[13] * e[25] +
        e[25] * e[24] * e[15] + e[25] * e[26] * e[17] + e[10] * e[19] * e[25] +
        e[10] * e[18] * e[24] + e[10] * e[20] * e[26] - 0.5 * e[16] * e2[18] -
        0.5 * e[16] * e2[23] + 0.5 * e[16] * e2[19] - 0.5 * e[16] * e2[20] -
        0.5 * e[16] * e2[21] + 0.5 * e2[22] * e[16] + 1.5 * e2[25] * e[16] +
        0.5 * e[16] * e2[24] + 0.5 * e[16] * e2[26];
    a[31] = 0.5 * e[0] * e2[12] + 0.5 * e[0] * e2[15] + 0.5 * e[0] * e2[11] +
        1.5 * e[0] * e2[9] + 0.5 * e[0] * e2[10] - 0.5 * e[0] * e2[16] -
        0.5 * e[0] * e2[17] - 0.5 * e[0] * e2[14] - 0.5 * e[0] * e2[13] +
        e[12] * e[9] * e[3] + e[12] * e[10] * e[4] + e[12] * e[1] * e[13] +
        e[12] * e[11] * e[5] + e[12] * e[2] * e[14] + e[15] * e[9] * e[6] +
        e[15] * e[10] * e[7] + e[15] * e[1] * e[16] + e[15] * e[11] * e[8] +
        e[15] * e[2] * e[17] + e[6] * e[11] * e[17] + e[6] * e[10] * e[16] +
        e[3] * e[11] * e[14] + e[3] * e[10] * e[13] + e[9] * e[10] * e[1] +
        e[9] * e[11] * e[2] - e[9] * e[13] * e[4] - e[9] * e[16] * e[7] -
        e[9] * e[14] * e[5] - e[9] * e[17] * e[8];
    a[148] =
        e[19] * e[11] * e[35] + e[28] * e[18] * e[15] + e[28] * e[9] * e[24] +
        e[28] * e[20] * e[17] + e[28] * e[11] * e[26] - e[25] * e[27] * e[9] -
        e[25] * e[30] * e[12] - e[25] * e[32] * e[14] + e[25] * e[33] * e[15] +
        e[25] * e[35] * e[17] - e[25] * e[29] * e[11] - e[34] * e[23] * e[14] +
        e[34] * e[24] * e[15] + e[34] * e[26] * e[17] - e[34] * e[20] * e[11] -
        e[34] * e[18] * e[9] - e[34] * e[21] * e[12] + e[13] * e[30] * e[24] +
        e[13] * e[21] * e[33] + e[13] * e[31] * e[25] + e[13] * e[22] * e[34] +
        e[13] * e[32] * e[26] + e[13] * e[23] * e[35] + e[10] * e[27] * e[24] +
        e[10] * e[18] * e[33] + e[10] * e[28] * e[25] + e[10] * e[19] * e[34] +
        e[10] * e[29] * e[26] + e[10] * e[20] * e[35] + e[22] * e[30] * e[15] +
        e[22] * e[12] * e[33] + e[22] * e[32] * e[17] + e[22] * e[14] * e[35] +
        e[22] * e[31] * e[16] + e[31] * e[21] * e[15] + e[31] * e[12] * e[24] +
        e[31] * e[23] * e[17] + e[31] * e[14] * e[26] - e[16] * e[27] * e[18] +
        e[16] * e[33] * e[24] - e[16] * e[30] * e[21] - e[16] * e[29] * e[20] +
        e[16] * e[35] * e[26] - e[16] * e[32] * e[23] + e[16] * e[28] * e[19] +
        3. * e[16] * e[34] * e[25] + e[19] * e[27] * e[15] +
        e[19] * e[9] * e[33] + e[19] * e[29] * e[17];
    a[52] = e[4] * e[27] * e[3] + e[4] * e[0] * e[30] + e[4] * e[29] * e[5] +
        e[4] * e[2] * e[32] + e[31] * e[0] * e[3] + e[31] * e[2] * e[5] +
        e[7] * e[27] * e[6] + e[7] * e[0] * e[33] + e[7] * e[29] * e[8] +
        e[7] * e[2] * e[35] + e[34] * e[0] * e[6] + e[34] * e[2] * e[8] +
        e[1] * e[27] * e[0] + e[1] * e[29] * e[2] + e[1] * e[34] * e[7] -
        e[1] * e[32] * e[5] - e[1] * e[33] * e[6] - e[1] * e[30] * e[3] -
        e[1] * e[35] * e[8] + e[1] * e[31] * e[4] + 1.5 * e[28] * e2[1] +
        0.5 * e[28] * e2[4] + 0.5 * e[28] * e2[0] - 0.5 * e[28] * e2[6] -
        0.5 * e[28] * e2[5] + 0.5 * e[28] * e2[7] - 0.5 * e[28] * e2[3] +
        0.5 * e[28] * e2[2] - 0.5 * e[28] * e2[8];
    a[99] = -e[35] * e[10] * e[1] - e[35] * e[13] * e[4] + e[35] * e[16] * e[7] +
        e[35] * e[15] * e[6] - e[35] * e[9] * e[0] - e[35] * e[12] * e[3] +
        e[32] * e[12] * e[6] + e[32] * e[3] * e[15] + e[32] * e[13] * e[7] +
        e[32] * e[4] * e[16] - e[8] * e[27] * e[9] - e[8] * e[30] * e[12] -
        e[8] * e[28] * e[10] - e[8] * e[31] * e[13] + e[8] * e[29] * e[11] +
        e[11] * e[27] * e[6] + e[11] * e[0] * e[33] + e[11] * e[28] * e[7] +
        e[11] * e[1] * e[34] + e[29] * e[9] * e[6] + e[29] * e[0] * e[15] +
        e[29] * e[10] * e[7] + e[29] * e[1] * e[16] + e[5] * e[30] * e[15] +
        e[5] * e[12] * e[33] + e[5] * e[32] * e[17] + e[5] * e[14] * e[35] +
        e[5] * e[31] * e[16] + e[5] * e[13] * e[34] + e[8] * e[33] * e[15] +
        3. * e[8] * e[35] * e[17] + e[8] * e[34] * e[16] +
        e[2] * e[27] * e[15] + e[2] * e[9] * e[33] + e[2] * e[29] * e[17] +
        e[2] * e[11] * e[35] + e[2] * e[28] * e[16] + e[2] * e[10] * e[34] -
        e[17] * e[27] * e[0] + e[17] * e[34] * e[7] + e[17] * e[33] * e[6] -
        e[17] * e[30] * e[3] - e[17] * e[28] * e[1] - e[17] * e[31] * e[4] +
        e[14] * e[30] * e[6] + e[14] * e[3] * e[33] + e[14] * e[31] * e[7] +
        e[14] * e[4] * e[34] + e[14] * e[32] * e[8];
    a[82] = e[19] * e[11] * e[2] + e[4] * e[18] * e[12] + e[4] * e[9] * e[21] +
        e[4] * e[20] * e[14] + e[4] * e[11] * e[23] + e[4] * e[19] * e[13] +
        e[4] * e[10] * e[22] + e[7] * e[18] * e[15] + e[7] * e[9] * e[24] +
        e[7] * e[20] * e[17] + e[7] * e[11] * e[26] + e[7] * e[19] * e[16] +
        e[7] * e[10] * e[25] + e[1] * e[18] * e[9] + e[1] * e[20] * e[11] -
        e[10] * e[21] * e[3] - e[10] * e[26] * e[8] - e[10] * e[23] * e[5] -
        e[10] * e[24] * e[6] + e[13] * e[18] * e[3] + e[13] * e[0] * e[21] +
        e[13] * e[1] * e[22] + e[13] * e[20] * e[5] + e[13] * e[2] * e[23] -
        e[19] * e[15] * e[6] - e[19] * e[14] * e[5] - e[19] * e[12] * e[3] -
        e[19] * e[17] * e[8] + e[22] * e[9] * e[3] + e[22] * e[0] * e[12] +
        e[22] * e[11] * e[5] + e[22] * e[2] * e[14] + e[16] * e[18] * e[6] +
        e[16] * e[0] * e[24] + e[16] * e[1] * e[25] + e[16] * e[20] * e[8] +
        e[16] * e[2] * e[26] - e[1] * e[23] * e[14] - e[1] * e[24] * e[15] -
        e[1] * e[26] * e[17] - e[1] * e[21] * e[12] + e[25] * e[9] * e[6] +
        e[25] * e[0] * e[15] + e[25] * e[11] * e[8] + e[25] * e[2] * e[17] +
        e[10] * e[18] * e[0] + 3. * e[10] * e[19] * e[1] +
        e[10] * e[20] * e[2] + e[19] * e[9] * e[0];
    a[169] = 0.5 * e2[23] * e[26] + 0.5 * e[26] * e2[25] + 0.5 * e2[20] * e[26] -
        0.5 * e[26] * e2[18] + 0.5 * e3[26] + 0.5 * e[26] * e2[24] +
        e[20] * e[19] * e[25] - 0.5 * e[26] * e2[19] - 0.5 * e[26] * e2[21] +
        e[20] * e[18] * e[24] - 0.5 * e[26] * e2[22] +
        e[23] * e[21] * e[24] + e[23] * e[22] * e[25];
    a[72] = e[16] * e[9] * e[33] + e[16] * e[29] * e[17] + e[16] * e[11] * e[35] +
        e[16] * e[10] * e[34] + e[34] * e[11] * e[17] + e[34] * e[9] * e[15] -
        e[10] * e[30] * e[12] - e[10] * e[32] * e[14] -
        e[10] * e[33] * e[15] - e[10] * e[35] * e[17] + e[10] * e[27] * e[9] +
        e[10] * e[29] * e[11] + e[13] * e[27] * e[12] + e[13] * e[9] * e[30] +
        e[13] * e[29] * e[14] + e[13] * e[11] * e[32] +
        e[13] * e[10] * e[31] + e[31] * e[11] * e[14] + e[31] * e[9] * e[12] +
        e[16] * e[27] * e[15] + 1.5 * e[28] * e2[10] + 0.5 * e[28] * e2[16] +
        0.5 * e[28] * e2[9] + 0.5 * e[28] * e2[11] - 0.5 * e[28] * e2[12] -
        0.5 * e[28] * e2[15] - 0.5 * e[28] * e2[17] - 0.5 * e[28] * e2[14] +
        0.5 * e[28] * e2[13];
    a[179] =
        0.5 * e2[20] * e[35] + 0.5 * e2[23] * e[35] + 1.5 * e[35] * e2[26] +
        0.5 * e[35] * e2[25] + 0.5 * e[35] * e2[24] - 0.5 * e[35] * e2[18] -
        0.5 * e[35] * e2[19] - 0.5 * e[35] * e2[22] - 0.5 * e[35] * e2[21] +
        e[20] * e[27] * e[24] + e[20] * e[18] * e[33] + e[20] * e[28] * e[25] +
        e[20] * e[19] * e[34] + e[20] * e[29] * e[26] + e[29] * e[19] * e[25] +
        e[29] * e[18] * e[24] + e[23] * e[30] * e[24] + e[23] * e[21] * e[33] +
        e[23] * e[31] * e[25] + e[23] * e[22] * e[34] + e[23] * e[32] * e[26] +
        e[32] * e[22] * e[25] + e[32] * e[21] * e[24] + e[26] * e[33] * e[24] +
        e[26] * e[34] * e[25] - e[26] * e[27] * e[18] - e[26] * e[30] * e[21] -
        e[26] * e[31] * e[22] - e[26] * e[28] * e[19];
    a[2] = e[4] * e[2] * e[5] + 0.5 * e[1] * e2[0] - 0.5 * e[1] * e2[6] +
        e[7] * e[0] * e[6] + 0.5 * e[1] * e2[7] + 0.5 * e[1] * e2[4] -
        0.5 * e[1] * e2[8] + 0.5 * e[1] * e2[2] - 0.5 * e[1] * e2[3] +
        0.5 * e3[1] + e[7] * e[2] * e[8] - 0.5 * e[1] * e2[5] +
        e[4] * e[0] * e[3];
    a[19] = -0.5 * e[17] * e2[13] - 0.5 * e[17] * e2[9] + 0.5 * e[17] * e2[16] +
        0.5 * e[17] * e2[15] + 0.5 * e3[17] - 0.5 * e[17] * e2[10] +
        e[14] * e[13] * e[16] + e[14] * e[12] * e[15] + 0.5 * e2[14] * e[17] +
        e[11] * e[10] * e[16] - 0.5 * e[17] * e2[12] + 0.5 * e2[11] * e[17] +
        e[11] * e[9] * e[15];
    a[122] = e[4] * e[27] * e[30] + e[4] * e[29] * e[32] + e[4] * e[28] * e[31] +
        e[31] * e[27] * e[3] + e[31] * e[0] * e[30] + e[31] * e[29] * e[5] +
        e[31] * e[2] * e[32] + e[7] * e[27] * e[33] + e[7] * e[29] * e[35] +
        e[7] * e[28] * e[34] + e[28] * e[27] * e[0] + e[28] * e[29] * e[2] +
        e[34] * e[27] * e[6] + e[34] * e[0] * e[33] + e[34] * e[29] * e[8] +
        e[34] * e[2] * e[35] - e[28] * e[32] * e[5] - e[28] * e[33] * e[6] -
        e[28] * e[30] * e[3] - e[28] * e[35] * e[8] + 0.5 * e[1] * e2[27] +
        0.5 * e[1] * e2[29] + 1.5 * e[1] * e2[28] + 0.5 * e[1] * e2[31] -
        0.5 * e[1] * e2[32] - 0.5 * e[1] * e2[33] - 0.5 * e[1] * e2[30] +
        0.5 * e[1] * e2[34] - 0.5 * e[1] * e2[35];
    a[79] = 0.5 * e2[11] * e[35] + 0.5 * e[35] * e2[16] - 0.5 * e[35] * e2[9] -
        0.5 * e[35] * e2[12] + 0.5 * e[35] * e2[15] + 1.5 * e[35] * e2[17] -
        0.5 * e[35] * e2[10] + 0.5 * e[35] * e2[14] - 0.5 * e[35] * e2[13] +
        e[11] * e[27] * e[15] + e[11] * e[9] * e[33] + e[11] * e[29] * e[17] +
        e[11] * e[28] * e[16] + e[11] * e[10] * e[34] + e[29] * e[9] * e[15] +
        e[29] * e[10] * e[16] + e[14] * e[30] * e[15] +
        e[14] * e[12] * e[33] + e[14] * e[32] * e[17] +
        e[14] * e[31] * e[16] + e[14] * e[13] * e[34] +
        e[32] * e[12] * e[15] + e[32] * e[13] * e[16] +
        e[17] * e[33] * e[15] + e[17] * e[34] * e[16] - e[17] * e[27] * e[9] -
        e[17] * e[30] * e[12] - e[17] * e[28] * e[10] - e[17] * e[31] * e[13];
    a[192] = e[34] * e[27] * e[33] + e[34] * e[29] * e[35] -
        0.5 * e[28] * e2[30] - 0.5 * e[28] * e2[35] + 0.5 * e3[28] +
        0.5 * e[28] * e2[27] + 0.5 * e[28] * e2[29] + e[31] * e[27] * e[30] +
        e[31] * e[29] * e[32] - 0.5 * e[28] * e2[32] - 0.5 * e[28] * e2[33] +
        0.5 * e[28] * e2[31] + 0.5 * e[28] * e2[34];
    a[9] = 0.5 * e2[5] * e[8] + e[2] * e[0] * e[6] + 0.5 * e2[2] * e[8] +
        0.5 * e3[8] - 0.5 * e[8] * e2[0] + e[5] * e[4] * e[7] +
        e[5] * e[3] * e[6] + 0.5 * e[8] * e2[7] + e[2] * e[1] * e[7] -
        0.5 * e[8] * e2[1] - 0.5 * e[8] * e2[4] - 0.5 * e[8] * e2[3] +
        0.5 * e[8] * e2[6];
    a[152] =
        e[28] * e[27] * e[9] + e[28] * e[29] * e[11] - e[28] * e[30] * e[12] +
        e[28] * e[31] * e[13] - e[28] * e[32] * e[14] - e[28] * e[33] * e[15] -
        e[28] * e[35] * e[17] + e[31] * e[27] * e[12] + e[31] * e[9] * e[30] +
        e[31] * e[29] * e[14] + e[31] * e[11] * e[32] + e[13] * e[27] * e[30] +
        e[13] * e[29] * e[32] + e[16] * e[27] * e[33] + e[16] * e[29] * e[35] +
        e[34] * e[27] * e[15] + e[34] * e[9] * e[33] + e[34] * e[29] * e[17] +
        e[34] * e[11] * e[35] + e[34] * e[28] * e[16] + 0.5 * e[10] * e2[27] +
        0.5 * e[10] * e2[29] + 1.5 * e[10] * e2[28] - 0.5 * e[10] * e2[32] +
        0.5 * e[10] * e2[31] - 0.5 * e[10] * e2[33] - 0.5 * e[10] * e2[30] +
        0.5 * e[10] * e2[34] - 0.5 * e[10] * e2[35];
    a[59] = -0.5 * e[35] * e2[1] + 0.5 * e[35] * e2[7] - 0.5 * e[35] * e2[3] +
        0.5 * e2[2] * e[35] + 1.5 * e[35] * e2[8] - 0.5 * e[35] * e2[4] -
        0.5 * e[35] * e2[0] + 0.5 * e[35] * e2[6] + 0.5 * e[35] * e2[5] +
        e[2] * e[27] * e[6] + e[2] * e[0] * e[33] + e[2] * e[28] * e[7] +
        e[2] * e[1] * e[34] + e[2] * e[29] * e[8] - e[8] * e[27] * e[0] +
        e[8] * e[34] * e[7] + e[8] * e[32] * e[5] + e[8] * e[33] * e[6] -
        e[8] * e[30] * e[3] - e[8] * e[28] * e[1] - e[8] * e[31] * e[4] +
        e[29] * e[1] * e[7] + e[29] * e[0] * e[6] + e[5] * e[30] * e[6] +
        e[5] * e[3] * e[33] + e[5] * e[31] * e[7] + e[5] * e[4] * e[34] +
        e[32] * e[4] * e[7] + e[32] * e[3] * e[6];
    a[182] =
        e[28] * e[27] * e[18] + e[28] * e[29] * e[20] + e[22] * e[27] * e[30] +
        e[22] * e[29] * e[32] + e[22] * e[28] * e[31] + e[31] * e[27] * e[21] +
        e[31] * e[18] * e[30] + e[31] * e[29] * e[23] + e[31] * e[20] * e[32] +
        e[25] * e[27] * e[33] + e[25] * e[29] * e[35] + e[25] * e[28] * e[34] +
        e[34] * e[27] * e[24] + e[34] * e[18] * e[33] + e[34] * e[29] * e[26] +
        e[34] * e[20] * e[35] - e[28] * e[33] * e[24] - e[28] * e[30] * e[21] -
        e[28] * e[35] * e[26] - e[28] * e[32] * e[23] - 0.5 * e[19] * e2[33] -
        0.5 * e[19] * e2[30] - 0.5 * e[19] * e2[35] + 0.5 * e[19] * e2[27] +
        0.5 * e[19] * e2[29] + 1.5 * e[19] * e2[28] + 0.5 * e[19] * e2[31] +
        0.5 * e[19] * e2[34] - 0.5 * e[19] * e2[32];
    a[89] = e[23] * e[3] * e[15] - e[17] * e[19] * e[1] - e[17] * e[22] * e[4] -
        e[17] * e[18] * e[0] + e[17] * e[25] * e[7] + e[17] * e[24] * e[6] +
        e[14] * e[21] * e[6] + e[14] * e[3] * e[24] + e[14] * e[22] * e[7] +
        e[14] * e[4] * e[25] + e[14] * e[23] * e[8] - e[26] * e[10] * e[1] -
        e[26] * e[13] * e[4] + e[26] * e[16] * e[7] + e[26] * e[15] * e[6] -
        e[26] * e[9] * e[0] - e[26] * e[12] * e[3] + e[23] * e[12] * e[6] +
        e[11] * e[18] * e[6] + e[11] * e[0] * e[24] + e[11] * e[19] * e[7] +
        e[11] * e[1] * e[25] + e[11] * e[20] * e[8] + e[11] * e[2] * e[26] +
        e[20] * e[9] * e[6] + e[20] * e[0] * e[15] + e[20] * e[10] * e[7] +
        e[20] * e[1] * e[16] + e[20] * e[2] * e[17] + e[5] * e[21] * e[15] +
        e[5] * e[12] * e[24] + e[5] * e[23] * e[17] + e[5] * e[14] * e[26] +
        e[5] * e[22] * e[16] + e[5] * e[13] * e[25] + e[8] * e[24] * e[15] +
        3. * e[8] * e[26] * e[17] + e[8] * e[25] * e[16] +
        e[2] * e[18] * e[15] + e[2] * e[9] * e[24] + e[2] * e[19] * e[16] +
        e[2] * e[10] * e[25] - e[17] * e[21] * e[3] + e[23] * e[4] * e[16] +
        e[23] * e[13] * e[7] - e[8] * e[18] * e[9] - e[8] * e[21] * e[12] -
        e[8] * e[19] * e[10] - e[8] * e[22] * e[13];
    a[62] = e[13] * e[18] * e[12] + e[13] * e[9] * e[21] + e[13] * e[20] * e[14] +
        e[13] * e[11] * e[23] + e[13] * e[10] * e[22] +
        e[22] * e[11] * e[14] + e[22] * e[9] * e[12] + e[16] * e[18] * e[15] +
        e[16] * e[9] * e[24] + e[16] * e[20] * e[17] + e[16] * e[11] * e[26] +
        e[16] * e[10] * e[25] + e[25] * e[11] * e[17] + e[25] * e[9] * e[15] -
        e[10] * e[23] * e[14] - e[10] * e[24] * e[15] -
        e[10] * e[26] * e[17] + e[10] * e[20] * e[11] + e[10] * e[18] * e[9] -
        e[10] * e[21] * e[12] + 0.5 * e[19] * e2[11] + 0.5 * e[19] * e2[9] +
        1.5 * e[19] * e2[10] + 0.5 * e[19] * e2[13] + 0.5 * e[19] * e2[16] -
        0.5 * e[19] * e2[12] - 0.5 * e[19] * e2[15] - 0.5 * e[19] * e2[17] -
        0.5 * e[19] * e2[14];
    a[88] = e[10] * e[18] * e[6] + e[10] * e[0] * e[24] + e[10] * e[19] * e[7] +
        e[10] * e[1] * e[25] + e[10] * e[20] * e[8] + e[10] * e[2] * e[26] +
        e[19] * e[9] * e[6] + e[19] * e[0] * e[15] + e[19] * e[1] * e[16] +
        e[19] * e[11] * e[8] + e[19] * e[2] * e[17] + e[4] * e[21] * e[15] +
        e[4] * e[12] * e[24] + e[4] * e[23] * e[17] + e[4] * e[14] * e[26] +
        e[4] * e[22] * e[16] + e[4] * e[13] * e[25] + e[7] * e[24] * e[15] +
        e[7] * e[26] * e[17] + 3. * e[7] * e[25] * e[16] +
        e[1] * e[18] * e[15] + e[1] * e[9] * e[24] + e[1] * e[20] * e[17] +
        e[1] * e[11] * e[26] - e[16] * e[21] * e[3] + e[16] * e[26] * e[8] -
        e[16] * e[20] * e[2] - e[16] * e[18] * e[0] - e[16] * e[23] * e[5] +
        e[16] * e[24] * e[6] + e[13] * e[21] * e[6] + e[13] * e[3] * e[24] +
        e[13] * e[22] * e[7] + e[13] * e[23] * e[8] + e[13] * e[5] * e[26] -
        e[25] * e[11] * e[2] + e[25] * e[15] * e[6] - e[25] * e[9] * e[0] -
        e[25] * e[14] * e[5] - e[25] * e[12] * e[3] + e[25] * e[17] * e[8] +
        e[22] * e[12] * e[6] + e[22] * e[3] * e[15] + e[22] * e[14] * e[8] +
        e[22] * e[5] * e[17] - e[7] * e[23] * e[14] - e[7] * e[20] * e[11] -
        e[7] * e[18] * e[9] - e[7] * e[21] * e[12];
    a[32] = e[13] * e[9] * e[3] + e[13] * e[0] * e[12] + e[13] * e[10] * e[4] +
        e[13] * e[11] * e[5] + e[13] * e[2] * e[14] + e[16] * e[9] * e[6] +
        e[16] * e[0] * e[15] + e[16] * e[10] * e[7] + e[16] * e[11] * e[8] +
        e[16] * e[2] * e[17] + e[7] * e[11] * e[17] + e[7] * e[9] * e[15] +
        e[4] * e[11] * e[14] + e[4] * e[9] * e[12] + e[10] * e[9] * e[0] +
        e[10] * e[11] * e[2] - e[10] * e[15] * e[6] - e[10] * e[14] * e[5] -
        e[10] * e[12] * e[3] - e[10] * e[17] * e[8] + 0.5 * e[1] * e2[11] +
        0.5 * e[1] * e2[9] + 1.5 * e[1] * e2[10] - 0.5 * e[1] * e2[12] -
        0.5 * e[1] * e2[15] - 0.5 * e[1] * e2[17] - 0.5 * e[1] * e2[14] +
        0.5 * e[1] * e2[13] + 0.5 * e[1] * e2[16];
    a[58] = e[1] * e[27] * e[6] + e[1] * e[0] * e[33] + e[1] * e[28] * e[7] +
        e[1] * e[29] * e[8] + e[1] * e[2] * e[35] - e[7] * e[27] * e[0] -
        e[7] * e[32] * e[5] + e[7] * e[33] * e[6] - e[7] * e[30] * e[3] +
        e[7] * e[35] * e[8] - e[7] * e[29] * e[2] + e[7] * e[31] * e[4] +
        e[28] * e[0] * e[6] + e[28] * e[2] * e[8] + e[4] * e[30] * e[6] +
        e[4] * e[3] * e[33] + e[4] * e[32] * e[8] + e[4] * e[5] * e[35] +
        e[31] * e[3] * e[6] + e[31] * e[5] * e[8] + 0.5 * e2[1] * e[34] +
        1.5 * e[34] * e2[7] + 0.5 * e[34] * e2[4] - 0.5 * e[34] * e2[0] +
        0.5 * e[34] * e2[6] - 0.5 * e[34] * e2[5] - 0.5 * e[34] * e2[3] -
        0.5 * e[34] * e2[2] + 0.5 * e[34] * e2[8];
    a[42] = e[4] * e[18] * e[3] + e[4] * e[0] * e[21] + e[4] * e[1] * e[22] +
        e[4] * e[20] * e[5] + e[4] * e[2] * e[23] + e[22] * e[0] * e[3] +
        e[22] * e[2] * e[5] + e[7] * e[18] * e[6] + e[7] * e[0] * e[24] +
        e[7] * e[1] * e[25] + e[7] * e[20] * e[8] + e[7] * e[2] * e[26] +
        e[25] * e[0] * e[6] + e[25] * e[2] * e[8] + e[1] * e[18] * e[0] +
        e[1] * e[20] * e[2] - e[1] * e[21] * e[3] - e[1] * e[26] * e[8] -
        e[1] * e[23] * e[5] - e[1] * e[24] * e[6] + 0.5 * e[19] * e2[4] +
        0.5 * e[19] * e2[0] - 0.5 * e[19] * e2[6] - 0.5 * e[19] * e2[5] +
        1.5 * e[19] * e2[1] + 0.5 * e[19] * e2[7] - 0.5 * e[19] * e2[3] +
        0.5 * e[19] * e2[2] - 0.5 * e[19] * e2[8];
    a[8] = -0.5 * e[7] * e2[0] + e[4] * e[5] * e[8] + 0.5 * e2[4] * e[7] -
        0.5 * e[7] * e2[2] + 0.5 * e[7] * e2[8] - 0.5 * e[7] * e2[5] +
        0.5 * e[7] * e2[6] + e[1] * e[0] * e[6] + 0.5 * e3[7] +
        e[4] * e[3] * e[6] + e[1] * e[2] * e[8] - 0.5 * e[7] * e2[3] +
        0.5 * e2[1] * e[7];
    a[112] = -e[1] * e[32] * e[23] - e[19] * e[32] * e[5] - e[19] * e[33] * e[6] -
        e[19] * e[30] * e[3] - e[19] * e[35] * e[8] - e[28] * e[21] * e[3] -
        e[28] * e[26] * e[8] - e[28] * e[23] * e[5] - e[28] * e[24] * e[6] +
        e[7] * e[27] * e[24] + e[7] * e[18] * e[33] + e[7] * e[29] * e[26] +
        e[7] * e[20] * e[35] + e[22] * e[27] * e[3] + e[22] * e[0] * e[30] +
        e[22] * e[29] * e[5] + e[22] * e[2] * e[32] + e[31] * e[18] * e[3] +
        e[31] * e[0] * e[21] + e[31] * e[20] * e[5] + e[31] * e[2] * e[23] +
        e[25] * e[27] * e[6] + e[25] * e[0] * e[33] + e[25] * e[28] * e[7] +
        e[25] * e[1] * e[34] + e[25] * e[29] * e[8] + e[25] * e[2] * e[35] +
        e[34] * e[18] * e[6] + e[34] * e[0] * e[24] + e[34] * e[19] * e[7] +
        e[34] * e[20] * e[8] + e[34] * e[2] * e[26] + e[1] * e[27] * e[18] +
        3. * e[1] * e[28] * e[19] + e[1] * e[29] * e[20] +
        e[19] * e[27] * e[0] + e[19] * e[29] * e[2] + e[28] * e[18] * e[0] +
        e[28] * e[20] * e[2] + e[4] * e[27] * e[21] + e[4] * e[18] * e[30] +
        e[4] * e[28] * e[22] + e[4] * e[19] * e[31] + e[4] * e[29] * e[23] +
        e[4] * e[20] * e[32] - e[1] * e[33] * e[24] - e[1] * e[30] * e[21] -
        e[1] * e[35] * e[26] + e[1] * e[31] * e[22];
    a[78] =
        e[10] * e[27] * e[15] + e[10] * e[9] * e[33] + e[10] * e[29] * e[17] +
        e[10] * e[11] * e[35] + e[10] * e[28] * e[16] + e[28] * e[11] * e[17] +
        e[28] * e[9] * e[15] + e[13] * e[30] * e[15] + e[13] * e[12] * e[33] +
        e[13] * e[32] * e[17] + e[13] * e[14] * e[35] + e[13] * e[31] * e[16] +
        e[31] * e[14] * e[17] + e[31] * e[12] * e[15] + e[16] * e[33] * e[15] +
        e[16] * e[35] * e[17] - e[16] * e[27] * e[9] - e[16] * e[30] * e[12] -
        e[16] * e[32] * e[14] - e[16] * e[29] * e[11] + 0.5 * e2[10] * e[34] +
        1.5 * e[34] * e2[16] - 0.5 * e[34] * e2[9] - 0.5 * e[34] * e2[11] -
        0.5 * e[34] * e2[12] + 0.5 * e[34] * e2[15] + 0.5 * e[34] * e2[17] -
        0.5 * e[34] * e2[14] + 0.5 * e[34] * e2[13];
    a[162] = 0.5 * e[19] * e2[18] + 0.5 * e[19] * e2[25] + 0.5 * e[19] * e2[22] +
        e[25] * e[20] * e[26] - 0.5 * e[19] * e2[21] + 0.5 * e[19] * e2[20] -
        0.5 * e[19] * e2[26] - 0.5 * e[19] * e2[23] - 0.5 * e[19] * e2[24] +
        0.5 * e3[19] + e[22] * e[20] * e[23] + e[25] * e[18] * e[24] +
        e[22] * e[18] * e[21];
    a[198] = 0.5 * e[34] * e2[33] + 0.5 * e[34] * e2[35] - 0.5 * e[34] * e2[27] -
        0.5 * e[34] * e2[32] - 0.5 * e[34] * e2[29] - 0.5 * e[34] * e2[30] +
        0.5 * e2[28] * e[34] + e[31] * e[30] * e[33] +
        e[31] * e[32] * e[35] + e[28] * e[27] * e[33] + 0.5 * e3[34] +
        e[28] * e[29] * e[35] + 0.5 * e2[31] * e[34];
    a[92] = e[4] * e[28] * e[13] + e[4] * e[10] * e[31] + e[7] * e[27] * e[15] +
        e[7] * e[9] * e[33] + e[7] * e[29] * e[17] + e[7] * e[11] * e[35] +
        e[7] * e[28] * e[16] + e[7] * e[10] * e[34] + e[1] * e[27] * e[9] +
        e[1] * e[29] * e[11] + 3. * e[1] * e[28] * e[10] +
        e[10] * e[27] * e[0] - e[10] * e[32] * e[5] - e[10] * e[33] * e[6] -
        e[10] * e[30] * e[3] - e[10] * e[35] * e[8] + e[10] * e[29] * e[2] +
        e[13] * e[27] * e[3] + e[13] * e[0] * e[30] + e[13] * e[1] * e[31] +
        e[13] * e[29] * e[5] + e[13] * e[2] * e[32] + e[28] * e[11] * e[2] -
        e[28] * e[15] * e[6] + e[28] * e[9] * e[0] - e[28] * e[14] * e[5] -
        e[28] * e[12] * e[3] - e[28] * e[17] * e[8] + e[31] * e[9] * e[3] +
        e[31] * e[0] * e[12] + e[31] * e[11] * e[5] + e[31] * e[2] * e[14] +
        e[16] * e[27] * e[6] + e[16] * e[0] * e[33] + e[16] * e[1] * e[34] +
        e[16] * e[29] * e[8] + e[16] * e[2] * e[35] - e[1] * e[30] * e[12] -
        e[1] * e[32] * e[14] - e[1] * e[33] * e[15] - e[1] * e[35] * e[17] +
        e[34] * e[9] * e[6] + e[34] * e[0] * e[15] + e[34] * e[11] * e[8] +
        e[34] * e[2] * e[17] + e[4] * e[27] * e[12] + e[4] * e[9] * e[30] +
        e[4] * e[29] * e[14] + e[4] * e[11] * e[32];
    a[128] = e[4] * e[30] * e[33] + e[4] * e[32] * e[35] + e[4] * e[31] * e[34] +
        e[31] * e[30] * e[6] + e[31] * e[3] * e[33] + e[31] * e[32] * e[8] +
        e[31] * e[5] * e[35] + e[28] * e[27] * e[6] + e[28] * e[0] * e[33] +
        e[28] * e[29] * e[8] + e[28] * e[2] * e[35] + e[34] * e[33] * e[6] +
        e[34] * e[35] * e[8] - e[34] * e[27] * e[0] - e[34] * e[32] * e[5] -
        e[34] * e[30] * e[3] - e[34] * e[29] * e[2] + e[1] * e[27] * e[33] +
        e[1] * e[29] * e[35] + e[1] * e[28] * e[34] + 0.5 * e2[31] * e[7] -
        0.5 * e[7] * e2[27] - 0.5 * e[7] * e2[32] + 0.5 * e[7] * e2[28] -
        0.5 * e[7] * e2[29] + 0.5 * e[7] * e2[33] - 0.5 * e[7] * e2[30] +
        1.5 * e[7] * e2[34] + 0.5 * e[7] * e2[35];
    a[12] = -0.5 * e[10] * e2[14] - 0.5 * e[10] * e2[17] - 0.5 * e[10] * e2[15] +
        e[13] * e[11] * e[14] + e[16] * e[11] * e[17] + 0.5 * e[10] * e2[13] +
        e[13] * e[9] * e[12] - 0.5 * e[10] * e2[12] + 0.5 * e3[10] +
        e[16] * e[9] * e[15] + 0.5 * e[10] * e2[16] + 0.5 * e[10] * e2[11] +
        0.5 * e[10] * e2[9];
    a[188] =
        e[22] * e[32] * e[35] + e[22] * e[31] * e[34] + e[31] * e[30] * e[24] +
        e[31] * e[21] * e[33] + e[31] * e[32] * e[26] + e[31] * e[23] * e[35] +
        e[34] * e[33] * e[24] + e[34] * e[35] * e[26] - e[34] * e[27] * e[18] -
        e[34] * e[30] * e[21] - e[34] * e[29] * e[20] - e[34] * e[32] * e[23] +
        e[19] * e[27] * e[33] + e[19] * e[29] * e[35] + e[19] * e[28] * e[34] +
        e[28] * e[27] * e[24] + e[28] * e[18] * e[33] + e[28] * e[29] * e[26] +
        e[28] * e[20] * e[35] + e[22] * e[30] * e[33] + 0.5 * e2[28] * e[25] +
        0.5 * e2[31] * e[25] + 0.5 * e[25] * e2[33] + 0.5 * e[25] * e2[35] +
        1.5 * e[25] * e2[34] - 0.5 * e[25] * e2[27] - 0.5 * e[25] * e2[32] -
        0.5 * e[25] * e2[29] - 0.5 * e[25] * e2[30];
    a[172] =
        -e[19] * e[35] * e[26] - e[19] * e[32] * e[23] + e[19] * e[27] * e[18] +
        e[19] * e[29] * e[20] + e[22] * e[27] * e[21] + e[22] * e[18] * e[30] +
        e[22] * e[19] * e[31] + e[22] * e[29] * e[23] + e[22] * e[20] * e[32] +
        e[31] * e[18] * e[21] + e[31] * e[20] * e[23] + e[25] * e[27] * e[24] +
        e[25] * e[18] * e[33] + e[25] * e[19] * e[34] + e[25] * e[29] * e[26] +
        e[25] * e[20] * e[35] + e[34] * e[18] * e[24] + e[34] * e[20] * e[26] -
        e[19] * e[33] * e[24] - e[19] * e[30] * e[21] + 1.5 * e[28] * e2[19] +
        0.5 * e[28] * e2[18] + 0.5 * e[28] * e2[20] + 0.5 * e[28] * e2[22] +
        0.5 * e[28] * e2[25] - 0.5 * e[28] * e2[26] - 0.5 * e[28] * e2[23] -
        0.5 * e[28] * e2[24] - 0.5 * e[28] * e2[21];
    a[158] =
        e[10] * e[27] * e[33] + e[10] * e[29] * e[35] + e[10] * e[28] * e[34] +
        e[34] * e[33] * e[15] + e[34] * e[35] * e[17] + e[28] * e[27] * e[15] +
        e[28] * e[9] * e[33] + e[28] * e[29] * e[17] + e[28] * e[11] * e[35] -
        e[34] * e[27] * e[9] - e[34] * e[30] * e[12] + e[34] * e[31] * e[13] -
        e[34] * e[32] * e[14] - e[34] * e[29] * e[11] + e[31] * e[30] * e[15] +
        e[31] * e[12] * e[33] + e[31] * e[32] * e[17] + e[31] * e[14] * e[35] +
        e[13] * e[30] * e[33] + e[13] * e[32] * e[35] - 0.5 * e[16] * e2[27] -
        0.5 * e[16] * e2[32] + 0.5 * e[16] * e2[28] - 0.5 * e[16] * e2[29] +
        0.5 * e[16] * e2[31] + 0.5 * e[16] * e2[33] - 0.5 * e[16] * e2[30] +
        1.5 * e[16] * e2[34] + 0.5 * e[16] * e2[35];
    a[153] =
        e[29] * e[32] * e[14] - e[29] * e[33] * e[15] - e[29] * e[34] * e[16] +
        e[32] * e[27] * e[12] + e[32] * e[9] * e[30] + e[32] * e[28] * e[13] +
        e[32] * e[10] * e[31] + e[14] * e[27] * e[30] + e[14] * e[28] * e[31] +
        e[17] * e[27] * e[33] + e[17] * e[28] * e[34] + e[35] * e[27] * e[15] +
        e[35] * e[9] * e[33] + e[35] * e[29] * e[17] + e[35] * e[28] * e[16] +
        e[35] * e[10] * e[34] + e[29] * e[27] * e[9] + e[29] * e[28] * e[10] -
        e[29] * e[30] * e[12] - e[29] * e[31] * e[13] + 0.5 * e[11] * e2[27] +
        1.5 * e[11] * e2[29] + 0.5 * e[11] * e2[28] + 0.5 * e[11] * e2[32] -
        0.5 * e[11] * e2[31] - 0.5 * e[11] * e2[33] - 0.5 * e[11] * e2[30] -
        0.5 * e[11] * e2[34] + 0.5 * e[11] * e2[35];
    a[118] = e[1] * e[20] * e[35] + e[19] * e[27] * e[6] + e[19] * e[0] * e[33] +
        e[19] * e[28] * e[7] + e[19] * e[29] * e[8] + e[19] * e[2] * e[35] +
        e[28] * e[18] * e[6] + e[28] * e[0] * e[24] + e[28] * e[20] * e[8] +
        e[28] * e[2] * e[26] + e[4] * e[30] * e[24] + e[4] * e[21] * e[33] +
        e[4] * e[31] * e[25] + e[4] * e[22] * e[34] + e[4] * e[32] * e[26] +
        e[4] * e[23] * e[35] - e[7] * e[27] * e[18] + e[7] * e[33] * e[24] -
        e[7] * e[30] * e[21] - e[7] * e[29] * e[20] + e[7] * e[35] * e[26] +
        e[7] * e[31] * e[22] - e[7] * e[32] * e[23] - e[25] * e[27] * e[0] -
        e[25] * e[32] * e[5] - e[25] * e[30] * e[3] - e[25] * e[29] * e[2] -
        e[34] * e[21] * e[3] - e[34] * e[20] * e[2] - e[34] * e[18] * e[0] -
        e[34] * e[23] * e[5] + e[22] * e[30] * e[6] + e[22] * e[3] * e[33] +
        e[22] * e[32] * e[8] + e[22] * e[5] * e[35] + e[31] * e[21] * e[6] +
        e[31] * e[3] * e[24] + e[31] * e[23] * e[8] + e[31] * e[5] * e[26] +
        e[34] * e[26] * e[8] + e[1] * e[27] * e[24] + e[1] * e[18] * e[33] +
        e[1] * e[28] * e[25] + e[1] * e[19] * e[34] + e[1] * e[29] * e[26] +
        e[34] * e[24] * e[6] + e[25] * e[33] * e[6] +
        3. * e[25] * e[34] * e[7] + e[25] * e[35] * e[8];
    a[183] =
        0.5 * e[20] * e2[27] + 1.5 * e[20] * e2[29] + 0.5 * e[20] * e2[28] +
        0.5 * e[20] * e2[32] + 0.5 * e[20] * e2[35] - 0.5 * e[20] * e2[31] -
        0.5 * e[20] * e2[33] - 0.5 * e[20] * e2[30] - 0.5 * e[20] * e2[34] +
        e[29] * e[27] * e[18] + e[29] * e[28] * e[19] + e[23] * e[27] * e[30] +
        e[23] * e[29] * e[32] + e[23] * e[28] * e[31] + e[32] * e[27] * e[21] +
        e[32] * e[18] * e[30] + e[32] * e[28] * e[22] + e[32] * e[19] * e[31] +
        e[26] * e[27] * e[33] + e[26] * e[29] * e[35] + e[26] * e[28] * e[34] +
        e[35] * e[27] * e[24] + e[35] * e[18] * e[33] + e[35] * e[28] * e[25] +
        e[35] * e[19] * e[34] - e[29] * e[33] * e[24] - e[29] * e[30] * e[21] -
        e[29] * e[31] * e[22] - e[29] * e[34] * e[25];
    a[48] = e[19] * e[1] * e[7] + e[19] * e[0] * e[6] + e[19] * e[2] * e[8] +
        e[4] * e[21] * e[6] + e[4] * e[3] * e[24] + e[4] * e[22] * e[7] +
        e[4] * e[23] * e[8] + e[4] * e[5] * e[26] + e[22] * e[3] * e[6] +
        e[22] * e[5] * e[8] + e[7] * e[24] * e[6] + e[7] * e[26] * e[8] +
        e[1] * e[18] * e[6] + e[1] * e[0] * e[24] + e[1] * e[20] * e[8] +
        e[1] * e[2] * e[26] - e[7] * e[21] * e[3] - e[7] * e[20] * e[2] -
        e[7] * e[18] * e[0] - e[7] * e[23] * e[5] + 0.5 * e[25] * e2[4] -
        0.5 * e[25] * e2[0] + 0.5 * e[25] * e2[6] - 0.5 * e[25] * e2[5] +
        0.5 * e[25] * e2[1] + 1.5 * e[25] * e2[7] - 0.5 * e[25] * e2[3] -
        0.5 * e[25] * e2[2] + 0.5 * e[25] * e2[8];
    a[123] = e[5] * e[27] * e[30] + e[5] * e[29] * e[32] + e[5] * e[28] * e[31] +
        e[32] * e[27] * e[3] + e[32] * e[0] * e[30] + e[32] * e[28] * e[4] +
        e[32] * e[1] * e[31] + e[8] * e[27] * e[33] + e[8] * e[29] * e[35] +
        e[8] * e[28] * e[34] + e[29] * e[27] * e[0] + e[29] * e[28] * e[1] +
        e[35] * e[27] * e[6] + e[35] * e[0] * e[33] + e[35] * e[28] * e[7] +
        e[35] * e[1] * e[34] - e[29] * e[34] * e[7] - e[29] * e[33] * e[6] -
        e[29] * e[30] * e[3] - e[29] * e[31] * e[4] + 0.5 * e[2] * e2[27] +
        1.5 * e[2] * e2[29] + 0.5 * e[2] * e2[28] + 0.5 * e[2] * e2[32] -
        0.5 * e[2] * e2[31] - 0.5 * e[2] * e2[33] - 0.5 * e[2] * e2[30] -
        0.5 * e[2] * e2[34] + 0.5 * e[2] * e2[35];
    a[38] = e[13] * e[12] * e[6] + e[13] * e[3] * e[15] + e[13] * e[4] * e[16] +
        e[13] * e[14] * e[8] + e[13] * e[5] * e[17] + e[16] * e[15] * e[6] +
        e[16] * e[17] * e[8] + e[1] * e[11] * e[17] + e[1] * e[9] * e[15] +
        e[1] * e[10] * e[16] + e[4] * e[14] * e[17] + e[4] * e[12] * e[15] +
        e[10] * e[9] * e[6] + e[10] * e[0] * e[15] + e[10] * e[11] * e[8] +
        e[10] * e[2] * e[17] - e[16] * e[11] * e[2] - e[16] * e[9] * e[0] -
        e[16] * e[14] * e[5] - e[16] * e[12] * e[3] + 0.5 * e2[13] * e[7] +
        1.5 * e2[16] * e[7] + 0.5 * e[7] * e2[17] + 0.5 * e[7] * e2[15] -
        0.5 * e[7] * e2[9] - 0.5 * e[7] * e2[11] - 0.5 * e[7] * e2[12] +
        0.5 * e[7] * e2[10] - 0.5 * e[7] * e2[14];
    a[193] = 0.5 * e[29] * e2[32] + 0.5 * e[29] * e2[35] - 0.5 * e[29] * e2[31] -
        0.5 * e[29] * e2[33] - 0.5 * e[29] * e2[30] - 0.5 * e[29] * e2[34] +
        e[32] * e[27] * e[30] + 0.5 * e3[29] + 0.5 * e[29] * e2[28] +
        e[35] * e[28] * e[34] + 0.5 * e[29] * e2[27] +
        e[35] * e[27] * e[33] + e[32] * e[28] * e[31];
    a[68] =
        -e[16] * e[21] * e[12] + e[10] * e[18] * e[15] + e[10] * e[9] * e[24] +
        e[10] * e[20] * e[17] + e[10] * e[11] * e[26] + e[19] * e[11] * e[17] +
        e[19] * e[9] * e[15] + e[19] * e[10] * e[16] + e[13] * e[21] * e[15] +
        e[13] * e[12] * e[24] + e[13] * e[23] * e[17] + e[13] * e[14] * e[26] +
        e[13] * e[22] * e[16] + e[22] * e[14] * e[17] + e[22] * e[12] * e[15] +
        e[16] * e[24] * e[15] + e[16] * e[26] * e[17] - e[16] * e[23] * e[14] -
        e[16] * e[20] * e[11] - e[16] * e[18] * e[9] + 0.5 * e2[13] * e[25] +
        1.5 * e[25] * e2[16] + 0.5 * e[25] * e2[17] + 0.5 * e[25] * e2[15] +
        0.5 * e2[10] * e[25] - 0.5 * e[25] * e2[9] - 0.5 * e[25] * e2[11] -
        0.5 * e[25] * e2[12] - 0.5 * e[25] * e2[14];
    a[102] = e[19] * e[20] * e[2] + e[22] * e[18] * e[3] + e[22] * e[0] * e[21] +
        e[22] * e[19] * e[4] + e[22] * e[20] * e[5] + e[22] * e[2] * e[23] -
        e[19] * e[21] * e[3] - e[19] * e[26] * e[8] + e[19] * e[25] * e[7] -
        e[19] * e[23] * e[5] - e[19] * e[24] * e[6] + e[4] * e[18] * e[21] +
        e[4] * e[20] * e[23] + e[25] * e[18] * e[6] + e[25] * e[0] * e[24] +
        e[25] * e[20] * e[8] + e[25] * e[2] * e[26] + e[7] * e[18] * e[24] +
        e[7] * e[20] * e[26] + e[19] * e[18] * e[0] + 1.5 * e2[19] * e[1] +
        0.5 * e[1] * e2[22] + 0.5 * e[1] * e2[18] + 0.5 * e[1] * e2[20] +
        0.5 * e[1] * e2[25] - 0.5 * e[1] * e2[26] - 0.5 * e[1] * e2[23] -
        0.5 * e[1] * e2[24] - 0.5 * e[1] * e2[21];
    a[178] =
        e[19] * e[27] * e[24] + e[19] * e[18] * e[33] + e[19] * e[28] * e[25] +
        e[19] * e[29] * e[26] + e[19] * e[20] * e[35] + e[28] * e[18] * e[24] +
        e[28] * e[20] * e[26] + e[22] * e[30] * e[24] + e[22] * e[21] * e[33] +
        e[22] * e[31] * e[25] + e[22] * e[32] * e[26] + e[22] * e[23] * e[35] +
        e[31] * e[21] * e[24] + e[31] * e[23] * e[26] + e[25] * e[33] * e[24] +
        e[25] * e[35] * e[26] - e[25] * e[27] * e[18] - e[25] * e[30] * e[21] -
        e[25] * e[29] * e[20] - e[25] * e[32] * e[23] - 0.5 * e[34] * e2[18] -
        0.5 * e[34] * e2[23] - 0.5 * e[34] * e2[20] - 0.5 * e[34] * e2[21] +
        0.5 * e2[19] * e[34] + 0.5 * e2[22] * e[34] + 1.5 * e[34] * e2[25] +
        0.5 * e[34] * e2[24] + 0.5 * e[34] * e2[26];
    a[22] = e[16] * e[0] * e[6] + e[16] * e[2] * e[8] + e[1] * e[11] * e[2] -
        e[1] * e[15] * e[6] + e[1] * e[9] * e[0] - e[1] * e[14] * e[5] -
        e[1] * e[12] * e[3] - e[1] * e[17] * e[8] + e[4] * e[9] * e[3] +
        e[4] * e[0] * e[12] + e[4] * e[1] * e[13] + e[4] * e[11] * e[5] +
        e[4] * e[2] * e[14] + e[13] * e[0] * e[3] + e[13] * e[2] * e[5] +
        e[7] * e[9] * e[6] + e[7] * e[0] * e[15] + e[7] * e[1] * e[16] +
        e[7] * e[11] * e[8] + e[7] * e[2] * e[17] - 0.5 * e[10] * e2[6] -
        0.5 * e[10] * e2[5] - 0.5 * e[10] * e2[3] - 0.5 * e[10] * e2[8] +
        1.5 * e[10] * e2[1] + 0.5 * e[10] * e2[0] + 0.5 * e[10] * e2[2] +
        0.5 * e[10] * e2[4] + 0.5 * e[10] * e2[7];
    a[18] = e[13] * e[14] * e[17] + e[13] * e[12] * e[15] + e[10] * e[9] * e[15] +
        0.5 * e[16] * e2[15] - 0.5 * e[16] * e2[11] - 0.5 * e[16] * e2[12] -
        0.5 * e[16] * e2[14] + e[10] * e[11] * e[17] + 0.5 * e2[10] * e[16] +
        0.5 * e3[16] - 0.5 * e[16] * e2[9] + 0.5 * e[16] * e2[17] +
        0.5 * e2[13] * e[16];
    a[142] =
        e[10] * e[29] * e[20] + e[22] * e[27] * e[12] + e[22] * e[9] * e[30] +
        e[22] * e[29] * e[14] + e[22] * e[11] * e[32] + e[22] * e[10] * e[31] +
        e[31] * e[18] * e[12] + e[31] * e[9] * e[21] + e[31] * e[20] * e[14] +
        e[31] * e[11] * e[23] - e[10] * e[33] * e[24] - e[10] * e[30] * e[21] -
        e[10] * e[35] * e[26] - e[10] * e[32] * e[23] + e[10] * e[34] * e[25] +
        e[19] * e[27] * e[9] + e[19] * e[29] * e[11] + e[28] * e[18] * e[9] +
        e[28] * e[20] * e[11] + e[16] * e[27] * e[24] + e[16] * e[18] * e[33] +
        e[16] * e[28] * e[25] + e[16] * e[19] * e[34] + e[16] * e[29] * e[26] +
        e[16] * e[20] * e[35] - e[19] * e[30] * e[12] - e[19] * e[32] * e[14] -
        e[19] * e[33] * e[15] - e[19] * e[35] * e[17] - e[28] * e[23] * e[14] -
        e[28] * e[24] * e[15] - e[28] * e[26] * e[17] - e[28] * e[21] * e[12] +
        e[25] * e[27] * e[15] + e[25] * e[9] * e[33] + e[25] * e[29] * e[17] +
        e[25] * e[11] * e[35] + e[34] * e[18] * e[15] + e[34] * e[9] * e[24] +
        e[34] * e[20] * e[17] + e[34] * e[11] * e[26] + e[13] * e[27] * e[21] +
        e[13] * e[18] * e[30] + e[13] * e[28] * e[22] + e[13] * e[19] * e[31] +
        e[13] * e[29] * e[23] + e[13] * e[20] * e[32] + e[10] * e[27] * e[18] +
        3. * e[10] * e[28] * e[19];
    a[98] = e[4] * e[30] * e[15] + e[4] * e[12] * e[33] + e[4] * e[32] * e[17] +
        e[4] * e[14] * e[35] + e[4] * e[31] * e[16] + e[4] * e[13] * e[34] +
        e[7] * e[33] * e[15] + e[7] * e[35] * e[17] +
        3. * e[7] * e[34] * e[16] + e[1] * e[27] * e[15] +
        e[1] * e[9] * e[33] + e[1] * e[29] * e[17] + e[1] * e[11] * e[35] +
        e[1] * e[28] * e[16] + e[1] * e[10] * e[34] - e[16] * e[27] * e[0] -
        e[16] * e[32] * e[5] + e[16] * e[33] * e[6] - e[16] * e[30] * e[3] +
        e[16] * e[35] * e[8] - e[16] * e[29] * e[2] + e[13] * e[30] * e[6] +
        e[13] * e[3] * e[33] + e[13] * e[31] * e[7] + e[13] * e[32] * e[8] +
        e[13] * e[5] * e[35] - e[34] * e[11] * e[2] + e[34] * e[15] * e[6] -
        e[34] * e[9] * e[0] - e[34] * e[14] * e[5] - e[34] * e[12] * e[3] +
        e[34] * e[17] * e[8] + e[31] * e[12] * e[6] + e[31] * e[3] * e[15] +
        e[31] * e[14] * e[8] + e[31] * e[5] * e[17] - e[7] * e[27] * e[9] -
        e[7] * e[30] * e[12] + e[7] * e[28] * e[10] - e[7] * e[32] * e[14] +
        e[10] * e[27] * e[6] + e[10] * e[0] * e[33] + e[10] * e[29] * e[8] +
        e[10] * e[2] * e[35] + e[28] * e[9] * e[6] + e[28] * e[0] * e[15] +
        e[28] * e[11] * e[8] + e[28] * e[2] * e[17] - e[7] * e[29] * e[11];
    a[132] =
        e[22] * e[18] * e[12] + e[22] * e[9] * e[21] + e[22] * e[20] * e[14] +
        e[22] * e[11] * e[23] + e[22] * e[19] * e[13] + e[25] * e[18] * e[15] +
        e[25] * e[9] * e[24] + e[25] * e[20] * e[17] + e[25] * e[11] * e[26] +
        e[25] * e[19] * e[16] + e[16] * e[18] * e[24] + e[16] * e[20] * e[26] +
        e[13] * e[18] * e[21] + e[13] * e[20] * e[23] + e[19] * e[18] * e[9] +
        e[19] * e[20] * e[11] - e[19] * e[23] * e[14] - e[19] * e[24] * e[15] -
        e[19] * e[26] * e[17] - e[19] * e[21] * e[12] + 0.5 * e[10] * e2[22] +
        0.5 * e[10] * e2[25] + 1.5 * e[10] * e2[19] + 0.5 * e[10] * e2[18] +
        0.5 * e[10] * e2[20] - 0.5 * e[10] * e2[26] - 0.5 * e[10] * e2[23] -
        0.5 * e[10] * e2[24] - 0.5 * e[10] * e2[21];
    a[168] = e[19] * e[20] * e[26] - 0.5 * e[25] * e2[20] +
        e[22] * e[21] * e[24] + e[19] * e[18] * e[24] +
        0.5 * e2[22] * e[25] - 0.5 * e[25] * e2[21] - 0.5 * e[25] * e2[23] +
        0.5 * e2[19] * e[25] - 0.5 * e[25] * e2[18] + 0.5 * e[25] * e2[24] +
        0.5 * e[25] * e2[26] + 0.5 * e3[25] + e[22] * e[23] * e[26];
    a[113] = -e[20] * e[33] * e[6] - e[20] * e[30] * e[3] - e[20] * e[31] * e[4] -
        e[29] * e[21] * e[3] - e[29] * e[22] * e[4] - e[29] * e[25] * e[7] -
        e[29] * e[24] * e[6] + e[8] * e[27] * e[24] + e[8] * e[18] * e[33] +
        e[8] * e[28] * e[25] + e[8] * e[19] * e[34] + e[23] * e[27] * e[3] +
        e[23] * e[0] * e[30] + e[23] * e[28] * e[4] + e[23] * e[1] * e[31] +
        e[32] * e[18] * e[3] + e[32] * e[0] * e[21] + e[32] * e[19] * e[4] +
        e[32] * e[1] * e[22] + e[26] * e[27] * e[6] + e[26] * e[0] * e[33] +
        e[26] * e[28] * e[7] + e[26] * e[1] * e[34] + e[26] * e[29] * e[8] +
        e[26] * e[2] * e[35] + e[35] * e[18] * e[6] + e[35] * e[0] * e[24] +
        e[35] * e[19] * e[7] + e[35] * e[1] * e[25] + e[35] * e[20] * e[8] +
        e[2] * e[27] * e[18] + e[2] * e[28] * e[19] +
        3. * e[2] * e[29] * e[20] + e[20] * e[27] * e[0] +
        e[20] * e[28] * e[1] + e[29] * e[18] * e[0] + e[29] * e[19] * e[1] +
        e[5] * e[27] * e[21] + e[5] * e[18] * e[30] + e[5] * e[28] * e[22] +
        e[5] * e[19] * e[31] + e[5] * e[29] * e[23] + e[5] * e[20] * e[32] -
        e[2] * e[33] * e[24] - e[2] * e[30] * e[21] - e[2] * e[31] * e[22] +
        e[2] * e[32] * e[23] - e[2] * e[34] * e[25] - e[20] * e[34] * e[7];
    a[43] = e[5] * e[18] * e[3] + e[5] * e[0] * e[21] + e[5] * e[19] * e[4] +
        e[5] * e[1] * e[22] + e[5] * e[2] * e[23] + e[23] * e[1] * e[4] +
        e[23] * e[0] * e[3] + e[8] * e[18] * e[6] + e[8] * e[0] * e[24] +
        e[8] * e[19] * e[7] + e[8] * e[1] * e[25] + e[8] * e[2] * e[26] +
        e[26] * e[1] * e[7] + e[26] * e[0] * e[6] + e[2] * e[18] * e[0] +
        e[2] * e[19] * e[1] - e[2] * e[21] * e[3] - e[2] * e[22] * e[4] -
        e[2] * e[25] * e[7] - e[2] * e[24] * e[6] - 0.5 * e[20] * e2[4] +
        0.5 * e[20] * e2[0] - 0.5 * e[20] * e2[6] + 0.5 * e[20] * e2[5] +
        0.5 * e[20] * e2[1] - 0.5 * e[20] * e2[7] - 0.5 * e[20] * e2[3] +
        1.5 * e[20] * e2[2] + 0.5 * e[20] * e2[8];
    a[33] = e[14] * e[9] * e[3] + e[14] * e[0] * e[12] + e[14] * e[10] * e[4] +
        e[14] * e[1] * e[13] + e[14] * e[11] * e[5] + e[17] * e[9] * e[6] +
        e[17] * e[0] * e[15] + e[17] * e[10] * e[7] + e[17] * e[1] * e[16] +
        e[17] * e[11] * e[8] + e[8] * e[9] * e[15] + e[8] * e[10] * e[16] +
        e[5] * e[9] * e[12] + e[5] * e[10] * e[13] + e[11] * e[9] * e[0] +
        e[11] * e[10] * e[1] - e[11] * e[13] * e[4] - e[11] * e[16] * e[7] -
        e[11] * e[15] * e[6] - e[11] * e[12] * e[3] + 0.5 * e[2] * e2[14] +
        0.5 * e[2] * e2[17] + 1.5 * e[2] * e2[11] + 0.5 * e[2] * e2[9] +
        0.5 * e[2] * e2[10] - 0.5 * e[2] * e2[16] - 0.5 * e[2] * e2[12] -
        0.5 * e[2] * e2[15] - 0.5 * e[2] * e2[13];
    a[63] = e[14] * e[18] * e[12] + e[14] * e[9] * e[21] + e[14] * e[11] * e[23] +
        e[14] * e[19] * e[13] + e[14] * e[10] * e[22] + e[23] * e[9] * e[12] +
        e[23] * e[10] * e[13] + e[17] * e[18] * e[15] + e[17] * e[9] * e[24] +
        e[17] * e[11] * e[26] + e[17] * e[19] * e[16] +
        e[17] * e[10] * e[25] + e[26] * e[9] * e[15] + e[26] * e[10] * e[16] -
        e[11] * e[24] * e[15] - e[11] * e[25] * e[16] + e[11] * e[18] * e[9] -
        e[11] * e[21] * e[12] + e[11] * e[19] * e[10] -
        e[11] * e[22] * e[13] + 1.5 * e[20] * e2[11] + 0.5 * e[20] * e2[9] +
        0.5 * e[20] * e2[10] + 0.5 * e[20] * e2[14] + 0.5 * e[20] * e2[17] -
        0.5 * e[20] * e2[16] - 0.5 * e[20] * e2[12] - 0.5 * e[20] * e2[15] -
        0.5 * e[20] * e2[13];
    a[143] =
        e[23] * e[10] * e[31] + e[32] * e[18] * e[12] + e[32] * e[9] * e[21] +
        e[32] * e[19] * e[13] + e[32] * e[10] * e[22] - e[11] * e[33] * e[24] -
        e[11] * e[30] * e[21] + e[11] * e[35] * e[26] - e[11] * e[31] * e[22] -
        e[11] * e[34] * e[25] + e[20] * e[27] * e[9] + e[20] * e[28] * e[10] +
        e[29] * e[18] * e[9] + e[29] * e[19] * e[10] + e[17] * e[27] * e[24] +
        e[17] * e[18] * e[33] + e[17] * e[28] * e[25] + e[17] * e[19] * e[34] +
        e[17] * e[29] * e[26] + e[17] * e[20] * e[35] - e[20] * e[30] * e[12] -
        e[20] * e[31] * e[13] - e[20] * e[33] * e[15] - e[20] * e[34] * e[16] -
        e[29] * e[24] * e[15] - e[29] * e[25] * e[16] - e[29] * e[21] * e[12] -
        e[29] * e[22] * e[13] + e[26] * e[27] * e[15] + e[26] * e[9] * e[33] +
        e[26] * e[28] * e[16] + e[26] * e[10] * e[34] + e[35] * e[18] * e[15] +
        e[35] * e[9] * e[24] + e[35] * e[19] * e[16] + e[35] * e[10] * e[25] +
        e[14] * e[27] * e[21] + e[14] * e[18] * e[30] + e[14] * e[28] * e[22] +
        e[14] * e[19] * e[31] + e[14] * e[29] * e[23] + e[14] * e[20] * e[32] +
        e[11] * e[27] * e[18] + e[11] * e[28] * e[19] +
        3. * e[11] * e[29] * e[20] + e[23] * e[27] * e[12] +
        e[23] * e[9] * e[30] + e[23] * e[11] * e[32] + e[23] * e[28] * e[13];
    a[133] =
        e[23] * e[18] * e[12] + e[23] * e[9] * e[21] + e[23] * e[20] * e[14] +
        e[23] * e[19] * e[13] + e[23] * e[10] * e[22] + e[26] * e[18] * e[15] +
        e[26] * e[9] * e[24] + e[26] * e[20] * e[17] + e[26] * e[19] * e[16] +
        e[26] * e[10] * e[25] + e[17] * e[19] * e[25] + e[17] * e[18] * e[24] +
        e[14] * e[19] * e[22] + e[14] * e[18] * e[21] + e[20] * e[18] * e[9] +
        e[20] * e[19] * e[10] - e[20] * e[24] * e[15] - e[20] * e[25] * e[16] -
        e[20] * e[21] * e[12] - e[20] * e[22] * e[13] + 0.5 * e[11] * e2[23] +
        0.5 * e[11] * e2[26] + 0.5 * e[11] * e2[19] + 0.5 * e[11] * e2[18] +
        1.5 * e[11] * e2[20] - 0.5 * e[11] * e2[22] - 0.5 * e[11] * e2[24] -
        0.5 * e[11] * e2[21] - 0.5 * e[11] * e2[25];
    a[103] = -e[20] * e[21] * e[3] + e[20] * e[26] * e[8] - e[20] * e[22] * e[4] -
        e[20] * e[25] * e[7] - e[20] * e[24] * e[6] + e[5] * e[19] * e[22] +
        e[5] * e[18] * e[21] + e[26] * e[18] * e[6] + e[26] * e[0] * e[24] +
        e[26] * e[19] * e[7] + e[26] * e[1] * e[25] + e[8] * e[19] * e[25] +
        e[8] * e[18] * e[24] + e[20] * e[18] * e[0] + e[20] * e[19] * e[1] +
        e[23] * e[18] * e[3] + e[23] * e[0] * e[21] + e[23] * e[19] * e[4] +
        e[23] * e[1] * e[22] + e[23] * e[20] * e[5] + 1.5 * e2[20] * e[2] +
        0.5 * e[2] * e2[23] + 0.5 * e[2] * e2[19] + 0.5 * e[2] * e2[18] +
        0.5 * e[2] * e2[26] - 0.5 * e[2] * e2[22] - 0.5 * e[2] * e2[24] -
        0.5 * e[2] * e2[21] - 0.5 * e[2] * e2[25];
    a[23] = -e[2] * e[15] * e[6] + e[2] * e[9] * e[0] - e[2] * e[12] * e[3] +
        e[5] * e[9] * e[3] + e[5] * e[0] * e[12] + e[5] * e[10] * e[4] +
        e[5] * e[1] * e[13] + e[5] * e[2] * e[14] + e[14] * e[1] * e[4] +
        e[14] * e[0] * e[3] + e[8] * e[9] * e[6] + e[8] * e[0] * e[15] +
        e[8] * e[10] * e[7] + e[8] * e[1] * e[16] + e[8] * e[2] * e[17] +
        e[17] * e[1] * e[7] + e[17] * e[0] * e[6] + e[2] * e[10] * e[1] -
        e[2] * e[13] * e[4] - e[2] * e[16] * e[7] + 0.5 * e[11] * e2[1] +
        0.5 * e[11] * e2[0] + 1.5 * e[11] * e2[2] + 0.5 * e[11] * e2[5] +
        0.5 * e[11] * e2[8] - 0.5 * e[11] * e2[4] - 0.5 * e[11] * e2[6] -
        0.5 * e[11] * e2[7] - 0.5 * e[11] * e2[3];
    a[83] = e[5] * e[19] * e[13] + e[5] * e[10] * e[22] + e[8] * e[18] * e[15] +
        e[8] * e[9] * e[24] + e[8] * e[20] * e[17] + e[8] * e[11] * e[26] +
        e[8] * e[19] * e[16] + e[8] * e[10] * e[25] + e[2] * e[18] * e[9] +
        e[2] * e[19] * e[10] - e[11] * e[21] * e[3] - e[11] * e[22] * e[4] -
        e[11] * e[25] * e[7] - e[11] * e[24] * e[6] + e[14] * e[18] * e[3] +
        e[14] * e[0] * e[21] + e[14] * e[19] * e[4] + e[14] * e[1] * e[22] +
        e[14] * e[2] * e[23] - e[20] * e[13] * e[4] - e[20] * e[16] * e[7] -
        e[20] * e[15] * e[6] - e[20] * e[12] * e[3] + e[23] * e[9] * e[3] +
        e[23] * e[0] * e[12] + e[23] * e[10] * e[4] + e[23] * e[1] * e[13] +
        e[17] * e[18] * e[6] + e[17] * e[0] * e[24] + e[17] * e[19] * e[7] +
        e[17] * e[1] * e[25] + e[17] * e[2] * e[26] - e[2] * e[24] * e[15] -
        e[2] * e[25] * e[16] - e[2] * e[21] * e[12] - e[2] * e[22] * e[13] +
        e[26] * e[9] * e[6] + e[26] * e[0] * e[15] + e[26] * e[10] * e[7] +
        e[26] * e[1] * e[16] + e[11] * e[18] * e[0] + e[11] * e[19] * e[1] +
        3. * e[11] * e[20] * e[2] + e[20] * e[9] * e[0] +
        e[20] * e[10] * e[1] + e[5] * e[18] * e[12] + e[5] * e[9] * e[21] +
        e[5] * e[20] * e[14] + e[5] * e[11] * e[23];
    a[53] = e[32] * e[1] * e[4] + e[32] * e[0] * e[3] + e[8] * e[27] * e[6] +
        e[8] * e[0] * e[33] + e[8] * e[28] * e[7] + e[8] * e[1] * e[34] +
        e[35] * e[1] * e[7] + e[35] * e[0] * e[6] + e[2] * e[27] * e[0] +
        e[2] * e[28] * e[1] - e[2] * e[34] * e[7] + e[2] * e[32] * e[5] -
        e[2] * e[33] * e[6] - e[2] * e[30] * e[3] + e[2] * e[35] * e[8] -
        e[2] * e[31] * e[4] + e[5] * e[27] * e[3] + e[5] * e[0] * e[30] +
        e[5] * e[28] * e[4] + e[5] * e[1] * e[31] + 1.5 * e[29] * e2[2] -
        0.5 * e[29] * e2[4] + 0.5 * e[29] * e2[0] - 0.5 * e[29] * e2[6] +
        0.5 * e[29] * e2[5] + 0.5 * e[29] * e2[1] - 0.5 * e[29] * e2[7] -
        0.5 * e[29] * e2[3] + 0.5 * e[29] * e2[8];
    a[3] = e[5] * e[0] * e[3] + e[8] * e[1] * e[7] + e[8] * e[0] * e[6] +
        e[5] * e[1] * e[4] - 0.5 * e[2] * e2[4] + 0.5 * e3[2] +
        0.5 * e[2] * e2[1] - 0.5 * e[2] * e2[3] + 0.5 * e[2] * e2[0] +
        0.5 * e[2] * e2[8] + 0.5 * e[2] * e2[5] - 0.5 * e[2] * e2[6] -
        0.5 * e[2] * e2[7];
    a[73] = e[35] * e[9] * e[15] + e[35] * e[10] * e[16] - e[11] * e[30] * e[12] -
        e[11] * e[31] * e[13] - e[11] * e[33] * e[15] -
        e[11] * e[34] * e[16] + e[11] * e[27] * e[9] + e[11] * e[28] * e[10] +
        e[14] * e[27] * e[12] + e[14] * e[9] * e[30] + e[14] * e[11] * e[32] +
        e[14] * e[28] * e[13] + e[14] * e[10] * e[31] + e[32] * e[9] * e[12] +
        e[32] * e[10] * e[13] + e[17] * e[27] * e[15] + e[17] * e[9] * e[33] +
        e[17] * e[11] * e[35] + e[17] * e[28] * e[16] +
        e[17] * e[10] * e[34] + 1.5 * e[29] * e2[11] - 0.5 * e[29] * e2[16] +
        0.5 * e[29] * e2[9] - 0.5 * e[29] * e2[12] - 0.5 * e[29] * e2[15] +
        0.5 * e[29] * e2[17] + 0.5 * e[29] * e2[10] + 0.5 * e[29] * e2[14] -
        0.5 * e[29] * e2[13];
    a[13] = e[14] * e[9] * e[12] + e[17] * e[10] * e[16] + e[17] * e[9] * e[15] +
        0.5 * e3[11] + e[14] * e[10] * e[13] + 0.5 * e[11] * e2[10] -
        0.5 * e[11] * e2[15] + 0.5 * e[11] * e2[14] - 0.5 * e[11] * e2[13] -
        0.5 * e[11] * e2[12] + 0.5 * e[11] * e2[9] - 0.5 * e[11] * e2[16] +
        0.5 * e[11] * e2[17];
    a[173] =
        e[20] * e[27] * e[18] + e[20] * e[28] * e[19] + e[23] * e[27] * e[21] +
        e[23] * e[18] * e[30] + e[23] * e[28] * e[22] + e[23] * e[19] * e[31] +
        e[23] * e[20] * e[32] + e[32] * e[19] * e[22] + e[32] * e[18] * e[21] +
        e[26] * e[27] * e[24] + e[26] * e[18] * e[33] + e[26] * e[28] * e[25] +
        e[26] * e[19] * e[34] + e[26] * e[20] * e[35] + e[35] * e[19] * e[25] +
        e[35] * e[18] * e[24] - e[20] * e[33] * e[24] - e[20] * e[30] * e[21] -
        e[20] * e[31] * e[22] - e[20] * e[34] * e[25] + 0.5 * e[29] * e2[23] +
        0.5 * e[29] * e2[26] - 0.5 * e[29] * e2[22] - 0.5 * e[29] * e2[24] -
        0.5 * e[29] * e2[21] - 0.5 * e[29] * e2[25] + 1.5 * e[29] * e2[20] +
        0.5 * e[29] * e2[19] + 0.5 * e[29] * e2[18];
    a[163] = 0.5 * e[20] * e2[26] + 0.5 * e[20] * e2[18] + 0.5 * e3[20] +
        0.5 * e[20] * e2[19] + e[26] * e[18] * e[24] + 0.5 * e[20] * e2[23] -
        0.5 * e[20] * e2[25] + e[23] * e[19] * e[22] - 0.5 * e[20] * e2[24] -
        0.5 * e[20] * e2[21] - 0.5 * e[20] * e2[22] + e[23] * e[18] * e[21] +
        e[26] * e[19] * e[25];
    a[93] = e[8] * e[28] * e[16] + e[8] * e[10] * e[34] + e[2] * e[27] * e[9] +
        3. * e[2] * e[29] * e[11] + e[2] * e[28] * e[10] +
        e[11] * e[27] * e[0] - e[11] * e[34] * e[7] - e[11] * e[33] * e[6] -
        e[11] * e[30] * e[3] + e[11] * e[28] * e[1] - e[11] * e[31] * e[4] +
        e[14] * e[27] * e[3] + e[14] * e[0] * e[30] + e[14] * e[28] * e[4] +
        e[14] * e[1] * e[31] + e[14] * e[2] * e[32] + e[29] * e[10] * e[1] -
        e[29] * e[13] * e[4] - e[29] * e[16] * e[7] - e[29] * e[15] * e[6] +
        e[29] * e[9] * e[0] - e[29] * e[12] * e[3] + e[32] * e[9] * e[3] +
        e[32] * e[0] * e[12] + e[32] * e[10] * e[4] + e[32] * e[1] * e[13] +
        e[17] * e[27] * e[6] + e[17] * e[0] * e[33] + e[17] * e[28] * e[7] +
        e[17] * e[1] * e[34] + e[17] * e[2] * e[35] - e[2] * e[30] * e[12] -
        e[2] * e[31] * e[13] - e[2] * e[33] * e[15] - e[2] * e[34] * e[16] +
        e[35] * e[9] * e[6] + e[35] * e[0] * e[15] + e[35] * e[10] * e[7] +
        e[35] * e[1] * e[16] + e[5] * e[27] * e[12] + e[5] * e[9] * e[30] +
        e[5] * e[29] * e[14] + e[5] * e[11] * e[32] + e[5] * e[28] * e[13] +
        e[5] * e[10] * e[31] + e[8] * e[27] * e[15] + e[8] * e[9] * e[33] +
        e[8] * e[29] * e[17] + e[8] * e[11] * e[35];
    a[94] = -e[12] * e[34] * e[7] + e[12] * e[32] * e[5] - e[12] * e[35] * e[8] -
        e[12] * e[29] * e[2] - e[12] * e[28] * e[1] + e[12] * e[31] * e[4] -
        e[30] * e[11] * e[2] - e[30] * e[10] * e[1] + e[30] * e[13] * e[4] -
        e[30] * e[16] * e[7] + e[30] * e[14] * e[5] - e[30] * e[17] * e[8] +
        e[15] * e[3] * e[33] + e[15] * e[31] * e[7] + e[15] * e[4] * e[34] +
        e[15] * e[32] * e[8] + e[15] * e[5] * e[35] + e[3] * e[27] * e[9] -
        e[3] * e[28] * e[10] - e[3] * e[34] * e[16] - e[3] * e[35] * e[17] -
        e[3] * e[29] * e[11] + e[33] * e[13] * e[7] + e[33] * e[4] * e[16] +
        e[33] * e[14] * e[8] + e[33] * e[5] * e[17] + e[9] * e[28] * e[4] +
        e[9] * e[1] * e[31] + e[9] * e[29] * e[5] + e[9] * e[2] * e[32] +
        e[27] * e[10] * e[4] + e[27] * e[1] * e[13] + e[27] * e[11] * e[5] +
        e[27] * e[2] * e[14] + 3. * e[3] * e[30] * e[12] +
        e[3] * e[32] * e[14] + e[3] * e[31] * e[13] + e[6] * e[30] * e[15] +
        e[6] * e[12] * e[33] + e[6] * e[32] * e[17] + e[6] * e[14] * e[35] +
        e[6] * e[31] * e[16] + e[6] * e[13] * e[34] + e[0] * e[27] * e[12] +
        e[0] * e[9] * e[30] + e[0] * e[29] * e[14] + e[0] * e[11] * e[32] +
        e[0] * e[28] * e[13] + e[0] * e[10] * e[31];
    a[164] = 0.5 * e[21] * e2[24] - 0.5 * e[21] * e2[25] + 0.5 * e[21] * e2[23] -
        0.5 * e[21] * e2[26] + 0.5 * e2[18] * e[21] + 0.5 * e[21] * e2[22] -
        0.5 * e[21] * e2[20] + e[24] * e[22] * e[25] +
        e[24] * e[23] * e[26] - 0.5 * e[21] * e2[19] +
        e[18] * e[19] * e[22] + e[18] * e[20] * e[23] + 0.5 * e3[21];
    a[174] =
        -0.5 * e[30] * e2[26] - 0.5 * e[30] * e2[19] - 0.5 * e[30] * e2[20] -
        0.5 * e[30] * e2[25] + 0.5 * e2[18] * e[30] + 1.5 * e[30] * e2[21] +
        0.5 * e[30] * e2[22] + 0.5 * e[30] * e2[23] + 0.5 * e[30] * e2[24] +
        e[18] * e[27] * e[21] + e[18] * e[28] * e[22] + e[18] * e[19] * e[31] +
        e[18] * e[29] * e[23] + e[18] * e[20] * e[32] + e[27] * e[19] * e[22] +
        e[27] * e[20] * e[23] + e[21] * e[31] * e[22] + e[21] * e[32] * e[23] +
        e[24] * e[21] * e[33] + e[24] * e[31] * e[25] + e[24] * e[22] * e[34] +
        e[24] * e[32] * e[26] + e[24] * e[23] * e[35] + e[33] * e[22] * e[25] +
        e[33] * e[23] * e[26] - e[21] * e[29] * e[20] - e[21] * e[35] * e[26] -
        e[21] * e[28] * e[19] - e[21] * e[34] * e[25];
    a[14] = 0.5 * e[12] * e2[15] - 0.5 * e[12] * e2[17] + e[15] * e[13] * e[16] -
        0.5 * e[12] * e2[10] + e[15] * e[14] * e[17] - 0.5 * e[12] * e2[16] -
        0.5 * e[12] * e2[11] + e[9] * e[10] * e[13] + 0.5 * e[12] * e2[13] +
        0.5 * e2[9] * e[12] + 0.5 * e3[12] + e[9] * e[11] * e[14] +
        0.5 * e[12] * e2[14];
    a[34] = e[12] * e[13] * e[4] + e[12] * e[14] * e[5] + e[15] * e[12] * e[6] +
        e[15] * e[13] * e[7] + e[15] * e[4] * e[16] + e[15] * e[14] * e[8] +
        e[15] * e[5] * e[17] + e[6] * e[14] * e[17] + e[6] * e[13] * e[16] +
        e[0] * e[11] * e[14] + e[0] * e[9] * e[12] + e[0] * e[10] * e[13] +
        e[9] * e[10] * e[4] + e[9] * e[1] * e[13] + e[9] * e[11] * e[5] +
        e[9] * e[2] * e[14] - e[12] * e[11] * e[2] - e[12] * e[10] * e[1] -
        e[12] * e[16] * e[7] - e[12] * e[17] * e[8] + 1.5 * e2[12] * e[3] +
        0.5 * e[3] * e2[15] - 0.5 * e[3] * e2[16] + 0.5 * e[3] * e2[9] -
        0.5 * e[3] * e2[11] - 0.5 * e[3] * e2[17] - 0.5 * e[3] * e2[10] +
        0.5 * e[3] * e2[14] + 0.5 * e[3] * e2[13];
    a[64] =
        e[18] * e[11] * e[14] + e[18] * e[9] * e[12] + e[18] * e[10] * e[13] +
        e[12] * e[23] * e[14] + e[12] * e[22] * e[13] + e[15] * e[12] * e[24] +
        e[15] * e[23] * e[17] + e[15] * e[14] * e[26] + e[15] * e[22] * e[16] +
        e[15] * e[13] * e[25] + e[24] * e[14] * e[17] + e[24] * e[13] * e[16] -
        e[12] * e[25] * e[16] - e[12] * e[26] * e[17] - e[12] * e[20] * e[11] -
        e[12] * e[19] * e[10] + e[9] * e[20] * e[14] + e[9] * e[11] * e[23] +
        e[9] * e[19] * e[13] + e[9] * e[10] * e[22] + 0.5 * e2[9] * e[21] -
        0.5 * e[21] * e2[16] - 0.5 * e[21] * e2[11] - 0.5 * e[21] * e2[17] -
        0.5 * e[21] * e2[10] + 1.5 * e[21] * e2[12] + 0.5 * e[21] * e2[14] +
        0.5 * e[21] * e2[13] + 0.5 * e[21] * e2[15];
    a[114] = -e[21] * e[35] * e[8] - e[21] * e[29] * e[2] - e[21] * e[28] * e[1] +
        e[21] * e[31] * e[4] - e[30] * e[26] * e[8] - e[30] * e[20] * e[2] -
        e[30] * e[19] * e[1] + e[30] * e[22] * e[4] - e[30] * e[25] * e[7] +
        e[30] * e[23] * e[5] + e[6] * e[31] * e[25] + e[6] * e[22] * e[34] +
        e[6] * e[32] * e[26] + e[6] * e[23] * e[35] + e[24] * e[30] * e[6] +
        e[24] * e[3] * e[33] + e[24] * e[31] * e[7] + e[24] * e[4] * e[34] +
        e[24] * e[32] * e[8] + e[24] * e[5] * e[35] + e[33] * e[21] * e[6] +
        e[33] * e[22] * e[7] + e[33] * e[4] * e[25] + e[33] * e[23] * e[8] +
        e[33] * e[5] * e[26] + e[0] * e[27] * e[21] + e[0] * e[18] * e[30] +
        e[0] * e[28] * e[22] + e[0] * e[19] * e[31] + e[0] * e[29] * e[23] +
        e[0] * e[20] * e[32] + e[18] * e[27] * e[3] + e[18] * e[28] * e[4] +
        e[18] * e[1] * e[31] + e[18] * e[29] * e[5] + e[18] * e[2] * e[32] +
        e[27] * e[19] * e[4] + e[27] * e[1] * e[22] + e[27] * e[20] * e[5] +
        e[27] * e[2] * e[23] + 3. * e[3] * e[30] * e[21] +
        e[3] * e[31] * e[22] + e[3] * e[32] * e[23] - e[3] * e[29] * e[20] -
        e[3] * e[35] * e[26] - e[3] * e[28] * e[19] - e[3] * e[34] * e[25] -
        e[21] * e[34] * e[7] + e[21] * e[32] * e[5];
    a[44] = e[18] * e[1] * e[4] + e[18] * e[0] * e[3] + e[18] * e[2] * e[5] +
        e[3] * e[22] * e[4] + e[3] * e[23] * e[5] + e[6] * e[3] * e[24] +
        e[6] * e[22] * e[7] + e[6] * e[4] * e[25] + e[6] * e[23] * e[8] +
        e[6] * e[5] * e[26] + e[24] * e[4] * e[7] + e[24] * e[5] * e[8] +
        e[0] * e[19] * e[4] + e[0] * e[1] * e[22] + e[0] * e[20] * e[5] +
        e[0] * e[2] * e[23] - e[3] * e[26] * e[8] - e[3] * e[20] * e[2] -
        e[3] * e[19] * e[1] - e[3] * e[25] * e[7] + 0.5 * e[21] * e2[4] +
        0.5 * e[21] * e2[0] + 0.5 * e[21] * e2[6] + 0.5 * e[21] * e2[5] -
        0.5 * e[21] * e2[1] - 0.5 * e[21] * e2[7] + 1.5 * e[21] * e2[3] -
        0.5 * e[21] * e2[2] - 0.5 * e[21] * e2[8];
    a[184] =
        0.5 * e2[27] * e[21] + 1.5 * e[21] * e2[30] + 0.5 * e[21] * e2[32] +
        0.5 * e[21] * e2[31] + 0.5 * e[21] * e2[33] - 0.5 * e[21] * e2[28] -
        0.5 * e[21] * e2[29] - 0.5 * e[21] * e2[34] - 0.5 * e[21] * e2[35] +
        e[18] * e[27] * e[30] + e[18] * e[29] * e[32] + e[18] * e[28] * e[31] +
        e[27] * e[28] * e[22] + e[27] * e[19] * e[31] + e[27] * e[29] * e[23] +
        e[27] * e[20] * e[32] + e[30] * e[31] * e[22] + e[30] * e[32] * e[23] +
        e[24] * e[30] * e[33] + e[24] * e[32] * e[35] + e[24] * e[31] * e[34] +
        e[33] * e[31] * e[25] + e[33] * e[22] * e[34] + e[33] * e[32] * e[26] +
        e[33] * e[23] * e[35] - e[30] * e[29] * e[20] - e[30] * e[35] * e[26] -
        e[30] * e[28] * e[19] - e[30] * e[34] * e[25];
    a[49] = -0.5 * e[26] * e2[4] - 0.5 * e[26] * e2[0] + 0.5 * e[26] * e2[6] +
        0.5 * e[26] * e2[5] - 0.5 * e[26] * e2[1] + 0.5 * e[26] * e2[7] -
        0.5 * e[26] * e2[3] + 0.5 * e[26] * e2[2] + 1.5 * e[26] * e2[8] +
        e[20] * e[0] * e[6] + e[20] * e[2] * e[8] + e[5] * e[21] * e[6] +
        e[5] * e[3] * e[24] + e[5] * e[22] * e[7] + e[5] * e[4] * e[25] +
        e[5] * e[23] * e[8] + e[23] * e[4] * e[7] + e[23] * e[3] * e[6] +
        e[8] * e[24] * e[6] + e[8] * e[25] * e[7] + e[2] * e[18] * e[6] +
        e[2] * e[0] * e[24] + e[2] * e[19] * e[7] + e[2] * e[1] * e[25] -
        e[8] * e[21] * e[3] - e[8] * e[19] * e[1] - e[8] * e[22] * e[4] -
        e[8] * e[18] * e[0] + e[20] * e[1] * e[7];
    a[154] =
        e[9] * e[27] * e[30] + e[9] * e[29] * e[32] + e[9] * e[28] * e[31] +
        e[33] * e[30] * e[15] + e[33] * e[32] * e[17] + e[33] * e[14] * e[35] +
        e[33] * e[31] * e[16] + e[33] * e[13] * e[34] + e[27] * e[29] * e[14] +
        e[27] * e[11] * e[32] + e[27] * e[28] * e[13] + e[27] * e[10] * e[31] -
        e[30] * e[28] * e[10] + e[30] * e[31] * e[13] + e[30] * e[32] * e[14] -
        e[30] * e[34] * e[16] - e[30] * e[35] * e[17] - e[30] * e[29] * e[11] +
        e[15] * e[32] * e[35] + e[15] * e[31] * e[34] - 0.5 * e[12] * e2[34] -
        0.5 * e[12] * e2[35] + 0.5 * e[12] * e2[27] + 0.5 * e[12] * e2[32] -
        0.5 * e[12] * e2[28] - 0.5 * e[12] * e2[29] + 0.5 * e[12] * e2[31] +
        0.5 * e[12] * e2[33] + 1.5 * e[12] * e2[30];
    a[119] = e[23] * e[30] * e[6] + e[23] * e[3] * e[33] + e[23] * e[31] * e[7] +
        e[23] * e[4] * e[34] + e[32] * e[21] * e[6] + e[32] * e[3] * e[24] +
        e[32] * e[22] * e[7] + e[32] * e[4] * e[25] + e[26] * e[33] * e[6] +
        e[26] * e[34] * e[7] + 3. * e[26] * e[35] * e[8] +
        e[35] * e[24] * e[6] + e[35] * e[25] * e[7] + e[2] * e[27] * e[24] +
        e[2] * e[18] * e[33] + e[2] * e[28] * e[25] + e[2] * e[19] * e[34] +
        e[2] * e[29] * e[26] + e[2] * e[20] * e[35] + e[20] * e[27] * e[6] +
        e[20] * e[0] * e[33] + e[20] * e[28] * e[7] + e[20] * e[1] * e[34] +
        e[20] * e[29] * e[8] + e[29] * e[18] * e[6] + e[29] * e[0] * e[24] +
        e[29] * e[19] * e[7] + e[29] * e[1] * e[25] + e[5] * e[30] * e[24] +
        e[5] * e[21] * e[33] + e[5] * e[31] * e[25] + e[5] * e[22] * e[34] +
        e[5] * e[32] * e[26] + e[5] * e[23] * e[35] - e[8] * e[27] * e[18] +
        e[8] * e[33] * e[24] - e[8] * e[30] * e[21] - e[8] * e[31] * e[22] +
        e[8] * e[32] * e[23] - e[8] * e[28] * e[19] + e[8] * e[34] * e[25] -
        e[26] * e[27] * e[0] - e[26] * e[30] * e[3] - e[26] * e[28] * e[1] -
        e[26] * e[31] * e[4] - e[35] * e[21] * e[3] - e[35] * e[19] * e[1] -
        e[35] * e[22] * e[4] - e[35] * e[18] * e[0];
    a[194] = e[27] * e[29] * e[32] + e[27] * e[28] * e[31] +
        e[33] * e[32] * e[35] + e[33] * e[31] * e[34] + 0.5 * e3[30] -
        0.5 * e[30] * e2[28] - 0.5 * e[30] * e2[29] - 0.5 * e[30] * e2[34] +
        0.5 * e[30] * e2[33] + 0.5 * e2[27] * e[30] + 0.5 * e[30] * e2[32] +
        0.5 * e[30] * e2[31] - 0.5 * e[30] * e2[35];
    a[69] =
        0.5 * e2[14] * e[26] + 1.5 * e[26] * e2[17] + 0.5 * e[26] * e2[15] +
        0.5 * e[26] * e2[16] + 0.5 * e2[11] * e[26] - 0.5 * e[26] * e2[9] -
        0.5 * e[26] * e2[12] - 0.5 * e[26] * e2[10] - 0.5 * e[26] * e2[13] +
        e[20] * e[11] * e[17] + e[20] * e[9] * e[15] + e[20] * e[10] * e[16] +
        e[14] * e[21] * e[15] + e[14] * e[12] * e[24] + e[14] * e[23] * e[17] +
        e[14] * e[22] * e[16] + e[14] * e[13] * e[25] + e[23] * e[12] * e[15] +
        e[23] * e[13] * e[16] + e[17] * e[24] * e[15] + e[17] * e[25] * e[16] -
        e[17] * e[18] * e[9] - e[17] * e[21] * e[12] - e[17] * e[19] * e[10] -
        e[17] * e[22] * e[13] + e[11] * e[18] * e[15] + e[11] * e[9] * e[24] +
        e[11] * e[19] * e[16] + e[11] * e[10] * e[25];
    a[124] = e[0] * e[27] * e[30] + e[0] * e[29] * e[32] + e[0] * e[28] * e[31] +
        e[30] * e[31] * e[4] + e[30] * e[32] * e[5] + e[6] * e[30] * e[33] +
        e[6] * e[32] * e[35] + e[6] * e[31] * e[34] + e[27] * e[28] * e[4] +
        e[27] * e[1] * e[31] + e[27] * e[29] * e[5] + e[27] * e[2] * e[32] +
        e[33] * e[31] * e[7] + e[33] * e[4] * e[34] + e[33] * e[32] * e[8] +
        e[33] * e[5] * e[35] - e[30] * e[34] * e[7] - e[30] * e[35] * e[8] -
        e[30] * e[29] * e[2] - e[30] * e[28] * e[1] + 1.5 * e[3] * e2[30] +
        0.5 * e[3] * e2[32] + 0.5 * e[3] * e2[31] + 0.5 * e[3] * e2[27] -
        0.5 * e[3] * e2[28] - 0.5 * e[3] * e2[29] + 0.5 * e[3] * e2[33] -
        0.5 * e[3] * e2[34] - 0.5 * e[3] * e2[35];
    a[39] = 0.5 * e2[14] * e[8] + 1.5 * e2[17] * e[8] + 0.5 * e[8] * e2[15] +
        0.5 * e[8] * e2[16] - 0.5 * e[8] * e2[9] + 0.5 * e[8] * e2[11] -
        0.5 * e[8] * e2[12] - 0.5 * e[8] * e2[10] - 0.5 * e[8] * e2[13] +
        e[14] * e[12] * e[6] + e[14] * e[3] * e[15] + e[14] * e[13] * e[7] +
        e[14] * e[4] * e[16] + e[14] * e[5] * e[17] + e[17] * e[15] * e[6] +
        e[17] * e[16] * e[7] + e[2] * e[11] * e[17] + e[2] * e[9] * e[15] +
        e[2] * e[10] * e[16] + e[5] * e[12] * e[15] + e[5] * e[13] * e[16] +
        e[11] * e[9] * e[6] + e[11] * e[0] * e[15] + e[11] * e[10] * e[7] +
        e[11] * e[1] * e[16] - e[17] * e[10] * e[1] - e[17] * e[13] * e[4] -
        e[17] * e[9] * e[0] - e[17] * e[12] * e[3];
    a[4] = -0.5 * e[3] * e2[1] - 0.5 * e[3] * e2[7] + 0.5 * e3[3] -
        0.5 * e[3] * e2[8] + e[0] * e[2] * e[5] + 0.5 * e[3] * e2[6] +
        0.5 * e[3] * e2[4] - 0.5 * e[3] * e2[2] + e[0] * e[1] * e[4] +
        e[6] * e[4] * e[7] + 0.5 * e2[0] * e[3] + 0.5 * e[3] * e2[5] +
        e[6] * e[5] * e[8];
    a[139] =
        0.5 * e2[23] * e[17] + 1.5 * e2[26] * e[17] + 0.5 * e[17] * e2[25] +
        0.5 * e[17] * e2[24] - 0.5 * e[17] * e2[18] - 0.5 * e[17] * e2[19] +
        0.5 * e[17] * e2[20] - 0.5 * e[17] * e2[22] - 0.5 * e[17] * e2[21] +
        e[23] * e[21] * e[15] + e[23] * e[12] * e[24] + e[23] * e[14] * e[26] +
        e[23] * e[22] * e[16] + e[23] * e[13] * e[25] + e[26] * e[24] * e[15] +
        e[26] * e[25] * e[16] + e[11] * e[19] * e[25] + e[11] * e[18] * e[24] +
        e[11] * e[20] * e[26] + e[14] * e[22] * e[25] + e[14] * e[21] * e[24] +
        e[20] * e[18] * e[15] + e[20] * e[9] * e[24] + e[20] * e[19] * e[16] +
        e[20] * e[10] * e[25] - e[26] * e[18] * e[9] - e[26] * e[21] * e[12] -
        e[26] * e[19] * e[10] - e[26] * e[22] * e[13];
    a[74] =
        -e[12] * e[34] * e[16] - e[12] * e[35] * e[17] - e[12] * e[29] * e[11] +
        e[9] * e[27] * e[12] + e[9] * e[29] * e[14] + e[9] * e[11] * e[32] +
        e[9] * e[28] * e[13] + e[9] * e[10] * e[31] + e[27] * e[11] * e[14] +
        e[27] * e[10] * e[13] + e[12] * e[32] * e[14] + e[12] * e[31] * e[13] +
        e[15] * e[12] * e[33] + e[15] * e[32] * e[17] + e[15] * e[14] * e[35] +
        e[15] * e[31] * e[16] + e[15] * e[13] * e[34] + e[33] * e[14] * e[17] +
        e[33] * e[13] * e[16] - e[12] * e[28] * e[10] + 0.5 * e2[9] * e[30] -
        0.5 * e[30] * e2[16] - 0.5 * e[30] * e2[11] + 1.5 * e[30] * e2[12] +
        0.5 * e[30] * e2[15] - 0.5 * e[30] * e2[17] - 0.5 * e[30] * e2[10] +
        0.5 * e[30] * e2[14] + 0.5 * e[30] * e2[13];
    a[149] =
        e[32] * e[22] * e[16] + e[32] * e[13] * e[25] - e[17] * e[27] * e[18] +
        e[17] * e[33] * e[24] - e[17] * e[30] * e[21] + e[17] * e[29] * e[20] +
        3. * e[17] * e[35] * e[26] - e[17] * e[31] * e[22] -
        e[17] * e[28] * e[19] + e[17] * e[34] * e[25] + e[20] * e[27] * e[15] +
        e[20] * e[9] * e[33] + e[20] * e[28] * e[16] + e[20] * e[10] * e[34] +
        e[29] * e[18] * e[15] + e[29] * e[9] * e[24] + e[29] * e[19] * e[16] +
        e[29] * e[10] * e[25] - e[26] * e[27] * e[9] - e[26] * e[30] * e[12] -
        e[26] * e[28] * e[10] - e[26] * e[31] * e[13] + e[26] * e[33] * e[15] +
        e[26] * e[34] * e[16] + e[35] * e[24] * e[15] + e[35] * e[25] * e[16] -
        e[35] * e[18] * e[9] - e[35] * e[21] * e[12] - e[35] * e[19] * e[10] -
        e[35] * e[22] * e[13] + e[14] * e[30] * e[24] + e[14] * e[21] * e[33] +
        e[14] * e[31] * e[25] + e[14] * e[22] * e[34] + e[14] * e[32] * e[26] +
        e[14] * e[23] * e[35] + e[11] * e[27] * e[24] + e[11] * e[18] * e[33] +
        e[11] * e[28] * e[25] + e[11] * e[19] * e[34] + e[11] * e[29] * e[26] +
        e[11] * e[20] * e[35] + e[23] * e[30] * e[15] + e[23] * e[12] * e[33] +
        e[23] * e[32] * e[17] + e[23] * e[31] * e[16] + e[23] * e[13] * e[34] +
        e[32] * e[21] * e[15] + e[32] * e[12] * e[24];
    a[84] = e[6] * e[23] * e[17] + e[6] * e[14] * e[26] + e[6] * e[22] * e[16] +
        e[6] * e[13] * e[25] + e[0] * e[20] * e[14] + e[0] * e[11] * e[23] +
        e[0] * e[19] * e[13] + e[0] * e[10] * e[22] - e[12] * e[26] * e[8] -
        e[12] * e[20] * e[2] - e[12] * e[19] * e[1] + e[12] * e[22] * e[4] -
        e[12] * e[25] * e[7] + e[12] * e[23] * e[5] - e[21] * e[11] * e[2] -
        e[21] * e[10] * e[1] + e[21] * e[13] * e[4] - e[21] * e[16] * e[7] +
        e[21] * e[14] * e[5] - e[21] * e[17] * e[8] + e[15] * e[3] * e[24] +
        e[15] * e[22] * e[7] + e[15] * e[4] * e[25] + e[15] * e[23] * e[8] +
        e[15] * e[5] * e[26] - e[3] * e[25] * e[16] - e[3] * e[26] * e[17] -
        e[3] * e[20] * e[11] - e[3] * e[19] * e[10] + e[24] * e[13] * e[7] +
        e[24] * e[4] * e[16] + e[24] * e[14] * e[8] + e[24] * e[5] * e[17] +
        e[9] * e[18] * e[3] + e[9] * e[0] * e[21] + e[9] * e[19] * e[4] +
        e[9] * e[1] * e[22] + e[9] * e[20] * e[5] + e[9] * e[2] * e[23] +
        e[18] * e[0] * e[12] + e[18] * e[10] * e[4] + e[18] * e[1] * e[13] +
        e[18] * e[11] * e[5] + e[18] * e[2] * e[14] +
        3. * e[3] * e[21] * e[12] + e[3] * e[23] * e[14] +
        e[3] * e[22] * e[13] + e[6] * e[21] * e[15] + e[6] * e[12] * e[24];
    a[29] = 0.5 * e2[5] * e[17] + 1.5 * e[17] * e2[8] + 0.5 * e[17] * e2[7] +
        0.5 * e[17] * e2[6] + 0.5 * e2[2] * e[17] - 0.5 * e[17] * e2[4] -
        0.5 * e[17] * e2[0] - 0.5 * e[17] * e2[1] - 0.5 * e[17] * e2[3] +
        e[11] * e[1] * e[7] + e[11] * e[0] * e[6] + e[11] * e[2] * e[8] +
        e[5] * e[12] * e[6] + e[5] * e[3] * e[15] + e[5] * e[13] * e[7] +
        e[5] * e[4] * e[16] + e[5] * e[14] * e[8] + e[14] * e[4] * e[7] +
        e[14] * e[3] * e[6] + e[8] * e[15] * e[6] + e[8] * e[16] * e[7] -
        e[8] * e[10] * e[1] - e[8] * e[13] * e[4] - e[8] * e[9] * e[0] -
        e[8] * e[12] * e[3] + e[2] * e[9] * e[6] + e[2] * e[0] * e[15] +
        e[2] * e[10] * e[7] + e[2] * e[1] * e[16];
    a[54] = e[6] * e[4] * e[34] + e[6] * e[32] * e[8] + e[6] * e[5] * e[35] +
        e[33] * e[4] * e[7] + e[33] * e[5] * e[8] + e[0] * e[27] * e[3] +
        e[0] * e[28] * e[4] + e[0] * e[1] * e[31] + e[0] * e[29] * e[5] +
        e[0] * e[2] * e[32] - e[3] * e[34] * e[7] + e[3] * e[32] * e[5] +
        e[3] * e[33] * e[6] - e[3] * e[35] * e[8] - e[3] * e[29] * e[2] -
        e[3] * e[28] * e[1] + e[3] * e[31] * e[4] + e[27] * e[1] * e[4] +
        e[27] * e[2] * e[5] + e[6] * e[31] * e[7] + 0.5 * e[30] * e2[4] +
        0.5 * e[30] * e2[6] + 0.5 * e[30] * e2[5] - 0.5 * e[30] * e2[1] -
        0.5 * e[30] * e2[7] - 0.5 * e[30] * e2[2] - 0.5 * e[30] * e2[8] +
        0.5 * e2[0] * e[30] + 1.5 * e[30] * e2[3];
    a[109] = 0.5 * e2[23] * e[8] + 1.5 * e2[26] * e[8] - 0.5 * e[8] * e2[18] -
        0.5 * e[8] * e2[19] - 0.5 * e[8] * e2[22] + 0.5 * e[8] * e2[24] -
        0.5 * e[8] * e2[21] + 0.5 * e[8] * e2[25] + 0.5 * e2[20] * e[8] +
        e[20] * e[18] * e[6] + e[20] * e[0] * e[24] + e[20] * e[19] * e[7] +
        e[20] * e[1] * e[25] + e[20] * e[2] * e[26] + e[23] * e[21] * e[6] +
        e[23] * e[3] * e[24] + e[23] * e[22] * e[7] + e[23] * e[4] * e[25] +
        e[23] * e[5] * e[26] - e[26] * e[21] * e[3] - e[26] * e[19] * e[1] -
        e[26] * e[22] * e[4] - e[26] * e[18] * e[0] + e[26] * e[25] * e[7] +
        e[26] * e[24] * e[6] + e[2] * e[19] * e[25] + e[2] * e[18] * e[24] +
        e[5] * e[22] * e[25] + e[5] * e[21] * e[24];
    a[175] =
        e[19] * e[27] * e[21] + e[19] * e[18] * e[30] + e[19] * e[28] * e[22] +
        e[19] * e[29] * e[23] + e[19] * e[20] * e[32] + e[28] * e[18] * e[21] +
        e[28] * e[20] * e[23] + e[22] * e[30] * e[21] + e[22] * e[32] * e[23] +
        e[25] * e[30] * e[24] + e[25] * e[21] * e[33] + e[25] * e[22] * e[34] +
        e[25] * e[32] * e[26] + e[25] * e[23] * e[35] + e[34] * e[21] * e[24] +
        e[34] * e[23] * e[26] - e[22] * e[27] * e[18] - e[22] * e[33] * e[24] -
        e[22] * e[29] * e[20] - e[22] * e[35] * e[26] + 0.5 * e2[19] * e[31] +
        1.5 * e[31] * e2[22] + 0.5 * e[31] * e2[21] + 0.5 * e[31] * e2[23] +
        0.5 * e[31] * e2[25] - 0.5 * e[31] * e2[26] - 0.5 * e[31] * e2[18] -
        0.5 * e[31] * e2[20] - 0.5 * e[31] * e2[24];
    a[15] = -0.5 * e[13] * e2[15] + 0.5 * e[13] * e2[16] + 0.5 * e[13] * e2[12] +
        e[16] * e[12] * e[15] + 0.5 * e3[13] + e[10] * e[11] * e[14] +
        0.5 * e[13] * e2[14] - 0.5 * e[13] * e2[17] - 0.5 * e[13] * e2[11] -
        0.5 * e[13] * e2[9] + 0.5 * e2[10] * e[13] + e[10] * e[9] * e[12] +
        e[16] * e[14] * e[17];
    a[95] = -e[13] * e[29] * e[2] - e[31] * e[11] * e[2] - e[31] * e[15] * e[6] -
        e[31] * e[9] * e[0] + e[31] * e[14] * e[5] + e[31] * e[12] * e[3] -
        e[31] * e[17] * e[8] + e[16] * e[30] * e[6] + e[16] * e[3] * e[33] +
        e[16] * e[4] * e[34] + e[16] * e[32] * e[8] + e[16] * e[5] * e[35] -
        e[4] * e[27] * e[9] + e[4] * e[28] * e[10] - e[4] * e[33] * e[15] -
        e[4] * e[35] * e[17] - e[4] * e[29] * e[11] + e[34] * e[12] * e[6] +
        e[34] * e[3] * e[15] + e[34] * e[14] * e[8] + e[34] * e[5] * e[17] +
        e[10] * e[27] * e[3] + e[10] * e[0] * e[30] + e[10] * e[29] * e[5] +
        e[10] * e[2] * e[32] + e[28] * e[9] * e[3] + e[28] * e[0] * e[12] +
        e[28] * e[11] * e[5] + e[28] * e[2] * e[14] + e[4] * e[30] * e[12] +
        e[4] * e[32] * e[14] + 3. * e[4] * e[31] * e[13] +
        e[7] * e[30] * e[15] + e[7] * e[12] * e[33] + e[7] * e[32] * e[17] +
        e[7] * e[14] * e[35] + e[7] * e[31] * e[16] + e[7] * e[13] * e[34] +
        e[1] * e[27] * e[12] + e[1] * e[9] * e[30] + e[1] * e[29] * e[14] +
        e[1] * e[11] * e[32] + e[1] * e[28] * e[13] + e[1] * e[10] * e[31] -
        e[13] * e[27] * e[0] + e[13] * e[32] * e[5] - e[13] * e[33] * e[6] +
        e[13] * e[30] * e[3] - e[13] * e[35] * e[8];
    a[165] = e[25] * e[23] * e[26] + e[19] * e[20] * e[23] +
        e[19] * e[18] * e[21] + e[25] * e[21] * e[24] + 0.5 * e3[22] +
        0.5 * e[22] * e2[23] + 0.5 * e2[19] * e[22] - 0.5 * e[22] * e2[18] -
        0.5 * e[22] * e2[24] + 0.5 * e[22] * e2[21] + 0.5 * e[22] * e2[25] -
        0.5 * e[22] * e2[20] - 0.5 * e[22] * e2[26];
    a[55] = e[34] * e[5] * e[8] + e[1] * e[27] * e[3] + e[1] * e[0] * e[30] +
        e[1] * e[28] * e[4] + e[1] * e[29] * e[5] + e[1] * e[2] * e[32] -
        e[4] * e[27] * e[0] + e[4] * e[34] * e[7] + e[4] * e[32] * e[5] -
        e[4] * e[33] * e[6] + e[4] * e[30] * e[3] - e[4] * e[35] * e[8] -
        e[4] * e[29] * e[2] + e[28] * e[0] * e[3] + e[28] * e[2] * e[5] +
        e[7] * e[30] * e[6] + e[7] * e[3] * e[33] + e[7] * e[32] * e[8] +
        e[7] * e[5] * e[35] + e[34] * e[3] * e[6] + 0.5 * e2[1] * e[31] +
        1.5 * e[31] * e2[4] - 0.5 * e[31] * e2[0] - 0.5 * e[31] * e2[6] +
        0.5 * e[31] * e2[5] + 0.5 * e[31] * e2[7] + 0.5 * e[31] * e2[3] -
        0.5 * e[31] * e2[2] - 0.5 * e[31] * e2[8];
    a[85] = e[1] * e[20] * e[14] + e[1] * e[11] * e[23] + e[13] * e[21] * e[3] -
        e[13] * e[26] * e[8] - e[13] * e[20] * e[2] - e[13] * e[18] * e[0] +
        e[13] * e[23] * e[5] - e[13] * e[24] * e[6] - e[22] * e[11] * e[2] -
        e[22] * e[15] * e[6] - e[22] * e[9] * e[0] + e[22] * e[14] * e[5] +
        e[22] * e[12] * e[3] - e[22] * e[17] * e[8] + e[16] * e[21] * e[6] +
        e[16] * e[3] * e[24] + e[16] * e[4] * e[25] + e[16] * e[23] * e[8] +
        e[16] * e[5] * e[26] - e[4] * e[24] * e[15] - e[4] * e[26] * e[17] -
        e[4] * e[20] * e[11] - e[4] * e[18] * e[9] + e[25] * e[12] * e[6] +
        e[25] * e[3] * e[15] + e[25] * e[14] * e[8] + e[25] * e[5] * e[17] +
        e[10] * e[18] * e[3] + e[10] * e[0] * e[21] + e[10] * e[19] * e[4] +
        e[10] * e[1] * e[22] + e[10] * e[20] * e[5] + e[10] * e[2] * e[23] +
        e[19] * e[9] * e[3] + e[19] * e[0] * e[12] + e[19] * e[1] * e[13] +
        e[19] * e[11] * e[5] + e[19] * e[2] * e[14] + e[4] * e[21] * e[12] +
        e[4] * e[23] * e[14] + 3. * e[4] * e[22] * e[13] +
        e[7] * e[21] * e[15] + e[7] * e[12] * e[24] + e[7] * e[23] * e[17] +
        e[7] * e[14] * e[26] + e[7] * e[22] * e[16] + e[7] * e[13] * e[25] +
        e[1] * e[18] * e[12] + e[1] * e[9] * e[21];
    a[75] =
        e[10] * e[27] * e[12] + e[10] * e[9] * e[30] + e[10] * e[29] * e[14] +
        e[10] * e[11] * e[32] + e[10] * e[28] * e[13] + e[28] * e[11] * e[14] +
        e[28] * e[9] * e[12] + e[13] * e[30] * e[12] + e[13] * e[32] * e[14] +
        e[16] * e[30] * e[15] + e[16] * e[12] * e[33] + e[16] * e[32] * e[17] +
        e[16] * e[14] * e[35] + e[16] * e[13] * e[34] + e[34] * e[14] * e[17] +
        e[34] * e[12] * e[15] - e[13] * e[27] * e[9] - e[13] * e[33] * e[15] -
        e[13] * e[35] * e[17] - e[13] * e[29] * e[11] + 0.5 * e2[10] * e[31] +
        0.5 * e[31] * e2[16] - 0.5 * e[31] * e2[9] - 0.5 * e[31] * e2[11] +
        0.5 * e[31] * e2[12] - 0.5 * e[31] * e2[15] - 0.5 * e[31] * e2[17] +
        0.5 * e[31] * e2[14] + 1.5 * e[31] * e2[13];
    a[5] = -0.5 * e[4] * e2[6] - 0.5 * e[4] * e2[0] + e[1] * e[2] * e[5] +
        0.5 * e[4] * e2[7] + e[1] * e[0] * e[3] + e[7] * e[5] * e[8] -
        0.5 * e[4] * e2[8] + 0.5 * e[4] * e2[3] + 0.5 * e[4] * e2[5] +
        e[7] * e[3] * e[6] - 0.5 * e[4] * e2[2] + 0.5 * e3[4] +
        0.5 * e2[1] * e[4];
    a[195] = e[34] * e[32] * e[35] - 0.5 * e[31] * e2[35] + 0.5 * e[31] * e2[34] +
        0.5 * e2[28] * e[31] + 0.5 * e3[31] + 0.5 * e[31] * e2[32] +
        e[34] * e[30] * e[33] - 0.5 * e[31] * e2[27] + 0.5 * e[31] * e2[30] -
        0.5 * e[31] * e2[33] - 0.5 * e[31] * e2[29] + e[28] * e[29] * e[32] +
        e[28] * e[27] * e[30];
    a[125] = e[1] * e[27] * e[30] + e[1] * e[29] * e[32] + e[1] * e[28] * e[31] +
        e[31] * e[30] * e[3] + e[31] * e[32] * e[5] + e[7] * e[30] * e[33] +
        e[7] * e[32] * e[35] + e[7] * e[31] * e[34] + e[28] * e[27] * e[3] +
        e[28] * e[0] * e[30] + e[28] * e[29] * e[5] + e[28] * e[2] * e[32] +
        e[34] * e[30] * e[6] + e[34] * e[3] * e[33] + e[34] * e[32] * e[8] +
        e[34] * e[5] * e[35] - e[31] * e[27] * e[0] - e[31] * e[33] * e[6] -
        e[31] * e[35] * e[8] - e[31] * e[29] * e[2] + 0.5 * e[4] * e2[30] +
        0.5 * e[4] * e2[32] + 1.5 * e[4] * e2[31] - 0.5 * e[4] * e2[27] +
        0.5 * e[4] * e2[28] - 0.5 * e[4] * e2[29] - 0.5 * e[4] * e2[33] +
        0.5 * e[4] * e2[34] - 0.5 * e[4] * e2[35];
    a[185] =
        0.5 * e[22] * e2[30] + 0.5 * e[22] * e2[32] + 1.5 * e[22] * e2[31] +
        0.5 * e[22] * e2[34] - 0.5 * e[22] * e2[27] - 0.5 * e[22] * e2[29] -
        0.5 * e[22] * e2[33] - 0.5 * e[22] * e2[35] + e[28] * e[18] * e[30] +
        e[28] * e[29] * e[23] + e[28] * e[20] * e[32] + e[31] * e[30] * e[21] +
        e[31] * e[32] * e[23] + e[25] * e[30] * e[33] + e[25] * e[32] * e[35] +
        e[25] * e[31] * e[34] + e[34] * e[30] * e[24] + e[34] * e[21] * e[33] +
        e[34] * e[32] * e[26] + e[34] * e[23] * e[35] - e[31] * e[27] * e[18] -
        e[31] * e[33] * e[24] - e[31] * e[29] * e[20] - e[31] * e[35] * e[26] +
        e[19] * e[27] * e[30] + e[19] * e[29] * e[32] + e[19] * e[28] * e[31] +
        e[28] * e[27] * e[21] + 0.5 * e2[28] * e[22];
    a[155] =
        e[16] * e[30] * e[33] + e[16] * e[32] * e[35] + e[10] * e[27] * e[30] +
        e[10] * e[29] * e[32] + e[10] * e[28] * e[31] + e[34] * e[30] * e[15] +
        e[34] * e[12] * e[33] + e[34] * e[32] * e[17] + e[34] * e[14] * e[35] +
        e[34] * e[31] * e[16] + e[28] * e[27] * e[12] + e[28] * e[9] * e[30] +
        e[28] * e[29] * e[14] + e[28] * e[11] * e[32] - e[31] * e[27] * e[9] +
        e[31] * e[30] * e[12] + e[31] * e[32] * e[14] - e[31] * e[33] * e[15] -
        e[31] * e[35] * e[17] - e[31] * e[29] * e[11] - 0.5 * e[13] * e2[27] +
        0.5 * e[13] * e2[32] + 0.5 * e[13] * e2[28] - 0.5 * e[13] * e2[29] +
        1.5 * e[13] * e2[31] - 0.5 * e[13] * e2[33] + 0.5 * e[13] * e2[30] +
        0.5 * e[13] * e2[34] - 0.5 * e[13] * e2[35];
    a[134] =
        e[21] * e[23] * e[14] + e[21] * e[22] * e[13] + e[24] * e[21] * e[15] +
        e[24] * e[23] * e[17] + e[24] * e[14] * e[26] + e[24] * e[22] * e[16] +
        e[24] * e[13] * e[25] + e[15] * e[22] * e[25] + e[15] * e[23] * e[26] +
        e[9] * e[19] * e[22] + e[9] * e[18] * e[21] + e[9] * e[20] * e[23] +
        e[18] * e[20] * e[14] + e[18] * e[11] * e[23] + e[18] * e[19] * e[13] +
        e[18] * e[10] * e[22] - e[21] * e[25] * e[16] - e[21] * e[26] * e[17] -
        e[21] * e[20] * e[11] - e[21] * e[19] * e[10] + 1.5 * e2[21] * e[12] +
        0.5 * e[12] * e2[24] - 0.5 * e[12] * e2[26] + 0.5 * e[12] * e2[18] +
        0.5 * e[12] * e2[23] - 0.5 * e[12] * e2[19] - 0.5 * e[12] * e2[20] +
        0.5 * e[12] * e2[22] - 0.5 * e[12] * e2[25];
    a[144] =
        -e[12] * e[29] * e[20] - e[12] * e[35] * e[26] - e[12] * e[28] * e[19] -
        e[12] * e[34] * e[25] + e[18] * e[29] * e[14] + e[18] * e[11] * e[32] +
        e[18] * e[28] * e[13] + e[18] * e[10] * e[31] + e[27] * e[20] * e[14] +
        e[27] * e[11] * e[23] + e[27] * e[19] * e[13] + e[27] * e[10] * e[22] +
        e[15] * e[30] * e[24] + e[15] * e[21] * e[33] + e[15] * e[31] * e[25] +
        e[15] * e[22] * e[34] + e[15] * e[32] * e[26] + e[15] * e[23] * e[35] -
        e[21] * e[28] * e[10] - e[21] * e[34] * e[16] - e[21] * e[35] * e[17] -
        e[21] * e[29] * e[11] - e[30] * e[25] * e[16] - e[30] * e[26] * e[17] -
        e[30] * e[20] * e[11] - e[30] * e[19] * e[10] + e[24] * e[32] * e[17] +
        e[24] * e[14] * e[35] + e[24] * e[31] * e[16] + e[24] * e[13] * e[34] +
        e[33] * e[23] * e[17] + e[33] * e[14] * e[26] + e[33] * e[22] * e[16] +
        e[33] * e[13] * e[25] + 3. * e[12] * e[30] * e[21] +
        e[12] * e[31] * e[22] + e[12] * e[32] * e[23] + e[9] * e[27] * e[21] +
        e[9] * e[18] * e[30] + e[9] * e[28] * e[22] + e[9] * e[19] * e[31] +
        e[9] * e[29] * e[23] + e[9] * e[20] * e[32] + e[21] * e[32] * e[14] +
        e[21] * e[31] * e[13] + e[30] * e[23] * e[14] + e[30] * e[22] * e[13] +
        e[12] * e[27] * e[18] + e[12] * e[33] * e[24];
    a[24] = e[0] * e[11] * e[5] + e[0] * e[2] * e[14] + e[9] * e[1] * e[4] +
        e[9] * e[0] * e[3] + e[9] * e[2] * e[5] + e[3] * e[13] * e[4] +
        e[3] * e[14] * e[5] + e[6] * e[3] * e[15] + e[6] * e[13] * e[7] +
        e[6] * e[4] * e[16] + e[6] * e[14] * e[8] + e[6] * e[5] * e[17] +
        e[15] * e[4] * e[7] + e[15] * e[5] * e[8] - e[3] * e[11] * e[2] -
        e[3] * e[10] * e[1] - e[3] * e[16] * e[7] - e[3] * e[17] * e[8] +
        e[0] * e[10] * e[4] + e[0] * e[1] * e[13] + 1.5 * e[12] * e2[3] +
        0.5 * e[12] * e2[4] + 0.5 * e[12] * e2[5] + 0.5 * e[12] * e2[6] +
        0.5 * e2[0] * e[12] - 0.5 * e[12] * e2[1] - 0.5 * e[12] * e2[7] -
        0.5 * e[12] * e2[2] - 0.5 * e[12] * e2[8];
    a[104] = e[21] * e[24] * e[6] + e[0] * e[19] * e[22] + e[0] * e[20] * e[23] +
        e[24] * e[22] * e[7] + e[24] * e[4] * e[25] + e[24] * e[23] * e[8] +
        e[24] * e[5] * e[26] + e[6] * e[22] * e[25] + e[6] * e[23] * e[26] +
        e[18] * e[0] * e[21] + e[18] * e[19] * e[4] + e[18] * e[1] * e[22] +
        e[18] * e[20] * e[5] + e[18] * e[2] * e[23] + e[21] * e[22] * e[4] +
        e[21] * e[23] * e[5] - e[21] * e[26] * e[8] - e[21] * e[20] * e[2] -
        e[21] * e[19] * e[1] - e[21] * e[25] * e[7] + 1.5 * e2[21] * e[3] +
        0.5 * e[3] * e2[22] + 0.5 * e[3] * e2[23] + 0.5 * e[3] * e2[24] -
        0.5 * e[3] * e2[26] - 0.5 * e[3] * e2[19] - 0.5 * e[3] * e2[20] -
        0.5 * e[3] * e2[25] + 0.5 * e2[18] * e[3];
    a[76] =
        e[11] * e[27] * e[12] + e[11] * e[9] * e[30] + e[11] * e[29] * e[14] +
        e[11] * e[28] * e[13] + e[11] * e[10] * e[31] + e[29] * e[9] * e[12] +
        e[29] * e[10] * e[13] + e[14] * e[30] * e[12] + e[14] * e[31] * e[13] +
        e[17] * e[30] * e[15] + e[17] * e[12] * e[33] + e[17] * e[14] * e[35] +
        e[17] * e[31] * e[16] + e[17] * e[13] * e[34] + e[35] * e[12] * e[15] +
        e[35] * e[13] * e[16] - e[14] * e[27] * e[9] - e[14] * e[28] * e[10] -
        e[14] * e[33] * e[15] - e[14] * e[34] * e[16] + 0.5 * e2[11] * e[32] -
        0.5 * e[32] * e2[16] - 0.5 * e[32] * e2[9] + 0.5 * e[32] * e2[12] -
        0.5 * e[32] * e2[15] + 0.5 * e[32] * e2[17] - 0.5 * e[32] * e2[10] +
        1.5 * e[32] * e2[14] + 0.5 * e[32] * e2[13];
    a[6] = e[8] * e[3] * e[6] + 0.5 * e2[2] * e[5] - 0.5 * e[5] * e2[0] +
        0.5 * e[5] * e2[4] - 0.5 * e[5] * e2[6] + 0.5 * e[5] * e2[8] +
        e[8] * e[4] * e[7] + 0.5 * e3[5] + e[2] * e[0] * e[3] +
        0.5 * e[5] * e2[3] - 0.5 * e[5] * e2[7] + e[2] * e[1] * e[4] -
        0.5 * e[5] * e2[1];
    a[56] = e[2] * e[27] * e[3] + e[2] * e[0] * e[30] + e[2] * e[28] * e[4] +
        e[2] * e[1] * e[31] + e[2] * e[29] * e[5] - e[5] * e[27] * e[0] -
        e[5] * e[34] * e[7] - e[5] * e[33] * e[6] + e[5] * e[30] * e[3] +
        e[5] * e[35] * e[8] - e[5] * e[28] * e[1] + e[5] * e[31] * e[4] +
        e[29] * e[1] * e[4] + e[29] * e[0] * e[3] + e[8] * e[30] * e[6] +
        e[8] * e[3] * e[33] + e[8] * e[31] * e[7] + e[8] * e[4] * e[34] +
        e[35] * e[4] * e[7] + e[35] * e[3] * e[6] + 0.5 * e2[2] * e[32] +
        1.5 * e[32] * e2[5] + 0.5 * e[32] * e2[4] - 0.5 * e[32] * e2[0] -
        0.5 * e[32] * e2[6] - 0.5 * e[32] * e2[1] - 0.5 * e[32] * e2[7] +
        0.5 * e[32] * e2[3] + 0.5 * e[32] * e2[8];
    a[86] = -e[14] * e[19] * e[1] + e[14] * e[22] * e[4] - e[14] * e[18] * e[0] -
        e[14] * e[25] * e[7] - e[14] * e[24] * e[6] - e[23] * e[10] * e[1] +
        e[23] * e[13] * e[4] - e[23] * e[16] * e[7] - e[23] * e[15] * e[6] -
        e[23] * e[9] * e[0] + e[23] * e[12] * e[3] + e[17] * e[21] * e[6] +
        e[17] * e[3] * e[24] + e[17] * e[22] * e[7] + e[17] * e[4] * e[25] +
        e[17] * e[5] * e[26] - e[5] * e[24] * e[15] - e[5] * e[25] * e[16] -
        e[5] * e[18] * e[9] - e[5] * e[19] * e[10] + e[26] * e[12] * e[6] +
        e[26] * e[3] * e[15] + e[26] * e[13] * e[7] + e[26] * e[4] * e[16] +
        e[11] * e[18] * e[3] + e[11] * e[0] * e[21] + e[11] * e[19] * e[4] +
        e[11] * e[1] * e[22] + e[11] * e[20] * e[5] + e[11] * e[2] * e[23] +
        e[20] * e[9] * e[3] + e[20] * e[0] * e[12] + e[20] * e[10] * e[4] +
        e[20] * e[1] * e[13] + e[20] * e[2] * e[14] + e[5] * e[21] * e[12] +
        3. * e[5] * e[23] * e[14] + e[5] * e[22] * e[13] +
        e[8] * e[21] * e[15] + e[8] * e[12] * e[24] + e[8] * e[23] * e[17] +
        e[8] * e[14] * e[26] + e[8] * e[22] * e[16] + e[8] * e[13] * e[25] +
        e[2] * e[18] * e[12] + e[2] * e[9] * e[21] + e[2] * e[19] * e[13] +
        e[2] * e[10] * e[22] + e[14] * e[21] * e[3];
    a[156] =
        -0.5 * e[14] * e2[27] + 1.5 * e[14] * e2[32] - 0.5 * e[14] * e2[28] +
        0.5 * e[14] * e2[29] + 0.5 * e[14] * e2[31] - 0.5 * e[14] * e2[33] +
        0.5 * e[14] * e2[30] - 0.5 * e[14] * e2[34] + 0.5 * e[14] * e2[35] +
        e[11] * e[27] * e[30] + e[11] * e[29] * e[32] + e[11] * e[28] * e[31] +
        e[35] * e[30] * e[15] + e[35] * e[12] * e[33] + e[35] * e[32] * e[17] +
        e[35] * e[31] * e[16] + e[35] * e[13] * e[34] + e[29] * e[27] * e[12] +
        e[29] * e[9] * e[30] + e[29] * e[28] * e[13] + e[29] * e[10] * e[31] -
        e[32] * e[27] * e[9] + e[32] * e[30] * e[12] - e[32] * e[28] * e[10] +
        e[32] * e[31] * e[13] - e[32] * e[33] * e[15] - e[32] * e[34] * e[16] +
        e[17] * e[30] * e[33] + e[17] * e[31] * e[34];
    a[186] =
        -0.5 * e[23] * e2[33] - 0.5 * e[23] * e2[34] + 0.5 * e2[29] * e[23] +
        0.5 * e[23] * e2[30] + 1.5 * e[23] * e2[32] + 0.5 * e[23] * e2[31] +
        0.5 * e[23] * e2[35] - 0.5 * e[23] * e2[27] - 0.5 * e[23] * e2[28] +
        e[32] * e[30] * e[21] + e[32] * e[31] * e[22] + e[26] * e[30] * e[33] +
        e[26] * e[32] * e[35] + e[26] * e[31] * e[34] + e[35] * e[30] * e[24] +
        e[35] * e[21] * e[33] + e[35] * e[31] * e[25] + e[35] * e[22] * e[34] -
        e[32] * e[27] * e[18] - e[32] * e[33] * e[24] - e[32] * e[28] * e[19] -
        e[32] * e[34] * e[25] + e[20] * e[27] * e[30] + e[20] * e[29] * e[32] +
        e[20] * e[28] * e[31] + e[29] * e[27] * e[21] + e[29] * e[18] * e[30] +
        e[29] * e[28] * e[22] + e[29] * e[19] * e[31];
    a[126] = e[2] * e[27] * e[30] + e[2] * e[29] * e[32] + e[2] * e[28] * e[31] +
        e[32] * e[30] * e[3] + e[32] * e[31] * e[4] + e[8] * e[30] * e[33] +
        e[8] * e[32] * e[35] + e[8] * e[31] * e[34] + e[29] * e[27] * e[3] +
        e[29] * e[0] * e[30] + e[29] * e[28] * e[4] + e[29] * e[1] * e[31] +
        e[35] * e[30] * e[6] + e[35] * e[3] * e[33] + e[35] * e[31] * e[7] +
        e[35] * e[4] * e[34] - e[32] * e[27] * e[0] - e[32] * e[34] * e[7] -
        e[32] * e[33] * e[6] - e[32] * e[28] * e[1] + 0.5 * e[5] * e2[30] +
        1.5 * e[5] * e2[32] + 0.5 * e[5] * e2[31] - 0.5 * e[5] * e2[27] -
        0.5 * e[5] * e2[28] + 0.5 * e[5] * e2[29] - 0.5 * e[5] * e2[33] -
        0.5 * e[5] * e2[34] + 0.5 * e[5] * e2[35];
    a[196] = 0.5 * e[32] * e2[31] + 0.5 * e[32] * e2[35] - 0.5 * e[32] * e2[27] +
        e[29] * e[27] * e[30] + e[29] * e[28] * e[31] +
        e[35] * e[30] * e[33] + e[35] * e[31] * e[34] +
        0.5 * e2[29] * e[32] + 0.5 * e3[32] - 0.5 * e[32] * e2[33] -
        0.5 * e[32] * e2[34] + 0.5 * e[32] * e2[30] - 0.5 * e[32] * e2[28];
    a[25] = e[10] * e[1] * e[4] + e[10] * e[0] * e[3] + e[10] * e[2] * e[5] +
        e[4] * e[12] * e[3] + e[4] * e[14] * e[5] + e[7] * e[12] * e[6] +
        e[7] * e[3] * e[15] + e[7] * e[4] * e[16] + e[7] * e[14] * e[8] +
        e[7] * e[5] * e[17] + e[16] * e[3] * e[6] + e[16] * e[5] * e[8] -
        e[4] * e[11] * e[2] - e[4] * e[15] * e[6] - e[4] * e[9] * e[0] -
        e[4] * e[17] * e[8] + e[1] * e[9] * e[3] + e[1] * e[0] * e[12] +
        e[1] * e[11] * e[5] + e[1] * e[2] * e[14] + 1.5 * e[13] * e2[4] +
        0.5 * e[13] * e2[3] + 0.5 * e[13] * e2[5] + 0.5 * e[13] * e2[7] +
        0.5 * e2[1] * e[13] - 0.5 * e[13] * e2[0] - 0.5 * e[13] * e2[6] -
        0.5 * e[13] * e2[2] - 0.5 * e[13] * e2[8];
    a[105] = e[25] * e[21] * e[6] + e[25] * e[3] * e[24] + e[25] * e[23] * e[8] +
        e[25] * e[5] * e[26] + e[7] * e[21] * e[24] + e[7] * e[23] * e[26] +
        e[19] * e[18] * e[3] + e[19] * e[0] * e[21] + e[19] * e[1] * e[22] +
        e[19] * e[20] * e[5] + e[19] * e[2] * e[23] + e[22] * e[21] * e[3] +
        e[22] * e[23] * e[5] - e[22] * e[26] * e[8] - e[22] * e[20] * e[2] -
        e[22] * e[18] * e[0] + e[22] * e[25] * e[7] - e[22] * e[24] * e[6] +
        e[1] * e[18] * e[21] + e[1] * e[20] * e[23] + 0.5 * e[4] * e2[25] -
        0.5 * e[4] * e2[26] - 0.5 * e[4] * e2[18] - 0.5 * e[4] * e2[20] -
        0.5 * e[4] * e2[24] + 0.5 * e2[19] * e[4] + 1.5 * e2[22] * e[4] +
        0.5 * e[4] * e2[21] + 0.5 * e[4] * e2[23];
    a[135] =
        e[22] * e[21] * e[12] + e[22] * e[23] * e[14] + e[25] * e[21] * e[15] +
        e[25] * e[12] * e[24] + e[25] * e[23] * e[17] + e[25] * e[14] * e[26] +
        e[25] * e[22] * e[16] + e[16] * e[21] * e[24] + e[16] * e[23] * e[26] +
        e[10] * e[19] * e[22] + e[10] * e[18] * e[21] + e[10] * e[20] * e[23] +
        e[19] * e[18] * e[12] + e[19] * e[9] * e[21] + e[19] * e[20] * e[14] +
        e[19] * e[11] * e[23] - e[22] * e[24] * e[15] - e[22] * e[26] * e[17] -
        e[22] * e[20] * e[11] - e[22] * e[18] * e[9] - 0.5 * e[13] * e2[26] -
        0.5 * e[13] * e2[18] + 0.5 * e[13] * e2[23] + 0.5 * e[13] * e2[19] -
        0.5 * e[13] * e2[20] - 0.5 * e[13] * e2[24] + 0.5 * e[13] * e2[21] +
        1.5 * e2[22] * e[13] + 0.5 * e[13] * e2[25];
    a[145] =
        e[13] * e[30] * e[21] + 3. * e[13] * e[31] * e[22] +
        e[13] * e[32] * e[23] + e[10] * e[27] * e[21] + e[10] * e[18] * e[30] +
        e[10] * e[28] * e[22] + e[10] * e[19] * e[31] + e[10] * e[29] * e[23] +
        e[10] * e[20] * e[32] + e[22] * e[30] * e[12] + e[22] * e[32] * e[14] +
        e[31] * e[21] * e[12] + e[31] * e[23] * e[14] - e[13] * e[27] * e[18] -
        e[13] * e[33] * e[24] - e[13] * e[29] * e[20] - e[13] * e[35] * e[26] +
        e[13] * e[28] * e[19] + e[13] * e[34] * e[25] + e[19] * e[27] * e[12] +
        e[19] * e[9] * e[30] + e[19] * e[29] * e[14] + e[19] * e[11] * e[32] +
        e[28] * e[18] * e[12] + e[28] * e[9] * e[21] + e[28] * e[20] * e[14] +
        e[28] * e[11] * e[23] + e[16] * e[30] * e[24] + e[16] * e[21] * e[33] +
        e[16] * e[31] * e[25] + e[16] * e[22] * e[34] + e[16] * e[32] * e[26] +
        e[16] * e[23] * e[35] - e[22] * e[27] * e[9] - e[22] * e[33] * e[15] -
        e[22] * e[35] * e[17] - e[22] * e[29] * e[11] - e[31] * e[24] * e[15] -
        e[31] * e[26] * e[17] - e[31] * e[20] * e[11] - e[31] * e[18] * e[9] +
        e[25] * e[30] * e[15] + e[25] * e[12] * e[33] + e[25] * e[32] * e[17] +
        e[25] * e[14] * e[35] + e[34] * e[21] * e[15] + e[34] * e[12] * e[24] +
        e[34] * e[23] * e[17] + e[34] * e[14] * e[26];
    a[65] =
        e[19] * e[11] * e[14] + e[19] * e[9] * e[12] + e[19] * e[10] * e[13] +
        e[13] * e[21] * e[12] + e[13] * e[23] * e[14] + e[16] * e[21] * e[15] +
        e[16] * e[12] * e[24] + e[16] * e[23] * e[17] + e[16] * e[14] * e[26] +
        e[16] * e[13] * e[25] + e[25] * e[14] * e[17] + e[25] * e[12] * e[15] -
        e[13] * e[24] * e[15] - e[13] * e[26] * e[17] - e[13] * e[20] * e[11] -
        e[13] * e[18] * e[9] + e[10] * e[18] * e[12] + e[10] * e[9] * e[21] +
        e[10] * e[20] * e[14] + e[10] * e[11] * e[23] + 1.5 * e[22] * e2[13] +
        0.5 * e[22] * e2[14] + 0.5 * e[22] * e2[12] + 0.5 * e[22] * e2[16] +
        0.5 * e2[10] * e[22] - 0.5 * e[22] * e2[9] - 0.5 * e[22] * e2[11] -
        0.5 * e[22] * e2[15] - 0.5 * e[22] * e2[17];
    a[35] = e[13] * e[12] * e[3] + e[13] * e[14] * e[5] + e[16] * e[12] * e[6] +
        e[16] * e[3] * e[15] + e[16] * e[13] * e[7] + e[16] * e[14] * e[8] +
        e[16] * e[5] * e[17] + e[7] * e[14] * e[17] + e[7] * e[12] * e[15] +
        e[1] * e[11] * e[14] + e[1] * e[9] * e[12] + e[1] * e[10] * e[13] +
        e[10] * e[9] * e[3] + e[10] * e[0] * e[12] + e[10] * e[11] * e[5] +
        e[10] * e[2] * e[14] - e[13] * e[11] * e[2] - e[13] * e[15] * e[6] -
        e[13] * e[9] * e[0] - e[13] * e[17] * e[8] + 1.5 * e2[13] * e[4] +
        0.5 * e[4] * e2[16] - 0.5 * e[4] * e2[9] - 0.5 * e[4] * e2[11] +
        0.5 * e[4] * e2[12] - 0.5 * e[4] * e2[15] - 0.5 * e[4] * e2[17] +
        0.5 * e[4] * e2[10] + 0.5 * e[4] * e2[14];
    a[45] = e[19] * e[1] * e[4] + e[19] * e[0] * e[3] + e[19] * e[2] * e[5] +
        e[4] * e[21] * e[3] + e[4] * e[23] * e[5] + e[7] * e[21] * e[6] +
        e[7] * e[3] * e[24] + e[7] * e[4] * e[25] + e[7] * e[23] * e[8] +
        e[7] * e[5] * e[26] + e[25] * e[3] * e[6] + e[25] * e[5] * e[8] +
        e[1] * e[18] * e[3] + e[1] * e[0] * e[21] + e[1] * e[20] * e[5] +
        e[1] * e[2] * e[23] - e[4] * e[26] * e[8] - e[4] * e[20] * e[2] -
        e[4] * e[18] * e[0] - e[4] * e[24] * e[6] + 1.5 * e[22] * e2[4] -
        0.5 * e[22] * e2[0] - 0.5 * e[22] * e2[6] + 0.5 * e[22] * e2[5] +
        0.5 * e[22] * e2[1] + 0.5 * e[22] * e2[7] + 0.5 * e[22] * e2[3] -
        0.5 * e[22] * e2[2] - 0.5 * e[22] * e2[8];
    a[115] = -e[31] * e[20] * e[2] - e[31] * e[18] * e[0] + e[31] * e[23] * e[5] -
        e[31] * e[24] * e[6] + e[7] * e[30] * e[24] + e[7] * e[21] * e[33] +
        e[7] * e[32] * e[26] + e[7] * e[23] * e[35] + e[25] * e[30] * e[6] +
        e[25] * e[3] * e[33] + e[25] * e[31] * e[7] + e[25] * e[4] * e[34] +
        e[25] * e[32] * e[8] + e[25] * e[5] * e[35] + e[34] * e[21] * e[6] +
        e[34] * e[3] * e[24] + e[34] * e[22] * e[7] + e[34] * e[23] * e[8] +
        e[34] * e[5] * e[26] + e[1] * e[27] * e[21] + e[1] * e[18] * e[30] +
        e[1] * e[28] * e[22] + e[1] * e[19] * e[31] + e[1] * e[29] * e[23] +
        e[1] * e[20] * e[32] + e[19] * e[27] * e[3] + e[19] * e[0] * e[30] +
        e[19] * e[28] * e[4] + e[19] * e[29] * e[5] + e[19] * e[2] * e[32] +
        e[28] * e[18] * e[3] + e[28] * e[0] * e[21] + e[28] * e[20] * e[5] +
        e[28] * e[2] * e[23] + e[4] * e[30] * e[21] +
        3. * e[4] * e[31] * e[22] + e[4] * e[32] * e[23] -
        e[4] * e[27] * e[18] - e[4] * e[33] * e[24] - e[4] * e[29] * e[20] -
        e[4] * e[35] * e[26] - e[22] * e[27] * e[0] + e[22] * e[32] * e[5] -
        e[22] * e[33] * e[6] + e[22] * e[30] * e[3] - e[22] * e[35] * e[8] -
        e[22] * e[29] * e[2] + e[31] * e[21] * e[3] - e[31] * e[26] * e[8];
}
void CEssentialMatrixEstimator_5Points::CalculateCoeffs(const Eigen::Matrix<double, 13, 3>& B, Eigen::Matrix<double, 11, 1>& coeffs) const
{
    const double* b = B.data();
    coeffs(0) = b[0] * b[17] * b[34] + b[26] * b[4] * b[21] -
        b[26] * b[17] * b[8] - b[13] * b[4] * b[34] -
        b[0] * b[21] * b[30] + b[13] * b[30] * b[8];
    coeffs(1) =
        b[26] * b[4] * b[22] + b[14] * b[30] * b[8] + b[13] * b[31] * b[8] +
        b[1] * b[17] * b[34] - b[13] * b[5] * b[34] + b[26] * b[5] * b[21] -
        b[0] * b[21] * b[31] - b[26] * b[17] * b[9] - b[1] * b[21] * b[30] +
        b[27] * b[4] * b[21] + b[0] * b[17] * b[35] - b[0] * b[22] * b[30] +
        b[13] * b[30] * b[9] + b[0] * b[18] * b[34] - b[27] * b[17] * b[8] -
        b[14] * b[4] * b[34] - b[13] * b[4] * b[35] - b[26] * b[18] * b[8];
    coeffs(2) =
        b[14] * b[30] * b[9] + b[14] * b[31] * b[8] + b[13] * b[31] * b[9] -
        b[13] * b[4] * b[36] - b[13] * b[5] * b[35] + b[15] * b[30] * b[8] -
        b[13] * b[6] * b[34] + b[13] * b[30] * b[10] + b[13] * b[32] * b[8] -
        b[14] * b[4] * b[35] - b[14] * b[5] * b[34] + b[26] * b[4] * b[23] +
        b[26] * b[5] * b[22] + b[26] * b[6] * b[21] - b[26] * b[17] * b[10] -
        b[15] * b[4] * b[34] - b[26] * b[18] * b[9] - b[26] * b[19] * b[8] +
        b[27] * b[4] * b[22] + b[27] * b[5] * b[21] - b[27] * b[17] * b[9] -
        b[27] * b[18] * b[8] - b[1] * b[21] * b[31] - b[0] * b[23] * b[30] -
        b[0] * b[21] * b[32] + b[28] * b[4] * b[21] - b[28] * b[17] * b[8] +
        b[2] * b[17] * b[34] + b[0] * b[18] * b[35] - b[0] * b[22] * b[31] +
        b[0] * b[17] * b[36] + b[0] * b[19] * b[34] - b[1] * b[22] * b[30] +
        b[1] * b[18] * b[34] + b[1] * b[17] * b[35] - b[2] * b[21] * b[30];
    coeffs(3) =
        b[14] * b[30] * b[10] + b[14] * b[32] * b[8] - b[3] * b[21] * b[30] +
        b[3] * b[17] * b[34] + b[13] * b[32] * b[9] + b[13] * b[33] * b[8] -
        b[13] * b[4] * b[37] - b[13] * b[5] * b[36] + b[15] * b[30] * b[9] +
        b[15] * b[31] * b[8] - b[16] * b[4] * b[34] - b[13] * b[6] * b[35] -
        b[13] * b[7] * b[34] + b[13] * b[30] * b[11] + b[13] * b[31] * b[10] +
        b[14] * b[31] * b[9] - b[14] * b[4] * b[36] - b[14] * b[5] * b[35] -
        b[14] * b[6] * b[34] + b[16] * b[30] * b[8] - b[26] * b[20] * b[8] +
        b[26] * b[4] * b[24] + b[26] * b[5] * b[23] + b[26] * b[6] * b[22] +
        b[26] * b[7] * b[21] - b[26] * b[17] * b[11] - b[15] * b[4] * b[35] -
        b[15] * b[5] * b[34] - b[26] * b[18] * b[10] - b[26] * b[19] * b[9] +
        b[27] * b[4] * b[23] + b[27] * b[5] * b[22] + b[27] * b[6] * b[21] -
        b[27] * b[17] * b[10] - b[27] * b[18] * b[9] - b[27] * b[19] * b[8] +
        b[0] * b[17] * b[37] - b[0] * b[23] * b[31] - b[0] * b[24] * b[30] -
        b[0] * b[21] * b[33] - b[29] * b[17] * b[8] + b[28] * b[4] * b[22] +
        b[28] * b[5] * b[21] - b[28] * b[17] * b[9] - b[28] * b[18] * b[8] +
        b[29] * b[4] * b[21] + b[1] * b[19] * b[34] - b[2] * b[21] * b[31] +
        b[0] * b[20] * b[34] + b[0] * b[19] * b[35] + b[0] * b[18] * b[36] -
        b[0] * b[22] * b[32] - b[1] * b[23] * b[30] - b[1] * b[21] * b[32] +
        b[1] * b[18] * b[35] - b[1] * b[22] * b[31] - b[2] * b[22] * b[30] +
        b[2] * b[17] * b[35] + b[1] * b[17] * b[36] + b[2] * b[18] * b[34];
    coeffs(4) =
        -b[14] * b[6] * b[35] - b[14] * b[7] * b[34] - b[3] * b[22] * b[30] -
        b[3] * b[21] * b[31] + b[3] * b[17] * b[35] + b[3] * b[18] * b[34] +
        b[13] * b[32] * b[10] + b[13] * b[33] * b[9] - b[13] * b[4] * b[38] -
        b[13] * b[5] * b[37] - b[15] * b[6] * b[34] + b[15] * b[30] * b[10] +
        b[15] * b[32] * b[8] - b[16] * b[4] * b[35] - b[13] * b[6] * b[36] -
        b[13] * b[7] * b[35] + b[13] * b[31] * b[11] + b[13] * b[30] * b[12] +
        b[14] * b[32] * b[9] + b[14] * b[33] * b[8] - b[14] * b[4] * b[37] -
        b[14] * b[5] * b[36] + b[16] * b[30] * b[9] + b[16] * b[31] * b[8] -
        b[26] * b[20] * b[9] + b[26] * b[4] * b[25] + b[26] * b[5] * b[24] +
        b[26] * b[6] * b[23] + b[26] * b[7] * b[22] - b[26] * b[17] * b[12] +
        b[14] * b[30] * b[11] + b[14] * b[31] * b[10] + b[15] * b[31] * b[9] -
        b[15] * b[4] * b[36] - b[15] * b[5] * b[35] - b[26] * b[18] * b[11] -
        b[26] * b[19] * b[10] - b[27] * b[20] * b[8] + b[27] * b[4] * b[24] +
        b[27] * b[5] * b[23] + b[27] * b[6] * b[22] + b[27] * b[7] * b[21] -
        b[27] * b[17] * b[11] - b[27] * b[18] * b[10] - b[27] * b[19] * b[9] -
        b[16] * b[5] * b[34] - b[29] * b[17] * b[9] - b[29] * b[18] * b[8] +
        b[28] * b[4] * b[23] + b[28] * b[5] * b[22] + b[28] * b[6] * b[21] -
        b[28] * b[17] * b[10] - b[28] * b[18] * b[9] - b[28] * b[19] * b[8] +
        b[29] * b[4] * b[22] + b[29] * b[5] * b[21] - b[2] * b[23] * b[30] +
        b[2] * b[18] * b[35] - b[1] * b[22] * b[32] - b[2] * b[21] * b[32] +
        b[2] * b[19] * b[34] + b[0] * b[19] * b[36] - b[0] * b[22] * b[33] +
        b[0] * b[20] * b[35] - b[0] * b[23] * b[32] - b[0] * b[25] * b[30] +
        b[0] * b[17] * b[38] + b[0] * b[18] * b[37] - b[0] * b[24] * b[31] +
        b[1] * b[17] * b[37] - b[1] * b[23] * b[31] - b[1] * b[24] * b[30] -
        b[1] * b[21] * b[33] + b[1] * b[20] * b[34] + b[1] * b[19] * b[35] +
        b[1] * b[18] * b[36] + b[2] * b[17] * b[36] - b[2] * b[22] * b[31];
    coeffs(5) =
        -b[14] * b[6] * b[36] - b[14] * b[7] * b[35] + b[14] * b[31] * b[11] -
        b[3] * b[23] * b[30] - b[3] * b[21] * b[32] + b[3] * b[18] * b[35] -
        b[3] * b[22] * b[31] + b[3] * b[17] * b[36] + b[3] * b[19] * b[34] +
        b[13] * b[32] * b[11] + b[13] * b[33] * b[10] - b[13] * b[5] * b[38] -
        b[15] * b[6] * b[35] - b[15] * b[7] * b[34] + b[15] * b[30] * b[11] +
        b[15] * b[31] * b[10] + b[16] * b[31] * b[9] - b[13] * b[6] * b[37] -
        b[13] * b[7] * b[36] + b[13] * b[31] * b[12] + b[14] * b[32] * b[10] +
        b[14] * b[33] * b[9] - b[14] * b[4] * b[38] - b[14] * b[5] * b[37] -
        b[16] * b[6] * b[34] + b[16] * b[30] * b[10] + b[16] * b[32] * b[8] -
        b[26] * b[20] * b[10] + b[26] * b[5] * b[25] + b[26] * b[6] * b[24] +
        b[26] * b[7] * b[23] + b[14] * b[30] * b[12] + b[15] * b[32] * b[9] +
        b[15] * b[33] * b[8] - b[15] * b[4] * b[37] - b[15] * b[5] * b[36] +
        b[29] * b[5] * b[22] + b[29] * b[6] * b[21] - b[26] * b[18] * b[12] -
        b[26] * b[19] * b[11] - b[27] * b[20] * b[9] + b[27] * b[4] * b[25] +
        b[27] * b[5] * b[24] + b[27] * b[6] * b[23] + b[27] * b[7] * b[22] -
        b[27] * b[17] * b[12] - b[27] * b[18] * b[11] - b[27] * b[19] * b[10] -
        b[28] * b[20] * b[8] - b[16] * b[4] * b[36] - b[16] * b[5] * b[35] -
        b[29] * b[17] * b[10] - b[29] * b[18] * b[9] - b[29] * b[19] * b[8] +
        b[28] * b[4] * b[24] + b[28] * b[5] * b[23] + b[28] * b[6] * b[22] +
        b[28] * b[7] * b[21] - b[28] * b[17] * b[11] - b[28] * b[18] * b[10] -
        b[28] * b[19] * b[9] + b[29] * b[4] * b[23] - b[2] * b[22] * b[32] -
        b[2] * b[21] * b[33] - b[1] * b[24] * b[31] + b[0] * b[18] * b[38] -
        b[0] * b[24] * b[32] + b[0] * b[19] * b[37] + b[0] * b[20] * b[36] -
        b[0] * b[25] * b[31] - b[0] * b[23] * b[33] + b[1] * b[19] * b[36] -
        b[1] * b[22] * b[33] + b[1] * b[20] * b[35] + b[2] * b[19] * b[35] -
        b[2] * b[24] * b[30] - b[2] * b[23] * b[31] + b[2] * b[20] * b[34] +
        b[2] * b[17] * b[37] - b[1] * b[25] * b[30] + b[1] * b[18] * b[37] +
        b[1] * b[17] * b[38] - b[1] * b[23] * b[32] + b[2] * b[18] * b[36];
    coeffs(6) =
        -b[14] * b[6] * b[37] - b[14] * b[7] * b[36] + b[14] * b[31] * b[12] +
        b[3] * b[17] * b[37] - b[3] * b[23] * b[31] - b[3] * b[24] * b[30] -
        b[3] * b[21] * b[33] + b[3] * b[20] * b[34] + b[3] * b[19] * b[35] +
        b[3] * b[18] * b[36] - b[3] * b[22] * b[32] + b[13] * b[32] * b[12] +
        b[13] * b[33] * b[11] - b[15] * b[6] * b[36] - b[15] * b[7] * b[35] +
        b[15] * b[31] * b[11] + b[15] * b[30] * b[12] + b[16] * b[32] * b[9] +
        b[16] * b[33] * b[8] - b[13] * b[6] * b[38] - b[13] * b[7] * b[37] +
        b[14] * b[32] * b[11] + b[14] * b[33] * b[10] - b[14] * b[5] * b[38] -
        b[16] * b[6] * b[35] - b[16] * b[7] * b[34] + b[16] * b[30] * b[11] +
        b[16] * b[31] * b[10] - b[26] * b[19] * b[12] - b[26] * b[20] * b[11] +
        b[26] * b[6] * b[25] + b[26] * b[7] * b[24] + b[15] * b[32] * b[10] +
        b[15] * b[33] * b[9] - b[15] * b[4] * b[38] - b[15] * b[5] * b[37] +
        b[29] * b[5] * b[23] + b[29] * b[6] * b[22] + b[29] * b[7] * b[21] -
        b[27] * b[20] * b[10] + b[27] * b[5] * b[25] + b[27] * b[6] * b[24] +
        b[27] * b[7] * b[23] - b[27] * b[18] * b[12] - b[27] * b[19] * b[11] -
        b[28] * b[20] * b[9] - b[16] * b[4] * b[37] - b[16] * b[5] * b[36] +
        b[0] * b[19] * b[38] - b[0] * b[24] * b[33] + b[0] * b[20] * b[37] -
        b[29] * b[17] * b[11] - b[29] * b[18] * b[10] - b[29] * b[19] * b[9] +
        b[28] * b[4] * b[25] + b[28] * b[5] * b[24] + b[28] * b[6] * b[23] +
        b[28] * b[7] * b[22] - b[28] * b[17] * b[12] - b[28] * b[18] * b[11] -
        b[28] * b[19] * b[10] - b[29] * b[20] * b[8] + b[29] * b[4] * b[24] +
        b[2] * b[18] * b[37] - b[0] * b[25] * b[32] + b[1] * b[18] * b[38] -
        b[1] * b[24] * b[32] + b[1] * b[19] * b[37] + b[1] * b[20] * b[36] -
        b[1] * b[25] * b[31] + b[2] * b[17] * b[38] + b[2] * b[19] * b[36] -
        b[2] * b[24] * b[31] - b[2] * b[22] * b[33] - b[2] * b[23] * b[32] +
        b[2] * b[20] * b[35] - b[1] * b[23] * b[33] - b[2] * b[25] * b[30];
    coeffs(7) =
        -b[14] * b[6] * b[38] - b[14] * b[7] * b[37] + b[3] * b[19] * b[36] -
        b[3] * b[22] * b[33] + b[3] * b[20] * b[35] - b[3] * b[23] * b[32] -
        b[3] * b[25] * b[30] + b[3] * b[17] * b[38] + b[3] * b[18] * b[37] -
        b[3] * b[24] * b[31] - b[15] * b[6] * b[37] - b[15] * b[7] * b[36] +
        b[15] * b[31] * b[12] + b[16] * b[32] * b[10] + b[16] * b[33] * b[9] +
        b[13] * b[33] * b[12] - b[13] * b[7] * b[38] + b[14] * b[32] * b[12] +
        b[14] * b[33] * b[11] - b[16] * b[6] * b[36] - b[16] * b[7] * b[35] +
        b[16] * b[31] * b[11] + b[16] * b[30] * b[12] + b[15] * b[32] * b[11] +
        b[15] * b[33] * b[10] - b[15] * b[5] * b[38] + b[29] * b[5] * b[24] +
        b[29] * b[6] * b[23] - b[26] * b[20] * b[12] + b[26] * b[7] * b[25] -
        b[27] * b[19] * b[12] - b[27] * b[20] * b[11] + b[27] * b[6] * b[25] +
        b[27] * b[7] * b[24] - b[28] * b[20] * b[10] - b[16] * b[4] * b[38] -
        b[16] * b[5] * b[37] + b[29] * b[7] * b[22] - b[29] * b[17] * b[12] -
        b[29] * b[18] * b[11] - b[29] * b[19] * b[10] + b[28] * b[5] * b[25] +
        b[28] * b[6] * b[24] + b[28] * b[7] * b[23] - b[28] * b[18] * b[12] -
        b[28] * b[19] * b[11] - b[29] * b[20] * b[9] + b[29] * b[4] * b[25] -
        b[2] * b[24] * b[32] + b[0] * b[20] * b[38] - b[0] * b[25] * b[33] +
        b[1] * b[19] * b[38] - b[1] * b[24] * b[33] + b[1] * b[20] * b[37] -
        b[2] * b[25] * b[31] + b[2] * b[20] * b[36] - b[1] * b[25] * b[32] +
        b[2] * b[19] * b[37] + b[2] * b[18] * b[38] - b[2] * b[23] * b[33];
    coeffs(8) =
        b[3] * b[18] * b[38] - b[3] * b[24] * b[32] + b[3] * b[19] * b[37] +
        b[3] * b[20] * b[36] - b[3] * b[25] * b[31] - b[3] * b[23] * b[33] -
        b[15] * b[6] * b[38] - b[15] * b[7] * b[37] + b[16] * b[32] * b[11] +
        b[16] * b[33] * b[10] - b[16] * b[5] * b[38] - b[16] * b[6] * b[37] -
        b[16] * b[7] * b[36] + b[16] * b[31] * b[12] + b[14] * b[33] * b[12] -
        b[14] * b[7] * b[38] + b[15] * b[32] * b[12] + b[15] * b[33] * b[11] +
        b[29] * b[5] * b[25] + b[29] * b[6] * b[24] - b[27] * b[20] * b[12] +
        b[27] * b[7] * b[25] - b[28] * b[19] * b[12] - b[28] * b[20] * b[11] +
        b[29] * b[7] * b[23] - b[29] * b[18] * b[12] - b[29] * b[19] * b[11] +
        b[28] * b[6] * b[25] + b[28] * b[7] * b[24] - b[29] * b[20] * b[10] +
        b[2] * b[19] * b[38] - b[1] * b[25] * b[33] + b[2] * b[20] * b[37] -
        b[2] * b[24] * b[33] - b[2] * b[25] * b[32] + b[1] * b[20] * b[38];
    coeffs(9) =
        b[29] * b[7] * b[24] - b[29] * b[20] * b[11] + b[2] * b[20] * b[38] -
        b[2] * b[25] * b[33] - b[28] * b[20] * b[12] + b[28] * b[7] * b[25] -
        b[29] * b[19] * b[12] - b[3] * b[24] * b[33] + b[15] * b[33] * b[12] +
        b[3] * b[19] * b[38] - b[16] * b[6] * b[38] + b[3] * b[20] * b[37] +
        b[16] * b[32] * b[12] + b[29] * b[6] * b[25] - b[16] * b[7] * b[37] -
        b[3] * b[25] * b[32] - b[15] * b[7] * b[38] + b[16] * b[33] * b[11];
    coeffs(10) = -b[29] * b[20] * b[12] + b[29] * b[7] * b[25] +
        b[16] * b[33] * b[12] - b[16] * b[7] * b[38] +
        b[3] * b[20] * b[38] - b[3] * b[25] * b[33];
}

CEssentialMatrixEstimator_8Points::CEssentialMatrixEstimator_8Points() :CEstimator(8)
{

}
vector<any> CEssentialMatrixEstimator_8Points::Estimate(const vector<any>& points1_Any, const vector<any>& points2_Any)
{
    CHECK(points1_Any.size() == points2_Any.size());
    CHECK(points1_Any.size() >= minNumSamples);
    CHECK(points1_Any[0].type() == typeid(Eigen::Vector2d) && points2_Any[0].type() == typeid(Eigen::Vector2d));

    return TypeVec2AnyVec(Estimate(AnyVec2TypeVec<Eigen::Vector2d>(points1_Any), AnyVec2TypeVec<Eigen::Vector2d>(points2_Any)));
}
vector<Eigen::Matrix3d> CEssentialMatrixEstimator_8Points::Estimate(const vector<Eigen::Vector2d>& points1, const vector<Eigen::Vector2d>& points2) const
{
    CHECK(points1.size() == points2.size());
    CHECK(points1.size() >= minNumSamples);

    // Step 1. 为了提高数值稳定性, 对图像点进行居中和归一化处理
    vector<Eigen::Vector2d> normedPoints1;
    vector<Eigen::Vector2d> normedPoints2;
    Eigen::Matrix3d originToResult1;
    Eigen::Matrix3d originToResult2;
    CenterAndNormalizeImagePoints(points1, normedPoints1, originToResult1);
    CenterAndNormalizeImagePoints(points2, normedPoints2, originToResult2);

    // Step 2. 建立齐次线性方程: x2' * F * x1 = 0
    Eigen::Matrix<double, Eigen::Dynamic, 9> cmatrix(points1.size(), 9);
    for (size_t i = 0; i < points1.size(); i++)
    {
        cmatrix.block<1, 3>(i, 0) = normedPoints1[i].homogeneous();
        cmatrix.block<1, 3>(i, 0) *= normedPoints2[i].x();
        cmatrix.block<1, 3>(i, 3) = normedPoints1[i].homogeneous();
        cmatrix.block<1, 3>(i, 3) *= normedPoints2[i].y();
        cmatrix.block<1, 3>(i, 6) = normedPoints1[i].homogeneous();
    }

    // Step 3. 求解约束矩阵的零空间
    Eigen::JacobiSVD<Eigen::Matrix<double, Eigen::Dynamic, 9>> cmatrix_SVD(cmatrix, Eigen::ComputeFullV);
    const Eigen::VectorXd ematrixNullspace = cmatrix_SVD.matrixV().col(8);
    const Eigen::Map<const Eigen::Matrix3d> ematrix_t(ematrixNullspace.data());

    // 将图像点反归一化
    const Eigen::Matrix3d E_raw = originToResult2.transpose() * ematrix_t.transpose() * originToResult1;

    // 强制内部约束, 即其中两个奇异值必须相等, 另外一个必须为零
    Eigen::JacobiSVD<Eigen::Matrix3d> E_raw_svd(E_raw, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Vector3d singularValues = E_raw_svd.singularValues();
    singularValues(0) = (singularValues(0) + singularValues(1)) / 2.0;
    singularValues(1) = singularValues(0);
    singularValues(2) = 0.0;
    const Eigen::Matrix3d E = E_raw_svd.matrixU() * singularValues.asDiagonal() * E_raw_svd.matrixV().transpose();

    return { E };

}
void CEssentialMatrixEstimator_8Points::Residuals(const vector<any>& points1_Any, const vector<any>& points2_Any, const any& E_any, vector<double>& residuals)
{
    CHECK(points1_Any.size() == points2_Any.size() && !points1_Any.empty());
    CHECK(points1_Any[0].type() == typeid(Eigen::Vector2d) && points2_Any[0].type() == typeid(Eigen::Vector2d) && E_any.type() == typeid(Eigen::Matrix3d));
    Residuals(AnyVec2TypeVec<Eigen::Vector2d>(points1_Any), AnyVec2TypeVec<Eigen::Vector2d>(points2_Any), any_cast<Eigen::Matrix3d>(E_any), residuals);
}
void CEssentialMatrixEstimator_8Points::Residuals(const vector<Eigen::Vector2d>& points1, const vector<Eigen::Vector2d>& points2, const Eigen::Matrix3d& E, vector<double>& residuals) const
{
    CHECK(points1.size() == points2.size() && !points1.empty());
    ComputeSquaredSampsonError(points1, points2, E, residuals);
}

CFundamentalMatrixEstimator_7Points::CFundamentalMatrixEstimator_7Points() :CEstimator(7)
{

}
vector<any> CFundamentalMatrixEstimator_7Points::Estimate(const vector<any>& points1_Any, const vector<any>& points2_Any)
{
    CHECK(points1_Any.size() == minNumSamples && points2_Any.size() == minNumSamples);
    CHECK(points1_Any[0].type() == typeid(Eigen::Vector2d) && points2_Any[0].type() == typeid(Eigen::Vector2d));

    return TypeVec2AnyVec(Estimate(AnyVec2TypeVec<Eigen::Vector2d>(points1_Any), AnyVec2TypeVec<Eigen::Vector2d>(points2_Any)));
}
vector<Eigen::Matrix3d> CFundamentalMatrixEstimator_7Points::Estimate(const vector<Eigen::Vector2d>& points1, const vector<Eigen::Vector2d>& points2) const
{
    CHECK(points1.size() == minNumSamples && points2.size() == minNumSamples);

    // 这里不需要对点进行归一化
    // 建立方程系统: [points2(i,:), 1]' * F * [points1(i,:), 1]'
    Eigen::Matrix<double, 7, 9> A;
    for (size_t i = 0; i < 7; i++)
    {
        const double x0 = points1[i](0);
        const double y0 = points1[i](1);
        const double x1 = points2[i](0);
        const double y1 = points2[i](1);
        A(i, 0) = x1 * x0;
        A(i, 1) = x1 * y0;
        A(i, 2) = x1;
        A(i, 3) = y1 * x0;
        A(i, 4) = y1 * y0;
        A(i, 5) = y1;
        A(i, 6) = x0;
        A(i, 7) = y0;
        A(i, 8) = 1;
    }

    // 有9个未知数和7个方程, 因此我们有一个2维零空间
    Eigen::JacobiSVD<Eigen::Matrix<double, 7, 9>> svd(A, Eigen::ComputeFullV);
    const Eigen::Matrix<double, 9, 9>& f = svd.matrixV();
    Eigen::Matrix<double, 1, 9> f1 = f.col(7);
    Eigen::Matrix<double, 1, 9> f2 = f.col(8);
    f1 -= f2;

    // 归一化, 使得λ+μ=1, 并添加约束det(F)=det(λ*f1+(1-λ)*f2)
    const double t0 = f1(4) * f1(8) - f1(5) * f1(7);
    const double t1 = f1(3) * f1(8) - f1(5) * f1(6);
    const double t2 = f1(3) * f1(7) - f1(4) * f1(6);
    const double t3 = f2(4) * f2(8) - f2(5) * f2(7);
    const double t4 = f2(3) * f2(8) - f2(5) * f2(6);
    const double t5 = f2(3) * f2(7) - f2(4) * f2(6);

    Eigen::Vector4d coeffs;
    coeffs(0) = f1(0) * t0 - f1(1) * t1 + f1(2) * t2;
    coeffs(1) = f2(0) * t0 - f2(1) * t1 + f2(2) * t2 -
        f2(3) * (f1(1) * f1(8) - f1(2) * f1(7)) +
        f2(4) * (f1(0) * f1(8) - f1(2) * f1(6)) -
        f2(5) * (f1(0) * f1(7) - f1(1) * f1(6)) +
        f2(6) * (f1(1) * f1(5) - f1(2) * f1(4)) -
        f2(7) * (f1(0) * f1(5) - f1(2) * f1(3)) +
        f2(8) * (f1(0) * f1(4) - f1(1) * f1(3));
    coeffs(2) = f1(0) * t3 - f1(1) * t4 + f1(2) * t5 -
        f1(3) * (f2(1) * f2(8) - f2(2) * f2(7)) +
        f1(4) * (f2(0) * f2(8) - f2(2) * f2(6)) -
        f1(5) * (f2(0) * f2(7) - f2(1) * f2(6)) +
        f1(6) * (f2(1) * f2(5) - f2(2) * f2(4)) -
        f1(7) * (f2(0) * f2(5) - f2(2) * f2(3)) +
        f1(8) * (f2(0) * f2(4) - f2(1) * f2(3));
    coeffs(3) = f2(0) * t3 - f2(1) * t4 + f2(2) * t5;

    Eigen::VectorXd rootsReal;
    Eigen::VectorXd rootsImag;
#ifdef FindPolynomialRoots_Fast
    if (!FindPolynomialRootsDurandKerner(coeffs, rootsReal, rootsImag))
    {
        return {};
    }
#else
    if (!FindPolynomialRootsCompanionMatrix(coeffs, rootsReal, rootsImag))
    {
        return {};
    }
#endif

    vector<Eigen::Matrix3d> models;
    models.reserve(rootsReal.size());

    for (Eigen::VectorXd::Index i = 0; i < rootsReal.size(); i++)
    {
        if (abs(rootsImag(i)) > 1e-10)
        {
            continue;
        }

        const double lambda = rootsReal(i);
        const double mu = 1;

        Eigen::MatrixXd F = lambda * f1 + mu * f2;

        F.resize(3, 3);

        if (abs(F(2, 2)) < 1e-10)
        {
            continue;
        }

        F /= F(2, 2);

        models.push_back(F.transpose());
    }

    return models;
}
CFundamentalMatrixEstimate_7PointsRANSACReport CFundamentalMatrixEstimator_7Points::EstimateLoRANSAC(const std::vector<Eigen::Vector2d>& X, const std::vector<Eigen::Vector2d>& Y, const CRANSACOptions& options, CSupportMeasurer* supportMeasurer, CSampler* sampler) const
{
    // Step 1. 初始化. 检查输入向量大小, 初始化结果, 检查特殊情况
    CHECK(X.size() == Y.size());

    const size_t numSamples = X.size();
    CFundamentalMatrixEstimate_7PointsRANSACReport report;
    if (numSamples < minNumSamples) // 如果当前样本数太少, 不足以用来估计模型, 则直接返回
    {
        return report;
    }

    bool isSupportMeasurerNull = false, isSamplerNull = false;
    if (!supportMeasurer)
    {
        isSupportMeasurerNull = true;
        supportMeasurer = new CInlierSupportMeasurer();
    }
    if (!sampler)
    {
        isSamplerNull = true;
        sampler = new CRandomSampler(minNumSamples);
    }


    // Step 2. RANSAC主循环
    bool isAbort = false;
    bool bestModelIsLocal = false;
    const double maxResidual = options.maxError * options.maxError; // 允许的最大残差(只有当残差不超过maxResidual, 才会被标记为内点)

    std::vector<double> residuals; // 模型与所有数据点的残差
    std::vector<double> bestLocalResiduals; // 模型与所有数据点的残差
    std::vector<Eigen::Vector2d> XInlier;
    std::vector<Eigen::Vector2d> YInlier;
    std::vector<Eigen::Vector2d> X_rand(minNumSamples);
    std::vector<Eigen::Vector2d> Y_rand(minNumSamples);
    sampler->Initialize(numSamples); // 初始化采样器

    CSupport bestSupport; // 当前得到的最好的支持度
    Eigen::Matrix3d bestModel;  // 当前得到的最好模型
    CEssentialMatrixEstimator_8Points localEstimator;

    size_t maxNumTrials = std::min(options.maxNumTrials, sampler->GetMaxNumSamples()); // 确定最大迭代次数
    size_t dynamicMaxNumTrials = maxNumTrials;
    for (report.numTrials = 0; report.numTrials < maxNumTrials; report.numTrials++)
    {
        if (isAbort)
        {
            report.numTrials++;
            break;
        }

        sampler->GetSampleXY(X, Y, X_rand, Y_rand); // 从X和Y中以采样器内部的采样规则采样
        const std::vector<Eigen::Matrix3d> sampledModels = Estimate(X_rand, Y_rand); // 使用随机样本来估计模型
        for (const Eigen::Matrix3d& sampledModel : sampledModels)
        {
            Residuals(X, Y, sampledModel, residuals); // 对于每一个估计出的模型, 计算其与所有数据点的残差
            CHECK(residuals.size() == numSamples);

            const CSupport support = supportMeasurer->Evaluate(residuals, maxResidual); // 评估这个模型的支持度

            if (supportMeasurer->Compare(support, bestSupport)) // 如果新的支持度比当前最好的支持度更好, 就做局部优化
            {
                bestSupport = support;
                bestModel = sampledModel;
                bestModelIsLocal = false;

                // 根据内点来局部估计更优模型
                if (support.numInliers > minNumSamples && support.numInliers >= localEstimator.minNumSamples)
                {
                    // 迭代式局部优化来扩大内点集
                    const size_t maxLocalTrials = 10;
                    for (size_t localNumTrials = 0; localNumTrials < maxLocalTrials; localNumTrials++)
                    {
                        XInlier.clear();
                        YInlier.clear();
                        XInlier.reserve(numSamples);
                        YInlier.reserve(numSamples);
                        for (size_t i = 0; i < residuals.size(); i++)
                        {
                            if (residuals[i] <= maxResidual)
                            {
                                XInlier.push_back(X[i]);
                                YInlier.push_back(Y[i]);
                            }
                        }
                        const std::vector<Eigen::Matrix3d> localModels = localEstimator.Estimate(XInlier, YInlier);
                        const size_t preBestNumInliers = bestSupport.numInliers;
                        for (const Eigen::Matrix3d& localModel : localModels)
                        {
                            localEstimator.Residuals(X, Y, localModel, residuals);
                            CHECK(residuals.size() == numSamples);
                            const CSupport localSupport = supportMeasurer->Evaluate(residuals, maxResidual);

                            // 检查局部优化模型是否更优
                            if (supportMeasurer->Compare(localSupport, bestSupport))
                            {
                                bestSupport = localSupport;
                                bestModel = localModel;
                                bestModelIsLocal = true;
                                std::swap(residuals, bestLocalResiduals); // 交换残差
                            }
                        }
                        // 只有当内点集变多了从而有机会进一步优化时, 才继续迭代
                        if (bestSupport.numInliers <= preBestNumInliers)
                        {
                            break;
                        }

                        //把残差再交换回来, 这样就可以在下一次局部优化的迭代中提取出最佳的内点集
                        std::swap(residuals, bestLocalResiduals);
                    }
                }
                dynamicMaxNumTrials = GetNumTrials(minNumSamples, bestSupport.numInliers, numSamples, options.confidence, options.maxIterNumTrialsMultiplier);
            }
            if (report.numTrials >= dynamicMaxNumTrials && report.numTrials >= options.minNumTrials)
            {
                isAbort = true;
                break;
            }
        }
    }

    // Step 3. 结果收集与返回
    report.support = bestSupport;
    report.model = bestModel;
    if (report.support.numInliers < minNumSamples) // 如果找到的最佳模型的内点数少于最小样本数, 那么说明失败了
    {
        if (isSamplerNull)
        {
            delete sampler;
        }
        if (isSupportMeasurerNull)
        {
            delete supportMeasurer;
        }
        return report;
    }

    // 这将对最佳模型的残差进行两次计算, 但避免了对每个评估模型都复制和填充内点掩码, 这种方法其实更快
    if (bestModelIsLocal)
    {
        localEstimator.Residuals(X, Y, report.model, residuals);
    }
    else
    {
        Residuals(X, Y, report.model, residuals);
    }
    CHECK(residuals.size() == numSamples);
    report.inlierMask.resize(numSamples);

    for (size_t i = 0; i < residuals.size(); i++) // 判断每个样本是否为内点
    {
        report.inlierMask[i] = (residuals[i] <= maxResidual);
    }
    report.isSuccess = true;

    if (isSamplerNull)
    {
        delete sampler;
    }
    if (isSupportMeasurerNull)
    {
        delete supportMeasurer;
    }
    return report;
}
void CFundamentalMatrixEstimator_7Points::Residuals(const vector<any>& points1_Any, const vector<any>& points2_Any, const any& F_any, vector<double>& residuals)
{
    CHECK(points1_Any.size() == points2_Any.size() && !points1_Any.empty());
    CHECK(points1_Any[0].type() == typeid(Eigen::Vector2d) && points2_Any[0].type() == typeid(Eigen::Vector2d) && F_any.type() == typeid(Eigen::Matrix3d));

    Residuals(AnyVec2TypeVec<Eigen::Vector2d>(points1_Any), AnyVec2TypeVec<Eigen::Vector2d>(points2_Any), any_cast<Eigen::Matrix3d>(F_any), residuals);
}
void CFundamentalMatrixEstimator_7Points::Residuals(const vector<Eigen::Vector2d>& points1, const vector<Eigen::Vector2d>& points2, const Eigen::Matrix3d& F, vector<double>& residuals) const
{
    CHECK(points1.size() == points2.size() && !points1.empty());
    ComputeSquaredSampsonError(points1, points2, F, residuals);
}

CFundamentalMatrixEstimator_8Points::CFundamentalMatrixEstimator_8Points() :CEstimator(8)
{

}
vector<any> CFundamentalMatrixEstimator_8Points::Estimate(const vector<any>& points1_Any, const vector<any>& points2_Any)
{
    CHECK(points1_Any.size() == points2_Any.size());
    CHECK(points1_Any.size() >= minNumSamples);
    CHECK(points1_Any[0].type() == typeid(Eigen::Vector2d) && points2_Any[0].type() == typeid(Eigen::Vector2d));

    return TypeVec2AnyVec(Estimate(AnyVec2TypeVec<Eigen::Vector2d>(points1_Any), AnyVec2TypeVec<Eigen::Vector2d>(points2_Any)));
}
vector<Eigen::Matrix3d> CFundamentalMatrixEstimator_8Points::Estimate(const vector<Eigen::Vector2d>& points1, const vector<Eigen::Vector2d>& points2) const
{
    CHECK(points1.size() == points2.size());
    CHECK(points1.size() >= minNumSamples);

    // 为了提高数值稳定性, 对图像点进行居中和归一化
    vector<Eigen::Vector2d> normedPoints1;
    vector<Eigen::Vector2d> normedPoints2;
    Eigen::Matrix3d originToResult1;
    Eigen::Matrix3d originToResult2;
    CenterAndNormalizeImagePoints(points1, normedPoints1, originToResult1);
    CenterAndNormalizeImagePoints(points2, normedPoints2, originToResult2);

    // 建立齐次线性方程: x2' * F *x1 = 0
    Eigen::Matrix<double, Eigen::Dynamic, 9> cmatrix(points1.size(), 9);
    for (size_t i = 0; i < points1.size(); i++)
    {
        cmatrix.block<1, 3>(i, 0) = normedPoints1[i].homogeneous();
        cmatrix.block<1, 3>(i, 0) *= normedPoints2[i].x();
        cmatrix.block<1, 3>(i, 3) = normedPoints1[i].homogeneous();
        cmatrix.block<1, 3>(i, 3) *= normedPoints2[i].y();
        cmatrix.block<1, 3>(i, 6) = normedPoints1[i].homogeneous();
    }

    // 求解约束矩阵的零空间
    Eigen::JacobiSVD<Eigen::Matrix<double, Eigen::Dynamic, 9>> cmatrix_svd(cmatrix, Eigen::ComputeFullV);
    const Eigen::VectorXd cmatrix_nullspace = cmatrix_svd.matrixV().col(8);
    const Eigen::Map<const Eigen::Matrix3d> ematrix_t(cmatrix_nullspace.data());

    // 强制内部约束, 即其中两个奇异值必须非零, 一个必须为零
    Eigen::JacobiSVD<Eigen::Matrix3d> fmatrix_svd(ematrix_t.transpose(), Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Vector3d singularValues = fmatrix_svd.singularValues();
    singularValues(2) = 0.0;
    const Eigen::Matrix3d F = fmatrix_svd.matrixU() * singularValues.asDiagonal() * fmatrix_svd.matrixV().transpose();

    return { originToResult2.transpose() * F * originToResult1 };
}
void CFundamentalMatrixEstimator_8Points::Residuals(const vector<any>& points1_Any, const vector<any>& points2_Any, const any& F_any, vector<double>& residuals)
{
    CHECK(points1_Any.size() == points2_Any.size() && !points1_Any.empty());
    CHECK(points1_Any[0].type() == typeid(Eigen::Vector2d) && points2_Any[0].type() == typeid(Eigen::Vector2d) && F_any.type() == typeid(Eigen::Matrix3d));

    Residuals(AnyVec2TypeVec<Eigen::Vector2d>(points1_Any), AnyVec2TypeVec<Eigen::Vector2d>(points2_Any), any_cast<Eigen::Matrix3d>(F_any), residuals);
}
void CFundamentalMatrixEstimator_8Points::Residuals(const vector<Eigen::Vector2d>& points1, const vector<Eigen::Vector2d>& points2, const Eigen::Matrix3d& F, vector<double>& residuals) const
{
    CHECK(points1.size() == points2.size() && !points1.empty());

    ComputeSquaredSampsonError(points1, points2, F, residuals);
}

CHomographyMatrixEstimator::CHomographyMatrixEstimator() :CEstimator(4)
{

}
vector<any> CHomographyMatrixEstimator::Estimate(const vector<any>& points1_Any, const vector<any>& points2_Any)
{
    CHECK(points1_Any.size() == points2_Any.size());
    CHECK(points1_Any.size() >= minNumSamples);
    CHECK(points1_Any[0].type() == typeid(Eigen::Vector2d) && points2_Any[0].type() == typeid(Eigen::Vector2d));
    return TypeVec2AnyVec(Estimate(AnyVec2TypeVec<Eigen::Vector2d>(points1_Any), AnyVec2TypeVec<Eigen::Vector2d>(points2_Any)));
}
vector<Eigen::Matrix3d> CHomographyMatrixEstimator::Estimate(const vector<Eigen::Vector2d>& points1, const vector<Eigen::Vector2d>& points2) const
{
    CHECK(points1.size() == points2.size());
    CHECK(points1.size() >= minNumSamples);

    const size_t N = points1.size();

    // 为了提高数值稳定性, 对图像点进行中心化和归一化
    vector<Eigen::Vector2d> normedPoints1;
    vector<Eigen::Vector2d> normedPoints2;
    Eigen::Matrix3d originToResult1;
    Eigen::Matrix3d originToResult2;
    CenterAndNormalizeImagePoints(points1, normedPoints1, originToResult1);
    CenterAndNormalizeImagePoints(points2, normedPoints2, originToResult2);

    // 设置约束矩阵
    Eigen::Matrix<double, Eigen::Dynamic, 9> A = Eigen::MatrixXd::Zero(2 * N, 9);

    for (size_t i = 0, j = N; i < points1.size(); i++, j++) 
    {
        const double s_0 = normedPoints1[i](0);
        const double s_1 = normedPoints1[i](1);
        const double d_0 = normedPoints2[i](0);
        const double d_1 = normedPoints2[i](1);

        A(i, 0) = -s_0;
        A(i, 1) = -s_1;
        A(i, 2) = -1;
        A(i, 6) = s_0 * d_0;
        A(i, 7) = s_1 * d_0;
        A(i, 8) = d_0;

        A(j, 3) = -s_0;
        A(j, 4) = -s_1;
        A(j, 5) = -1;
        A(j, 6) = s_0 * d_1;
        A(j, 7) = s_1 * d_1;
        A(j, 8) = d_1;
    }

    // 求解约束矩阵的零空间
    Eigen::JacobiSVD<Eigen::Matrix<double, Eigen::Dynamic, 9>> svd(A, Eigen::ComputeFullV);
    const Eigen::VectorXd nullspace = svd.matrixV().col(8);
    Eigen::Map<const Eigen::Matrix3d> H_t(nullspace.data());
    return { originToResult2.inverse() * H_t.transpose() * originToResult1 };
}
CHomographyMatrixEstimateRANSACReport CHomographyMatrixEstimator::EstimateLoRANSAC(const std::vector<Eigen::Vector2d>& X, const std::vector<Eigen::Vector2d>& Y, const CRANSACOptions& options, CSupportMeasurer* supportMeasurer, CSampler* sampler) const
{
    // Step 1. 初始化. 检查输入向量大小, 初始化结果, 检查特殊情况
    CHECK(X.size() == Y.size());

    const size_t numSamples = X.size();
    CHomographyMatrixEstimateRANSACReport report;
    if (numSamples < minNumSamples) // 如果当前样本数太少, 不足以用来估计模型, 则直接返回
    {
        return report;
    }

    bool isSupportMeasurerNull = false, isSamplerNull = false;
    if (!supportMeasurer)
    {
        isSupportMeasurerNull = true;
        supportMeasurer = new CInlierSupportMeasurer();
    }
    if (!sampler)
    {
        isSamplerNull = true;
        sampler = new CRandomSampler(minNumSamples);
    }

    // Step 2. RANSAC主循环
    bool isAbort = false;
    bool bestModelIsLocal = false;
    const double maxResidual = options.maxError * options.maxError; // 允许的最大残差(只有当残差不超过maxResidual, 才会被标记为内点)

    std::vector<double> residuals; // 模型与所有数据点的残差
    std::vector<double> bestLocalResiduals; // 模型与所有数据点的残差
    std::vector<Eigen::Vector2d> XInlier;
    std::vector<Eigen::Vector2d> YInlier;
    std::vector<Eigen::Vector2d> X_rand(minNumSamples);
    std::vector<Eigen::Vector2d> Y_rand(minNumSamples);
    sampler->Initialize(numSamples); // 初始化采样器

    CSupport bestSupport; // 当前得到的最好的支持度
    Eigen::Matrix3d bestModel;  // 当前得到的最好模型

    size_t maxNumTrials = std::min(options.maxNumTrials, sampler->GetMaxNumSamples()); // 确定最大迭代次数
    size_t dynamicMaxNumTrials = maxNumTrials;
    for (report.numTrials = 0; report.numTrials < maxNumTrials; report.numTrials++)
    {
        if (isAbort)
        {
            report.numTrials++;
            break;
        }

        sampler->GetSampleXY(X, Y, X_rand, Y_rand); // 从X和Y中以采样器内部的采样规则采样
        const std::vector<Eigen::Matrix3d> sampledModels = Estimate(X_rand, Y_rand); // 使用随机样本来估计模型
        for (const Eigen::Matrix3d& sampledModel : sampledModels)
        {
            Residuals(X, Y, sampledModel, residuals); // 对于每一个估计出的模型, 计算其与所有数据点的残差
            CHECK(residuals.size() == numSamples);

            const CSupport support = supportMeasurer->Evaluate(residuals, maxResidual); // 评估这个模型的支持度

            if (supportMeasurer->Compare(support, bestSupport)) // 如果新的支持度比当前最好的支持度更好, 就做局部优化
            {
                bestSupport = support;
                bestModel = sampledModel;
                bestModelIsLocal = false;

                // 根据内点来局部估计更优模型
                if (support.numInliers > minNumSamples && support.numInliers >= minNumSamples)
                {
                    // 迭代式局部优化来扩大内点集
                    const size_t maxLocalTrials = 10;
                    for (size_t localNumTrials = 0; localNumTrials < maxLocalTrials; localNumTrials++)
                    {
                        XInlier.clear();
                        YInlier.clear();
                        XInlier.reserve(numSamples);
                        YInlier.reserve(numSamples);
                        for (size_t i = 0; i < residuals.size(); i++)
                        {
                            if (residuals[i] <= maxResidual)
                            {
                                XInlier.push_back(X[i]);
                                YInlier.push_back(Y[i]);
                            }
                        }
                        const std::vector<Eigen::Matrix3d> localModels = Estimate(XInlier, YInlier);
                        const size_t preBestNumInliers = bestSupport.numInliers;
                        for (const Eigen::Matrix3d& localModel : localModels)
                        {
                            Residuals(X, Y, localModel, residuals);
                            CHECK(residuals.size() == numSamples);
                            const CSupport localSupport = supportMeasurer->Evaluate(residuals, maxResidual);

                            // 检查局部优化模型是否更优
                            if (supportMeasurer->Compare(localSupport, bestSupport))
                            {
                                bestSupport = localSupport;
                                bestModel = localModel;
                                bestModelIsLocal = true;
                                std::swap(residuals, bestLocalResiduals); // 交换残差
                            }
                        }
                        // 只有当内点集变多了从而有机会进一步优化时, 才继续迭代
                        if (bestSupport.numInliers <= preBestNumInliers)
                        {
                            break;
                        }

                        //把残差再交换回来, 这样就可以在下一次局部优化的迭代中提取出最佳的内点集
                        std::swap(residuals, bestLocalResiduals);
                    }
                }
                dynamicMaxNumTrials = GetNumTrials(minNumSamples, bestSupport.numInliers, numSamples, options.confidence, options.maxIterNumTrialsMultiplier);
            }
            if (report.numTrials >= dynamicMaxNumTrials && report.numTrials >= options.minNumTrials)
            {
                isAbort = true;
                break;
            }
        }
    }

    // Step 3. 结果收集与返回
    report.support = bestSupport;
    report.model = bestModel;
    if (report.support.numInliers < minNumSamples) // 如果找到的最佳模型的内点数少于最小样本数, 那么说明失败了
    {
        if (isSamplerNull)
        {
            delete sampler;
        }
        if (isSupportMeasurerNull)
        {
            delete supportMeasurer;
        }
        return report;
    }

    // 这将对最佳模型的残差进行两次计算, 但避免了对每个评估模型都复制和填充内点掩码, 这种方法其实更快
    Residuals(X, Y, report.model, residuals);
    CHECK(residuals.size() == numSamples);
    report.inlierMask.resize(numSamples);

    for (size_t i = 0; i < residuals.size(); i++) // 判断每个样本是否为内点
    {
        report.inlierMask[i] = (residuals[i] <= maxResidual);
    }
    report.isSuccess = true;

    if (isSamplerNull)
    {
        delete sampler;
    }
    if (isSupportMeasurerNull)
    {
        delete supportMeasurer;
    }
    return report;
}
void CHomographyMatrixEstimator::Residuals(const vector<any>& points1_Any, const vector<any>& points2_Any, const any& H_Any, vector<double>& residuals)
{
    CHECK(points1_Any.size() == points2_Any.size() && !points1_Any.empty());
    CHECK(points1_Any[0].type() == typeid(Eigen::Vector2d) && points2_Any[0].type() == typeid(Eigen::Vector2d) && H_Any.type() == typeid(Eigen::Matrix3d));

    Residuals(AnyVec2TypeVec<Eigen::Vector2d>(points1_Any), AnyVec2TypeVec<Eigen::Vector2d>(points2_Any), any_cast<Eigen::Matrix3d>(H_Any), residuals);
}
void CHomographyMatrixEstimator::Residuals(const vector<Eigen::Vector2d>& points1, const vector<Eigen::Vector2d>& points2, const Eigen::Matrix3d& H, vector<double>& residuals) const
{
    CHECK(points1.size() == points2.size() && !points1.empty());

    residuals.resize(points1.size());

    // 这段代码可能没有Eigen表达式那么优雅, 但在各种测试中, 它的速度明显更快
    const double H_00 = H(0, 0);
    const double H_01 = H(0, 1);
    const double H_02 = H(0, 2);
    const double H_10 = H(1, 0);
    const double H_11 = H(1, 1);
    const double H_12 = H(1, 2);
    const double H_20 = H(2, 0);
    const double H_21 = H(2, 1);
    const double H_22 = H(2, 2);

    for (size_t i = 0; i < points1.size(); i++)
    {
        const double s_0 = points1[i](0);
        const double s_1 = points1[i](1);
        const double d_0 = points2[i](0);
        const double d_1 = points2[i](1);

        const double pd_0 = H_00 * s_0 + H_01 * s_1 + H_02;
        const double pd_1 = H_10 * s_0 + H_11 * s_1 + H_12;
        const double pd_2 = H_20 * s_0 + H_21 * s_1 + H_22;

        const double inv_pd_2 = 1.0 / pd_2;
        const double dd_0 = d_0 - pd_0 * inv_pd_2;
        const double dd_1 = d_1 - pd_1 * inv_pd_2;

        residuals[i] = dd_0 * dd_0 + dd_1 * dd_1;
    }
}

CAffineTransformEstimator::CAffineTransformEstimator() :CEstimator(3)
{

}
vector<any> CAffineTransformEstimator::Estimate(const vector<any>& points1_Any, const vector<any>& points2_Any)
{
    CHECK(points1_Any.size() == points2_Any.size());
    CHECK(points1_Any.size() >= minNumSamples);
    CHECK(points1_Any[0].type() == typeid(Eigen::Vector2d) && points2_Any[0].type() == typeid(Eigen::Vector2d));
    return TypeVec2AnyVec(Estimate(AnyVec2TypeVec<Eigen::Vector2d>(points1_Any), AnyVec2TypeVec<Eigen::Vector2d>(points2_Any)));
}
vector<Eigen::Matrix<double, 2, 3>> CAffineTransformEstimator::Estimate(const vector<Eigen::Vector2d>& points1, const vector<Eigen::Vector2d>& points2) const
{
    CHECK(points1.size() == points2.size());
    CHECK(points1.size() >= minNumSamples);

    // 建立用来求解仿射变换的最小二乘解的线性系统
    Eigen::MatrixXd C(2 * points1.size(), 6);
    C.setZero();
    Eigen::VectorXd b(2 * points1.size(), 1);

    for (size_t i = 0; i < points1.size(); i++)
    {
        const Eigen::Vector2d& x1 = points1[i];
        const Eigen::Vector2d& x2 = points2[i];

        C(2 * i, 0) = x1(0);
        C(2 * i, 1) = x1(1);
        C(2 * i, 2) = 1.0f;
        b(2 * i) = x2(0);

        C(2 * i + 1, 3) = x1(0);
        C(2 * i + 1, 4) = x1(1);
        C(2 * i + 1, 5) = 1.0f;
        b(2 * i + 1) = x2(1);
    }
    const Eigen::VectorXd nullspace = C.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
    Eigen::Map<const Eigen::Matrix<double, 3, 2>> A_t(nullspace.data());
    return { A_t.transpose() };
}
void CAffineTransformEstimator::Residuals(const vector<any>& points1_Any, const vector<any>& points2_Any, const any& A_Any, vector<double>& residuals)
{
    CHECK(points1_Any.size() == points2_Any.size() && !points1_Any.empty());
    CHECK(points1_Any[0].type() == typeid(Eigen::Vector2d) && points2_Any[0].type() == typeid(Eigen::Vector2d) && A_Any.type() == typeid(Eigen::Matrix<double, 2, 3>));
    
    Residuals(AnyVec2TypeVec<Eigen::Vector2d>(points1_Any), AnyVec2TypeVec<Eigen::Vector2d>(points2_Any), any_cast<Eigen::Matrix<double, 2, 3>>(A_Any), residuals);
}
void CAffineTransformEstimator::Residuals(const vector<Eigen::Vector2d>& points1, const vector<Eigen::Vector2d>& points2, const Eigen::Matrix<double, 2, 3>& A, vector<double>& residuals) const
{
    CHECK(points1.size() == points2.size() && !points1.empty());
    residuals.resize(points1.size());

    // 这段代码可能没有Eigen表达式那么优雅, 但在各种测试中它明显更快
    const double A_00 = A(0, 0);
    const double A_01 = A(0, 1);
    const double A_02 = A(0, 2);
    const double A_10 = A(1, 0);
    const double A_11 = A(1, 1);
    const double A_12 = A(1, 2);

    for (size_t i = 0; i < points1.size(); i++)
    {
        const double s0 = points1[i](0);
        const double s1 = points1[i](1);
        const double d0 = points2[i](0);
        const double d1 = points2[i](1);

        const double pd0 = A_00 * s0 + A_01 * s1 + A_02;
        const double pd1 = A_10 * s0 + A_11 * s1 + A_12;

        const double dd0 = d0 - pd0;
        const double dd1 = d1 - pd1;

        residuals[i] = dd0 * dd0 + dd1 * dd1;
    }
}

CSimilarityTransformEstimator::CSimilarityTransformEstimator(bool isEstimateScale) : CEstimator(3)
{
    this->isEstimateScale = isEstimateScale;
}
vector<any> CSimilarityTransformEstimator::Estimate(const vector<any>& src_Any, const vector<any>& dst_Any)
{
    CHECK(src_Any.size() == dst_Any.size());
    CHECK(src_Any.size() >= minNumSamples);
    CHECK(src_Any[0].type() == typeid(Eigen::Vector3d) && dst_Any[0].type() == typeid(Eigen::Vector3d));
    return TypeVec2AnyVec(Estimate(AnyVec2TypeVec<Eigen::Vector3d>(src_Any), AnyVec2TypeVec<Eigen::Vector3d>(dst_Any)));
}
vector<Eigen::Matrix<double, 3, 4>> CSimilarityTransformEstimator::Estimate(const vector<Eigen::Vector3d>& src, const vector<Eigen::Vector3d>& dst) const
{
    CHECK(src.size() == dst.size());
    CHECK(src.size() >= minNumSamples);

    Eigen::Matrix<double, 3, Eigen::Dynamic> srcMat(3, src.size());
    Eigen::Matrix<double, 3, Eigen::Dynamic> dstMat(3, dst.size());
    for (size_t i = 0; i < src.size(); i++) 
    {
        srcMat.col(i) = src[i];
        dstMat.col(i) = dst[i];
    }
    const Eigen::Matrix<double, 3, 4> model = Eigen::umeyama(srcMat, dstMat, isEstimateScale).topLeftCorner<3, 4>();

    if (model.array().isNaN().any()) 
    {
        return {};
    }
    return { model };
}
void CSimilarityTransformEstimator::Residuals(const vector<any>& src_Any, const vector<any>& dst_Any, const any& matrix_Any, vector<double>& residuals)
{
    CHECK(src_Any.size() == dst_Any.size() && !src_Any.empty());
    CHECK(src_Any[0].type() == typeid(Eigen::Vector3d) && dst_Any[0].type() == typeid(Eigen::Vector3d) && matrix_Any.type() == typeid(Eigen::Matrix<double, 3, 4>));

    Residuals(AnyVec2TypeVec<Eigen::Vector3d>(src_Any), AnyVec2TypeVec<Eigen::Vector3d>(dst_Any), any_cast<Eigen::Matrix<double, 3, 4>>(matrix_Any), residuals);
}
void CSimilarityTransformEstimator::Residuals(const vector<Eigen::Vector3d>& src, const vector<Eigen::Vector3d>& dst, const Eigen::Matrix<double, 3, 4>& matrix, vector<double>& residuals) const
{
    CHECK(src.size() == dst.size() && !src.empty());
    residuals.resize(src.size());

    for (size_t i = 0; i < src.size(); i++) 
    {
        const Eigen::Vector3d transformedDst = matrix * src[i].homogeneous();
        residuals[i] = (dst[i] - transformedDst).squaredNorm();
    }
}

CTranslationTransformEstimator::CTranslationTransformEstimator() : CEstimator(1)
{

}
vector<any> CTranslationTransformEstimator::Estimate(const vector<any>& points1_Any, const vector<any>& points2_Any)
{
    CHECK(points1_Any.size() == points2_Any.size());
    CHECK(points1_Any.size() >= minNumSamples);
    CHECK(points1_Any[0].type() == typeid(Eigen::Vector2d) && points2_Any[0].type() == typeid(Eigen::Vector2d));
    return TypeVec2AnyVec(Estimate(AnyVec2TypeVec<Eigen::Vector2d>(points1_Any), AnyVec2TypeVec<Eigen::Vector2d>(points2_Any)));
}
vector<Eigen::Vector2d> CTranslationTransformEstimator::Estimate(const vector<Eigen::Vector2d>& points1, const vector<Eigen::Vector2d>& points2) const
{
    CHECK(points1.size() == points2.size());
    CHECK(points1.size() >= minNumSamples);

    Eigen::Vector2d srcMean = Eigen::Vector2d::Zero();
    Eigen::Vector2d dstMean = Eigen::Vector2d::Zero();

    for (size_t i = 0; i < points1.size(); i++) 
    {
        srcMean += points1[i];
        dstMean += points2[i];
    }

    srcMean /= points1.size();
    dstMean /= points2.size();

    return { dstMean - srcMean };
}
void CTranslationTransformEstimator::Residuals(const vector<any>& points1_Any, const vector<any>& points2_Any, const any& translation_Any, vector<double>& residuals)
{
    CHECK(points1_Any.size() == points2_Any.size() && !points1_Any.empty());
    CHECK(points1_Any[0].type() == typeid(Eigen::Vector2d) && points2_Any[0].type() == typeid(Eigen::Vector2d) && translation_Any.type() == typeid(Eigen::Vector2d));

    Residuals(AnyVec2TypeVec<Eigen::Vector2d>(points1_Any), AnyVec2TypeVec<Eigen::Vector2d>(points2_Any), any_cast<Eigen::Vector2d>(translation_Any), residuals);
}
void CTranslationTransformEstimator::Residuals(const vector<Eigen::Vector2d>& points1, const vector<Eigen::Vector2d>& points2, const Eigen::Vector2d& translation, vector<double>& residuals) const
{
    CHECK(points1.size() == points2.size() && !points1.empty());
    residuals.resize(points1.size());

    for (size_t i = 0; i < points1.size(); i++)
    {
        const Eigen::Vector2d diff = points2[i] - points1[i] - translation;
        residuals[i] = diff.squaredNorm();
    }
}

CTriangulationEstimator::CTriangulationEstimator(double minTriAngle, CTriangulationResidualType residualType) : CEstimator(2)
{
    CHECK(minTriAngle >= 0);
    this->minTriAngle = minTriAngle;
    this->residualType = residualType;
}
vector<any> CTriangulationEstimator::Estimate(const vector<any>& points_Any, const vector<any>& poses_Any)
{
    CHECK(points_Any.size() == poses_Any.size());
    CHECK(points_Any.size() >= minNumSamples);
    CHECK(points_Any[0].type() == typeid(CTriangulationPoint) && poses_Any[0].type() == typeid(CTriangulationPose));
    return TypeVec2AnyVec(Estimate(AnyVec2TypeVec<CTriangulationPoint>(points_Any), AnyVec2TypeVec<CTriangulationPose>(poses_Any)));
}
vector<Eigen::Vector3d> CTriangulationEstimator::Estimate(const vector<CTriangulationPoint>& points, const vector<CTriangulationPose>& poses) const
{
    CHECK(points.size() == poses.size());
    CHECK(points.size() >= minNumSamples);

    if (points.size() == 2)
    {
        // 双视三角测量
        const Eigen::Vector3d XYZ = TriangulatePoint(poses[0].projectionMatrix, poses[1].projectionMatrix, points[0].pointNormalized, points[1].pointNormalized);
        if (HasPointPositiveDepth(poses[0].projectionMatrix, XYZ) && HasPointPositiveDepth(poses[1].projectionMatrix, XYZ) &&
            CalculateTriangulationAngle(poses[0].projectionCenter, poses[1].projectionCenter, XYZ) >= minTriAngle)
        {
            return vector<Eigen::Vector3d>{XYZ};
        }
    }
    else
    {
        // 多视三角测量
        vector<Eigen::Matrix3x4d> projectionMatrices;
        projectionMatrices.reserve(points.size());
        vector<Eigen::Vector2d> points2D;
        points2D.reserve(points.size());
        for (size_t i = 0; i < points.size(); i++)
        {
            projectionMatrices.push_back(poses[i].projectionMatrix);
            points2D.push_back(points[i].pointNormalized);
        }
        const Eigen::Vector3d XYZ = TriangulateMultiViewPoint(projectionMatrices, points2D);

        // 正景深约束测试
        for (const auto& pose : poses)
        {
            if (!HasPointPositiveDepth(pose.projectionMatrix, XYZ))
            {
                return vector<Eigen::Vector3d>();
            }
        }

        // 检查是否有足够大的交会角
        for (size_t i = 0; i < poses.size(); i++)
        {
            for (size_t j = 0; j < i; j++)
            {
                const double tri_angle = CalculateTriangulationAngle(poses[i].projectionCenter, poses[j].projectionCenter, XYZ);
                if (tri_angle >= minTriAngle)
                {
                    return vector<Eigen::Vector3d>{XYZ};
                }
            }
        }
    }
    return vector<Eigen::Vector3d>();
}
void CTriangulationEstimator::Residuals(const vector<any>& points_Any, const vector<any>& poses_Any, const any& XYZ_Any, vector<double>& residuals)
{
    CHECK(points_Any.size() == poses_Any.size() && !points_Any.empty());
    CHECK(points_Any[0].type() == typeid(CTriangulationPoint) && poses_Any[0].type() == typeid(CTriangulationPose) && XYZ_Any.type() == typeid(Eigen::Vector3d));

    Residuals(AnyVec2TypeVec<CTriangulationPoint>(points_Any), AnyVec2TypeVec<CTriangulationPose>(poses_Any), any_cast<Eigen::Vector3d>(XYZ_Any), residuals);
}
void CTriangulationEstimator::Residuals(const vector<CTriangulationPoint>& points, const vector<CTriangulationPose>& poses, const Eigen::Vector3d& XYZ, vector<double>& residuals) const
{
    CHECK(points.size() == poses.size() && !points.empty());
    residuals.resize(points.size());

    if (residualType == CTriangulationResidualType::CReprojectionError)
    {
        for (size_t i = 0; i < points.size(); i++)
        {
            CHECK(poses[i].camera);
            residuals[i] = CalculateSquaredReprojectionError(points[i].point, XYZ, poses[i].projectionMatrix, *poses[i].camera);
        }
    }
    else if (residualType == CTriangulationResidualType::CAngularError)
    {
        for (size_t i = 0; i < points.size(); i++)
        {
            const double angularError = CalculateNormalizedAngularError(points[i].pointNormalized, XYZ, poses[i].projectionMatrix);
            residuals[i] = angularError * angularError;
        }
    }
    else
    {
        CHECK(false, "Unknown residualType");
    }
}


