#include "HomographyMatrix.h"
#include "Pose.h"

#include "../Base/Math.h"

using namespace std;



double ComputeOppositeOfMinor(const Eigen::Matrix3d& matrix, const size_t row, const size_t col) 
{
    const size_t col1 = col == 0 ? 1 : 0;
    const size_t col2 = col == 2 ? 1 : 2;
    const size_t row1 = row == 0 ? 1 : 0;
    const size_t row2 = row == 2 ? 1 : 2;
    return (matrix(row1, col2) * matrix(row2, col1) - matrix(row1, col1) * matrix(row2, col2));
}
Eigen::Matrix3d ComputeHomographyRotation(const Eigen::Matrix3d& H_normalized, const Eigen::Vector3d& tstar, const Eigen::Vector3d& n, const double v)
{
    return H_normalized * (Eigen::Matrix3d::Identity() - (2.0 / v) * tstar * n.transpose());
}

void DecomposeHomographyMatrix(const Eigen::Matrix3d& H, const Eigen::Matrix3d& K1, const Eigen::Matrix3d& K2, vector<Eigen::Matrix3d>& R, vector<Eigen::Vector3d>& t, vector<Eigen::Vector3d>& n)
{
    Eigen::Matrix3d H_normalized = K2.inverse() * H * K1;

    // 从归一化的单应性矩阵中移除尺度因子
    Eigen::JacobiSVD<Eigen::Matrix3d> H_norm_svd(H_normalized);
    H_normalized.array() /= H_norm_svd.singularValues()[1];

    // 确保我们总是返回旋转, 而不是反射
    //
    // 取det(H_normalized)>0就足够了
    //
    // - 在论文中: R := H_normalized * (Id + x y^t)^{-1} (32页).
    // - 可以验证R是正交的: RR^t = Id.
    // - 为了返回一个旋转, 我们还需要 det(R) > 0.
    // - 根据Sylvester恒等式: det(Id + x y^t) = (1 + x^t y), 通过选择合适的x和y, 这个行列式的值可以为正数 (24页).
    // - 所以 det(R) 和 det(H_normalized) 是同号的
    if (H_normalized.determinant() < 0) 
    {
        H_normalized.array() *= -1.0;
    }

    const Eigen::Matrix3d S = H_normalized.transpose() * H_normalized - Eigen::Matrix3d::Identity();

    // 检查H是否是旋转矩阵
    if (S.lpNorm<Eigen::Infinity>() < 1e-3)
    {  // 纯旋转
        R = { H_normalized };
        t = { Eigen::Vector3d::Zero() };
        n = { Eigen::Vector3d::Zero() };
        return;
    }

    const double M00 = ComputeOppositeOfMinor(S, 0, 0);
    const double M11 = ComputeOppositeOfMinor(S, 1, 1);
    const double M22 = ComputeOppositeOfMinor(S, 2, 2);

    const double rtM00 = sqrt(M00);
    const double rtM11 = sqrt(M11);
    const double rtM22 = sqrt(M22);

    const double M01 = ComputeOppositeOfMinor(S, 0, 1);
    const double M12 = ComputeOppositeOfMinor(S, 1, 2);
    const double M02 = ComputeOppositeOfMinor(S, 0, 2);

    const int e12 = SignOfNumber(M12);
    const int e02 = SignOfNumber(M02);
    const int e01 = SignOfNumber(M01);

    const double nS00 = abs(S(0, 0));
    const double nS11 = abs(S(1, 1));
    const double nS22 = abs(S(2, 2));

    const array<double, 3> nS{ {nS00, nS11, nS22} };
    const size_t index = distance(nS.begin(), max_element(nS.begin(), nS.end()));

    Eigen::Vector3d np1;
    Eigen::Vector3d np2;
    if (index == 0)
    {
        np1[0] = S(0, 0);
        np2[0] = S(0, 0);
        np1[1] = S(0, 1) + rtM22;
        np2[1] = S(0, 1) - rtM22;
        np1[2] = S(0, 2) + e12 * rtM11;
        np2[2] = S(0, 2) - e12 * rtM11;
    }
    else if (index == 1)
    {
        np1[0] = S(0, 1) + rtM22;
        np2[0] = S(0, 1) - rtM22;
        np1[1] = S(1, 1);
        np2[1] = S(1, 1);
        np1[2] = S(1, 2) - e02 * rtM00;
        np2[2] = S(1, 2) + e02 * rtM00;
    }
    else if (index == 2)
    {
        np1[0] = S(0, 2) + e01 * rtM11;
        np2[0] = S(0, 2) - e01 * rtM11;
        np1[1] = S(1, 2) + rtM00;
        np2[1] = S(1, 2) - rtM00;
        np1[2] = S(2, 2);
        np2[2] = S(2, 2);
    }

    const double traceS = S.trace();
    const double v = 2.0 * sqrt(1.0 + traceS - M00 - M11 - M22);

    const double ESii = SignOfNumber(S(index, index));
    const double r_2 = 2 + traceS + v;
    const double nt_2 = 2 + traceS - v;

    const double r = sqrt(r_2);
    const double n_t = sqrt(nt_2);

    const Eigen::Vector3d n1 = np1.normalized();
    const Eigen::Vector3d n2 = np2.normalized();

    const double half_nt = 0.5 * n_t;
    const double esii_t_r = ESii * r;

    const Eigen::Vector3d t1_star = half_nt * (esii_t_r * n2 - n_t * n1);
    const Eigen::Vector3d t2_star = half_nt * (esii_t_r * n1 - n_t * n2);

    const Eigen::Matrix3d R1 = ComputeHomographyRotation(H_normalized, t1_star, n1, v);
    const Eigen::Vector3d t1 = R1 * t1_star;

    const Eigen::Matrix3d R2 = ComputeHomographyRotation(H_normalized, t2_star, n2, v);
    const Eigen::Vector3d t2 = R2 * t2_star;

    R = { R1, R1, R2, R2 };
    t = { t1, -t1, t2, -t2 };
    n = { -n1, n1, -n2, n2 };
}
void HomographyMatrixToPose(const Eigen::Matrix3d& H, const Eigen::Matrix3d& K1, const Eigen::Matrix3d& K2, const vector<Eigen::Vector2d>& points1, const vector<Eigen::Vector2d>& points2, Eigen::Matrix3d& R, Eigen::Vector3d& t, Eigen::Vector3d& n, vector<Eigen::Vector3d>& points3D)
{
    CHECK(points1.size() == points2.size());
    vector<Eigen::Matrix3d> R_cmbs;
    vector<Eigen::Vector3d> t_cmbs;
    vector<Eigen::Vector3d> n_cmbs;

    DecomposeHomographyMatrix(H, K1, K2, R_cmbs, t_cmbs, n_cmbs);
    points3D.clear();
    for (size_t i = 0; i < R_cmbs.size(); i++)
    {
        vector<Eigen::Vector3d> points3D_cmb;
        CheckCheirality(R_cmbs[i], t_cmbs[i], points1, points2, points3D_cmb);
        if (points3D_cmb.size() >= points3D.size())
        {
            R = R_cmbs[i];
            t = t_cmbs[i];
            n = n_cmbs[i];
            points3D = points3D_cmb;
        }
    }
}
Eigen::Matrix3d PoseToHomographyMatrix(const Eigen::Matrix3d& K1, const Eigen::Matrix3d& K2, const Eigen::Matrix3d& R, const Eigen::Vector3d& t, const Eigen::Vector3d& n, double d)
{
    CHECK(d > 0);
    return K2 * (R - t * n.normalized().transpose() / d) * K1.inverse();
}