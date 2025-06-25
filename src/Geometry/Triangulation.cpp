#include "Triangulation.h"
#include "Pose.h"
#include "../Estimator/EssentialMatrix.h"
#include "../Estimator/Sampler.h"
#include "../Scene/Projection.h"
#include "../Base/logging.h"

#include <Eigen/Dense>
#include <Eigen/Geometry>


Eigen::Vector3d TriangulatePoint(const Eigen::Matrix3x4d& cam1_from_world,
    const Eigen::Matrix3x4d& cam2_from_world,
    const Eigen::Vector2d& point1,
    const Eigen::Vector2d& point2) {
    Eigen::Matrix4d A;

    A.row(0) = point1(0) * cam1_from_world.row(2) - cam1_from_world.row(0);
    A.row(1) = point1(1) * cam1_from_world.row(2) - cam1_from_world.row(1);
    A.row(2) = point2(0) * cam2_from_world.row(2) - cam2_from_world.row(0);
    A.row(3) = point2(1) * cam2_from_world.row(2) - cam2_from_world.row(1);

    Eigen::JacobiSVD<Eigen::Matrix4d> svd(A, Eigen::ComputeFullV);

    return svd.matrixV().col(3).hnormalized();
}

std::vector<Eigen::Vector3d> TriangulatePoints(
    const Eigen::Matrix3x4d& cam1_from_world,
    const Eigen::Matrix3x4d& cam2_from_world,
    const std::vector<Eigen::Vector2d>& points1,
    const std::vector<Eigen::Vector2d>& points2) {
    CHECK_EQ(points1.size(), points2.size());

    std::vector<Eigen::Vector3d> points3D(points1.size());

    for (size_t i = 0; i < points3D.size(); ++i) {
        points3D[i] = TriangulatePoint(
            cam1_from_world, cam2_from_world, points1[i], points2[i]);
    }

    return points3D;
}

Eigen::Vector3d TriangulateMultiViewPoint(
    const std::vector<Eigen::Matrix3x4d>& cams_from_world,
    const std::vector<Eigen::Vector2d>& points) {
    CHECK_EQ(cams_from_world.size(), points.size());

    Eigen::Matrix4d A = Eigen::Matrix4d::Zero();

    for (size_t i = 0; i < points.size(); i++) {
        const Eigen::Vector3d point = points[i].homogeneous().normalized();
        const Eigen::Matrix3x4d term =
            cams_from_world[i] - point * point.transpose() * cams_from_world[i];
        A += term.transpose() * term;
    }

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> eigen_solver(A);

    return eigen_solver.eigenvectors().col(0).hnormalized();
}

Eigen::Vector3d TriangulateOptimalPoint(
    const Eigen::Matrix3x4d& cam1_from_world_mat,
    const Eigen::Matrix3x4d& cam2_from_world_mat,
    const Eigen::Vector2d& point1,
    const Eigen::Vector2d& point2) {
    const Rigid3d cam1_from_world(
        Eigen::Quaterniond(cam1_from_world_mat.leftCols<3>()),
        cam1_from_world_mat.col(3));
    const Rigid3d cam2_from_world(
        Eigen::Quaterniond(cam2_from_world_mat.leftCols<3>()),
        cam2_from_world_mat.col(3));
    const Rigid3d cam2_from_cam1 = cam2_from_world * Inverse(cam1_from_world);
    const Eigen::Matrix3d E = EssentialMatrixFromPose(cam2_from_cam1);

    Eigen::Vector2d optimal_point1;
    Eigen::Vector2d optimal_point2;
    FindOptimalImageObservations(
        E, point1, point2, &optimal_point1, &optimal_point2);

    return TriangulatePoint(
        cam1_from_world_mat, cam2_from_world_mat, optimal_point1, optimal_point2);
}

std::vector<Eigen::Vector3d> TriangulateOptimalPoints(
    const Eigen::Matrix3x4d& cam1_from_world,
    const Eigen::Matrix3x4d& cam2_from_world,
    const std::vector<Eigen::Vector2d>& points1,
    const std::vector<Eigen::Vector2d>& points2) {
    std::vector<Eigen::Vector3d> points3D(points1.size());

    for (size_t i = 0; i < points3D.size(); ++i) {
        points3D[i] = TriangulateOptimalPoint(
            cam1_from_world, cam2_from_world, points1[i], points2[i]);
    }

    return points3D;
}

double CalculateTriangulationAngle(const Eigen::Vector3d& proj_center1,
    const Eigen::Vector3d& proj_center2,
    const Eigen::Vector3d& point3D) {
    const double baseline_length_squared =
        (proj_center1 - proj_center2).squaredNorm();

    const double ray_length_squared1 = (point3D - proj_center1).squaredNorm();
    const double ray_length_squared2 = (point3D - proj_center2).squaredNorm();

    // Using "law of cosines" to compute the enclosing angle between rays.
    const double denominator =
        2.0 * std::sqrt(ray_length_squared1 * ray_length_squared2);
    if (denominator == 0.0) {
        return 0.0;
    }
    const double nominator =
        ray_length_squared1 + ray_length_squared2 - baseline_length_squared;
    const double angle = std::abs(std::acos(nominator / denominator));

    // Triangulation is unstable for acute angles (far away points) and
    // obtuse angles (close points), so always compute the minimum angle
    // between the two intersecting rays.
    return std::min(angle, M_PI - angle);
}

std::vector<double> CalculateTriangulationAngles(
    const Eigen::Vector3d& proj_center1,
    const Eigen::Vector3d& proj_center2,
    const std::vector<Eigen::Vector3d>& points3D) {
    // Baseline length between camera centers.
    const double baseline_length_squared =
        (proj_center1 - proj_center2).squaredNorm();

    std::vector<double> angles(points3D.size());

    for (size_t i = 0; i < points3D.size(); ++i) {
        // Ray lengths from cameras to point.
        const double ray_length_squared1 =
            (points3D[i] - proj_center1).squaredNorm();
        const double ray_length_squared2 =
            (points3D[i] - proj_center2).squaredNorm();

        // Using "law of cosines" to compute the enclosing angle between rays.
        const double denominator =
            2.0 * std::sqrt(ray_length_squared1 * ray_length_squared2);
        if (denominator == 0.0) {
            angles[i] = 0.0;
            continue;
        }
        const double nominator =
            ray_length_squared1 + ray_length_squared2 - baseline_length_squared;
        const double angle = std::abs(std::acos(nominator / denominator));

        // Triangulation is unstable for acute angles (far away points) and
        // obtuse angles (close points), so always compute the minimum angle
        // between the two intersecting rays.
        angles[i] = std::min(angle, M_PI - angle);
    }

    return angles;
}

void TriangulationEstimator::SetMinTriAngle(const double min_tri_angle) {
    CHECK_GE(min_tri_angle, 0);
    min_tri_angle_ = min_tri_angle;
}

void TriangulationEstimator::SetResidualType(const ResidualType residual_type) {
    residual_type_ = residual_type;
}

std::vector<TriangulationEstimator::M_t> TriangulationEstimator::Estimate(
    const std::vector<X_t>& point_data,
    const std::vector<Y_t>& pose_data) const {
    CHECK_GE(point_data.size(), 2);
    CHECK_EQ(point_data.size(), pose_data.size());

    if (point_data.size() == 2) {
        // Two-view triangulation.

        const M_t xyz = TriangulatePoint(pose_data[0].proj_matrix,
            pose_data[1].proj_matrix,
            point_data[0].point_normalized,
            point_data[1].point_normalized);

        if (HasPointPositiveDepth(pose_data[0].proj_matrix, xyz) &&
            HasPointPositiveDepth(pose_data[1].proj_matrix, xyz) &&
            CalculateTriangulationAngle(pose_data[0].proj_center,
                pose_data[1].proj_center,
                xyz) >= min_tri_angle_) {
            return std::vector<M_t>{xyz};
        }
    }
    else {
        // Multi-view triangulation.

        std::vector<Eigen::Matrix3x4d> proj_matrices;
        proj_matrices.reserve(point_data.size());
        std::vector<Eigen::Vector2d> points;
        points.reserve(point_data.size());
        for (size_t i = 0; i < point_data.size(); ++i) {
            proj_matrices.push_back(pose_data[i].proj_matrix);
            points.push_back(point_data[i].point_normalized);
        }

        const M_t xyz = TriangulateMultiViewPoint(proj_matrices, points);

        // Check for cheirality constraint.
        for (const auto& pose : pose_data) {
            if (!HasPointPositiveDepth(pose.proj_matrix, xyz)) {
                return std::vector<M_t>();
            }
        }

        // Check for sufficient triangulation angle.
        for (size_t i = 0; i < pose_data.size(); ++i) {
            for (size_t j = 0; j < i; ++j) {
                const double tri_angle = CalculateTriangulationAngle(
                    pose_data[i].proj_center, pose_data[j].proj_center, xyz);
                if (tri_angle >= min_tri_angle_) {
                    return std::vector<M_t>{xyz};
                }
            }
        }
    }

    return std::vector<M_t>();
}

void TriangulationEstimator::Residuals(const std::vector<X_t>& point_data,
    const std::vector<Y_t>& pose_data,
    const M_t& xyz,
    std::vector<double>* residuals) const {
    CHECK_EQ(point_data.size(), pose_data.size());

    residuals->resize(point_data.size());

    for (size_t i = 0; i < point_data.size(); ++i) {
        if (residual_type_ == ResidualType::REPROJECTION_ERROR) {
            (*residuals)[i] =
                CalculateSquaredReprojectionError(point_data[i].point,
                    xyz,
                    pose_data[i].proj_matrix,
                    *pose_data[i].camera);
        }
        else if (residual_type_ == ResidualType::ANGULAR_ERROR) {
            const double angular_error = CalculateNormalizedAngularError(
                point_data[i].point_normalized, xyz, pose_data[i].proj_matrix);
            (*residuals)[i] = angular_error * angular_error;
        }
    }
}

bool EstimateTriangulation(
    const EstimateTriangulationOptions& options,
    const std::vector<TriangulationEstimator::PointData>& point_data,
    const std::vector<TriangulationEstimator::PoseData>& pose_data,
    std::vector<char>* inlier_mask,
    Eigen::Vector3d* xyz) {
    CHECK_NOTNULL(inlier_mask);
    CHECK_NOTNULL(xyz);
    CHECK_GE(point_data.size(), 2);
    CHECK_EQ(point_data.size(), pose_data.size());
    options.Check();

    // Robustly estimate track using LORANSAC.
    LORANSAC<TriangulationEstimator,
        TriangulationEstimator,
        InlierSupportMeasurer,
        CombinationSampler>
        ransac(options.ransac_options);
    ransac.estimator.SetMinTriAngle(options.min_tri_angle);
    ransac.estimator.SetResidualType(options.residual_type);
    ransac.local_estimator.SetMinTriAngle(options.min_tri_angle);
    ransac.local_estimator.SetResidualType(options.residual_type);
    const auto report = ransac.Estimate(point_data, pose_data);
    if (!report.success) {
        return false;
    }

    *inlier_mask = report.inlier_mask;
    *xyz = report.model;

    return report.success;
}