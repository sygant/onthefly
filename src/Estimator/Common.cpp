#include "Common.h"
#include "../Base/logging.h"
#include <Eigen/Geometry>

void CenterAndNormalizeImagePoints(const std::vector<Eigen::Vector2d>& points,
    std::vector<Eigen::Vector2d>* normed_points,
    Eigen::Matrix3d* normed_from_orig) {
    const size_t num_points = points.size();
    CHECK_GT(num_points, 0);

    // Calculate centroid.
    Eigen::Vector2d centroid(0, 0);
    for (const Eigen::Vector2d& point : points) {
        centroid += point;
    }
    centroid /= num_points;

    // Root mean square distance to centroid of all points.
    double rms_mean_dist = 0;
    for (const Eigen::Vector2d& point : points) {
        rms_mean_dist += (point - centroid).squaredNorm();
    }
    rms_mean_dist = std::sqrt(rms_mean_dist / num_points);

    // Compose normalization matrix.
    const double norm_factor = std::sqrt(2.0) / rms_mean_dist;
    *normed_from_orig << norm_factor, 0, -norm_factor * centroid(0), 0,
        norm_factor, -norm_factor * centroid(1), 0, 0, 1;

    // Apply normalization matrix.
    normed_points->resize(num_points);
    for (size_t i = 0; i < num_points; ++i) {
        (*normed_points)[i] =
            (*normed_from_orig * points[i].homogeneous()).hnormalized();
    }
}

void ComputeSquaredSampsonError(const std::vector<Eigen::Vector2d>& points1,
    const std::vector<Eigen::Vector2d>& points2,
    const Eigen::Matrix3d& E,
    std::vector<double>* residuals) {
    const size_t num_points1 = points1.size();
    CHECK_EQ(num_points1, points2.size());
    residuals->resize(num_points1);
    for (size_t i = 0; i < num_points1; ++i) {
        const Eigen::Vector3d epipolar_line1 = E * points1[i].homogeneous();
        const Eigen::Vector3d point2_homogeneous = points2[i].homogeneous();
        const double num = point2_homogeneous.dot(epipolar_line1);
        const Eigen::Vector4d denom(point2_homogeneous.dot(E.col(0)),
            point2_homogeneous.dot(E.col(1)),
            epipolar_line1.x(),
            epipolar_line1.y());
        (*residuals)[i] = num * num / denom.squaredNorm();
    }
}

void ComputeSquaredReprojectionError(
    const std::vector<Eigen::Vector2d>& points2D,
    const std::vector<Eigen::Vector3d>& points3D,
    const Eigen::Matrix3x4d& cam_from_world,
    std::vector<double>* residuals) {
    const size_t num_points2D = points2D.size();
    CHECK_EQ(num_points2D, points3D.size());
    residuals->resize(num_points2D);
    for (size_t i = 0; i < num_points2D; ++i) {
        const Eigen::Vector3d point3D_in_cam =
            cam_from_world * points3D[i].homogeneous();
        // Check if 3D point is in front of camera.
        if (point3D_in_cam.z() > std::numeric_limits<double>::epsilon()) {
            (*residuals)[i] =
                (point3D_in_cam.hnormalized() - points2D[i]).squaredNorm();
        }
        else {
            (*residuals)[i] = std::numeric_limits<double>::max();
        }
    }
}