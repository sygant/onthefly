#pragma once
#include "../Base/math.h"
#include "../Base/types.h"
#include "../Estimator/RANSAC.h"
#include "../Scene/Camera.h"
#include <vector>
#include <Eigen/Core>


// Triangulate 3D point from corresponding image point observations.
//
// Implementation of the direct linear transform triangulation method in
//   R. Hartley and A. Zisserman, Multiple View Geometry in Computer Vision,
//   Cambridge Univ. Press, 2003.
//
// @param cam_from_world1   Projection matrix of the first image as 3x4 matrix.
// @param cam_from_world2   Projection matrix of the second image as 3x4 matrix.
// @param point1         Corresponding 2D point in first image.
// @param point2         Corresponding 2D point in second image.
//
// @return               Triangulated 3D point.
Eigen::Vector3d TriangulatePoint(const Eigen::Matrix3x4d& cam_from_world1,
    const Eigen::Matrix3x4d& cam_from_world2,
    const Eigen::Vector2d& point1,
    const Eigen::Vector2d& point2);

// Triangulate multiple 3D points from multiple image correspondences.
std::vector<Eigen::Vector3d> TriangulatePoints(
    const Eigen::Matrix3x4d& cam_from_world1,
    const Eigen::Matrix3x4d& cam_from_world2,
    const std::vector<Eigen::Vector2d>& points1,
    const std::vector<Eigen::Vector2d>& points2);

// Triangulate point from multiple views minimizing the L2 error.
//
// @param cams_from_world       Projection matrices of multi-view observations.
// @param points              Image observations of multi-view observations.
//
// @return                    Estimated 3D point.
Eigen::Vector3d TriangulateMultiViewPoint(
    const std::vector<Eigen::Matrix3x4d>& cams_from_world,
    const std::vector<Eigen::Vector2d>& points);

// Triangulate optimal 3D point from corresponding image point observations by
// finding the optimal image observations.
//
// Note that camera poses should be very good in order for this method to yield
// good results. Otherwise just use `TriangulatePoint`.
//
// Implementation of the method described in
//   P. Lindstrom, "Triangulation Made Easy," IEEE Computer Vision and Pattern
//   Recognition 2010, pp. 1554-1561, June 2010.
//
// @param cam_from_world1   Projection matrix of the first image as 3x4 matrix.
// @param cam_from_world2   Projection matrix of the second image as 3x4 matrix.
// @param point1         Corresponding 2D point in first image.
// @param point2         Corresponding 2D point in second image.
//
// @return               Triangulated optimal 3D point.
Eigen::Vector3d TriangulateOptimalPoint(
    const Eigen::Matrix3x4d& cam_from_world1,
    const Eigen::Matrix3x4d& cam_from_world2,
    const Eigen::Vector2d& point1,
    const Eigen::Vector2d& point2);

// Triangulate multiple optimal 3D points from multiple image correspondences.
std::vector<Eigen::Vector3d> TriangulateOptimalPoints(
    const Eigen::Matrix3x4d& cam_from_world1,
    const Eigen::Matrix3x4d& cam_from_world2,
    const std::vector<Eigen::Vector2d>& points1,
    const std::vector<Eigen::Vector2d>& points2);

// Calculate angle in radians between the two rays of a triangulated point.
double CalculateTriangulationAngle(const Eigen::Vector3d& proj_center1,
    const Eigen::Vector3d& proj_center2,
    const Eigen::Vector3d& point3D);
std::vector<double> CalculateTriangulationAngles(
    const Eigen::Vector3d& proj_center1,
    const Eigen::Vector3d& proj_center2,
    const std::vector<Eigen::Vector3d>& points3D);


// Triangulation estimator to estimate 3D point from multiple observations.
// The triangulation must satisfy the following constraints:
//    - Sufficient triangulation angle between observation pairs.
//    - All observations must satisfy cheirality constraint.
//
// An observation is composed of an image measurement and the corresponding
// camera pose and calibration.
class TriangulationEstimator {
public:
    enum class ResidualType {
        ANGULAR_ERROR,
        REPROJECTION_ERROR,
    };

    struct PointData {
        PointData() {}
        PointData(const Eigen::Vector2d& point_, const Eigen::Vector2d& point_N_)
            : point(point_), point_normalized(point_N_) {}
        // Image observation in pixels. Only needs to be set for REPROJECTION_ERROR.
        Eigen::Vector2d point;
        // Normalized image observation. Must always be set.
        Eigen::Vector2d point_normalized;
    };

    struct PoseData {
        PoseData() : camera(nullptr) {}
        PoseData(const Eigen::Matrix3x4d& proj_matrix_,
            const Eigen::Vector3d& pose_,
            const Camera* camera_)
            : proj_matrix(proj_matrix_), proj_center(pose_), camera(camera_) {}
        // The projection matrix for the image of the observation.
        Eigen::Matrix3x4d proj_matrix;
        // The projection center for the image of the observation.
        Eigen::Vector3d proj_center;
        // The camera for the image of the observation.
        const Camera* camera;
    };

    typedef PointData X_t;
    typedef PoseData Y_t;
    typedef Eigen::Vector3d M_t;

    // Specify settings for triangulation estimator.
    void SetMinTriAngle(double min_tri_angle);
    void SetResidualType(ResidualType residual_type);

    // The minimum number of samples needed to estimate a model.
    static const int kMinNumSamples = 2;

    // Estimate a 3D point from a two-view observation.
    //
    // @param point_data        Image measurement.
    // @param point_data        Camera poses.
    //
    // @return                  Triangulated point if successful, otherwise none.
    std::vector<M_t> Estimate(const std::vector<X_t>& point_data,
        const std::vector<Y_t>& pose_data) const;

    // Calculate residuals in terms of squared reprojection or angular error.
    //
    // @param point_data        Image measurements.
    // @param point_data        Camera poses.
    // @param xyz               3D point.
    //
    // @return                  Residual for each observation.
    void Residuals(const std::vector<X_t>& point_data,
        const std::vector<Y_t>& pose_data,
        const M_t& xyz,
        std::vector<double>* residuals) const;

private:
    ResidualType residual_type_ = ResidualType::REPROJECTION_ERROR;
    double min_tri_angle_ = 0.0;
};

struct EstimateTriangulationOptions {
    // Minimum triangulation angle in radians.
    double min_tri_angle = 0.0;

    // The employed residual type.
    TriangulationEstimator::ResidualType residual_type =
        TriangulationEstimator::ResidualType::ANGULAR_ERROR;

    // RANSAC options for TriangulationEstimator.
    RANSACOptions ransac_options;

    void Check() const {
        CHECK_GE(min_tri_angle, 0.0);
        ransac_options.Check();
    }
};

// Robustly estimate 3D point from observations in multiple views using RANSAC
// and a subsequent non-linear refinement using all inliers. Returns true
// if the estimated number of inliers has more than two views.
bool EstimateTriangulation(
    const EstimateTriangulationOptions& options,
    const std::vector<TriangulationEstimator::PointData>& point_data,
    const std::vector<TriangulationEstimator::PoseData>& pose_data,
    std::vector<char>* inlier_mask,
    Eigen::Vector3d* xyz);