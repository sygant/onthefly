#pragma once
#include "../Base/types.h"
#include "../Geometry/Rigid3D.h"
#include <vector>
#include <Eigen/Core>
#include <ceres/ceres.h>

// Essential matrix estimator from corresponding normalized point pairs.
//
// This algorithm solves the 5-Point problem based on the following paper:
//
//    D. Nister, An efficient solution to the five-point relative pose problem,
//    IEEE-T-PAMI, 26(6), 2004.
//    http://citeseerx.ist.psu.edu/viewdoc/summary?doi=10.1.1.86.8769
class EssentialMatrixFivePointEstimator {
public:
    typedef Eigen::Vector2d X_t;
    typedef Eigen::Vector2d Y_t;
    typedef Eigen::Matrix3d M_t;

    // The minimum number of samples needed to estimate a model.
    static const int kMinNumSamples = 5;

    // Estimate up to 10 possible essential matrix solutions from a set of
    // corresponding points.
    //
    //  The number of corresponding points must be at least 5.
    //
    // @param points1  First set of corresponding points.
    // @param points2  Second set of corresponding points.
    //
    // @return         Up to 10 solutions as a vector of 3x3 essential matrices.
    static std::vector<M_t> Estimate(const std::vector<X_t>& points1,
        const std::vector<Y_t>& points2);

    // Calculate the residuals of a set of corresponding points and a given
    // essential matrix.
    //
    // Residuals are defined as the squared Sampson error.
    //
    // @param points1    First set of corresponding points.
    // @param points2    Second set of corresponding points.
    // @param E          3x3 essential matrix.
    // @param residuals  Output vector of residuals.
    static void Residuals(const std::vector<X_t>& points1,
        const std::vector<Y_t>& points2,
        const M_t& E,
        std::vector<double>* residuals);
};

// Essential matrix estimator from corresponding normalized point pairs.
//
// This algorithm solves the 8-Point problem based on the following paper:
//
//    Hartley and Zisserman, Multiple View Geometry, algorithm 11.1, page 282.
class EssentialMatrixEightPointEstimator {
public:
    typedef Eigen::Vector2d X_t;
    typedef Eigen::Vector2d Y_t;
    typedef Eigen::Matrix3d M_t;

    // The minimum number of samples needed to estimate a model.
    static const int kMinNumSamples = 8;

    // Estimate essential matrix solutions from  set of corresponding points.
    //
    // The number of corresponding points must be at least 8.
    //
    // @param points1  First set of corresponding points.
    // @param points2  Second set of corresponding points.
    static std::vector<M_t> Estimate(const std::vector<X_t>& points1,
        const std::vector<Y_t>& points2);

    // Calculate the residuals of a set of corresponding points and a given
    // essential matrix.
    //
    // Residuals are defined as the squared Sampson error.
    //
    // @param points1    First set of corresponding points.
    // @param points2    Second set of corresponding points.
    // @param E          3x3 essential matrix.
    // @param residuals  Output vector of residuals.
    static void Residuals(const std::vector<X_t>& points1,
        const std::vector<Y_t>& points2,
        const M_t& E,
        std::vector<double>* residuals);
};


// Decompose an essential matrix into the possible rotations and translations.
//
// The first pose is assumed to be P = [I | 0] and the set of four other
// possible second poses are defined as: {[R1 | t], [R2 | t],
//                                        [R1 | -t], [R2 | -t]}
//
// @param E          3x3 essential matrix.
// @param R1         First possible 3x3 rotation matrix.
// @param R2         Second possible 3x3 rotation matrix.
// @param t          3x1 possible translation vector (also -t possible).
void DecomposeEssentialMatrix(const Eigen::Matrix3d& E,
    Eigen::Matrix3d* R1,
    Eigen::Matrix3d* R2,
    Eigen::Vector3d* t);

// Recover the most probable pose from the given essential matrix.
//
// The pose of the first image is assumed to be P = [I | 0].
//
// @param E            3x3 essential matrix.
// @param points1      First set of corresponding points.
// @param points2      Second set of corresponding points.
// @param inlier_mask  Only points with `true` in the inlier mask are
//                     considered in the cheirality test. Size of the
//                     inlier mask must match the number of points N.
// @param R            Most probable 3x3 rotation matrix.
// @param t            Most probable 3x1 translation vector.
// @param points3D     Triangulated 3D points infront of camera.
void PoseFromEssentialMatrix(const Eigen::Matrix3d& E,
    const std::vector<Eigen::Vector2d>& points1,
    const std::vector<Eigen::Vector2d>& points2,
    Eigen::Matrix3d* R,
    Eigen::Vector3d* t,
    std::vector<Eigen::Vector3d>* points3D);

// Compose essential matrix from relative camera poses.
//
// Assumes that first camera pose has projection matrix P = [I | 0], and
// pose of second camera is given as transformation from world to camera system.
//
// @param R             3x3 rotation matrix.
// @param t             3x1 translation vector.
//
// @return              3x3 essential matrix.
Eigen::Matrix3d EssentialMatrixFromPose(const Rigid3d& cam2_from_cam1);

// Find optimal image points, such that:
//
//     optimal_point1^t * E * optimal_point2 = 0
//
// as described in:
//
//   Lindstrom, P., "Triangulation made easy",
//   Computer Vision and Pattern Recognition (CVPR),
//   2010 IEEE Conference on , vol., no., pp.1554,1561, 13-18 June 2010
//
// @param E                Essential or fundamental matrix.
// @param point1           Corresponding 2D point in first image.
// @param point2           Corresponding 2D point in second image.
// @param optimal_point1   Estimated optimal image point in the first image.
// @param optimal_point2   Estimated optimal image point in the second image.
void FindOptimalImageObservations(const Eigen::Matrix3d& E,
    const Eigen::Vector2d& point1,
    const Eigen::Vector2d& point2,
    Eigen::Vector2d* optimal_point1,
    Eigen::Vector2d* optimal_point2);

// Compute the location of the epipole in homogeneous coordinates.
//
// @param E           3x3 essential matrix.
// @param left_image  If true, epipole in left image is computed,
//                    else in right image.
//
// @return            Epipole in homogeneous coordinates.
Eigen::Vector3d EpipoleFromEssentialMatrix(const Eigen::Matrix3d& E,
    bool left_image);

// Invert the essential matrix, i.e. if the essential matrix E describes the
// transformation from camera A to B, the inverted essential matrix E' describes
// the transformation from camera B to A.
//
// @param E      3x3 essential matrix.
//
// @return       Inverted essential matrix.
Eigen::Matrix3d InvertEssentialMatrix(const Eigen::Matrix3d& matrix);