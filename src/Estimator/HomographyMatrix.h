#pragma once
#include "../Base/types.h"
#include <vector>
#include <Eigen/Core>

// Direct linear transformation algorithm to compute the homography between
// point pairs. This algorithm computes the least squares estimate for
// the homography from at least 4 correspondences.
class HomographyMatrixEstimator {
public:
    typedef Eigen::Vector2d X_t;
    typedef Eigen::Vector2d Y_t;
    typedef Eigen::Matrix3d M_t;

    // The minimum number of samples needed to estimate a model.
    static const int kMinNumSamples = 4;

    // Estimate the projective transformation (homography).
    //
    // The number of corresponding points must be at least 4.
    //
    // @param points1    First set of corresponding points.
    // @param points2    Second set of corresponding points.
    //
    // @return         3x3 homogeneous transformation matrix.
    static std::vector<M_t> Estimate(const std::vector<X_t>& points1,
        const std::vector<Y_t>& points2);

    // Calculate the transformation error for each corresponding point pair.
    //
    // Residuals are defined as the squared transformation error when
    // transforming the source to the destination coordinates.
    //
    // @param points1    First set of corresponding points.
    // @param points2    Second set of corresponding points.
    // @param H          3x3 projective matrix.
    // @param residuals  Output vector of residuals.
    static void Residuals(const std::vector<X_t>& points1,
        const std::vector<Y_t>& points2,
        const M_t& H,
        std::vector<double>* residuals);
};


// Decompose an homography matrix into the possible rotations, translations,
// and plane normal vectors, according to:
//
//    Malis, Ezio, and Manuel Vargas. "Deeper understanding of the homography
//    decomposition for vision-based control." (2007): 90.
//
// The first pose is assumed to be P = [I | 0]. Note that the homography is
// plane-induced if `R.size() == t.size() == n.size() == 4`. If `R.size() ==
// t.size() == n.size() == 1` the homography is pure-rotational.
//
// @param H          3x3 homography matrix.
// @param K          3x3 calibration matrix.
// @param R          Possible 3x3 rotation matrices.
// @param t          Possible translation vectors.
// @param n          Possible normal vectors.
void DecomposeHomographyMatrix(const Eigen::Matrix3d& H,
    const Eigen::Matrix3d& K1,
    const Eigen::Matrix3d& K2,
    std::vector<Eigen::Matrix3d>* R,
    std::vector<Eigen::Vector3d>* t,
    std::vector<Eigen::Vector3d>* n);

// Recover the most probable pose from the given homography matrix.
//
// The pose of the first image is assumed to be P = [I | 0].
//
// @param H            3x3 homography matrix.
// @param K1           3x3 calibration matrix of first camera.
// @param K2           3x3 calibration matrix of second camera.
// @param points1      First set of corresponding points.
// @param points2      Second set of corresponding points.
// @param inlier_mask  Only points with `true` in the inlier mask are
//                     considered in the cheirality test. Size of the
//                     inlier mask must match the number of points N.
// @param R            Most probable 3x3 rotation matrix.
// @param t            Most probable 3x1 translation vector.
// @param n            Most probable 3x1 normal vector.
// @param points3D     Triangulated 3D points infront of camera
//                     (only if homography is not pure-rotational).
void PoseFromHomographyMatrix(const Eigen::Matrix3d& H,
    const Eigen::Matrix3d& K1,
    const Eigen::Matrix3d& K2,
    const std::vector<Eigen::Vector2d>& points1,
    const std::vector<Eigen::Vector2d>& points2,
    Eigen::Matrix3d* R,
    Eigen::Vector3d* t,
    Eigen::Vector3d* n,
    std::vector<Eigen::Vector3d>* points3D);

// Compose homography matrix from relative pose.
//
// @param K1      3x3 calibration matrix of first camera.
// @param K2      3x3 calibration matrix of second camera.
// @param R       Most probable 3x3 rotation matrix.
// @param t       Most probable 3x1 translation vector.
// @param n       Most probable 3x1 normal vector.
// @param d       Orthogonal distance from plane.
//
// @return        3x3 homography matrix.
Eigen::Matrix3d HomographyMatrixFromPose(const Eigen::Matrix3d& K1,
    const Eigen::Matrix3d& K2,
    const Eigen::Matrix3d& R,
    const Eigen::Vector3d& t,
    const Eigen::Vector3d& n,
    double d);