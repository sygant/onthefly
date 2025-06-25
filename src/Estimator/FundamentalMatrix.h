#pragma once
#include "HomographyMatrix.h"
#include "../Base/types.h"

#include <vector>
#include <Eigen/Core>

// Fundamental matrix estimator from corresponding point pairs.
//
// This algorithm solves the 7-Point problem and is based on the following
// paper:
//
//    Zhengyou Zhang and T. Kanade, Determining the Epipolar Geometry and its
//    Uncertainty: A Review, International Journal of Computer Vision, 1998.
//    http://citeseerx.ist.psu.edu/viewdoc/summary?doi=10.1.1.33.4540
class FundamentalMatrixSevenPointEstimator {
public:
    typedef Eigen::Vector2d X_t;
    typedef Eigen::Vector2d Y_t;
    typedef Eigen::Matrix3d M_t;

    // The minimum number of samples needed to estimate a model.
    static const int kMinNumSamples = 7;

    // Estimate either 1 or 3 possible fundamental matrix solutions from a set of
    // corresponding points.
    //
    // The number of corresponding points must be exactly 7.
    //
    // @param points1  First set of corresponding points.
    // @param points2  Second set of corresponding points
    //
    // @return         Up to 4 solutions as a vector of 3x3 fundamental matrices.
    static std::vector<M_t> Estimate(const std::vector<X_t>& points1,
        const std::vector<Y_t>& points2);

    // Calculate the residuals of a set of corresponding points and a given
    // fundamental matrix.
    //
    // Residuals are defined as the squared Sampson error.
    //
    // @param points1    First set of corresponding points as Nx2 matrix.
    // @param points2    Second set of corresponding points as Nx2 matrix.
    // @param F          3x3 fundamental matrix.
    // @param residuals  Output vector of residuals.
    static void Residuals(const std::vector<X_t>& points1,
        const std::vector<Y_t>& points2,
        const M_t& F,
        std::vector<double>* residuals);
};

// Fundamental matrix estimator from corresponding point pairs.
//
// This algorithm solves the 8-Point problem based on the following paper:
//
//    Hartley and Zisserman, Multiple View Geometry, algorithm 11.1, page 282.
class FundamentalMatrixEightPointEstimator {
public:
    typedef Eigen::Vector2d X_t;
    typedef Eigen::Vector2d Y_t;
    typedef Eigen::Matrix3d M_t;

    // The minimum number of samples needed to estimate a model.
    static const int kMinNumSamples = 8;

    // Estimate fundamental matrix solutions from a set of corresponding points.
    //
    // The number of corresponding points must be at least 8.
    //
    // @param points1  First set of corresponding points.
    // @param points2  Second set of corresponding points
    //
    // @return         Single solution as a vector of 3x3 fundamental matrices.
    static std::vector<M_t> Estimate(const std::vector<X_t>& points1,
        const std::vector<Y_t>& points2);

    // Calculate the residuals of a set of corresponding points and a given
    // fundamental matrix.
    //
    // Residuals are defined as the squared Sampson error.
    //
    // @param points1    First set of corresponding points as Nx2 matrix.
    // @param points2    Second set of corresponding points as Nx2 matrix.
    // @param F          3x3 fundamental matrix.
    // @param residuals  Output vector of residuals.
    static void Residuals(const std::vector<X_t>& points1,
        const std::vector<Y_t>& points2,
        const M_t& F,
        std::vector<double>* residuals);
};