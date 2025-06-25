#pragma once
#include "../Base/types.h"
#include <vector>
#include <windows.h>
#include <Eigen/Core>


// Center and normalize image points.
//
// The points are transformed in a two-step procedure that is expressed
// as a transformation matrix. The matrix of the resulting points is usually
// better conditioned than the matrix of the original points.
//
// Center the image points, such that the new coordinate system has its
// origin at the centroid of the image points.
//
// Normalize the image points, such that the mean distance from the points
// to the coordinate system is sqrt(2).
//
// @param points            Image coordinates.
// @param normed_points     Transformed image coordinates.
// @param normed_from_orig  3x3 transformation matrix.
void CenterAndNormalizeImagePoints(const std::vector<Eigen::Vector2d>& points,
    std::vector<Eigen::Vector2d>* normed_points,
    Eigen::Matrix3d* normed_from_orig);

// Calculate the residuals of a set of corresponding points and a given
// fundamental or essential matrix.
//
// Residuals are defined as the squared Sampson error.
//
// @param points1     First set of corresponding points as Nx2 matrix.
// @param points2     Second set of corresponding points as Nx2 matrix.
// @param E           3x3 fundamental or essential matrix.
// @param residuals   Output vector of residuals.
void ComputeSquaredSampsonError(const std::vector<Eigen::Vector2d>& points1,
    const std::vector<Eigen::Vector2d>& points2,
    const Eigen::Matrix3d& E,
    std::vector<double>* residuals);

// Calculate the squared reprojection error given a set of 2D-3D point
// correspondences and a projection matrix. Returns DBL_MAX if a 3D point is
// behind the given camera.
//
// @param points2D      Normalized 2D image points.
// @param points3D      3D world points.
// @param proj_matrix   3x4 projection matrix.
// @param residuals     Output vector of residuals.
void ComputeSquaredReprojectionError(
    const std::vector<Eigen::Vector2d>& points2D,
    const std::vector<Eigen::Vector3d>& points3D,
    const Eigen::Matrix3x4d& cam_from_world,
    std::vector<double>* residuals);