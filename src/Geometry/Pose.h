#pragma once
#include "Rigid3D.h"
#include "Sim3D.h"
#include "../Base/types.h"
#include "../Base/logging.h"
#include "../Estimator/RANSAC.h"
#include "../Scene/Camera.h"
#include "../Scene/CameraModel.h"

#include <array>
#include <vector>
#include <Eigen/Core>
#include <ceres/ceres.h>

// Compute the closes rotation matrix with the closest Frobenius norm by setting
// the singular values of the given matrix to 1.
Eigen::Matrix3d ComputeClosestRotationMatrix(const Eigen::Matrix3d& matrix);

// Decompose projection matrix into intrinsic camera matrix, rotation matrix and
// translation vector. Returns false if decomposition fails.
bool DecomposeProjectionMatrix(const Eigen::Matrix3x4d& proj_matrix,
    Eigen::Matrix3d* K,
    Eigen::Matrix3d* R,
    Eigen::Vector3d* T);

// Compose the skew symmetric cross product matrix from a vector.
Eigen::Matrix3d CrossProductMatrix(const Eigen::Vector3d& vector);

// Convert 3D rotation matrix to Euler angles.
//
// The convention `R = Rx * Ry * Rz` is used,
// using a right-handed coordinate system.
//
// @param R              3x3 rotation matrix.
// @param rx, ry, rz     Euler angles in radians.
void RotationMatrixToEulerAngles(const Eigen::Matrix3d& R,
    double* rx,
    double* ry,
    double* rz);

// Convert Euler angles to 3D rotation matrix.
//
// The convention `R = Rz * Ry * Rx` is used,
// using a right-handed coordinate system.
//
// @param rx, ry, rz     Euler angles in radians.
//
// @return               3x3 rotation matrix.
Eigen::Matrix3d EulerAnglesToRotationMatrix(double rx, double ry, double rz);

// Compute the weighted average of multiple Quaternions according to:
//
//    Markley, F. Landis, et al. "Averaging quaternions."
//    Journal of Guidance, Control, and Dynamics 30.4 (2007): 1193-1197.
//
// @param quats         The Quaternions to be averaged.
// @param weights       Non-negative weights.
//
// @return              The average Quaternion.
Eigen::Quaterniond AverageQuaternions(
    const std::vector<Eigen::Quaterniond>& quats,
    const std::vector<double>& weights);

// Linearly interpolate camera pose.
Rigid3d InterpolateCameraPoses(const Rigid3d& cam_from_world1,
    const Rigid3d& cam_from_world2,
    double t);

// Perform cheirality constraint test, i.e., determine which of the triangulated
// correspondences lie in front of of both cameras. The first camera has the
// projection matrix P1 = [I | 0] and the second camera has the projection
// matrix P2 = [R | t].
//
// @param R            3x3 rotation matrix of second projection matrix.
// @param t            3x1 translation vector of second projection matrix.
// @param points1      First set of corresponding points.
// @param points2      Second set of corresponding points.
// @param points3D     Points that lie in front of both cameras.
bool CheckCheirality(const Eigen::Matrix3d& R,
    const Eigen::Vector3d& t,
    const std::vector<Eigen::Vector2d>& points1,
    const std::vector<Eigen::Vector2d>& points2,
    std::vector<Eigen::Vector3d>* points3D);

Rigid3d TransformCameraWorld(const Sim3d& new_from_old_world,
    const Rigid3d& cam_from_world);

struct AbsolutePoseEstimationOptions {
    // Whether to estimate the focal length.
    bool estimate_focal_length = false;

    // Number of discrete samples for focal length estimation.
    size_t num_focal_length_samples = 30;

    // Minimum focal length ratio for discrete focal length sampling
    // around focal length of given camera.
    double min_focal_length_ratio = 0.2;

    // Maximum focal length ratio for discrete focal length sampling
    // around focal length of given camera.
    double max_focal_length_ratio = 5;

    // Number of threads for parallel estimation of focal length.
    // int num_threads = ThreadPool::kMaxNumThreads;

    // Options used for P3P RANSAC.
    RANSACOptions ransac_options;

    void Check() const {
        CHECK_GT(num_focal_length_samples, 0);
        CHECK_GT(min_focal_length_ratio, 0);
        CHECK_GT(max_focal_length_ratio, 0);
        CHECK_LT(min_focal_length_ratio, max_focal_length_ratio);
        ransac_options.Check();
    }
};

struct AbsolutePoseRefinementOptions {
    // Convergence criterion.
    double gradient_tolerance = 1.0;

    // Maximum number of solver iterations.
    int max_num_iterations = 100;

    // Scaling factor determines at which residual robustification takes place.
    double loss_function_scale = 1.0;

    // Whether to refine the focal length parameter group.
    bool refine_focal_length = true;

    // Whether to refine the extra parameter group.
    bool refine_extra_params = true;

#ifdef _DEBUG
    // Whether to print final summary.
    bool print_summary = true;
#else
    // Whether to print final summary.
    bool print_summary = false;
#endif // _DEBUG

    

    void Check() const {
        CHECK_GE(gradient_tolerance, 0.0);
        CHECK_GE(max_num_iterations, 0);
        CHECK_GE(loss_function_scale, 0.0);
    }
};

// Analytic solver for the P3P (Perspective-Three-Point) problem.
//
// The algorithm is based on the following paper:
//
//    X.S. Gao, X.-R. Hou, J. Tang, H.-F. Chang. Complete Solution
//    Classification for the Perspective-Three-Point Problem.
//    http://www.mmrc.iss.ac.cn/~xgao/paper/ieee.pdf
class P3PEstimator {
public:
    // The 2D image feature observations.
    typedef Eigen::Vector2d X_t;
    // The observed 3D features in the world frame.
    typedef Eigen::Vector3d Y_t;
    // The transformation from the world to the camera frame.
    typedef Eigen::Matrix3x4d M_t;

    // The minimum number of samples needed to estimate a model.
    static const int kMinNumSamples = 3;

    // Estimate the most probable solution of the P3P problem from a set of
    // three 2D-3D point correspondences.
    //
    // @param points2D   Normalized 2D image points as 3x2 matrix.
    // @param points3D   3D world points as 3x3 matrix.
    //
    // @return           Most probable pose as length-1 vector of a 3x4 matrix.
    static std::vector<M_t> Estimate(const std::vector<X_t>& points2D,
        const std::vector<Y_t>& points3D);

    // Calculate the squared reprojection error given a set of 2D-3D point
    // correspondences and a projection matrix.
    //
    // @param points2D     Normalized 2D image points as Nx2 matrix.
    // @param points3D     3D world points as Nx3 matrix.
    // @param proj_matrix  3x4 projection matrix.
    // @param residuals    Output vector of residuals.
    static void Residuals(const std::vector<X_t>& points2D,
        const std::vector<Y_t>& points3D,
        const M_t& proj_matrix,
        std::vector<double>* residuals);
};

// EPNP solver for the PNP (Perspective-N-Point) problem. The solver needs a
// minimum of 4 2D-3D correspondences.
//
// The algorithm is based on the following paper:
//
//    Lepetit, Vincent, Francesc Moreno-Noguer, and Pascal Fua.
//    "Epnp: An accurate o (n) solution to the pnp problem."
//    International journal of computer vision 81.2 (2009): 155-166.
//
// The implementation is based on their original open-source release, but is
// ported to Eigen and contains several improvements over the original code.
class EPNPEstimator {
public:
    // The 2D image feature observations.
    typedef Eigen::Vector2d X_t;
    // The observed 3D features in the world frame.
    typedef Eigen::Vector3d Y_t;
    // The transformation from the world to the camera frame.
    typedef Eigen::Matrix3x4d M_t;

    // The minimum number of samples needed to estimate a model.
    static const int kMinNumSamples = 4;

    // Estimate the most probable solution of the P3P problem from a set of
    // three 2D-3D point correspondences.
    //
    // @param points2D   Normalized 2D image points as 3x2 matrix.
    // @param points3D   3D world points as 3x3 matrix.
    //
    // @return           Most probable pose as length-1 vector of a 3x4 matrix.
    static std::vector<M_t> Estimate(const std::vector<X_t>& points2D,
        const std::vector<Y_t>& points3D);

    // Calculate the squared reprojection error given a set of 2D-3D point
    // correspondences and a projection matrix.
    //
    // @param points2D     Normalized 2D image points as Nx2 matrix.
    // @param points3D     3D world points as Nx3 matrix.
    // @param proj_matrix  3x4 projection matrix.
    // @param residuals    Output vector of residuals.
    static void Residuals(const std::vector<X_t>& points2D,
        const std::vector<Y_t>& points3D,
        const M_t& proj_matrix,
        std::vector<double>* residuals);

private:
    bool ComputePose(const std::vector<Eigen::Vector2d>& points2D,
        const std::vector<Eigen::Vector3d>& points3D,
        Eigen::Matrix3x4d* proj_matrix);

    void ChooseControlPoints();
    bool ComputeBarycentricCoordinates();

    Eigen::Matrix<double, Eigen::Dynamic, 12> ComputeM();
    Eigen::Matrix<double, 6, 10> ComputeL6x10(
        const Eigen::Matrix<double, 12, 12>& Ut);
    Eigen::Matrix<double, 6, 1> ComputeRho();

    void FindBetasApprox1(const Eigen::Matrix<double, 6, 10>& L_6x10,
        const Eigen::Matrix<double, 6, 1>& rho,
        Eigen::Vector4d* betas);
    void FindBetasApprox2(const Eigen::Matrix<double, 6, 10>& L_6x10,
        const Eigen::Matrix<double, 6, 1>& rho,
        Eigen::Vector4d* betas);
    void FindBetasApprox3(const Eigen::Matrix<double, 6, 10>& L_6x10,
        const Eigen::Matrix<double, 6, 1>& rho,
        Eigen::Vector4d* betas);

    void RunGaussNewton(const Eigen::Matrix<double, 6, 10>& L_6x10,
        const Eigen::Matrix<double, 6, 1>& rho,
        Eigen::Vector4d* betas);

    double ComputeRT(const Eigen::Matrix<double, 12, 12>& Ut,
        const Eigen::Vector4d& betas,
        Eigen::Matrix3d* R,
        Eigen::Vector3d* t);

    void ComputeCcs(const Eigen::Vector4d& betas,
        const Eigen::Matrix<double, 12, 12>& Ut);
    void ComputePcs();

    void SolveForSign();

    void EstimateRT(Eigen::Matrix3d* R, Eigen::Vector3d* t);

    double ComputeTotalReprojectionError(const Eigen::Matrix3d& R,
        const Eigen::Vector3d& t);

    const std::vector<Eigen::Vector2d>* points2D_ = nullptr;
    const std::vector<Eigen::Vector3d>* points3D_ = nullptr;
    std::vector<Eigen::Vector3d> pcs_;
    std::vector<Eigen::Vector4d> alphas_;
    std::array<Eigen::Vector3d, 4> cws_;
    std::array<Eigen::Vector3d, 4> ccs_;
};







// Estimate absolute pose (optionally focal length) from 2D-3D correspondences.
//
// Focal length estimation is performed using discrete sampling around the
// focal length of the given camera. The focal length that results in the
// maximal number of inliers is assigned to the given camera.
//
// @param options              Absolute pose estimation options.
// @param points2D             Corresponding 2D points.
// @param points3D             Corresponding 3D points.
// @param cam_from_world       Estimated absolute camera pose.
// @param camera               Camera for which to estimate pose. Modified
//                             in-place to store the estimated focal length.
// @param num_inliers          Number of inliers in RANSAC.
// @param inlier_mask          Inlier mask for 2D-3D correspondences.
//
// @return                     Whether pose is estimated successfully.
bool EstimateAbsolutePose(const AbsolutePoseEstimationOptions& options,
    const std::vector<Eigen::Vector2d>& points2D,
    const std::vector<Eigen::Vector3d>& points3D,
    Rigid3d* cam_from_world,
    Camera* camera,
    size_t* num_inliers,
    std::vector<char>* inlier_mask);

// Estimate relative from 2D-2D correspondences.
//
// Pose of first camera is assumed to be at the origin without rotation. Pose
// of second camera is given as world-to-image transformation,
// i.e. `x2 = [R | t] * X2`.
//
// @param ransac_options       RANSAC options.
// @param points1              Corresponding 2D points.
// @param points2              Corresponding 2D points.
// @param cam2_from_cam1       Estimated pose between cameras.
//
// @return                     Number of RANSAC inliers.
size_t EstimateRelativePose(const RANSACOptions& ransac_options,
    const std::vector<Eigen::Vector2d>& points1,
    const std::vector<Eigen::Vector2d>& points2,
    Rigid3d* cam2_from_cam1);

// Refine absolute pose (optionally focal length) from 2D-3D correspondences.
//
// @param options              Refinement options.
// @param inlier_mask          Inlier mask for 2D-3D correspondences.
// @param points2D             Corresponding 2D points.
// @param points3D             Corresponding 3D points.
// @param cam_from_world       Refined absolute camera pose.
// @param camera               Camera for which to estimate pose. Modified
//                             in-place to store the estimated focal length.
// @param cam_from_world_cov   Estimated 6x6 covariance matrix of
//                             the rotation (as axis-angle, in tangent space)
//                             and translation terms (optional).
//
// @return                     Whether the solution is usable.
bool RefineAbsolutePose(const AbsolutePoseRefinementOptions& options,
    const std::vector<char>& inlier_mask,
    const std::vector<Eigen::Vector2d>& points2D,
    const std::vector<Eigen::Vector3d>& points3D,
    Rigid3d* cam_from_world,
    Camera* camera,
    Eigen::Matrix6d* cam_from_world_cov = nullptr);

// Refine relative pose of two cameras.
//
// Minimizes the Sampson error between corresponding normalized points using
// a robust cost function, i.e. the corresponding points need not necessarily
// be inliers given a sufficient initial guess for the relative pose.
//
// Assumes that first camera pose has projection matrix P = [I | 0], and
// pose of second camera is given as transformation from world to camera system.
//
// Assumes that the given translation vector is normalized, and refines
// the translation up to an unknown scale (i.e. refined translation vector
// is a unit vector again).
//
// @param options          Solver options.
// @param points1          First set of corresponding points.
// @param points2          Second set of corresponding points.
// @param cam_from_world   Refined pose between cameras.
//
// @return                 Flag indicating if solution is usable.
bool RefineRelativePose(const ceres::Solver::Options& options,
    const std::vector<Eigen::Vector2d>& points1,
    const std::vector<Eigen::Vector2d>& points2,
    Rigid3d* cam_from_world);

// Refine essential matrix.
//
// Decomposes the essential matrix into rotation and translation components
// and refines the relative pose using the function `RefineRelativePose`.
//
// @param E                3x3 essential matrix.
// @param points1          First set of corresponding points.
// @param points2          Second set of corresponding points.
// @param inlier_mask      Inlier mask for corresponding points.
// @param options          Solver options.
//
// @return                 Flag indicating if solution is usable.
bool RefineEssentialMatrix(const ceres::Solver::Options& options,
    const std::vector<Eigen::Vector2d>& points1,
    const std::vector<Eigen::Vector2d>& points2,
    const std::vector<char>& inlier_mask,
    Eigen::Matrix3d* E);