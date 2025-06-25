#pragma once
#include "../Scene/Reconstruction.h"
#include "../Base/macros.h"

#include <memory>
#include <unordered_set>
#include <Eigen/Core>
#include <ceres/ceres.h>

struct BundleAdjustmentOptions {
    // Loss function types: Trivial (non-robust) and Cauchy (robust) loss.
    enum class LossFunctionType { TRIVIAL, SOFT_L1, CAUCHY };
    LossFunctionType loss_function_type = LossFunctionType::TRIVIAL;

    // Scaling factor determines residual at which robustification takes place.
    double loss_function_scale = 1.0;

    // Whether to refine the focal length parameter group.
    bool refine_focal_length = true;

    // Whether to refine the principal point parameter group.
    bool refine_principal_point = false;

    // Whether to refine the extra parameter group.
    bool refine_extra_params = true;

    // Whether to refine the extrinsic parameter group.
    bool refine_extrinsics = true;

    // Whether to print a final summary.
#ifdef _DEBUG
    bool print_summary = true;
#else
    bool print_summary = false;
#endif
    

    // Minimum number of residuals to enable multi-threading. Note that
    // single-threaded is typically better for small bundle adjustment problems
    // due to the overhead of threading.
    int min_num_residuals_for_multi_threading = 50000;

    // Ceres-Solver options.
    ceres::Solver::Options solver_options;

    BundleAdjustmentOptions() {
        solver_options.function_tolerance = 0.0;
        solver_options.gradient_tolerance = 0.0;
        solver_options.parameter_tolerance = 0.0;
        solver_options.minimizer_progress_to_stdout = false;
        solver_options.max_num_iterations = 100;
        solver_options.max_linear_solver_iterations = 200;
        solver_options.max_num_consecutive_invalid_steps = 10;
        solver_options.max_consecutive_nonmonotonic_steps = 10;
        solver_options.num_threads = -1;
#if CERES_VERSION_MAJOR < 2
        solver_options.num_linear_solver_threads = -1;
#endif  // CERES_VERSION_MAJOR
    }

    // Create a new loss function based on the specified options. The caller
    // takes ownership of the loss function.
    ceres::LossFunction* CreateLossFunction() const;

    bool Check() const;
};

// Configuration container to setup bundle adjustment problems.
class BundleAdjustmentConfig {
public:
    BundleAdjustmentConfig();

    size_t NumImages() const;
    size_t NumPoints() const;
    size_t NumConstantCamIntrinsics() const;
    size_t NumConstantCamPoses() const;
    size_t NumConstantCamPositionss() const;
    size_t NumVariablePoints() const;
    size_t NumConstantPoints() const;

    // Determine the number of residuals for the given reconstruction. The number
    // of residuals equals the number of observations times two.
    size_t NumResiduals(const Reconstruction& reconstruction) const;

    // Add / remove images from the configuration.
    void AddImage(image_t image_id);
    bool HasImage(image_t image_id) const;
    void RemoveImage(image_t image_id);

    // Set cameras of added images as constant or variable. By default all
    // cameras of added images are variable. Note that the corresponding images
    // have to be added prior to calling these methods.
    void SetConstantCamIntrinsics(camera_t camera_id);
    void SetVariableCamIntrinsics(camera_t camera_id);
    bool HasConstantCamIntrinsics(camera_t camera_id) const;

    // Set the pose of added images as constant. The pose is defined as the
    // rotational and translational part of the projection matrix.
    void SetConstantCamPose(image_t image_id);
    void SetVariableCamPose(image_t image_id);
    bool HasConstantCamPose(image_t image_id) const;

    // Set the translational part of the pose, hence the constant pose
    // indices may be in [0, 1, 2] and must be unique. Note that the
    // corresponding images have to be added prior to calling these methods.
    void SetConstantCamPositions(image_t image_id, const std::vector<int>& idxs);
    void RemoveConstantCamPositions(image_t image_id);
    bool HasConstantCamPositions(image_t image_id) const;

    // Add / remove points from the configuration. Note that points can either
    // be variable or constant but not both at the same time.
    void AddVariablePoint(point3D_t point3D_id);
    void AddConstantPoint(point3D_t point3D_id);
    bool HasPoint(point3D_t point3D_id) const;
    bool HasVariablePoint(point3D_t point3D_id) const;
    bool HasConstantPoint(point3D_t point3D_id) const;
    void RemoveVariablePoint(point3D_t point3D_id);
    void RemoveConstantPoint(point3D_t point3D_id);

    // Access configuration data.
    const std::unordered_set<image_t>& Images() const;
    const std::unordered_set<point3D_t>& VariablePoints() const;
    const std::unordered_set<point3D_t>& ConstantPoints() const;
    const std::vector<int>& ConstantCamPositions(image_t image_id) const;

#ifdef USE_HIERARCHICAL_WEIGHT_BA
    std::unordered_map<image_t, double> weights;
#endif


private:
    std::unordered_set<camera_t> constant_intrinsics_;
    std::unordered_set<image_t> image_ids_;
    std::unordered_set<point3D_t> variable_point3D_ids_;
    std::unordered_set<point3D_t> constant_point3D_ids_;
    std::unordered_set<image_t> constant_cam_poses_;
    std::unordered_map<image_t, std::vector<int>> constant_cam_positions_;
};

// Bundle adjustment based on Ceres-Solver. Enables most flexible configurations
// and provides best solution quality.
class BundleAdjuster {
public:
    BundleAdjuster(const BundleAdjustmentOptions& options,
        const BundleAdjustmentConfig& config);

    bool Solve(Reconstruction* reconstruction, double& finalCost);

    // Get the Ceres solver summary for the last call to `Solve`.
    const ceres::Solver::Summary& Summary() const;

private:

#ifdef USE_HIERARCHICAL_WEIGHT_BA
    ceres::LossFunction* trivialLossFunction = nullptr;
    ceres::LossFunction* softL1LossFunction = nullptr;
    ceres::LossFunction* CauchyLossFunction = nullptr;

    void SetUp(Reconstruction* reconstruction);
    void TearDown(Reconstruction* reconstruction);
    void AddImageToProblem(image_t image_id, Reconstruction* reconstruction);
    void AddPointToProblem(point3D_t point3D_id, Reconstruction* reconstruction);

#else
    void SetUp(Reconstruction* reconstruction, ceres::LossFunction* loss_function);
    void TearDown(Reconstruction* reconstruction);
    void AddImageToProblem(image_t image_id, Reconstruction* reconstruction, ceres::LossFunction* loss_function);
    void AddPointToProblem(point3D_t point3D_id, Reconstruction* reconstruction, ceres::LossFunction* loss_function);
#endif


    

protected:
    void ParameterizeCameras(Reconstruction* reconstruction);
    void ParameterizePoints(Reconstruction* reconstruction);

    const BundleAdjustmentOptions options_;
    BundleAdjustmentConfig config_;
    std::unique_ptr<ceres::Problem> problem_;
    ceres::Solver::Summary summary_;
    std::unordered_set<camera_t> camera_ids_;
    std::unordered_map<point3D_t, size_t> point3D_num_observations_;
};

void PrintSolverSummary(const ceres::Solver::Summary& summary);