#pragma once
#include "../Base/types.h"
#include <vector>
#include <string>
#include <unordered_map>

// Camera class that holds the intrinsic parameters. Cameras may be shared
// between multiple images, e.g., if the same "physical" camera took multiple
// pictures with the exact same lens and intrinsics (focal length, etc.).
// This class has a specific distortion model defined by a camera model class.
class Camera {
public:
    Camera();

    // Access the unique identifier of the camera.
    inline camera_t CameraId() const;
    inline void SetCameraId(camera_t camera_id);

    // Access the camera model.
    inline int ModelId() const;
    std::string ModelName() const;
    void SetModelId(int model_id);
    void SetModelIdFromName(const std::string& model_name);

    // Access dimensions of the camera sensor.
    inline size_t Width() const;
    inline size_t Height() const;
    inline void SetWidth(size_t width);
    inline void SetHeight(size_t height);

    // Access focal length parameters.
    double MeanFocalLength() const;
    double FocalLength() const;
    double FocalLengthX() const;
    double FocalLengthY() const;
    void SetFocalLength(double focal_length);
    void SetFocalLengthX(double focal_length_x);
    void SetFocalLengthY(double focal_length_y);

    // Check if camera has prior focal length.
    inline bool HasPriorFocalLength() const;
    inline void SetPriorFocalLength(bool prior);

    // Access principal point parameters. Only works if there are two
    // principal point parameters.
    double PrincipalPointX() const;
    double PrincipalPointY() const;
    void SetPrincipalPointX(double ppx);
    void SetPrincipalPointY(double ppy);

    // Get the indices of the parameter groups in the parameter vector.
    span<const size_t> FocalLengthIdxs() const;
    span<const size_t> PrincipalPointIdxs() const;
    span<const size_t> ExtraParamsIdxs() const;

    // Get intrinsic calibration matrix composed from focal length and principal
    // point parameters, excluding distortion parameters.
    Eigen::Matrix3d CalibrationMatrix() const;

    // Get human-readable information about the parameter vector ordering.
    std::string ParamsInfo() const;

    // Access the raw parameter vector.
    inline size_t NumParams() const;
    inline const std::vector<double>& Params() const;
    inline std::vector<double>& Params();
    inline double Params(size_t idx) const;
    inline double& Params(size_t idx);
    inline const double* ParamsData() const;
    inline double* ParamsData();
    inline void SetParams(const std::vector<double>& params);

    // Concatenate parameters as comma-separated list.
    std::string ParamsToString() const;

    // Set camera parameters from comma-separated list.
    bool SetParamsFromString(const std::string& string);

    // Check whether parameters are valid, i.e. the parameter vector has
    // the correct dimensions that match the specified camera model.
    bool VerifyParams() const;

    // Check whether camera is already undistorted
    bool IsUndistorted() const;

    // Check whether camera has bogus parameters.
    bool HasBogusParams(double min_focal_length_ratio,
        double max_focal_length_ratio,
        double max_extra_param) const;

    // Initialize parameters for given camera model and focal length, and set
    // the principal point to be the image center.
    void InitializeWithId(int model_id,
        double focal_length,
        size_t width,
        size_t height);
    void InitializeWithName(const std::string& model_name,
        double focal_length,
        size_t width,
        size_t height);

    // Project point in image plane to world / infinity.
    Eigen::Vector2d CamFromImg(const Eigen::Vector2d& image_point) const;

    // Convert pixel threshold in image plane to camera frame.
    double CamFromImgThreshold(double threshold) const;

    // Project point from camera frame to image plane.
    Eigen::Vector2d ImgFromCam(const Eigen::Vector2d& cam_point) const;

    // Rescale camera dimensions and accordingly the focal length and
    // and the principal point.
    void Rescale(double scale);
    void Rescale(size_t width, size_t height);

private:
    // The unique identifier of the camera. If the identifier is not specified
    // it is set to `kInvalidCameraId`.
    camera_t camera_id_;

    // The identifier of the camera model. If the camera model is not specified
    // the identifier is `kInvalidCameraModelId`.
    int model_id_;

    // The dimensions of the image, 0 if not initialized.
    size_t width_;
    size_t height_;

    // The focal length, principal point, and extra parameters. If the camera
    // model is not specified, this vector is empty.
    std::vector<double> params_;

    // Whether there is a safe prior for the focal length,
    // e.g. manually provided or extracted from EXIF
    bool prior_focal_length_;
};

////////////////////////////////////////////////////////////////////////////////
// Implementation
////////////////////////////////////////////////////////////////////////////////

camera_t Camera::CameraId() const { return camera_id_; }

void Camera::SetCameraId(const camera_t camera_id) { camera_id_ = camera_id; }

int Camera::ModelId() const { return model_id_; }

size_t Camera::Width() const { return width_; }

size_t Camera::Height() const { return height_; }

void Camera::SetWidth(const size_t width) { width_ = width; }

void Camera::SetHeight(const size_t height) { height_ = height; }

bool Camera::HasPriorFocalLength() const { return prior_focal_length_; }

void Camera::SetPriorFocalLength(const bool prior) {
    prior_focal_length_ = prior;
}

size_t Camera::NumParams() const { return params_.size(); }

const std::vector<double>& Camera::Params() const { return params_; }

std::vector<double>& Camera::Params() { return params_; }

double Camera::Params(const size_t idx) const { return params_[idx]; }

double& Camera::Params(const size_t idx) { return params_[idx]; }

const double* Camera::ParamsData() const { return params_.data(); }

double* Camera::ParamsData() { return params_.data(); }

void Camera::SetParams(const std::vector<double>& params) { params_ = params; }

typedef std::vector<std::pair<std::string, float>> camera_make_specs_t;
typedef std::unordered_map<std::string, camera_make_specs_t> camera_specs_t;

// Database that contains sensor widths for many cameras, which is useful
// to automatically extract the focal length if EXIF information is incomplete.
class CameraDatabase
{
public:
    CameraDatabase() = default;

    size_t NumEntries() const { return specs_.size(); }

    bool QuerySensorWidth(const std::string& make,
        const std::string& model,
        double* sensor_width);

private:
    static const camera_specs_t specs_;
};


camera_specs_t InitializeCameraSpecs();
