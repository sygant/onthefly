#pragma once
#include <cstdint>
#include <vector>
#include <windows.h>

// Define non-copyable or non-movable classes.
// NOLINTNEXTLINE(bugprone-macro-parentheses)
#define NON_COPYABLE(class_name)          \
  class_name(class_name const&) = delete; \
  void operator=(class_name const& obj) = delete;
// NOLINTNEXTLINE(bugprone-macro-parentheses)
#define NON_MOVABLE(class_name) class_name(class_name&&) = delete;


#include <Eigen/Core>

namespace Eigen
{
	typedef Eigen::Matrix<float, 3, 4> Matrix3x4f;
	typedef Eigen::Matrix<double, 3, 4> Matrix3x4d;
	typedef Eigen::Matrix<double, 6, 6> Matrix6d;
	typedef Eigen::Matrix<uint8_t, 3, 1> Vector3ub;
	typedef Eigen::Matrix<uint8_t, 4, 1> Vector4ub;
	typedef Eigen::Matrix<double, 6, 1> Vector6d;
}

typedef uint32_t camera_t;

// Unique identifier for images.
typedef uint32_t image_t;

// Each image pair gets a unique ID, see `Database::ImagePairToPairId`.
typedef uint64_t image_pair_t;

// Index per image, i.e. determines maximum number of 2D points per image.
typedef uint32_t point2D_t;

// Unique identifier per added 3D point. Since we add many 3D points,
// delete them, and possibly re-add them again, the maximum number of allowed
// unique indices should be large.
typedef uint64_t point3D_t;


const camera_t kInvalidCameraId = std::numeric_limits<camera_t>::max();
const image_t kInvalidImageId = std::numeric_limits<image_t>::max();
const image_pair_t kInvalidImagePairId = std::numeric_limits<image_pair_t>::max();
const point2D_t kInvalidPoint2DIdx = std::numeric_limits<point2D_t>::max();
const point3D_t kInvalidPoint3DId = std::numeric_limits<point3D_t>::max();


template <typename T>
class span 
{
	T* ptr_;
	const size_t size_;

public:
	span(T* ptr, size_t len) noexcept : ptr_{ ptr }, size_{ len } {}

	T& operator[](size_t i) noexcept { return ptr_[i]; }
	T const& operator[](size_t i) const noexcept { return ptr_[i]; }

	size_t size() const noexcept { return size_; }

	T* begin() noexcept { return ptr_; }
	T* end() noexcept { return ptr_ + size_; }
	const T* begin() const noexcept { return ptr_; }
	const T* end() const noexcept { return ptr_ + size_; }
};


namespace std
{
	// Hash function specialization for uint32_t pairs, e.g., image_t or camera_t.
	template <>
	struct hash<std::pair<uint32_t, uint32_t>> 
	{
		std::size_t operator()(const std::pair<uint32_t, uint32_t>& p) const 
		{
			const uint64_t s = (static_cast<uint64_t>(p.first) << 32) + static_cast<uint64_t>(p.second);
			return std::hash<uint64_t>()(s);
		}
	};
}


struct FeatureKeypoint
{
	FeatureKeypoint();
	FeatureKeypoint(float x, float y);
	FeatureKeypoint(float x, float y, float scale, float orientation);
	FeatureKeypoint(float x, float y, float a11, float a12, float a21, float a22);

	static FeatureKeypoint FromShapeParameters(float x,
		float y,
		float scale_x,
		float scale_y,
		float orientation,
		float shear);

	// Rescale the feature location and shape size by the given scale factor.
	void Rescale(float scale);
	void Rescale(float scale_x, float scale_y);

	// Compute shape parameters from affine shape.
	float ComputeScale() const;
	float ComputeScaleX() const;
	float ComputeScaleY() const;
	float ComputeOrientation() const;
	float ComputeShear() const;

	// Location of the feature, with the origin at the upper left image corner,
	// i.e. the upper left pixel has the coordinate (0.5, 0.5).
	float x;
	float y;

	// Affine shape of the feature.
	float a11;
	float a12;
	float a21;
	float a22;
};

typedef Eigen::Matrix<uint8_t, 1, Eigen::Dynamic, Eigen::RowMajor> FeatureDescriptor;
typedef std::vector<FeatureKeypoint> FeatureKeypoints;
typedef Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> FeatureDescriptors;
typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> FeatureDescriptorsFloat;
typedef std::vector<float> GlobalFeature;

struct FeatureMatch {
	FeatureMatch()
		: point2D_idx1(kInvalidPoint2DIdx), point2D_idx2(kInvalidPoint2DIdx) {}
	FeatureMatch(const point2D_t point2D_idx1, const point2D_t point2D_idx2)
		: point2D_idx1(point2D_idx1), point2D_idx2(point2D_idx2) {}

	// Feature index in first image.
	point2D_t point2D_idx1 = kInvalidPoint2DIdx;

	// Feature index in second image.
	point2D_t point2D_idx2 = kInvalidPoint2DIdx;
};

typedef std::vector<FeatureMatch> FeatureMatches;