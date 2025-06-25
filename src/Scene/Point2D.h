#pragma once
#include "../Base/types.h"
#include <Eigen/Core>

// 2D point class corresponds to a feature in an image. It may or may not have a
// corresponding 3D point if it is part of a triangulated track.
struct Point2D
{
	// The image coordinates in pixels, starting at upper left corner with 0.
	Eigen::Vector2d xy = Eigen::Vector2d::Zero();

	// The identifier of the 3D point. If the 2D point is not part of a 3D point
	// track the identifier is `kInvalidPoint3DId` and `HasPoint3D() = false`.
	point3D_t point3D_id = kInvalidPoint3DId;

	// Determin whether the 2D point observes a 3D point.
	inline bool HasPoint3D() const { return point3D_id != kInvalidPoint3DId; }
};