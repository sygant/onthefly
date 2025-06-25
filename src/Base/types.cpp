#include "types.h"
#include "logging.h"

FeatureKeypoint::FeatureKeypoint() : FeatureKeypoint(0, 0) {}

FeatureKeypoint::FeatureKeypoint(const float x, const float y)
    : FeatureKeypoint(x, y, 1, 0, 0, 1) {}

FeatureKeypoint::FeatureKeypoint(const float x_,
    const float y_,
    const float scale,
    const float orientation)
    : x(x_), y(y_) {
    CHECK_GE(scale, 0.0);
    const float scale_cos_orientation = scale * std::cos(orientation);
    const float scale_sin_orientation = scale * std::sin(orientation);
    a11 = scale_cos_orientation;
    a12 = -scale_sin_orientation;
    a21 = scale_sin_orientation;
    a22 = scale_cos_orientation;
}

FeatureKeypoint::FeatureKeypoint(const float x_,
    const float y_,
    const float a11_,
    const float a12_,
    const float a21_,
    const float a22_)
    : x(x_), y(y_), a11(a11_), a12(a12_), a21(a21_), a22(a22_) {}

FeatureKeypoint FeatureKeypoint::FromShapeParameters(const float x,
    const float y,
    const float scale_x,
    const float scale_y,
    const float orientation,
    const float shear) {
    CHECK_GE(scale_x, 0.0);
    CHECK_GE(scale_y, 0.0);
    return FeatureKeypoint(x,
        y,
        scale_x * std::cos(orientation),
        -scale_y * std::sin(orientation + shear),
        scale_x * std::sin(orientation),
        scale_y * std::cos(orientation + shear));
}

void FeatureKeypoint::Rescale(const float scale) { Rescale(scale, scale); }

void FeatureKeypoint::Rescale(const float scale_x, const float scale_y) {
    CHECK_GT(scale_x, 0);
    CHECK_GT(scale_y, 0);
    x *= scale_x;
    y *= scale_y;
    a11 *= scale_x;
    a12 *= scale_y;
    a21 *= scale_x;
    a22 *= scale_y;
}

float FeatureKeypoint::ComputeScale() const {
    return (ComputeScaleX() + ComputeScaleY()) / 2.0f;
}

float FeatureKeypoint::ComputeScaleX() const {
    return std::sqrt(a11 * a11 + a21 * a21);
}

float FeatureKeypoint::ComputeScaleY() const {
    return std::sqrt(a12 * a12 + a22 * a22);
}

float FeatureKeypoint::ComputeOrientation() const {
    return std::atan2(a21, a11);
}

float FeatureKeypoint::ComputeShear() const {
    return std::atan2(-a12, a22) - ComputeOrientation();
}