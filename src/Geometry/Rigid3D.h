#pragma once
#include "../Base/types.h"
#include <Eigen/Geometry>

struct Rigid3d {
public:
    Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();
    Eigen::Vector3d translation = Eigen::Vector3d::Zero();

    Rigid3d() = default;
    Rigid3d(const Eigen::Quaterniond& rotation,
        const Eigen::Vector3d& translation)
        : rotation(rotation), translation(translation) {}

    inline Eigen::Matrix3x4d ToMatrix() const {
        Eigen::Matrix3x4d matrix;
        matrix.leftCols<3>() = rotation.toRotationMatrix();
        matrix.col(3) = translation;
        return matrix;
    }
};

// Return inverse transform.
inline Rigid3d Inverse(const Rigid3d& b_from_a) {
    Rigid3d a_from_b;
    a_from_b.rotation = b_from_a.rotation.inverse();
    a_from_b.translation = a_from_b.rotation * -b_from_a.translation;
    return a_from_b;
}

// Apply transform to point such that one can write expressions like:
//      x_in_b = b_from_a * x_in_a
//
// Be careful when including multiple transformations in the same expression, as
// the multiply operator in C++ is evaluated left-to-right.
// For example, the following expression:
//      x_in_c = d_from_c * c_from_b * b_from_a * x_in_a
// will be executed in the following order:
//      x_in_c = ((d_from_c * c_from_b) * b_from_a) * x_in_a
// This will first concatenate all transforms and then apply it to the point.
// While you may want to instead write and execute it as:
//      x_in_c = d_from_c * (c_from_b * (b_from_a * x_in_a))
// which will apply the transformations as a chain on the point.
inline Eigen::Vector3d operator*(const Rigid3d& t, const Eigen::Vector3d& x) {
    return t.rotation * x + t.translation;
}

// Concatenate transforms such one can write expressions like:
//      d_from_a = d_from_c * c_from_b * b_from_a
inline Rigid3d operator*(const Rigid3d& c_from_b, const Rigid3d& b_from_a) {
    Rigid3d cFromA;
    cFromA.rotation = (c_from_b.rotation * b_from_a.rotation).normalized();
    cFromA.translation =
        c_from_b.translation + (c_from_b.rotation * b_from_a.translation);
    return cFromA;
}