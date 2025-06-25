#pragma once
#include "../Base/types.h"
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

// 3D similarity transform with 7 degrees of freedom.
// Transforms point x from a to b as: x_in_b = scale * R * x_in_a + t.
struct Sim3d {
    double scale = 1;
    Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();
    Eigen::Vector3d translation = Eigen::Vector3d::Zero();

    Sim3d() = default;
    Sim3d(double scale,
        const Eigen::Quaterniond& rotation,
        const Eigen::Vector3d& translation)
        : scale(scale), rotation(rotation), translation(translation) {}

    inline Eigen::Matrix3x4d ToMatrix() const {
        Eigen::Matrix3x4d matrix;
        matrix.leftCols<3>() = scale * rotation.toRotationMatrix();
        matrix.col(3) = translation;
        return matrix;
    }

    static inline Sim3d FromMatrix(const Eigen::Matrix3x4d& matrix) {
        Sim3d t;
        t.scale = matrix.col(0).norm();
        t.rotation =
            Eigen::Quaterniond(matrix.leftCols<3>() / t.scale).normalized();
        t.translation = matrix.rightCols<1>();
        return t;
    }

    // Read from or write to text file without loss of precision.
    void ToFile(const std::string& path) const;
    static Sim3d FromFile(const std::string& path);
};

// Return inverse transform.
inline Sim3d Inverse(const Sim3d& b_from_a) {
    Sim3d a_from_b;
    a_from_b.scale = 1 / b_from_a.scale;
    a_from_b.rotation = b_from_a.rotation.inverse();
    a_from_b.translation =
        (a_from_b.rotation * b_from_a.translation) / -b_from_a.scale;
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
inline Eigen::Vector3d operator*(const Sim3d& t, const Eigen::Vector3d& x) {
    return t.scale * (t.rotation * x) + t.translation;
}

// Concatenate transforms such one can write expressions like:
//      d_from_a = d_from_c * c_from_b * b_from_a
inline Sim3d operator*(const Sim3d& c_from_b, const Sim3d& b_from_a) {
    Sim3d cFromA;
    cFromA.scale = c_from_b.scale * b_from_a.scale;
    cFromA.rotation = (c_from_b.rotation * b_from_a.rotation).normalized();
    cFromA.translation =
        c_from_b.translation +
        (c_from_b.scale * (c_from_b.rotation * b_from_a.translation));
    return cFromA;
}