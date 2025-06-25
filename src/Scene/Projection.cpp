#include "Projection.h"
#include "../Geometry/Pose.h"
#include "../Base/math.h"


double CalculateSquaredReprojectionError(const Eigen::Vector2d& point2D,
    const Eigen::Vector3d& point3D,
    const Rigid3d& cam_from_world,
    const Camera& camera) {
    const Eigen::Vector3d point3D_in_cam = cam_from_world * point3D;

    // Check that point is infront of camera.
    if (point3D_in_cam.z() < std::numeric_limits<double>::epsilon()) {
        return std::numeric_limits<double>::max();
    }

    return (camera.ImgFromCam(point3D_in_cam.hnormalized()) - point2D)
        .squaredNorm();
}

double CalculateSquaredReprojectionError(
    const Eigen::Vector2d& point2D,
    const Eigen::Vector3d& point3D,
    const Eigen::Matrix3x4d& cam_from_world,
    const Camera& camera) {
    const double proj_z = cam_from_world.row(2).dot(point3D.homogeneous());

    // Check that point is infront of camera.
    if (proj_z < std::numeric_limits<double>::epsilon()) {
        return std::numeric_limits<double>::max();
    }

    const double proj_x = cam_from_world.row(0).dot(point3D.homogeneous());
    const double proj_y = cam_from_world.row(1).dot(point3D.homogeneous());
    const double inv_proj_z = 1.0 / proj_z;

    const Eigen::Vector2d proj_point2D = camera.ImgFromCam(
        Eigen::Vector2d(inv_proj_z * proj_x, inv_proj_z * proj_y));

    return (proj_point2D - point2D).squaredNorm();
}

double CalculateAngularError(const Eigen::Vector2d& point2D,
    const Eigen::Vector3d& point3D,
    const Rigid3d& cam_from_world,
    const Camera& camera) {
    return CalculateNormalizedAngularError(
        camera.CamFromImg(point2D), point3D, cam_from_world);
}

double CalculateAngularError(const Eigen::Vector2d& point2D,
    const Eigen::Vector3d& point3D,
    const Eigen::Matrix3x4d& cam_from_world,
    const Camera& camera) {
    return CalculateNormalizedAngularError(
        camera.CamFromImg(point2D), point3D, cam_from_world);
}

double CalculateNormalizedAngularError(const Eigen::Vector2d& point2D,
    const Eigen::Vector3d& point3D,
    const Rigid3d& cam_from_world) {
    const Eigen::Vector3d ray1 = point2D.homogeneous();
    const Eigen::Vector3d ray2 = cam_from_world * point3D;
    return std::acos(ray1.normalized().transpose() * ray2.normalized());
}

double CalculateNormalizedAngularError(
    const Eigen::Vector2d& point2D,
    const Eigen::Vector3d& point3D,
    const Eigen::Matrix3x4d& cam_from_world) {
    const Eigen::Vector3d ray1 = point2D.homogeneous();
    const Eigen::Vector3d ray2 = cam_from_world * point3D.homogeneous();
    return std::acos(ray1.normalized().transpose() * ray2.normalized());
}

bool HasPointPositiveDepth(const Eigen::Matrix3x4d& cam_from_world,
    const Eigen::Vector3d& point3D) {
    return cam_from_world.row(2).dot(point3D.homogeneous()) >=
        std::numeric_limits<double>::epsilon();
}