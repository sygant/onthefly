#pragma once
#define EIGEN_USE_MKL_ALL
#include <vector>
#include <array>
#include <Eigen/Core>
#include <Eigen/Dense>

#include "../Base/Base.h"
#include "Rigid3D.h"
#include "Pose.h"

// 将本质矩阵E分解为两个可能的旋转矩阵(R1和R2)以及两个可能的平移向量(t和-t)
void DecomposeEssentialMatrix(const Eigen::Matrix3d& E, Eigen::Matrix3d& R1, Eigen::Matrix3d& R2, Eigen::Vector3d& t);

// 从给定的本质矩阵E中分解最可能的位姿. 第一张影像的位姿假设为P=[I | 0]
void EssentialMatrixToPose(const Eigen::Matrix3d& E, const std::vector<Eigen::Vector2d>& points1, const std::vector<Eigen::Vector2d>& points2, Eigen::Matrix3d& R, Eigen::Vector3d& t, std::vector<Eigen::Vector3d>& points3D);

// 根据相对相机位姿组成本质矩阵E. 假设第一张影像的位姿的投影矩阵P=[I | 0], 并且第二张影像的位姿是从世界坐标系到相机坐标系的变换
Eigen::Matrix3d PoseToEssentialMatrix(const CRigid3D& camera1ToCamera2);

// 寻找"最优影像点", 其满足该等式: optimal_point1 ^ T * E * optimal_point2 = 0, E为本质矩阵或基础矩阵
// 参考文献: Lindstrom, P., "Triangulation made easy", Computer Vision and Pattern Recognition (CVPR), 2010 IEEE Conference on , vol., no., pp.1554,1561, 13-18 June 2010
void FindOptimalImageObservations(const Eigen::Matrix3d& E, const Eigen::Vector2d& point1, const Eigen::Vector2d& point2, Eigen::Vector2d& optimalPoint1, Eigen::Vector2d& optimalPoint2);

// 计算极点的位置(用齐次坐标表示). isLeftImage为true则会计算左影像的极点, 为false则会计算右影像的极点
Eigen::Vector3d EssentialMatrixToEpipole(const Eigen::Matrix3d& E, bool isLeftImage);

// 求本质矩阵的逆. 根据相机A到相机B的转换, 计算相机B到相机A的转换
Eigen::Matrix3d InvertEssentialMatrix(const Eigen::Matrix3d& E);