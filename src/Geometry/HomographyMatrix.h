#pragma once
#define EIGEN_USE_MKL_ALL
#include <vector>
#include <array>
#include <Eigen/Core>
#include <Eigen/Dense>

// 将单应矩阵H分解成可能的旋转R, 平移t和平面法向量n. 参考文献: Malis, Ezio, and Manuel Vargas. "Deeper understanding of the homography decomposition for vision-based control." (2007): 90.
// 第一个位姿假设为P=[I | 0]. 注意: 如果返回的R,t和n的size为4, 则说明单应性是由平面诱导的. 如果返回的R,t和n的size为1, 则说明单应性是纯旋转的.
// 
// @param H          3x3单应矩阵
// @param K1         相机一的内参矩阵
// @param K2         相机二的内参矩阵
// @param R          输出: 所有可能的3×3旋转矩阵
// @param t          输出: 所有可能的平移向量
// @param n          输出: 所有可能的法向量
void DecomposeHomographyMatrix(const Eigen::Matrix3d& H, const Eigen::Matrix3d& K1, const Eigen::Matrix3d& K2, std::vector<Eigen::Matrix3d>& R, std::vector<Eigen::Vector3d>& t, std::vector<Eigen::Vector3d>& n);

// 从单应矩阵H恢复最有可能的位姿(第一个位姿假设为P=[I | 0]). 
// 
// @param H          3x3单应矩阵
// @param K1         相机一的内参矩阵
// @param K2         相机二的内参矩阵
// @param points1    影像一上的对应点
// @param points2    影像二上的对应点
// @param R          输出: 最有可能的3×3旋转矩阵
// @param t          输出: 最有可能的平移向量
// @param n          输出: 最有可能的法向量
// @param points3D   输出: 位于相机前方的三角化后的3D点(仅当单应性不是纯旋转时)
void HomographyMatrixToPose(const Eigen::Matrix3d& H, const Eigen::Matrix3d& K1, const Eigen::Matrix3d& K2, const std::vector<Eigen::Vector2d>& points1, const std::vector<Eigen::Vector2d>& points2, Eigen::Matrix3d& R, Eigen::Vector3d& t, Eigen::Vector3d& n, std::vector<Eigen::Vector3d>& points3D);


// 根据相对位姿构建单应矩阵
//
// @param K1         相机一的内参矩阵
// @param K2         相机二的内参矩阵
// @param R          3×3旋转矩阵
// @param t          平移向量
// @param n          法向量
// @param d          与平面的垂直距离
//
// @return           3x3单应矩阵
Eigen::Matrix3d PoseToHomographyMatrix(const Eigen::Matrix3d& K1, const Eigen::Matrix3d& K2, const Eigen::Matrix3d& R, const Eigen::Vector3d& t, const Eigen::Vector3d& n, double d);
