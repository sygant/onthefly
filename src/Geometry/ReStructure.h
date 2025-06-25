#pragma once

#include "../Scene/Database.h"
#include "../Geometry/Pose.h"
#include "../Feature/ImageReader.h"
#include "../Feature/FeatureExtractor.h"
#include "../Feature/FeatureMatcher.h"
#include "../Workflow/Workflow.h"
#include "../Base/misc.h"

using namespace std;

//struct MatchRelation 
//{
//    image_t imageID1;
//    image_t imageID2;
//    std::vector<size_t> G_ij;   //影像i的全部特征点中，未和j匹配，但和其他影像匹配的特征点数，为一个向量
//    std::vector<size_t> G_ji;   //影像j的全部特征点中，未和i匹配，但和其他影像匹配的特征点数，为一个向量
//    double RS;
//    float wmst;
//    Rigid3d cam2_from_cam1;
//};

// 判断特征点是否在匹配对集合中
bool isMatched(size_t pointIndex, const FeatureMatches& matches);

//将vector转换为Eigen矩阵列向量
Eigen::Matrix<size_t, Eigen::Dynamic, 1> Vector2Eigen(std::vector<size_t> vector);

// 找到集合S_i中未匹配的特征点D_ij
//可优化
std::vector<size_t> findUnmatchedPoints(const FeatureKeypoints& keypoints, const FeatureMatches& matches);

// 找到集合S_i中匹配的特征点Q_ij
// 后面可优化（直接提取matches中的point2D_idx1或point2D_idx2）
std::vector<size_t> findMatchedPoints(const FeatureKeypoints& keypoints, const FeatureMatches& matches);

//计算影像对的G向量
std::vector<size_t> CalculateG(Database& database, const image_t imageID, const std::vector<size_t> unmatchedPoint, const image_t ImageIDs);
std::vector<size_t> CalculateG(Database& database, const image_t imageID, const std::vector<size_t> unmatchedPoint, const image_t ImageIDs, vector<image_t>RetrievalPairs);

//计算所有影像对的匹配关系
void CalculateMR(Database& database, const image_t ImageIDs);
void CalculateMR(Database& database, const image_t ImageIDs, vector<image_t>RetrievalPairs);
//std::vector<std::vector<MatchRelation>> CalculateMR(Database& database, const vector<std::string> ImagePaths);

//生成minimum spanning tree
std::vector<TwoViewGeometry> GenerateMinSpanTree(Database& database, const image_t ImageIDs);
//std::vector<MatchRelation> GenerateMinSpanTree(std::vector<std::vector<MatchRelation>> matchRelations, const vector<std::string> ImagePaths);

// 最小优化
std::vector<size_t>SolveMinOptimize(Database& database, const image_t ImageIDs, std::vector<TwoViewGeometry> minSpanTree);
// std::vector<size_t>SolveMinOptimize(vector<std::string> ImagePaths, std::vector<std::vector<MatchRelation>>matchRelation, std::vector<MatchRelation> minSpanTree);