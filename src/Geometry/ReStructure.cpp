#include "fusion.h"
#include "ReStructure.h"

using namespace std;

// 判断特征点是否在匹配对集合中
bool isMatched(size_t pointIndex, const FeatureMatches& matches) {
    // 在匹配对集合中查找特征点的索引值
    auto it = std::find_if(matches.begin(), matches.end(),
        [pointIndex](const FeatureMatch& match) {
            return match.point2D_idx1 == pointIndex;
        });
    // 如果找到了匹配对，it指向匹配对，如果没找到，it指向集合末尾；因此找到匹配对，则返回true；否则返回false
    return it != matches.end();
}

//判断元素是否在vector中
bool isInVector(image_t imageID, vector<image_t>RetrievalPairs)
{
    auto it = std::find(RetrievalPairs.begin(), RetrievalPairs.end(), imageID);
    return it != RetrievalPairs.end();
}

//将vector转换为Eigen矩阵列向量
Eigen::Matrix<size_t, Eigen::Dynamic, 1> Vector2Eigen(std::vector<size_t> vector)
{
    Eigen::Matrix<size_t, Eigen::Dynamic, 1> eigen(vector.size());
    for (size_t i = 0; i < vector.size(); ++i) {
        eigen(i) = vector[i];
    }
    return eigen;
}

// 找到集合S_i中未匹配的特征点D_ij
//可优化
std::vector<size_t> findUnmatchedPoints(const FeatureKeypoints& keypoints, const FeatureMatches& matches)
{
    std::vector<size_t> unmatchedPoints;

    // 遍历集合S_i中的每个特征点
    for (size_t i = 0; i < keypoints.size(); i++)
    {
        // 如果特征点不在匹配对集合中，则将其索引添加到未匹配点集合中
        if (!isMatched(i, matches))
        {
            unmatchedPoints.push_back(i);
        }
    }
    return unmatchedPoints;
}

// 找到集合S_i中匹配的特征点Q_ij
// 后面可优化（直接提取matches中的point2D_idx1或point2D_idx2）
std::vector<size_t> findMatchedPoints(const FeatureKeypoints& keypoints, const FeatureMatches& matches) {
    std::vector<size_t> matchedPoints;

    // 遍历集合S_i中的每个特征点
    for (size_t i = 0; i < keypoints.size(); i++)
    {
        // 如果特征点在匹配对集合中，则将其索引添加到匹配点集合中
        if (isMatched(i, matches))
        {
            matchedPoints.push_back(i);
        }
    }
    return matchedPoints;
}

//计算影像对的G向量
std::vector<size_t> CalculateG(Database& database, const image_t imageID, const std::vector<size_t> unmatchedPoint, const image_t ImageIDs)
{
    vector<size_t>G;
    //遍历除自身以外的每张影像
    for (size_t k = 0; k < ImageIDs; k++)
    {
        if (k + 1 == imageID || database.ExistsMatches(imageID, k + 1) != true)
        {
            G.push_back(0);
            continue;
        }
        const image_t imageID_k = k + 1;
        // const FeatureMatches matches_ik = database.ReadTwoViewGeometry(imageID, imageID_k).inlier_matches;
        const FeatureMatches matches_ik = database.ReadMatches(imageID, imageID_k);
        size_t numG = 0;
        for (size_t n = 0; n < unmatchedPoint.size(); n++)
        {
            if (isMatched(unmatchedPoint[n], matches_ik))
            {
                numG = numG + 1;
            }
        }
        G.push_back(numG);
    }
    return G;
}

std::vector<size_t> CalculateG(Database& database, const image_t imageID, const std::vector<size_t> unmatchedPoint, const image_t ImageIDs, vector<image_t>RetrievalPairs)
{
    vector<size_t>G;
    //遍历除自身以外的每张影像
    for (size_t k = 0; k < ImageIDs; k++)
    {
        if (k + 1 == imageID || database.ExistsMatches(imageID, k + 1) != true)
        {
            G.push_back(0);
            continue;
        }
        const image_t imageID_k = k + 1;
        if (!isInVector(imageID_k, RetrievalPairs))
        {
            G.push_back(0);
            continue;
        }
        // const FeatureMatches matches_ik = database.ReadTwoViewGeometry(imageID, imageID_k).inlier_matches;
        const FeatureMatches matches_ik = database.ReadMatches(imageID, imageID_k);
        size_t numG = 0;
        for (size_t n = 0; n < unmatchedPoint.size(); n++)
        {
            if (isMatched(unmatchedPoint[n], matches_ik))
            {
                numG = numG + 1;
            }
        }
        G.push_back(numG);
    }
    return G;
}

//计算所有影像对的匹配关系(初始化)
void CalculateMR(Database& database, const image_t ImageIDs)
{
    float maxRS = 0;
    float minRS = std::numeric_limits<float>::max();

    for (size_t i = 0; i < ImageIDs; i++)
    {
        for (size_t j = 0; j < ImageIDs; j++)
        {
            const image_t imageID1 = i + 1;
            const image_t imageID2 = j + 1;
            if (i == j || database.ExistsMatches(imageID1, imageID2) != true)
            {
                //cout << "MatchRelation Finish:" << i + 1 << " in " << ImageIDs << ";" << j + 1 << " in " << ImageIDs << endl;
                continue;
            }
            else if (i > j && database.ExistsMatches(imageID1, imageID2) == true)
            {
                TwoViewGeometry geometry;
                if (database.ExistsInlierMatches(imageID1, imageID2))
                {
                    geometry = database.ReadTwoViewGeometry(imageID1, imageID2);
                }
                geometry.imageID1 = imageID1;
                geometry.imageID2 = imageID2;
                geometry.G_ij = database.ReadTwoViewGeometry(imageID2, imageID1).G_ji;
                geometry.G_ji = database.ReadTwoViewGeometry(imageID2, imageID1).G_ij;
                geometry.RS = database.ReadTwoViewGeometry(imageID2, imageID1).RS;
                database.WriteTwoViewGeometry(imageID1, imageID2, geometry);
                //cout << "MatchRelation Finish:" << i + 1 << " in " << ImageIDs << ";" << j + 1 << " in " << ImageIDs << endl;
                continue;
            }
            const FeatureKeypoints keypoint1 = database.ReadKeypoints(imageID1);
            const FeatureKeypoints keypoint2 = database.ReadKeypoints(imageID2);
            //const FeatureMatches matches1 = database.ReadTwoViewGeometry(imageID1, imageID2).inlier_matches;
            //const FeatureMatches matches2 = database.ReadTwoViewGeometry(imageID2, imageID1).inlier_matches;
            const FeatureMatches matches1 = database.ReadMatches(imageID1, imageID2);
            const FeatureMatches matches2 = database.ReadMatches(imageID2, imageID1);

            std::vector<size_t> matchedPoint1 = findMatchedPoints(keypoint1, matches1);
            std::vector<size_t> unmatchedPoint1 = findUnmatchedPoints(keypoint1, matches1);
            std::vector<size_t> matchedPoint2 = findMatchedPoints(keypoint2, matches2);
            std::vector<size_t> unmatchedPoint2 = findUnmatchedPoints(keypoint2, matches2);

            //求该匹配对的G向量
            vector<size_t>g_i = CalculateG(database, imageID1, unmatchedPoint1, ImageIDs);
            vector<size_t>g_j = CalculateG(database, imageID2, unmatchedPoint2, ImageIDs);

            Eigen::Matrix<size_t, Eigen::Dynamic, 1>g_i_e = Vector2Eigen(g_i);
            Eigen::Matrix<size_t, Eigen::Dynamic, 1>g_j_e = Vector2Eigen(g_j);

            Eigen::Matrix<size_t, Eigen::Dynamic, Eigen::Dynamic> g_g = g_i_e.transpose() * g_j_e;
            //计算该匹配对的RS
            float RS = (unmatchedPoint1.size() + unmatchedPoint2.size()) * (g_g(0)) / (matchedPoint1.size() + matchedPoint2.size());

            //TwoViewGeometry geometry = database.ReadTwoViewGeometry(imageID1, imageID2);
            TwoViewGeometry geometry;
            if (database.ExistsInlierMatches(imageID1, imageID2))
            {
                geometry = database.ReadTwoViewGeometry(imageID1, imageID2);
            }
            geometry.imageID1 = imageID1;
            geometry.imageID2 = imageID2;
            geometry.G_ij = g_i;
            geometry.G_ji = g_j;
            geometry.RS = RS;

            if (geometry.RS > maxRS)
            {
                maxRS = geometry.RS;
            }
            if (geometry.RS < minRS)
            {
                //minRS = geometry.RS / 1000000.0;
                minRS = geometry.RS;
            }
            database.WriteTwoViewGeometry(imageID1, imageID2, geometry);
            //cout << "MatchRelation Finish:" << i + 1 << " in " << ImageIDs << ";" << j + 1 << " in " << ImageIDs << endl;
        }
    }

    // 计算权值
    for (size_t i = 0; i < ImageIDs; i++)
    {
        for (size_t j = 0; j < ImageIDs; j++)
        {
            if (database.ExistsMatches(i + 1, j + 1) == true)
            {
                TwoViewGeometry geometry = database.ReadTwoViewGeometry(i + 1, j + 1);
                geometry.wmst = (geometry.RS - minRS) / (maxRS - minRS);
                database.WriteTwoViewGeometry(i + 1, j + 1, geometry);
            }
        }
    }
}

//计算所有影像对的匹配关系(后续动态剔除)
void CalculateMR(Database& database, const image_t ImageIDs, vector<image_t>RetrievalPairs)
{
    float maxRS = 0;
    float minRS = std::numeric_limits<float>::max();

    for (size_t i = 0; i < ImageIDs; i++)
    {
        for (size_t j = 0; j < ImageIDs; j++)
        {
            const image_t imageID1 = i + 1;
            const image_t imageID2 = j + 1;
            //如果检索结果中没有imageID1和imageID2，则不需要更新RS，跳过
            if (!isInVector(imageID1, RetrievalPairs) && !isInVector(imageID2,RetrievalPairs))
            {
                continue;
            }
            if (i == j || database.ExistsMatches(imageID1, imageID2) != true)
            {
                //cout << "MatchRelation Finish:" << i + 1 << " in " << ImageIDs << ";" << j + 1 << " in " << ImageIDs << endl;
                continue;
            }
            else if (i > j && database.ExistsMatches(imageID1, imageID2) == true)
            {
                TwoViewGeometry geometry;
                if (database.ExistsInlierMatches(imageID1, imageID2))
                {
                    geometry = database.ReadTwoViewGeometry(imageID1, imageID2);
                }
                geometry.imageID1 = imageID1;
                geometry.imageID2 = imageID2;
                geometry.G_ij = database.ReadTwoViewGeometry(imageID2, imageID1).G_ji;
                geometry.G_ji = database.ReadTwoViewGeometry(imageID2, imageID1).G_ij;
                geometry.RS = database.ReadTwoViewGeometry(imageID2, imageID1).RS;
                database.WriteTwoViewGeometry(imageID1, imageID2, geometry);
                //cout << "MatchRelation Finish:" << i + 1 << " in " << ImageIDs << ";" << j + 1 << " in " << ImageIDs << endl;
                continue;
            }
            const FeatureKeypoints keypoint1 = database.ReadKeypoints(imageID1);
            const FeatureKeypoints keypoint2 = database.ReadKeypoints(imageID2);
            //const FeatureMatches matches1 = database.ReadTwoViewGeometry(imageID1, imageID2).inlier_matches;
            //const FeatureMatches matches2 = database.ReadTwoViewGeometry(imageID2, imageID1).inlier_matches;
            const FeatureMatches matches1 = database.ReadMatches(imageID1, imageID2);
            const FeatureMatches matches2 = database.ReadMatches(imageID2, imageID1);

            std::vector<size_t> matchedPoint1 = findMatchedPoints(keypoint1, matches1);
            std::vector<size_t> unmatchedPoint1 = findUnmatchedPoints(keypoint1, matches1);
            std::vector<size_t> matchedPoint2 = findMatchedPoints(keypoint2, matches2);
            std::vector<size_t> unmatchedPoint2 = findUnmatchedPoints(keypoint2, matches2);

            //求该匹配对的G向量
            vector<size_t>g_i = CalculateG(database, imageID1, unmatchedPoint1, ImageIDs, RetrievalPairs);
            vector<size_t>g_j = CalculateG(database, imageID2, unmatchedPoint2, ImageIDs, RetrievalPairs);

            Eigen::Matrix<size_t, Eigen::Dynamic, 1>g_i_e = Vector2Eigen(g_i);
            Eigen::Matrix<size_t, Eigen::Dynamic, 1>g_j_e = Vector2Eigen(g_j);

            Eigen::Matrix<size_t, Eigen::Dynamic, Eigen::Dynamic> g_g = g_i_e.transpose() * g_j_e;
            //计算该匹配对的RS
            float RS = (unmatchedPoint1.size() + unmatchedPoint2.size()) * (g_g(0)) / (matchedPoint1.size() + matchedPoint2.size());

            //TwoViewGeometry geometry = database.ReadTwoViewGeometry(imageID1, imageID2);
            TwoViewGeometry geometry;
            if (database.ExistsInlierMatches(imageID1, imageID2))
            {
                geometry = database.ReadTwoViewGeometry(imageID1, imageID2);
            }
            geometry.imageID1 = imageID1;
            geometry.imageID2 = imageID2;
            geometry.G_ij = g_i;
            geometry.G_ji = g_j;
            geometry.RS = RS;

            if (geometry.RS > maxRS)
            {
                maxRS = geometry.RS;
            }
            if (geometry.RS < minRS)
            {
                //minRS = geometry.RS / 1000000.0;
                minRS = geometry.RS;
            }
            database.WriteTwoViewGeometry(imageID1, imageID2, geometry);
            //cout << "MatchRelation Finish:" << i + 1 << " in " << ImageIDs << ";" << j + 1 << " in " << ImageIDs << endl;
        }
    }

    // 计算权值
    for (size_t i = 0; i < ImageIDs; i++)
    {
        for (size_t j = 0; j < ImageIDs; j++)
        {
            if (database.ExistsMatches(i + 1, j + 1) == true)
            {
                TwoViewGeometry geometry = database.ReadTwoViewGeometry(i + 1, j + 1);
                geometry.wmst = (geometry.RS - minRS) / (maxRS - minRS);
                database.WriteTwoViewGeometry(i + 1, j + 1, geometry);
            }
        }
    }
}

//计算所有影像对的匹配关系
//std::vector<std::vector<MatchRelation>> CalculateMR(Database& database, const vector<std::string> ImagePaths)
//{
//    std::vector<std::vector<MatchRelation>> matchRelations;
//    std::vector<MatchRelation> matchrelations_temp;
//    MatchRelation matchrelation;
//
//    for (size_t i = 0; i < ImagePaths.size(); i++)
//    {
//        matchrelations_temp.clear();
//        for (size_t j = 0; j < ImagePaths.size(); j++)
//        {
//            if (i == j || database.ReadTwoViewGeometry(i + 1, j + 1).inlier_matches.empty())
//            {
//                matchrelation.imageID1 = 0;
//                matchrelation.imageID2 = 0;
//                matchrelation.RS = -1;
//                matchrelations_temp.push_back(matchrelation);
//                cout << "MatchRelation Finish:" << i + 1 << " in " << ImagePaths.size() << ";" << j + 1 << " in " << ImagePaths.size() << endl;
//                continue;
//            }
//            else if (i > j)
//            {
//                matchrelation.imageID1 = matchRelations[j][i].imageID2;
//                matchrelation.imageID2 = matchRelations[j][i].imageID1;
//                //matchrelation.G_ij = matchRelations[j][i].G_ji;
//                //matchrelation.G_ji = matchRelations[j][i].G_ij;
//                matchrelation.RS = matchRelations[j][i].RS;
//                matchrelation.cam2_from_cam1 = database.ReadTwoViewGeometry(matchrelation.imageID1, matchrelation.imageID2).cam2_from_cam1;
//                matchrelations_temp.push_back(matchrelation);
//                cout << "MatchRelation Finish:" << i + 1 << " in " << ImagePaths.size() << ";" << j + 1 << " in " << ImagePaths.size() << endl;
//                continue;
//            }
//            const image_t imageID1 = i + 1;
//            const image_t imageID2 = j + 1;
//            const FeatureKeypoints keypoint1 = database.ReadKeypoints(imageID1);
//            const FeatureKeypoints keypoint2 = database.ReadKeypoints(imageID2);
//            const FeatureMatches matches1 = database.ReadMatches(imageID1, imageID2);
//            const FeatureMatches matches2 = database.ReadMatches(imageID2, imageID1);
//
//            std::vector<size_t> matchedPoint1 = findMatchedPoints(keypoint1, matches1);
//            std::vector<size_t> unmatchedPoint1 = findUnmatchedPoints(keypoint1, matches1);
//            std::vector<size_t> matchedPoint2 = findMatchedPoints(keypoint2, matches2);
//            std::vector<size_t> unmatchedPoint2 = findUnmatchedPoints(keypoint2, matches2);
//
//            //求该匹配对的G向量
//            vector<size_t>g_i = CalculateG(database, imageID1, unmatchedPoint1, ImagePaths);
//            vector<size_t>g_j = CalculateG(database, imageID2, unmatchedPoint2, ImagePaths);
//
//            Eigen::Matrix<size_t, Eigen::Dynamic, 1>g_i_e = Vector2Eigen(g_i);
//            Eigen::Matrix<size_t, Eigen::Dynamic, 1>g_j_e = Vector2Eigen(g_j);
//
//            Eigen::Matrix<size_t, Eigen::Dynamic, Eigen::Dynamic> g_g = g_i_e.transpose() * g_j_e;
//            //计算该匹配对的RS
//            float RS = (unmatchedPoint1.size() + unmatchedPoint2.size()) * (g_g(0)) / (matchedPoint1.size() + matchedPoint2.size());
//
//            matchrelation.imageID1 = imageID1;
//            matchrelation.imageID2 = imageID2;
//            //matchrelation.G_ij = g_i;
//            //matchrelation.G_ji = g_j;
//            matchrelation.RS = RS;
//            matchrelation.cam2_from_cam1 = database.ReadTwoViewGeometry(imageID1, imageID2).cam2_from_cam1;
//            matchrelations_temp.push_back(matchrelation);
//
//            cout << "MatchRelation Finish:" << i + 1 << " in " << ImagePaths.size() << ";" << j + 1 << " in " << ImagePaths.size() << endl;
//        }
//        matchRelations.push_back(matchrelations_temp);
//    }
//
//    //计算权值
//    float maxRS = 0;
//    float minRS = std::numeric_limits<float>::max();
//    for (size_t i = 0; i < ImagePaths.size(); ++i)
//    {
//        for (size_t j = 0; j < ImagePaths.size(); ++j)
//        {
//            if (matchRelations[i][j].RS > maxRS)
//            {
//                maxRS = matchRelations[i][j].RS;
//            }
//            if (matchRelations[i][j].RS < minRS && matchRelations[i][j].RS != -1)
//            {
//                minRS = matchRelations[i][j].RS;
//            }
//        }
//    }
//
//    //如果仅有一组匹配关系
//    if (maxRS == minRS)
//    {
//        for (size_t i = 0; i < ImagePaths.size(); ++i)
//        {
//            for (size_t j = 0; j < ImagePaths.size(); ++j)
//            {
//                if (matchRelations[i][j].RS != -1)
//                {
//                    matchRelations[i][j].wmst = 0.000000001;
//                }
//
//                else
//                {
//                    matchRelations[i][j].wmst = 100;
//                }
//            }
//        }
//    }
//
//    else
//    {
//        for (size_t i = 0; i < ImagePaths.size(); ++i)
//        {
//            for (size_t j = 0; j < ImagePaths.size(); ++j)
//            {
//                if (matchRelations[i][j].RS != -1)
//                {
//                    matchRelations[i][j].wmst = (matchRelations[i][j].RS - minRS) / (maxRS - minRS);
//                }
//
//                else
//                {
//                    matchRelations[i][j].wmst = 100;
//                }
//            }
//        }
//    }
//
//    return matchRelations;
//}

//生成minimum spanning tree
std::vector<TwoViewGeometry> GenerateMinSpanTree(Database& database, const image_t ImageIDs)
{
    const size_t numImages = ImageIDs;
    std::vector<TwoViewGeometry> MinSpanTree;     //储存匹配关系O
    std::set<int> L; // 存储已经包含的影像索引,该索引从1开始,和数据库一致
    std::set<int> DS; // 存储待选的影像索引，从1开始

    // 初始化 DS，将所有影像索引加入其中
    for (int i = 0; i < numImages; i++)
    {
        size_t imageID = i + 1;
        DS.insert(imageID);
    }

    // 1. 选择权值最小的 RO，将其加入 O 和对应的影像索引加入 L
    float min_rs = std::numeric_limits<float>::max();
    size_t min_imageID1 = -1;
    size_t min_imageID2 = -1;

    for (size_t i = 0; i < numImages; i++)
    {
        for (size_t j = i; j < numImages; j++)
        {
            if (database.ExistsMatches(i + 1, j + 1) == true && database.ReadTwoViewGeometry(i + 1, j + 1).RS < min_rs)
            {
                min_rs = database.ReadTwoViewGeometry(i + 1, j + 1).RS;
                min_imageID1 = i + 1;
                min_imageID2 = j + 1;
            }
        }
    }

    if (min_imageID1 == -1 || min_imageID2 == -1)
    {
        std::cout << "没有找到符合条件的 RO" << endl;
        return MinSpanTree;
    }

    MinSpanTree.push_back(database.ReadTwoViewGeometry(min_imageID1, min_imageID2));
    L.insert(min_imageID1);
    L.insert(min_imageID2);

    // 更新 DS，将已经加入 L 的影像索引从 DS 中删除
    for (auto it = L.begin(); it != L.end(); ++it)
    {
        DS.erase(*it);
    }

    // 2. 找到 DS 中与 L 中影像索引相连的影像索引,直到DS中所有影像都加入tree中
    while (!DS.empty())
    {
        std::set<int> connectedVertices;
        for (size_t i : L)
        {
            for (size_t j : DS)
            {
                if (j != i && database.ExistsMatches(i, j) == true)
                {
                    connectedVertices.insert(j);
                }
            }
        }
        // 如果没有与 L 中影像索引相连的影像索引，则跳过本次循环
        if (connectedVertices.empty())
        {
            std::cout << "DS不为空，但已无影像与L中的节点相连" << endl;
            break;
        }

        // 选择与 L 中影像索引相连的影像索引中权值最小的那个，加入 O 和 L
        min_rs = std::numeric_limits<float>::max();
        size_t min_connectedImageID = -1;
        size_t min_connectedL = -1;
        for (size_t imageID : connectedVertices)
        {
            for (size_t j : L)
            {
                if (database.ExistsMatches(j, imageID) == true && database.ReadTwoViewGeometry(j, imageID).RS < min_rs)
                {
                    min_rs = database.ReadTwoViewGeometry(j, imageID).RS;
                    min_connectedImageID = imageID;
                    min_connectedL = j;
                }
            }
        }

        if (min_connectedImageID == -1) {
            break; // 如果没有找到符合条件的影像索引，则结束循环
        }

        MinSpanTree.push_back(database.ReadTwoViewGeometry(min_connectedImageID, min_connectedL));
        L.insert(min_connectedImageID);
        DS.erase(min_connectedImageID);
    }
    return MinSpanTree;
}
//std::vector<MatchRelation> GenerateMinSpanTree(std::vector<std::vector<MatchRelation>> matchRelations, const vector<std::string> ImagePaths)
//{
//    const size_t numImages = ImagePaths.size();
//    std::vector<MatchRelation> MinSpanTree;     //储存匹配关系O
//    std::set<int> L; // 存储已经包含的影像索引,该索引从0开始
//    std::set<int> DS; // 存储待选的影像索引
//
//    // 初始化 DS，将所有影像索引加入其中
//    for (int i = 0; i < numImages; i++)
//    {
//        size_t imageID = i;
//        DS.insert(imageID);
//    }
//
//    // 1. 选择权值最小的 RO，将其加入 O 和对应的影像索引加入 L
//    float min_wmst = std::numeric_limits<float>::max();
//    size_t min_imageID1 = -1;
//    size_t min_imageID2 = -1;
//
//    for (size_t i = 0; i < numImages; i++)
//    {
//        for (size_t j = i + 1; j < numImages; j++)
//        {
//            if (matchRelations[i][j].RS != -1 && matchRelations[i][j].RS < min_wmst)
//            {
//                min_wmst = matchRelations[i][j].RS;
//                min_imageID1 = i;
//                min_imageID2 = j;
//            }
//        }
//    }
//
//    if (min_imageID1 == -1 || min_imageID2 == -1)
//    {
//        std::cout << "没有找到符合条件的 RO" << endl;
//        return MinSpanTree;
//    }
//
//    MinSpanTree.push_back(matchRelations[min_imageID1][min_imageID2]);
//    L.insert(min_imageID1);
//    L.insert(min_imageID2);
//
//    // 更新 DS，将已经加入 L 的影像索引从 DS 中删除
//    for (auto it = L.begin(); it != L.end(); ++it)
//    {
//        DS.erase(*it);
//    }
//
//    // 2. 找到 DS 中与 L 中影像索引相连的影像索引,直到DS中所有影像都加入tree中
//    while (!DS.empty())
//    {
//        std::set<int> connectedVertices;
//        for (size_t i : L)
//        {
//            for (size_t j : DS)
//            {
//                if (j != i && matchRelations[i][j].RS != -1)
//                {
//                    connectedVertices.insert(j);
//                }
//            }
//        }
//        // 如果没有与 L 中影像索引相连的影像索引，则跳过本次循环
//        if (connectedVertices.empty())
//        {
//            std::cout << "DS不为空，但已无影像与L中的节点相连" << endl;
//            break;
//        }
//
//        // 选择与 L 中影像索引相连的影像索引中权值最小的那个，加入 O 和 L
//        min_wmst = std::numeric_limits<float>::max();
//        size_t min_connectedImageID = 0;
//        size_t min_connectedL = 0;
//        for (size_t imageID : connectedVertices)
//        {
//            for (size_t j : L)
//            {
//                if (matchRelations[j][imageID].RS != -1 && matchRelations[j][imageID].RS < min_wmst)
//                {
//                    min_wmst = matchRelations[j][imageID].RS;
//                    min_connectedImageID = imageID;
//                    min_connectedL = j;
//                }
//            }
//        }
//
//        if (min_connectedImageID == -1) {
//            break; // 如果没有找到符合条件的影像索引，则结束循环
//        }
//
//        MinSpanTree.push_back(matchRelations[min_connectedImageID][min_connectedL]);
//        L.insert(min_connectedImageID);
//        DS.erase(min_connectedImageID);
//    }
//    return MinSpanTree;
//}

// 最小优化4
//std::vector<size_t>SolveMinOptimize(Database& database, const image_t ImageIDs, std::vector<TwoViewGeometry> minSpanTree)
//{
//    //进行优化
//    // 定义影像总数、ROs 数量和每张影像保留的 ROs 数量
//    int Ne = ImageIDs;
//    int Nr = 2;  // 每张影像保留的 ROs 数量
//    int Mr = 3;  // RS_ij 最小的 M_r * N_e 个 ROs 需要保留下来
//    size_t numVar = Ne * Ne;
//
//    Model::t M = new Model("MinRS"); auto _M = finally([&]() { M->dispose(); });
//
//    //创建变量x，数量为numVar，限定条件为整数，且在[0,1]之间，即只能取0或者1
//    Variable::t x = M->variable("x", numVar, Domain::integral(Domain::inRange(0.0, 1.0)));
//
//    // 找RS最大值
//    float max_rs = 0;
//    for (size_t i = 0; i < ImageIDs; ++i)
//    {
//        for (size_t j = i; j < ImageIDs; ++j)
//        {
//            if (database.ExistsMatches(i + 1, j + 1) == true && database.ReadTwoViewGeometry(i + 1, j + 1).RS > max_rs)
//            {
//                max_rs = database.ReadTwoViewGeometry(i + 1, j + 1).RS;
//            }
//        }     
//    }
//
//    //设定变量x的系数阵c，其值为rs
//    std::vector<double>rs_vec;
//    for (size_t i = 0; i < ImageIDs; ++i)
//    {
//        for (size_t j = 0; j < ImageIDs; ++j)
//        {
//            if (database.ExistsMatches(i + 1, j + 1) == true)
//            {
//                rs_vec.push_back(database.ReadTwoViewGeometry(i + 1, j + 1).RS);
//            }
//            else if (database.ExistsMatches(i + 1, j + 1) != true)
//            {
//                rs_vec.push_back(max_rs + 10000);
//            }
//        }
//    }
//    auto c = new_array_ptr<double>(rs_vec);
//
//    //添加限制条件
//    //1.对于任意一张影像i，所有变量a_ij之和，应大于Nr
//    for (size_t imageID = 0; imageID < Ne; ++imageID)
//    {
//        std::vector<double>A1_vec;
//        A1_vec.clear();
//        for (size_t i = 0; i < Ne; ++i)
//        {
//            for (size_t j = 0; j < Ne; j++)
//            {
//                if (i == imageID)
//                {
//                    A1_vec.push_back(1);
//                }
//                else
//                {
//                    A1_vec.push_back(0);
//                }
//            }
//        }
//        auto A1 = new_array_ptr<double>(A1_vec);
//        std::string cIndex = "c1_" + std::to_string(imageID);
//        M->constraint(cIndex, Expr::dot(A1, x), Domain::greaterThan(Nr));
//    }
//
//    //2.对于集合O（minSpanTree）中的匹配关系，变量a_ij均为1
//    for (auto ro : minSpanTree)
//    {
//        size_t indexVar1 = (ro.imageID1 - 1) * Ne + (ro.imageID2 - 1);
//        size_t indexVar2 = (ro.imageID2 - 1) * Ne + (ro.imageID1 - 1);
//
//        M->constraint(x->index(indexVar1), Domain::equalsTo(1.0));
//        M->constraint(x->index(indexVar2), Domain::equalsTo(1.0));
//    }
//
//    //3.对于所有匹配关系，变量a_ij的和等于Mr*Ne
//    std::vector<double> A3_vec(numVar);
//    for (size_t i = 0; i < numVar; ++i)
//    {
//        A3_vec[i] = 1.0;
//    }
//    auto A3 = new_array_ptr<double>(A3_vec);
//    M->constraint("c3", Expr::dot(A3, x), Domain::equalsTo(Mr * Ne));
//
//    //4.对于本身就未能匹配的影像对（包括自身），变量a_ij直接为0
//    for (size_t i = 0; i < Ne; ++i)
//    {
//        for (size_t j = 0; j < Ne; j++)
//        {
//            if (database.ExistsMatches(i + 1, j + 1) != true)
//            {
//                size_t indexVar = i * Ne + j;
//                M->constraint(x->index(indexVar), Domain::equalsTo(0.0));
//            }
//        }
//    }
//    //设置目标函数
//    M->objective("obj", ObjectiveSense::Minimize, Expr::dot(c, x));
//
//    //开始解算
//    M->solve();
//
//    //解算结果
//    auto sol = x->level();
//
//    //结果写入a_after
//    std::vector<size_t>result(numVar);
//    for (size_t i = 0; i < numVar; ++i)
//    {
//        result[i] = (*sol)[i];
//    }
//
//    return result;
//}
//std::vector<size_t>SolveMinOptimize(vector<std::string> ImagePaths, std::vector<std::vector<MatchRelation>>matchRelation, std::vector<MatchRelation> minSpanTree)
//{
//    //进行优化
//    // 定义影像总数、ROs 数量和每张影像保留的 ROs 数量
//    int Ne = ImagePaths.size();
//    int Nr = 8;  // 每张影像保留的 ROs 数量
//    int Mr = 18;  // RS_ij 最小的 M_r * N_e 个 ROs 需要保留下来
//    size_t numVar = Ne * Ne;
//
//    Model::t M = new Model("MinRS"); auto _M = finally([&]() { M->dispose(); });
//
//    //创建变量x，数量为numVar，限定条件为整数，且在[0,1]之间，即只能取0或者1
//    Variable::t x = M->variable("x", numVar, Domain::integral(Domain::inRange(0.0, 1.0)));
//
//    //设定变量x的系数阵c，其值为wmst
//    std::vector<double>wmst_vec;
//    for (size_t i = 0; i < ImagePaths.size(); ++i)
//    {
//        for (size_t j = 0; j < ImagePaths.size(); ++j)
//        {
//            wmst_vec.push_back(matchRelation[i][j].wmst);
//        }
//    }
//    auto c = new_array_ptr<double>(wmst_vec);
//
//    //添加限制条件
//    //1.对于任意一张影像i，所有变量a_ij之和，应大于Nr
//    for (size_t imageID = 0; imageID < Ne; ++imageID)
//    {
//        std::vector<double>A1_vec;
//        A1_vec.clear();
//        for (size_t i = 0; i < Ne; ++i)
//        {
//            for (size_t j = 0; j < Ne; j++)
//            {
//                if (i == imageID)
//                {
//                    A1_vec.push_back(1);
//                }
//                else
//                {
//                    A1_vec.push_back(0);
//                }
//            }
//        }
//        auto A1 = new_array_ptr<double>(A1_vec);
//        std::string cIndex = "c1_" + std::to_string(imageID);
//        M->constraint(cIndex, Expr::dot(A1, x), Domain::greaterThan(Nr));
//    }
//
//    //2.对于集合O（minSpanTree）中的匹配关系，变量a_ij均为1
//    for (auto ro : minSpanTree)
//    {
//        size_t indexVar1 = (ro.imageID1 - 1) * Ne + (ro.imageID2 - 1);
//        size_t indexVar2 = (ro.imageID2 - 1) * Ne + (ro.imageID1 - 1);
//
//        M->constraint(x->index(indexVar1), Domain::equalsTo(1.0));
//        M->constraint(x->index(indexVar2), Domain::equalsTo(1.0));
//    }
//
//    //3.对于所有匹配关系，变量a_ij的和等于Mr*Ne
//    std::vector<double> A3_vec(numVar);
//    for (size_t i = 0; i < numVar; ++i)
//    {
//        A3_vec[i] = 1.0;
//    }
//    auto A3 = new_array_ptr<double>(A3_vec);
//    M->constraint("c3", Expr::dot(A3, x), Domain::equalsTo(Mr * Ne));
//
//    //4.对于本身就未能匹配的影像对（包括自身），变量a_ij直接为0
//    for (size_t i = 0; i < Ne; ++i)
//    {
//        for (size_t j = 0; j < Ne; j++)
//        {
//            if (matchRelation[i][j].RS == -1)
//            {
//                size_t indexVar = i * Ne + j;
//                M->constraint(x->index(indexVar), Domain::equalsTo(0.0));
//            }
//        }
//    }
//    //设置目标函数
//    M->objective("obj", ObjectiveSense::Minimize, Expr::dot(c, x));
//
//    //开始解算
//    M->solve();
//
//    //解算结果
//    auto sol = x->level();
//
//    //结果写入a_after
//    std::vector<size_t>result(numVar);
//    for (size_t i = 0; i < numVar; ++i)
//    {
//        result[i] = (*sol)[i];
//    }
//
//    return result;
//}



