#include "Model.h"

using namespace std;


CModel::CModel(size_t modelID, CDatabase* database) :database(database)
{
	CHECK(database);
	this->modelID = modelID;
	regImageIDs.clear();
	points3D.clear();
	nextPoint3DID = 0;
}
unordered_set<size_t> CModel::GetAllPoints3D() const
{
	unordered_set<size_t> result;
	result.reserve(points3D.size());
	for (const auto& pair : points3D)
	{
		result.insert(pair.first);
	}
	return result;
}
size_t CModel::AddPoint3D(const Eigen::Vector3d& XYZ, const CTrack& track, const Eigen::Vector3ub& color)
{
	CHECK(database);
	size_t point3DID = nextPoint3DID;
	while (points3D.find(nextPoint3DID) != points3D.end())
	{
		nextPoint3DID++;
	}
	CHECK(point3DID != nextPoint3DID);

	const vector<CTrackElement> trackElements = track.GetAllElements();
	for (const CTrackElement& trackElement : trackElements)
	{
		CHECK(regImageIDs.find(trackElement.imageID) != regImageIDs.end());
		CImage& image = database->GetImage(trackElement.imageID);
		CHECK(!image.IsPoint2DHasPoint3D(modelID, trackElement.point2DIndex));
		image.SetPoint3DForPoint2D(trackElement.point2DIndex, point3DID, modelID);
		CHECK(image.GetNumPoints2D() >= image.GetNumPoints3D(modelID));
	}
	for (const CTrackElement& trackElement : trackElements)
	{
		SetObservationAsTriangulated(trackElement.imageID, trackElement.point2DIndex, false);
	}

	points3D[point3DID] = CPoint3D(XYZ);
	points3D[point3DID].SetTrack(track);
	points3D[point3DID].SetColor(color);
	return point3DID;
}
void CModel::AddObservation(size_t point3DID, const CTrackElement& trackElement)
{
	CHECK(database);
	CHECK(points3D.find(point3DID) != points3D.end());

	CImage& image = database->GetImage(trackElement.imageID);
	CHECK(!image.IsPoint2DHasPoint3D(modelID, trackElement.point2DIndex));
	image.SetPoint3DForPoint2D(trackElement.point2DIndex, point3DID, modelID);
	CHECK(image.GetNumPoints3D(modelID) <= image.GetNumPoints2D());

	points3D[point3DID].GetTrack().AddElement(trackElement);
	SetObservationAsTriangulated(trackElement.imageID, trackElement.point2DIndex, true);
}
size_t CModel::MergePoints3D(size_t point3D1ID, size_t point3D2ID)
{
	CHECK(points3D.find(point3D1ID) != points3D.end() && points3D.find(point3D2ID) != points3D.end());

	const CPoint3D& point1 = points3D[point3D1ID];
	const CPoint3D& point2 = points3D[point3D2ID];

	const size_t trackLength1 = point1.GetTrack().GetTrackLength();
	const size_t trackLength2 = point2.GetTrack().GetTrackLength();

	const Eigen::Vector3d mergedXYZ = (trackLength1 * point1.GetXYZ() + trackLength2 * point2.GetXYZ()) / (trackLength1 + trackLength2);
	const Eigen::Vector3d mergedRGB = (trackLength1 * point1.GetColor().cast<double>() + trackLength2 * point2.GetColor().cast<double>()) / (trackLength1 + trackLength2);

	CTrack mergedTrack;
	mergedTrack.Reserve(trackLength1 + trackLength2);
	mergedTrack.AddElements(point1.GetTrack().GetAllElements());
	mergedTrack.AddElements(point2.GetTrack().GetAllElements());
	DeletePoint3D(point3D1ID);
	DeletePoint3D(point3D2ID);
	const size_t mergedPoint3DID = AddPoint3D(mergedXYZ, mergedTrack, mergedRGB.cast<uint8_t>());
	return mergedPoint3DID;
}
void CModel::DeletePoint3D(size_t point3DID)
{
	// 注意：不要更改这些代码的顺序
	CHECK(database);
	CHECK(points3D.find(point3DID) != points3D.end());
	const CTrack& track = points3D[point3DID].GetTrack();
	const vector<CTrackElement> trackElements = track.GetAllElements();
	for (const CTrackElement& trackElement : trackElements)
	{
		ResetTriObservations(trackElement.imageID, trackElement.point2DIndex, true);
	}
	for (const CTrackElement& trackElement : trackElements)
	{
		database->GetImage(trackElement.imageID).ResetPoint3DForPoint2D(trackElement.point2DIndex, modelID);
	}
	points3D.erase(point3DID);
	nextPoint3DID = min(nextPoint3DID, point3DID);
}
void CModel::DeleteObservation(size_t imageID, size_t point2DID)
{
	// 注意：不要更改这些代码的顺序
	CHECK(database);
	CImage& image = database->GetImage(imageID);
	const size_t point3DID = image.GetPoint3DID(point2DID, modelID);
	
	const auto it = points3D.find(point3DID);
	CHECK(it != points3D.end());
	CPoint3D& point3D = it->second;
	CTrack& track = point3D.GetTrack();
	if (track.GetTrackLength() <= 2)
	{
		DeletePoint3D(point3DID);
		return;
	}

	track.DeleteElement(imageID, point2DID);
	ResetTriObservations(imageID, point2DID, false);
	image.ResetPoint3DForPoint2D(point2DID, modelID);
}
void CModel::RegisterImage(size_t imageID)
{
	CHECK(database);
	CImage& image = database->GetImage(imageID);
	const auto it = regImageIDs.find(modelID);
	if (!image.IsRegistered(modelID))
	{
		CHECK(it == regImageIDs.end());
		image.SetRegistered(modelID, true);
		regImageIDs.insert(imageID);
	}
	CHECK(it != regImageIDs.end());
}
void CModel::DeRegisterImage(size_t imageID)
{
	CHECK(database);
	CImage& image = database->GetImage(imageID);
	const size_t numPoints2D = image.GetNumPoints2D();
	for (size_t point2DID = 0; point2DID < numPoints2D; point2DID++)
	{
		if (image.IsPoint2DHasPoint3D(modelID, point2DID))
		{
			DeleteObservation(imageID, point2DID);
		}
	}
	image.SetRegistered(modelID, false);
	regImageIDs.erase(imageID);
}
void CModel::Normalize(double extent, double p0, double p1, bool isUseImage)
{
	if ((isUseImage && regImageIDs.size() < 2) || (!isUseImage && points3D.size() < 2))
	{
		return;
	}
	CHECK(extent > 0 && p0 >= 0 && p0 <= 1 && p1 >= 0 && p1 <= 1 && p0 <= p1);
	const tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d> boundsAndCentroid = ComputeBoundsAndCentroid(p0, p1, isUseImage);

	// 计算平移和缩放量, 以便在缩放之前执行平移
	const double oldExtent = (get<1>(boundsAndCentroid) - get<0>(boundsAndCentroid)).norm();
	double scale;
	if (oldExtent < numeric_limits<double>::epsilon())
	{
		scale = 1;
	}
	else
	{
		scale = extent / oldExtent;
	}
	CSim3D tform(scale, Eigen::Quaterniond::Identity(), -scale * get<2>(boundsAndCentroid));
	Transform(tform);
}
Eigen::Vector3d CModel::ComputeCentroid(double p0, double p1) const
{
	CHECK(p0 >= 0 && p0 <= 1 && p1 >= 0 && p1 <= 1 && p0 <= p1);
	return get<2>(ComputeBoundsAndCentroid(p0, p1, false));
}
pair<Eigen::Vector3d, Eigen::Vector3d> CModel::ComputeBoundingBox(double p0, double p1) const
{
	CHECK(p0 >= 0 && p0 <= 1 && p1 >= 0 && p1 <= 1 && p0 <= p1);
	const tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d> boundsAndCentroid = ComputeBoundsAndCentroid(p0, p1, false);
	return { get<0>(boundsAndCentroid), get<1>(boundsAndCentroid) };
}
void CModel::Transform(const CSim3D& oldWorldToNewWorld)
{
	CHECK(database);
	// TODO: 原Colmap的Transform函数会转换整个database所有的image, 本人认为仅需要转换当前模型注册的影像即可, 不被当前影像注册的影像应该不用转换
	for (size_t regImageID : regImageIDs)
	{
		CImage& image = database->GetImage(regImageID);
		image.GetWorldToCamera(modelID) = TransformCameraWorld(oldWorldToNewWorld, image.GetWorldToCamera(modelID));
	}
	for (auto& pair : points3D)
	{
		pair.second.SetXYZ(oldWorldToNewWorld * pair.second.GetXYZ());
	}
}
vector<size_t> CModel::FindCommonRegisteredImages(const CModel& otherModel) const
{
	CHECK(database && database == otherModel.database);
	vector<size_t> result;
	result.reserve(regImageIDs.size());
	for (size_t regImageID : regImageIDs)
	{
		const CImage& image = database->GetImage(regImageID);
		if (otherModel.regImageIDs.find(regImageID) != otherModel.regImageIDs.end())
		{
			CHECK(image.IsRegistered(modelID) && image.IsRegistered(otherModel.modelID));
			result.push_back(regImageID);
		}
		else
		{
			CHECK(image.IsRegistered(modelID) && !image.IsRegistered(otherModel.modelID));
		}
	}
	return result;
}
size_t CModel::FilterPoints3D(double maxReprojectionError, double minTriAngle, const unordered_set<size_t>& pointsToBeFiltered)
{
	size_t numFiltered = 0;
	numFiltered += FilterPoints3DWithLargeReprojectionError(maxReprojectionError, pointsToBeFiltered);
	numFiltered += FilterPoints3DWithSmallTriangulationAngle(minTriAngle, pointsToBeFiltered);
	return numFiltered;
}
size_t CModel::FilterPoints3DInImages(double maxReprojectionError, double minTriAngle, const unordered_set<size_t>& imageIDs)
{
	CHECK(database);
	unordered_set<size_t> pointsToBeFiltered;
	for (size_t imageID : imageIDs)
	{
		const CImage& image = database->GetImage(imageID);
		const size_t numPoints2D = image.GetNumPoints2D();
		for (size_t point2DID = 0; point2DID < numPoints2D; point2DID++)
		{
			if (image.IsPoint2DHasPoint3D(modelID, point2DID))
			{
				const size_t point3DID = image.GetPoint3DID(point2DID, modelID);
				CHECK(points3D.find(point3DID) != points3D.end());
				pointsToBeFiltered.insert(point3DID);
			}
		}
	}
	return FilterPoints3D(maxReprojectionError, minTriAngle, pointsToBeFiltered);
}
size_t CModel::FilterAllPoints3D(double maxReprojectionError, double minTriAngle)
{
	// 首先应该过滤掉重投影误差太大的观测和点
	unordered_set<size_t> allPoints3D = GetAllPoints3D();
	size_t numFiltered = 0;
	numFiltered += FilterPoints3DWithLargeReprojectionError(maxReprojectionError, allPoints3D);
	numFiltered += FilterPoints3DWithSmallTriangulationAngle(minTriAngle, allPoints3D);
	return numFiltered;
}
size_t CModel::FilterObservationsWithNegativeDepth()
{
	size_t numFiltered = 0;
	for (size_t imageID : regImageIDs)
	{
		const CImage& image = database->GetImage(imageID);
		const size_t numPoints2D = image.GetNumPoints2D();
		const Eigen::Matrix3x4d worldToCamera = image.GetWorldToCamera(modelID).ToMatrix();
		for (size_t point2DID = 0; point2DID < numPoints2D; point2DID++)
		{
			if (image.IsPoint2DHasPoint3D(modelID, point2DID))
			{
				const size_t point3DID = image.GetPoint3DID(point2DID, modelID);
				CHECK(points3D.find(point3DID) != points3D.end());
				const CPoint3D& point3D = points3D[point3DID];
				if (!HasPointPositiveDepth(worldToCamera, point3D.GetXYZ()))
				{
					DeleteObservation(imageID, point2DID);
					numFiltered++;
				}
			}
		}
	}
	return numFiltered;
}
vector<size_t> CModel::FilterImages(double minFocalLengthRatio, double maxFocalLengthRatio, double maxExtraParam)
{

}







