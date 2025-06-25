#include "DatabaseCache.h"
#include "../Feature/Common.h"
#include "../Base/string.h"
#include "../Base/timer.h"
#include <unordered_set>


size_t DatabaseCache::lastCreateNumImages = 0;
std::shared_ptr<DatabaseCache> DatabaseCache::lastCreateCache = nullptr;

std::shared_ptr<DatabaseCache> DatabaseCache::Create(
    const Database& database,
    const size_t min_num_matches,
    const bool ignore_watermarks,
    const std::unordered_set<std::string>& image_names) {

    if (database.NumImages() == lastCreateNumImages && !lastCreateCache)
    {
        return lastCreateCache;
    }

    auto cache = std::make_shared<DatabaseCache>();

    //////////////////////////////////////////////////////////////////////////////
    // Load cameras
    //////////////////////////////////////////////////////////////////////////////

    Timer timer;

    timer.Start();
#ifdef _DEBUG
    std::cout << "Loading cameras..." << std::flush;
#endif // _DEBUG

    {
        std::vector<class Camera> cameras = database.ReadAllCameras();
        cache->cameras_.reserve(cameras.size());
        for (auto& camera : cameras) {
            const camera_t camera_id = camera.CameraId();
            cache->cameras_.emplace(camera_id, std::move(camera));
        }
    }
#ifdef _DEBUG
    std::cout << StringPrintf(" %d in %.3fs",
        cache->cameras_.size(),
        timer.ElapsedSeconds())
        << std::endl;
#endif // _DEBUG

    //////////////////////////////////////////////////////////////////////////////
    // Load matches
    //////////////////////////////////////////////////////////////////////////////

    timer.Restart();
#ifdef _DEBUG
    std::cout << "Loading matches..." << std::flush;
#endif // _DEBUG

    std::vector<image_pair_t> image_pair_ids;
    std::vector<TwoViewGeometry> two_view_geometries;
    database.ReadTwoViewGeometries(&image_pair_ids, &two_view_geometries);

#ifdef _DEBUG
    std::cout << StringPrintf(" %d in %.3fs",
        image_pair_ids.size(),
        timer.ElapsedSeconds())
        << std::endl;
#endif // _DEBUG

    auto UseInlierMatchesCheck = [min_num_matches, ignore_watermarks](
        const TwoViewGeometry& two_view_geometry) {
            return static_cast<size_t>(two_view_geometry.inlier_matches.size()) >=
                min_num_matches &&
                (!ignore_watermarks ||
                    two_view_geometry.config != TwoViewGeometry::WATERMARK);
        };

    //////////////////////////////////////////////////////////////////////////////
    // Load images
    //////////////////////////////////////////////////////////////////////////////

    timer.Restart();

#ifdef _DEBUG
    std::cout << "Loading images..." << std::flush; 
#endif // _DEBUG


    std::unordered_set<image_t> image_ids;

    {
        std::vector<class Image> images = database.ReadAllImages();
        const size_t num_images = images.size();

        // Determines for which images data should be loaded.
        if (image_names.empty()) {
            for (const auto& image : images) {
                image_ids.insert(image.ImageId());
            }
        }
        else {
            for (const auto& image : images) {
                if (image_names.count(image.Name()) > 0) {
                    image_ids.insert(image.ImageId());
                }
            }
        }

        // Collect all images that are connected in the correspondence graph.
        std::unordered_set<image_t> connected_image_ids;
        connected_image_ids.reserve(image_ids.size());
        for (size_t i = 0; i < image_pair_ids.size(); ++i) {
            if (UseInlierMatchesCheck(two_view_geometries[i])) {
                image_t image_id1;
                image_t image_id2;
                Database::PairIdToImagePair(image_pair_ids[i], &image_id1, &image_id2);
                if (image_ids.count(image_id1) > 0 && image_ids.count(image_id2) > 0) {
                    connected_image_ids.insert(image_id1);
                    connected_image_ids.insert(image_id2);
                }
            }
        }

        // Load images with correspondences and discard images without
        // correspondences, as those images are useless for SfM.
        cache->images_.reserve(connected_image_ids.size());
        for (auto& image : images) {
            const image_t image_id = image.ImageId();
            if (image_ids.count(image_id) > 0 &&
                connected_image_ids.count(image_id) > 0) {
                image.SetPoints2D(
                    FeatureKeypointsToPointsVector(database.ReadKeypoints(image_id)));
                cache->images_.emplace(image_id, std::move(image));
            }
        }

#ifdef _DEBUG
        std::cout << StringPrintf(" %d in %.3fs (connected %d)",
            num_images,
            timer.ElapsedSeconds(),
            connected_image_ids.size())
            << std::endl;
#endif // _DEBUG
    }

    //////////////////////////////////////////////////////////////////////////////
    // Build correspondence graph
    //////////////////////////////////////////////////////////////////////////////

    timer.Restart();

#ifdef _DEBUG
    std::cout << "Building correspondence graph..." << std::flush;
#endif // _DEBUG


    cache->correspondence_graph_ = std::make_shared<class CorrespondenceGraph>();

    for (const auto& image : cache->images_) {
        cache->correspondence_graph_->AddImage(image.first,
            image.second.NumPoints2D());
    }

    size_t num_ignored_image_pairs = 0;
    for (size_t i = 0; i < image_pair_ids.size(); ++i) {
        if (UseInlierMatchesCheck(two_view_geometries[i])) {
            image_t image_id1;
            image_t image_id2;
            Database::PairIdToImagePair(image_pair_ids[i], &image_id1, &image_id2);
            if (image_ids.count(image_id1) > 0 && image_ids.count(image_id2) > 0) {
                cache->correspondence_graph_->AddCorrespondences(
                    image_id1, image_id2, two_view_geometries[i].inlier_matches);
            }
            else {
                num_ignored_image_pairs += 1;
            }
        }
        else {
            num_ignored_image_pairs += 1;
        }
    }

    cache->correspondence_graph_->Finalize();

    // Set number of observations and correspondences per image.
    for (auto& image : cache->images_) {
        image.second.SetNumObservations(
            cache->correspondence_graph_->NumObservationsForImage(image.first));
        image.second.SetNumCorrespondences(
            cache->correspondence_graph_->NumCorrespondencesForImage(image.first));
    }

#ifdef _DEBUG
    std::cout << StringPrintf(" in %.3fs (ignored %d)",
        timer.ElapsedSeconds(),
        num_ignored_image_pairs)
        << std::endl;
#endif // _DEBUG

    lastCreateNumImages = database.NumImages();
    lastCreateCache = cache;
    return cache;
}

const class Image* DatabaseCache::FindImageWithName(
    const std::string& name) const {
    for (const auto& image : images_) {
        if (image.second.Name() == name) {
            return &image.second;
        }
    }
    return nullptr;
}