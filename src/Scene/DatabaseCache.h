#pragma once
#include "Camera.h"
#include "CameraModel.h"
#include "CorrespondenceGraph.h"
#include "Database.h"
#include "Image.h"
#include "../Base/types.h"

#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <Eigen/Core>

// A class that caches the contents of the database in memory, used to quickly
// create new reconstruction instances when multiple models are reconstructed.
class DatabaseCache {
public:
    // Load cameras, images, features, and matches from database.
    //
    // @param database              Source database from which to load data.
    // @param min_num_matches       Only load image pairs with a minimum number
    //                              of matches.
    // @param ignore_watermarks     Whether to ignore watermark image pairs.
    // @param image_names           Whether to use only load the data for a subset
    //                              of the images. All images are used if empty.
    static std::shared_ptr<DatabaseCache> Create(const Database& database,size_t min_num_matches,bool ignore_watermarks, const std::unordered_set<std::string>& image_names);

    // Get number of objects.
    inline size_t NumCameras() const
    {
        return cameras_.size();
    }
    inline size_t NumImages() const
    {
        return images_.size();
    }

    // Get specific objects.
    inline class Camera& Camera(camera_t camera_id)
    {
        return cameras_.at(camera_id);
    }
    inline const class Camera& Camera(camera_t camera_id) const
    {
        return cameras_.at(camera_id);
    }
    inline class Image& Image(image_t image_id)
    {
        return images_.at(image_id);
    }
    inline const class Image& Image(image_t image_id) const
    {
        return images_.at(image_id);
    }

    // Get all objects.
    inline const std::unordered_map<camera_t, class Camera>& Cameras() const
    {
        return cameras_;
    }
    inline const std::unordered_map<image_t, class Image>& Images() const
    {
        return images_;
    }

    // Check whether specific object exists.
    inline bool ExistsCamera(camera_t camera_id) const
    {
        return cameras_.find(camera_id) != cameras_.end();
    }
    inline bool ExistsImage(image_t image_id) const
    {
        return images_.find(image_id) != images_.end();
    }

    // Get reference to const correspondence graph.
    inline std::shared_ptr<const class CorrespondenceGraph> CorrespondenceGraph() const
    {
        return correspondence_graph_;
    }

    // Find specific image by name. Note that this uses linear search.
    const class Image* FindImageWithName(const std::string& name) const;

private:
    std::shared_ptr<class CorrespondenceGraph> correspondence_graph_;
    static size_t lastCreateNumImages;
    static std::shared_ptr<DatabaseCache> lastCreateCache;

    std::unordered_map<camera_t, class Camera> cameras_;
    std::unordered_map<image_t, class Image> images_;
};