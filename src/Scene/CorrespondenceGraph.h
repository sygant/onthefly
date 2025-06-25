#pragma once
#include "Database.h"
#include "../Base/types.h"

#include <unordered_map>
#include <vector>


// Scene graph represents the graph of image to image and feature to feature
// correspondences of a dataset. It should be accessed from the DatabaseCache.
class CorrespondenceGraph {
public:
    struct Correspondence {
        Correspondence()
            : image_id(kInvalidImageId), point2D_idx(kInvalidPoint2DIdx) {}
        Correspondence(const image_t image_id, const point2D_t point2D_idx)
            : image_id(image_id), point2D_idx(point2D_idx) {}

        // The identifier of the corresponding image.
        image_t image_id;

        // The index of the corresponding point in the corresponding image.
        point2D_t point2D_idx;
    };

    // Range of correspondences from [beg, end). Empty if beg == end.
    struct CorrespondenceRange {
        const Correspondence* beg = nullptr;
        const Correspondence* end = nullptr;
    };

    CorrespondenceGraph() = default;

    // Number of added images.
    inline size_t NumImages() const
    {
        return images_.size();
    }

    // Number of added images.
    inline size_t NumImagePairs() const
    {
        return image_pairs_.size();
    }

    // Check whether image exists.
    inline bool ExistsImage(image_t image_id) const
    {
        return images_.find(image_id) != images_.end();
    }

    // Get the number of observations in an image. An observation is an image
    // point that has at least one correspondence.
    inline point2D_t NumObservationsForImage(image_t image_id) const
    {
        return images_.at(image_id).num_observations;
    }

    // Get the number of correspondences per image.
    inline point2D_t NumCorrespondencesForImage(image_t image_id) const
    {
        return images_.at(image_id).num_correspondences;
    }

    // Get the number of correspondences between a pair of images.
    inline point2D_t NumCorrespondencesBetweenImages(image_t image_id1,
        image_t image_id2) const
    {
        const image_pair_t pair_id =
            Database::ImagePairToPairId(image_id1, image_id2);
        const auto it = image_pairs_.find(pair_id);
        if (it == image_pairs_.end()) {
            return 0;
        }
        else {
            return it->second.num_correspondences;
        }
    }

    // Get the number of correspondences between all images.
    std::unordered_map<image_pair_t, point2D_t> NumCorrespondencesBetweenImages()
        const;

    // Finalize the database manager.
    //
    // - Calculates the number of observations per image by counting the number
    //   of image points that have at least one correspondence.
    // - Deletes images without observations, as they are useless for SfM.
    // - Shrinks the correspondence vectors to their size to save memory.
    void Finalize();

    // Add new image to the correspondence graph.
    void AddImage(image_t image_id, size_t num_points2D);

    // Add correspondences between images. This function ignores invalid
    // correspondences where the point indices are out of bounds or duplicate
    // correspondences between the same image points. Whenever either of the two
    // cases occur this function prints a warning to the standard output.
    void AddCorrespondences(image_t image_id1,
        image_t image_id2,
        const FeatureMatches& matches);

    // Find range of correspondences of an image observation to all other images.
    CorrespondenceRange FindCorrespondences(image_t image_id,
        point2D_t point2D_idx) const;

    // Helper method to extract found correspondences into a vector.
    void ExtractCorrespondences(image_t image_id,
        point2D_t point2D_idx,
        std::vector<Correspondence>* corrs) const;

    // Extract correspondences to the given observation.
    //
    // Transitively collects correspondences to the given observation by first
    // finding correspondences to the given observation, then looking for
    // correspondences to the collected correspondences in the first step, and so
    // forth until the transitivity is exhausted or no more correspondences are
    // found. The returned list does not contain duplicates and contains
    // the given observation.
    void ExtractTransitiveCorrespondences(
        image_t image_id,
        point2D_t point2D_idx,
        size_t transitivity,
        std::vector<Correspondence>* corrs) const;

    // Find all correspondences between two images.
    FeatureMatches FindCorrespondencesBetweenImages(image_t image_id1,
        image_t image_id2) const;

    // Check whether the image point has correspondences.
    inline bool HasCorrespondences(image_t image_id, point2D_t point2D_idx) const
    {
        const CorrespondenceRange range = FindCorrespondences(image_id, point2D_idx);
        return range.beg != range.end;
    }

    // Check whether the given observation is part of a two-view track, i.e.
    // it only has one correspondence and that correspondence has the given
    // observation as its only correspondence.
    bool IsTwoViewObservation(image_t image_id, point2D_t point2D_idx) const;

private:
    struct Image {
        // Number of 2D points with at least one correspondence to another image.
        point2D_t num_observations = 0;

        // Total number of correspondences to other images. This measure is useful
        // to find a good initial pair, that is connected to many images.
        point2D_t num_correspondences = 0;

        // Correspondences to other images per image point.
        // Added correspondences before Finalize().
        std::vector<std::vector<Correspondence>> corrs;
        // Flattened correspondences after Finalize().
        std::vector<Correspondence> flat_corrs;
        // For each point, determines the beginning of the correspondences in the
        // flat_corrs vector. The end of point i is determined by the beginning of
        // the next point. The length of this vector is num_points2D + 1, where the
        // last element is equivalent to the size of flat_corrs.
        std::vector<point2D_t> flat_corr_begs;
    };

    struct ImagePair {
        // The number of correspondences between pairs of images.
        point2D_t num_correspondences = 0;
    };

    bool finalized_ = false;
    tbb::concurrent_unordered_map<image_t, Image> images_;
    tbb::concurrent_unordered_map<image_pair_t, ImagePair> image_pairs_;
};