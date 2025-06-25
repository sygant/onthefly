#pragma once
#include "../Geometry/Sim3D.h"
#include "../Base/types.h"
#include "Camera.h"
#include "Image.h"
#include "Database.h"
#include "Point2D.h"
#include "Point3D.h"
#include "Track.h"

#include <memory>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <tbb/concurrent_unordered_map.h>

#include <Eigen/Core>


struct PlyPoint;
struct RANSACOptions;
class DatabaseCache;
class CorrespondenceGraph;

// Reconstruction class holds all information about a single reconstructed
// model. It is used by the mapping and bundle adjustment classes and can be
// written to and read from disk.
class Reconstruction {
public:
    struct ImagePairStat {
        // The number of triangulated correspondences between two images.
        size_t num_tri_corrs = 0;
        // The number of total correspondences/matches between two images.
        size_t num_total_corrs = 0;
    };

    Reconstruction();
    ~Reconstruction();

    // Get number of objects.
    inline size_t NumCameras() const
    {
        return cameras_.size();
    }
    inline size_t NumImages() const
    {
        return images_.size();
    }
    inline size_t NumRegImages() const
    {
        return reg_image_ids_.size();
    }
    inline size_t NumPoints3D() const
    {
        return points3D_.size();
    }
    inline size_t NumImagePairs() const
    {
        return image_pair_stats_.size();
    }

    // Get const objects.
    inline const class Camera& Camera(camera_t camera_id) const
    {
        return cameras_.at(camera_id);
    }
    inline const class Image& Image(image_t image_id) const
    {
        return images_.at(image_id);
    }
    inline const class Point3D& Point3D(point3D_t point3D_id) const
    {
        return points3D_.at(point3D_id);
    }
    inline const ImagePairStat& ImagePair(image_pair_t pair_id) const
    {
        return image_pair_stats_.at(pair_id);
    }
    inline ImagePairStat& ImagePair(image_t image_id1, image_t image_id2)
    {
        const auto pair_id = Database::ImagePairToPairId(image_id1, image_id2);
        return image_pair_stats_.at(pair_id);
    }

    // Get mutable objects.
    inline class Camera& Camera(camera_t camera_id)
    {
        return cameras_.at(camera_id);
    }
    inline class Image& Image(image_t image_id)
    {
        return images_.at(image_id);
    }
    inline class Point3D& Point3D(point3D_t point3D_id)
    {
        return points3D_.at(point3D_id);
    }
    inline ImagePairStat& ImagePair(image_pair_t pair_id)
    {
        return image_pair_stats_.at(pair_id);
    }
    inline const ImagePairStat& ImagePair(image_t image_id1, image_t image_id2) const
    {
        const auto pair_id = Database::ImagePairToPairId(image_id1, image_id2);
        return image_pair_stats_.at(pair_id);
    }

    // Get reference to all objects.
    inline const tbb::concurrent_unordered_map<camera_t, class Camera> Cameras() const
    {
        return cameras_;
    }
    inline const tbb::concurrent_unordered_map<image_t, class Image>& Images() const
    {
        return images_;
    }
    inline const std::vector<image_t>& RegImageIds() const
    {
        return reg_image_ids_;
    }
    inline const tbb::concurrent_unordered_map<point3D_t, class Point3D>& Points3D() const
    {
        return points3D_;
    }
    inline const tbb::concurrent_unordered_map<image_pair_t, ImagePairStat>& ImagePairs() const
    {
        return image_pair_stats_;
    }

    // Identifiers of all 3D points.
    std::unordered_set<point3D_t> Point3DIds() const;

    // Check whether specific object exists.
    inline bool ExistsCamera(camera_t camera_id) const
    {
        return cameras_.find(camera_id) != cameras_.end();
    }
    inline bool ExistsImage(image_t image_id) const
    {
        return images_.find(image_id) != images_.end();
    }
    inline bool ExistsPoint3D(point3D_t point3D_id) const
    {
        return points3D_.find(point3D_id) != points3D_.end();
    }
    inline bool ExistsImagePair(image_pair_t pair_id) const
    {
        return image_pair_stats_.find(pair_id) != image_pair_stats_.end();
    }

    // Load data from given `DatabaseCache`.
    void Load(const DatabaseCache& database_cache);

    // Setup all relevant data structures before reconstruction. Note the
    // correspondence graph object must live until `TearDown` is called.
    void SetUp(std::shared_ptr<const CorrespondenceGraph> correspondence_graph);

    // Finalize the Reconstruction after the reconstruction has finished.
    //
    // Once a scene has been finalized, it cannot be used for reconstruction.
    //
    // This removes all not yet registered images and unused cameras, in order to
    // save memory.
    void TearDown();

    // Add new camera. There is only one camera per image, while multiple images
    // might be taken by the same camera.
    void AddCamera(class Camera camera);

    // Add new image.
    void AddImage(class Image image);

    // Add new 3D object, and return its unique ID.
    point3D_t AddPoint3D(
        const Eigen::Vector3d& xyz,
        Track track,
        const Eigen::Vector3ub& color = Eigen::Vector3ub::Zero());

    // Add observation to existing 3D point.
    void AddObservation(point3D_t point3D_id, const TrackElement& track_el);

    // Merge two 3D points and return new identifier of new 3D point.
    // The location of the merged 3D point is a weighted average of the two
    // original 3D point's locations according to their track lengths.
    point3D_t MergePoints3D(point3D_t point3D_id1, point3D_t point3D_id2);

    // Delete a 3D point, and all its references in the observed images.
    void DeletePoint3D(point3D_t point3D_id);

    // Delete one observation from an image and the corresponding 3D point.
    // Note that this deletes the entire 3D point, if the track has two elements
    // prior to calling this method.
    void DeleteObservation(image_t image_id, point2D_t point2D_idx);

    // Delete all 2D points of all images and all 3D points.
    void DeleteAllPoints2DAndPoints3D();

    // Register an existing image.
    void RegisterImage(image_t image_id);

    // De-register an existing image, and all its references.
    void DeRegisterImage(image_t image_id);

    // Check if image is registered.
    inline bool IsImageRegistered(image_t image_id) const
    {
        return Image(image_id).IsRegistered();
    }

    // Normalize scene by scaling and translation to avoid degenerate
    // visualization after bundle adjustment and to improve numerical
    // stability of algorithms.
    //
    // Translates scene such that the mean of the camera centers or point
    // locations are at the origin of the coordinate system.
    //
    // Scales scene such that the minimum and maximum camera centers are at the
    // given `extent`, whereas `p0` and `p1` determine the minimum and
    // maximum percentiles of the camera centers considered.
    void Normalize(double extent = 10.0,
        double p0 = 0.1,
        double p1 = 0.9,
        bool use_images = true);

    // Compute the centroid of the 3D points
    Eigen::Vector3d ComputeCentroid(double p0 = 0.1, double p1 = 0.9) const;

    // Compute the bounding box corners of the 3D points
    std::pair<Eigen::Vector3d, Eigen::Vector3d> ComputeBoundingBox(
        double p0 = 0.0, double p1 = 1.0) const;

    // Apply the 3D similarity transformation to all images and points.
    void Transform(const Sim3d& new_from_old_world);

    // Find specific image by name. Note that this uses linear search.
    const class Image* FindImageWithName(const std::string& name) const;

    // Find images that are both present in this and the given reconstruction.
    // Matching of images is performed based on common image names.
    std::vector<std::pair<image_t, image_t>> FindCommonRegImageIds(
        const Reconstruction& other) const;

    // Update the image identifiers to match the ones in the database by matching
    // the names of the images.
    void TranscribeImageIdsToDatabase(const Database& database);

    // Filter 3D points with large reprojection error, negative depth, or
    // insufficient triangulation angle.
    //
    // @param max_reproj_error    The maximum reprojection error.
    // @param min_tri_angle       The minimum triangulation angle.
    // @param point3D_ids         The points to be filtered.
    //
    // @return                    The number of filtered observations.
    size_t FilterPoints3D(double max_reproj_error,
        double min_tri_angle,
        const std::unordered_set<point3D_t>& point3D_ids);
    size_t FilterPoints3DInImages(double max_reproj_error,
        double min_tri_angle,
        const std::unordered_set<image_t>& image_ids);
    size_t FilterAllPoints3D(double max_reproj_error, double min_tri_angle);

    // Filter observations that have negative depth.
    //
    // @return    The number of filtered observations.
    size_t FilterObservationsWithNegativeDepth();

    // Filter images without observations or bogus camera parameters.
    //
    // @return    The identifiers of the filtered images.
    std::vector<image_t> FilterImages(double min_focal_length_ratio,
        double max_focal_length_ratio,
        double max_extra_param);

    // Compute statistics for scene.
    size_t ComputeNumObservations() const;
    double ComputeMeanTrackLength() const;
    double ComputeMeanObservationsPerRegImage() const;
    double ComputeMeanReprojectionError() const;

    // Updates mean reprojection errors for all 3D points.
    void UpdatePoint3DErrors();

    // Read data from text or binary file. Prefer binary data if it exists.
    void Read(const std::string& path);
    void Write(const std::string& path) const;

    // Read data from binary/text file.
    void ReadText(const std::string& path);
    void ReadBinary(const std::string& path);

    // Write data from binary/text file.
    void WriteText(const std::string& path) const;
    void WriteBinary(const std::string& path) const;

    // Convert 3D points in reconstruction to PLY point cloud.
    std::vector<PlyPoint> ConvertToPLY() const;

    // Import from other data formats. Note that these import functions are
    // only intended for visualization of data and usable for reconstruction.
    void ImportPLY(const std::string& path);
    void ImportPLY(const std::vector<PlyPoint>& ply_points);

    // Export to other data formats.

    // Exports in NVM format http://ccwu.me/vsfm/doc.html#nvm. Only supports
    // SIMPLE_RADIAL camera model when exporting distortion parameters. When
    // skip_distortion == true it supports all camera models with the caveat that
    // it's using the mean focal length which will be inaccurate for camera models
    // with two focal lengths and distortion.
    bool ExportNVM(const std::string& path, bool skip_distortion = false) const;

    // Exports in CAM format which is a simple text file that contains pose
    // information and camera intrinsics for each image and exports one file per
    // image; it does not include information on the 3D points. The format is as
    // follows (2 lines of text with space separated numbers):
    // <Tvec; 3 values> <Rotation matrix in row-major format; 9 values>
    // <focal_length> <k1> <k2> 1.0 <principal point X> <principal point Y>
    // Note that focal length is relative to the image max(width, height),
    // and principal points x and y are relative to width and height respectively.
    //
    // Only supports SIMPLE_RADIAL and RADIAL camera models when exporting
    // distortion parameters. When skip_distortion == true it supports all camera
    // models with the caveat that it's using the mean focal length which will be
    // inaccurate for camera models with two focal lengths and distortion.
    bool ExportCam(const std::string& path, bool skip_distortion = false) const;

    // Exports in Recon3D format which consists of three text files with the
    // following format and content:
    // 1) imagemap_0.txt: a list of image numeric IDs with one entry per line.
    // 2) urd-images.txt: A list of images with one entry per line as:
    //    <image file name> <width> <height>
    // 3) synth_0.out: Contains information for image poses, camera intrinsics,
    //    and 3D points as:
    //    <N; num images> <M; num points>
    //    <N lines of image entries>
    //    <M lines of point entries>
    //
    //    Each image entry consists of 5 lines as:
    //    <focal length> <k1> <k2>
    //    <Rotation matrix; 3x3 array>
    //    <Tvec; 3 values>
    //    Note that the focal length is scaled by 1 / max(width, height)
    //
    //    Each point entry consists of 3 lines as:
    //    <point x, y, z coordinates>
    //    <point RGB color>
    //    <K; num track elements> <Track Element 1> ... <Track Element K>
    //
    //    Each track elemenet is a sequence of 5 values as:
    //    <image ID> <2D point ID> -1.0 <X> <Y>
    //    Note that the 2D point coordinates are centered around the principal
    //    point and scaled by 1 / max(width, height).
    //
    // When skip_distortion == true it supports all camera models with the
    // caveat that it's using the mean focal length which will be inaccurate
    // for camera models with two focal lengths and distortion.
    bool ExportRecon3D(const std::string& path,
        bool skip_distortion = false) const;

    // Exports in Bundler format https://www.cs.cornell.edu/~snavely/bundler/.
    // Supports SIMPLE_PINHOLE, PINHOLE, SIMPLE_RADIAL and RADIAL camera models
    // when exporting distortion parameters. When skip_distortion == true it
    // supports all camera models with the caveat that it's using the mean focal
    // length which will be inaccurate for camera models with two focal lengths
    // and distortion.
    bool ExportBundler(const std::string& path,
        const std::string& list_path,
        bool skip_distortion = false) const;

    // Exports 3D points only in PLY format.
    void ExportPLY(const std::string& path) const;

    // Exports in VRML format https://en.wikipedia.org/wiki/VRML.
    void ExportVRML(const std::string& images_path,
        const std::string& points3D_path,
        double image_scale,
        const Eigen::Vector3d& image_rgb) const;

    // Extract colors for 3D points of given image. Colors will be extracted
    // only for 3D points which are completely black.
    //
    // @param image_id      Identifier of the image for which to extract colors.
    // @param path          Absolute or relative path to root folder of image.
    //                      The image path is determined by concatenating the
    //                      root path and the name of the image.
    //
    // @return              True if image could be read at given path.
    bool ExtractColorsForImage(image_t image_id, const std::string& path);

    // Extract colors for all 3D points by computing the mean color of all images.
    //
    // @param path          Absolute or relative path to root folder of image.
    //                      The image path is determined by concatenating the
    //                      root path and the name of the image.
    void ExtractColorsForAllImages(const std::string& path);

    // Create all image sub-directories in the given path.
    void CreateImageDirs(const std::string& path) const;

    void OutputDebugResult(const std::string& outputDir) const;


    std::atomic_size_t reconstructTime = 0;  // 单位: 毫秒

    double finalGloablBAError = 0;                               // 最后一次全局平差的反投影误差
    size_t finalGlobalBATime = 0;                                // 最后一次全局平差的耗时(毫秒)
    tbb::concurrent_vector<size_t> numLocalBAImages;             // 每一次局部平差涉及的影像数
    tbb::concurrent_vector<std::vector<double>> localBAErrors;   // 每一次局部平差各个迭代的反投影误差
    tbb::concurrent_vector<std::vector<size_t>> localBATimes;    // 每一次局部平差各个迭代的耗时(毫秒)
    tbb::concurrent_vector<size_t> registerTimes;                // 每一次注册影像的耗时(毫秒)
    tbb::concurrent_vector<size_t> triangulateTimes;             // 每一次前方交会的耗时(毫秒)
   
private:
    size_t FilterPoints3DWithSmallTriangulationAngle(
        double min_tri_angle, const std::unordered_set<point3D_t>& point3D_ids);
    size_t FilterPoints3DWithLargeReprojectionError(
        double max_reproj_error,
        const std::unordered_set<point3D_t>& point3D_ids);

    std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d>
        ComputeBoundsAndCentroid(double p0, double p1, bool use_images) const;

    void ReadCamerasText(const std::string& path);
    void ReadImagesText(const std::string& path);
    void ReadPoints3DText(const std::string& path);
    void ReadCamerasBinary(const std::string& path);
    void ReadImagesBinary(const std::string& path);
    void ReadPoints3DBinary(const std::string& path);

    void WriteCamerasText(const std::string& path) const;
    void WriteImagesText(const std::string& path) const;
    void WritePoints3DText(const std::string& path) const;
    void WriteCamerasBinary(const std::string& path) const;
    void WriteImagesBinary(const std::string& path) const;
    void WritePoints3DBinary(const std::string& path) const;

    void SetObservationAsTriangulated(image_t image_id, point2D_t point2D_idx, bool is_continued_point3D);
    void ResetTriObservations(image_t image_id, point2D_t point2D_idx, bool is_deleted_point3D);

    std::shared_ptr<const CorrespondenceGraph> correspondence_graph_;

    tbb::concurrent_unordered_map<camera_t, class Camera> cameras_;
    tbb::concurrent_unordered_map<image_t, class Image> images_;
    tbb::concurrent_unordered_map<point3D_t, class Point3D> points3D_;

    tbb::concurrent_unordered_map<image_pair_t, ImagePairStat> image_pair_stats_;

    // { image_id, ... } where `images_.at(image_id).registered == true`.
    std::vector<image_t> reg_image_ids_;

    // Total number of added 3D points, used to generate unique identifiers.
    point3D_t num_added_points3D_;
};


bool AlignReconstructionToLocations(
    const Reconstruction& reconstruction,
    const std::vector<std::string>& image_names,
    const std::vector<Eigen::Vector3d>& locations,
    int min_common_images,
    const RANSACOptions& ransac_options,
    Sim3d* tform);

// Robustly compute alignment between reconstructions by finding images that
// are registered in both reconstructions. The alignment is then estimated
// robustly inside RANSAC from corresponding projection centers. An alignment
// is verified by reprojecting common 3D point observations.
// The min_inlier_observations threshold determines how many observations
// in a common image must reproject within the given threshold.
bool AlignReconstructionsViaReprojections(
    const Reconstruction& src_reconstruction,
    const Reconstruction& tgt_reconstruction,
    double min_inlier_observations,
    double max_reproj_error,
    Sim3d* tgt_from_src);

// Robustly compute alignment between reconstructions by finding images that
// are registered in both reconstructions. The alignment is then estimated
// robustly inside RANSAC from corresponding projection centers and by
// minimizing the Euclidean distance between them in world space.
bool AlignReconstructionsViaProjCenters(
    const Reconstruction& src_reconstruction,
    const Reconstruction& tgt_reconstruction,
    double max_proj_center_error,
    Sim3d* tgt_from_src);

// Robustly compute the alignment between reconstructions that share the
// same 2D points. It is estimated by minimizing the 3D distance between
// corresponding 3D points.
bool AlignReconstructionsViaPoints(const Reconstruction& src_reconstruction,
    const Reconstruction& tgt_reconstruction,
    size_t min_common_observations,
    double max_error,
    double min_inlier_ratio,
    Sim3d* tgt_from_src);

// Compute image alignment errors in the target coordinate frame.
struct ImageAlignmentError {
    std::string image_name;
    double rotation_error_deg = -1;
    double proj_center_error = -1;
};
std::vector<ImageAlignmentError> ComputeImageAlignmentError(
    const Reconstruction& src_reconstruction,
    const Reconstruction& tgt_reconstruction,
    const Sim3d& tgt_from_src);

// Aligns the source to the target reconstruction and merges cameras, images,
// points3D into the target using the alignment. Returns false on failure.
bool MergeReconstructions(double max_reproj_error,
    const Reconstruction& src_reconstruction,
    Reconstruction* tgt_reconstruction);