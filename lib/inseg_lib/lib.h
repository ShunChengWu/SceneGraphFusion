#ifndef INSEG_LIB_LIB_H_
#define INSEG_LIB_LIB_H_

#include <string>

#include <opencv2/core/core.hpp>

#include "image_pyramid.h"
#include "inseg.h"
#include "inseg_config.h"
#include "main_config.h"
#include "map.h"
#include "map_config.h"
#include "segmentation_config.h"

namespace inseg_lib {

// Interface class for InSegLib.
class InSegLibInterface {
public:
    bool initialized = false;

    // Process frame function for the main thread.
    virtual void ProcessFrame(const cv::Mat& depth_map,
                              const cv::Mat& color_map) = 0;
    // Initialize camera intrinsics for image pyramid.
    virtual void InitializeCamera(const Eigen::Matrix3d& intrinsics,
                                  const int image_width,
                                  const int image_height) = 0;
    // Returns the current camera pose in world coordinates.
    virtual const Eigen::Matrix4f& pose() const = 0;
    // Set pose from outside (e.g. tango).
    virtual void set_pose(const Eigen::Matrix4f& pose) = 0;
    virtual void InitializeMap(const Eigen::Matrix4f& pose) = 0;
    virtual void Reset() = 0;
    // Get the current reconstructed and segmented map.
    virtual Map& map() = 0;
    // Returns image pyramid.
    virtual ImagePyramid& image_pyramid() = 0;
    // Returns the segmentation of the depth image.
    virtual const cv::Mat& GetSegmentedLabelMap() = 0;
    virtual const cv::Mat& GetFrameLabelMap() = 0;

    virtual void SetLabelMap() = 0;
    virtual void SetPropagatedLabelMap() = 0;
    virtual int GetLabelSize() = 0;
};

// class InSegReconstruction;
class InSegLib : public InSegLibInterface {
public:
    InSegLib(std::shared_ptr<inseg_lib::SegmentsInterface> segments,
             InSegConfig& config,
             MainConfig& main_config,
             MapConfig& map_config,
             const SegmentationConfig& segmentation_config);
    // Main process.
    void ProcessFrame(const cv::Mat& depth_map,
                      const cv::Mat& color_map) override;

    // Initialize map data and camera.
    void InitializeMap(const Eigen::Matrix4f& pose) override;
    void InitializeCamera(const Eigen::Matrix3d& intrinsics,
                          const int image_width,
                          const int image_height) override;
    void Reset() override;
    // Returns the current camera pose in world coordinates.
    const Eigen::Matrix4f& pose() const override;
    void SetLabelMap() override;
    void SetPropagatedLabelMap() override;
    
    // Set pose from outside (e.g. tango).
    void set_pose(const Eigen::Matrix4f& pose) override;
    // Get the current reconstructed and segmented map.
    Map& map() override;
    // Returns image pyramid.
    ImagePyramid& image_pyramid() override;
    // Returns the segmentation of the depth image.
    const cv::Mat& GetSegmentedLabelMap() override;
    const cv::Mat& GetFrameLabelMap() override;

    int GetLabelSize() override;
private:
    InSegReconstruction inseg_;
    // segmented map from depth image.
    cv::Mat segmented_labels_;
};
}  // namespace inseg_lib

#endif  // INSEG_LIB_LIB_H_
