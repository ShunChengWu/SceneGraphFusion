#ifndef INSEG_LIB_INSEG_H_
#define INSEG_LIB_INSEG_H_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <opencv2/core/core.hpp>

#include "image_pyramid.h"
#include "inseg_config.h"
#include "main_config.h"
#include "map.h"
#include "map_config.h"
#include "segmentation.h"
#include "segmentation_config.h"

namespace inseg_lib {

// Depth based inseg main class.
class InSegReconstruction {
public:    
    InSegReconstruction(std::shared_ptr<inseg_lib::SegmentsInterface> segments,
                        InSegConfig& config,
                        MainConfig& main_config,
                        MapConfig& map_config,
                        const SegmentationConfig& segmentation_config);
    
    // Prepares image pyramid and also applies segmentation and tracking.
    // The current pose is used to propage the labels from the global model.
    // The confidence of correctly predicted segments is increased and added
    // to the global segmentation model.
    // If two segments overlap in consecutive frames they are merged.
    void ProcessFrame(const cv::Mat& depth_map, const cv::Mat& color_image);

    // Get current camera pose.
    const Eigen::Matrix4f& pose() const { return pose_; }
    
    // Set the pose e.g. tangos visual odomertry.
    void set_pose(const Eigen::Matrix4f& pose);
    
    // Returns the map (used for point cloud visualization).
    Map& map() { return map_; }
    
    // Returns image pyramid.
    ImagePyramid& image_pyramid() {
        return image_pyramid_;
    }
        
    // Initializes the global (segmentation) map. Once the map is initialized the
    // incremental segmentation starts.
    void InitializeMap(const Eigen::Matrix4f& pose);
    // Initializes the camera parameters for the image pyramid.
    void InitializeCamera(const CamParams& cparam);
    void Reset();

    const cv::Mat& GetFrameLabelMap();
    
    void SetLabelMap();
    void SetPropagatedLabelMap();
    
    int GetLabelSize() const;
        
    MapConfig& map_config;
    InSegConfig& config;
    MainConfig& main_config;
private:
    // Segments the image and labels it by using point cloud and normal map.
    void Label();
    
    // Make index map for correspondences between input frame and surfels on
    // world map.
    void MakeIndexMap();
    
    // Updates the Global Model (surfels and segments). Labels are updated, at
    // each new frame with segmentation information associated with the current
    // depth map. Segment are potentially merged.
    void UpdateMap();
    
    // In the label propagation function correspondences between the visible
    // segments of the GSM, and those on the current depth map are estimated
    // by checking whether the underlying surface of both segments are the same.
    void PropagateLabels();
    
    // Computes the number of labels currently in the label_map.
    void CalculateLabelSizeList(const cv::Mat* label_map,
                                const int label_num,
                                std::vector<int16_t>* label_size_list);
    
    InSegCamera camera_;
    std::unique_ptr<SegmentationInterface> segmentation_;
    ImagePyramid image_pyramid_;
    
    // Point cloud map in the world coordinate system.
    Map map_;
        
    // Camera pose in the world coordinate system.
    Eigen::Matrix4f pose_;
    
    // Look up data structure for labels to be merged.
    DisjointSet world_label_match_list_;
    
    // Index map.
    // TODO(WaldJohannaU): Change this to Eigen Matrix.
    cv::Mat index_map_;
    
    int label_num_ = 0;
    int label_num_frame_ = 0;
    
    cv::Mat label_map_frame_;
    cv::Mat label_map_propagated_frame_;
    
    // Number of labels in the global segmentation map.
    int world_label_num_ = 0;
    std::vector<int16_t> label_size_list_;
    std::set<int> instance_labels;

    // Label map to represent the world map.
    cv::Mat world_label_map_;
    // The label_connect_confidence_list_[i][j] is the merging confidence to merge
    // the segment with label i with another segment with label j. To do so,
    // label_connect_confidence_list_[i] stores a hash-map to save the confidence
    // values for the merging candidates for label j (key of inner hashmap). In
    // most cases the inner hash map has only one or no entries. The confidence
    // value of the map is increased every time the segments overlap.
    std::map<int, std::map<int, int>> label_connect_confidence_list_;
};
    
}  // namespace inseg_lib

#endif  // INSEG_LIB_INSEG_H_
