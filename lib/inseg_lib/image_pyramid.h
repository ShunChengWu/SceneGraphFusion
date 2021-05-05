#ifndef INSEG_LIB_IMAGE_PYRAMID_H_
#define INSEG_LIB_IMAGE_PYRAMID_H_

#include <opencv2/core.hpp>

#include "camera.h"
#include "map_config.h"

namespace inseg_lib {

// Class represents frame pyramid with depth map, point cloud, normal map.
// Model frame is frame from the model view.
class ImagePyramid {
public:
    explicit ImagePyramid(const MapConfig& config,
                          const int max_level, const int min_level);
    
    void CreateDepthPyramid(const cv::Mat& depth_image,
                            const cv::Mat& color_image,
                            const bool normalize_depth,
                            const InSegCamera& camera,
                            const float uncertainty_coefficient_depth);
    
    // Returns point cloud (vertex map) at the level.
    cv::Mat& GetFramePointCloud();
    const cv::Mat& GetFramePointCloud(int level) const;
    
    // Returns normal map at the level.
    const cv::Mat& GetFrameNormalMap(int level = 0) const;
    
    const cv::Mat& GetRGBImage() const;
    const cv::Mat& GetRGBImageHD() const;
    
    // Returns normal map.
    cv::Mat& GetFrameLabelMapNotConst();
    cv::Mat& GetModelLabelMapNotConst();
    
    // Returns the segmented depth map.
    const cv::Mat& GetFrameLabelMap() const;
    
    void PyrModelDown(const int max_level);
    void PyrFrameDown(const int max_level);
    void PyrDownMap(const cv::Mat& upper, cv::Mat* cur);
    
    // Returns point cloud map
    cv::Mat& GetModelPointCloud();
    const cv::Mat& GetModelPointCloud(int level) const;
    
    // Returns normal map at the level.
    cv::Mat& GetModelNormalMapNotConst(int level = 0);
    cv::Mat& GetFrameNormalMapNotConst(int level = 0);
    
    const cv::Mat& GetModelNormalMap(int level = 0) const;
    
    // Sets label map from model view.
    void SetModelLabelMap(cv::Mat* label);
    void SetFrameLabelMap(cv::Mat* label);

    // Propagation of segmentation (rendered point).
    const cv::Mat& GetModelLabelMap() const;    
private:
    const int min_level_;
    const MapConfig& config_;
    
    // Image pyramids of depth, point cloud and normal map.
    std::vector<cv::Mat> frame_pyr_depth_img_;
    std::vector<cv::Mat> frame_pyr_point_cloud_;
    // Image pyramids of depth map. These pyramid levels are needed for tracking.
    std::vector<cv::Mat> model_pyr_point_cloud_;
    std::vector<cv::Mat> frame_pyr_rgb_img_;
    cv::Mat frame_rgb_img_;
    
    // Image pyramids of normal map.
    std::vector<cv::Mat> model_pyr_normal_map_;
    std::vector<cv::Mat> frame_pyr_normal_map_;
    
    // Label map.
    cv::Mat frame_label_map_;
    // Image pyramids of label map.
    cv::Mat model_label_map_;
    
    // Image pyramids of smoothed depth image and point cloud.
    std::vector<cv::Mat> frame_pyr_smooth_depth_img_;
    std::vector<cv::Mat> frame_pyr_smooth_point_cloud_;
};
    
// Calculate normal Map from depth map.
void FilterBilateral3x3(const cv::Mat& source,
                        const float uncertainty_coef_depth,
                        cv::Mat* destination);

float AddBilateralFilterElem(const float depth,
                             const int16_t coefficient,
                             const float reference_depth,
                             const float depth_variance);

void CalculateNormalMap(const cv::Mat& point_cloud, cv::Mat* normal_image);
void CalculatePointCloud(const cv::Mat& depth_image,
                         const CamParams& cam_params,
                         const bool normalize_depth,
                         cv::Mat* point_cloud);
    
}  // namespace inseg_lib

#endif  // INSEG_LIB_IMAGE_PYRAMID_H_
