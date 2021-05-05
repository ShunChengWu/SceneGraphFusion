#ifndef INSEG_LIB_CAMERA_H_
#define INSEG_LIB_CAMERA_H_

#include <vector>

#include <Eigen/Dense>

namespace inseg_lib {
    
// Camera parameter class to represent camera intrinsics. Focal length and
// principle point, image width and height.
class CamParams {
public:
    CamParams() = default;
    // Get a scaled version of the camera parameters.
    CamParams CreateScaledCamParams(const float scale) const;
    
    // Denormalize and normalize 2D point according to intrinsics.
    Eigen::Vector2f Denormalize(const Eigen::Vector2f& xy) const;
    Eigen::Vector2f Normalize(const Eigen::Vector2f& uv) const;
    
    // Focal Length.
    float fx = 0;
    float fy = 0;
    // Principal Point.
    float cx = 0;
    float cy = 0;
    // Image Size.
    int img_width = 0;
    int img_height = 0;
};

class InSegCamera {
public:
    // Set camera paramter list (for image pyramid). maximum levels is the height
    // of the pyramid, minimum level is the level that is used for the processing
    // such as labeling, segmentation and tracking. cparam is the base of pyramid.
    void Init(const CamParams& cparam,
              const int max_levels,
              const int min_levels);
    
    // Return camera paramters for a specific level of the image pyramid.
    const CamParams& GetCamParams(const int level) const;
    // Returns the camera paramater of the index image whereas the index map is 4
    // times as big as the main image.
    const CamParams& GetIndexCamParams() const;
    // Returns height of the image pyramid which is also the max level parameter.
    int GetMaxLevel() const;
private:
    // Camera paramaters of index image.
    CamParams camera_param_index_;
    // Vector of camera parameters that represents the image pyramid.
    std::vector<CamParams> camera_params_;
};
}  // namespace inseg_lib

#endif  // INSEG_LIB_CAMERA_H_
