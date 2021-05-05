#ifndef INSEG_LIB_MAP_CONFIG_H_
#define INSEG_LIB_MAP_CONFIG_H_

namespace inseg_lib {
    
struct MapConfig {
    // Near threshold for depth values in mm. All other values are dropped.
    double near_depth_threshold = 10;
    
    // Far threshold for depth values in mm. All other values are dropped.
    double far_depth_threshold = 10000;
    
    // Number of bilateral filter iterations that should be applied
    // (0=disabled).
    int num_iterations_bilateral_filtering = 0;
    
    // Normalize depth data (for synthetic renderings).
    bool normalize_depth = false;
};
}  // namespace inseg_lib

#endif  // INSEG_LIB_MAP_CONFIG_H_
