#ifndef INSEG_LIB_INSEG_CONFIG_H_
#define INSEG_LIB_INSEG_CONFIG_H_

namespace inseg_lib {

// Settings for the InSegReconstruction class.
struct InSegConfig {
    // Defines if dynamic points should be removed.
    // This is useful if the scene is not static.
    // Might remove noisy depth input on object borders.
    bool remove_dynamic_point = false;

    // Threshold when a surfel is set to invalid (in frames).
    int surfel_invalid_frame_threshold = 30;

    // Frame threshold when a point is considered to be stable.
    int surfel_stable_frame_threshold = 2;

    // Angle threshold of normal vectors in the map update stage.
    // Point are considered invalid if angle (degrees) is higher than threshold.
    float update_point_angle_threshold = 75;

    // Threshold for the dot product of the normal of the surfels.
    // If dot product is less than the threshold we have a valid normal.
    float update_point_dot_product_threshold = 30;

    // Confidence value frame threshold when two label are merged.
    // This happens when they are when overlapping.
    int label_merge_confidence_threshold = 3; 

    // Overlap threshold to identify correct segments in propagation step.
    // If two labels overlap by this ratio they are merged.
    float label_merge_overlap_ratio = 0.3; // 0.2

    // Smalles size of a segment to be labled.
    // This threshold helps to remove noise.
    int label_min_size = 10;
    
    // This angle threshold is used to determine segment correspondences in the
    // label propagation step. If vertices are close (along viewing ray) and the
    // angle between the corresponding normals are lower than this threshold
    // the segments are considered to overlap.
    float label_point_dot_product_threshold = 10;
    
    // Threshold for the dot product of edges in segmentation step.
    // This defines how the surfaces are segmented.
//    float depth_edge_threshold = 0.98f; // segments_0 0.992f
    //TODO: add a IO for this
    float depth_edge_threshold = 0.98f;// for 3RScan
//    float depth_edge_threshold = 0.90f;// for ScanNet

    // Label size threshold used for the merging process.
    int label_size_threshold = 15;

    // Label region ratio for the merging process.
    // If two labels overlap by this ratio they will be merged.
    float label_region_ratio = 0.3;
};
}  // namespace inseg_lib

#endif  // INSEG_LIB_INSEG_CONFIG_H_
