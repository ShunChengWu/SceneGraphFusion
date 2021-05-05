#ifndef INSEG_LIB_SEGMENTATION_CONFIG_H_
#define INSEG_LIB_SEGMENTATION_CONFIG_H_

namespace inseg_lib {
    
struct SegmentationConfig {
    // Minimum region size in pixel of a label to be propageded.
    int min_region_size = 10;
};
}  // namespace inseg_lib

#endif  // INSEG_LIB_SEGMENTATION_CONFIG_H_
