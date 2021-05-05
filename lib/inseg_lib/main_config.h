#ifndef INSEG_LIB_MAIN_CONFIG_H_
#define INSEG_LIB_MAIN_CONFIG_H_

#include <stdint.h>
#include <string.h>

namespace inseg_lib {

// Represents an invalid label.
constexpr int16_t kLabelUnknown = 0;
// Represents an edge in the edge image.
constexpr int16_t kEdge = 0;
// Represents no edge in the edge image.
constexpr int16_t kNoEdge = 255;

struct MainConfig {
    // Depth uncertainty coefficient.
    double uncertainty_coefficient_depth = 0.0000285f;
    
    // Max pyramid level for main the process.
    int max_pyr_level = 6;
    
    // Main pyramid level for main process.
    int min_pyr_level = 3;
};

}  // namespace inseg_lib

#endif  // INSEG_LIB_MAIN_CONFIG_H_
