//
// Created by sc on 1/13/21.
//

#ifndef LIBSURFELRECONSTRUCTION_DATASET_BASE_H
#define LIBSURFELRECONSTRUCTION_DATASET_BASE_H
#include "dataset_types.h"
#include <string>
namespace PSLAM {
    class DatasetDefinitionBase {
    public:
        DatasetDefinitionBase() = default;
        virtual ~DatasetDefinitionBase() = default;

        INPUTE_TYPE datasetType;
        std::string folder = "";

        // TODO change to true if V2
        bool rotate_pose_img = true;

        int frame_index_counter = 1;
        int number_length = 0;
        int number_pose = 0;

        int max_depth = 5000; // mm

        std::string prefix_depth = "";
        std::string prefix_rgb = "";
        std::string prefix_pose = "";

        std::string suffix_depth = "";
        std::string suffix_rgb = "";
        std::string suffix_pose = "";

        std::string folder_depth = "";
        std::string folder_rgb = "";
        std::string folder_pose = "";

        int min_pyr_level = 1;
    };
}

#endif //LIBSURFELRECONSTRUCTION_DATASET_BASE_H
