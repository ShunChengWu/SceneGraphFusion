//
// Created by sc on 1/13/21.
//

#ifndef LIBSURFELRECONSTRUCTION_DATASETSCANNET_H
#define LIBSURFELRECONSTRUCTION_DATASETSCANNET_H

#include "dataset_base.h"

namespace PSLAM {
    class ScanNetDataset : public DatasetDefinitionBase {
    public:
        ScanNetDataset(INPUTE_TYPE type, const std::string &path) {
            datasetType = type;
            folder = path;
            frame_index_counter = 2; //TODO: make a IO for this
            number_length = 1;

            prefix_pose = "/frame-";
            prefix_depth = "/frame-";
            prefix_rgb = "/frame-";

            suffix_depth = ".depth.pgm";
            suffix_rgb = ".color.png";
            suffix_pose = ".pose.txt";
            if (rotate_pose_img) {
                suffix_depth = ".rendered.depth.png";
                suffix_pose = ".pose.txt";
            }

            min_pyr_level = 3;
            number_pose = 6;
            number_length = 6;
        }
    };
}

#endif //LIBSURFELRECONSTRUCTION_DATASETSCANNET_H
