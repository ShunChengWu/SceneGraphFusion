//
// Created by sc on 1/13/21.
//

#ifndef LIBSURFELRECONSTRUCTION_DATASET3RSCAN_H
#define LIBSURFELRECONSTRUCTION_DATASET3RSCAN_H

#include "dataset_base.h"

namespace PSLAM {
    class Scan3RDataset : public DatasetDefinitionBase {
    public:
        Scan3RDataset(INPUTE_TYPE type, const std::string &path)  {
            datasetType = type;
            folder = path;
            frame_index_counter = 1;
            number_length = 1;
            prefix_pose = "/frame-";
            prefix_depth = "/frame-";
            prefix_rgb = "/frame-";

            suffix_depth = ".depth.pgm";
            suffix_rgb = ".color.jpg";
            suffix_pose = ".pose.txt";
            if (rotate_pose_img) {
//                suffix_depth = ".rendered.depth.png";
//                suffix_pose = ".align.pose.txt";
                suffix_depth = ".depth.pgm";
                suffix_pose = ".pose.txt";
//                suffix_pose = use_aligned_pose? ".align.pose.txt" : ".pose.txt";
            }

            min_pyr_level = 3;
            number_pose = 6;
            number_length = 6;
        }
        bool use_aligned_pose = true;
    };
}

#endif //LIBSURFELRECONSTRUCTION_DATASET3RSCAN_H
