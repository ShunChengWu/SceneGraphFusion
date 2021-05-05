//
// Created by sc on 1/13/21.
//

#ifndef LIBSURFELRECONSTRUCTION_UTIL_H
#define LIBSURFELRECONSTRUCTION_UTIL_H
#include <string>
#include <fstream>
#include <Eigen/Core>
namespace PSLAM {
    static inline bool isFileExist(const std::string &filename) {
        std::ifstream file_tmp(filename);
        if (!file_tmp.is_open()) {
            return false;
        }
        file_tmp.close();
        return true;
    }

    static inline void LoadPose(Eigen::Matrix4f &pose, const std::string &path, bool rotate) {
        pose.setIdentity();
        std::ifstream file(path);
        assert(file.is_open());
        if (file.is_open()) {
            for (int i = 0; i < 4; i++)
                for (int j = 0; j < 4; j++)
                    file >> pose(i, j);
            pose.block<3, 1>(0, 3) *= 1000.0f;
            file.close();
        }
//        std::cout << "pose\n"<< pose << "\n";
    }
}



#endif //LIBSURFELRECONSTRUCTION_UTIL_H
