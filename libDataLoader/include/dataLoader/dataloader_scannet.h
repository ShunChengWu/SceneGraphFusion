//
// Created by sc on 2/27/21.
//

#ifndef GRAPHSLAM_DATALOADER_SCANNET_H
#define GRAPHSLAM_DATALOADER_SCANNET_H

#include "dataset_loader.h"
#include <opencv2/core.hpp>
#include <Eigen/Core>
#include <string>
#include "../../ScanNetLoader/sensorData.h"
//#include "ScanNetLoader/sensorData.h"
//#include "CameraParameters.h"
//#include "../Objects/Camera/CameraParameters.h"

namespace PSLAM {
    class DatasetLoader_ScanNet : public DatasetLoader_base {
    public:
        explicit DatasetLoader_ScanNet(std::shared_ptr<DatasetDefinitionBase> dataset);
        void Reset() override;
        bool Retrieve() override;
    private:
        std::unique_ptr<::ml::SensorData> m_loader;
        bool GetDepth();
        bool GetRGB();
        bool GetPose();

        const std::string GetFileName(const std::string& folder,
                                                             const std::string& subfolder,
                                                             const std::string& prefix,
                                                             const std::string& suffix,
                                                             int number_length) const;
    };
}

#endif //GRAPHSLAM_DATALOADER_SCANNET_H
