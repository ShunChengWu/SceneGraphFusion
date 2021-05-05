#ifndef _H_INC_DATASET_LOADER_
#define _H_INC_DATASET_LOADER_

#include <fstream>
#include <string>
#include <utility>
#include <vector>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <Eigen/Core>
#include <string>
#include "dataset_base.h"
#include "CameraParameters.h"

namespace PSLAM {
    class DatasetLoader_base {
    public:
        explicit DatasetLoader_base(std::shared_ptr<DatasetDefinitionBase> dataset):
                m_dataset(std::move(dataset)){
        }
        virtual ~DatasetLoader_base() = default;

        virtual void Reset() = 0;
        virtual bool Retrieve() = 0;

        [[nodiscard]] const cv::Mat& GetRGBImage() const { return m_rgb; }
        [[nodiscard]] const cv::Mat& GetDepthImage() const { return m_d; }

        [[nodiscard]] const Eigen::Matrix4f& GetPose() const { return m_pose; }
        [[nodiscard]] const CameraParameters &GetCamParamDepth() const { return m_cam_param_d; }
        [[nodiscard]] const CameraParameters &GetCamParamRGB() const { return m_cam_param_rgb; }
        const DatasetDefinitionBase * GetDataBase(){return m_dataset.get();}

        const int& GetFrameIndex() const { return frame_index; }

        const DatasetDefinitionBase* GetBase() const {return m_dataset.get(); }

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    protected:
        std::shared_ptr<DatasetDefinitionBase> m_dataset;
        int frame_index = 0, frame_index_max = 0;
        cv::Mat m_rgb, m_d;
        Eigen::Matrix4f m_pose;
        CameraParameters m_cam_param_rgb, m_cam_param_d;
    };





}
#endif
