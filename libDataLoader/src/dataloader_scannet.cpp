#include <dataLoader/dataloader_scannet.h>
//#include "dataset_loader.h"
#include <fstream>
#include <iomanip>
//#include "../ORUtils/Logging.h"
using namespace PSLAM;

bool isFileExist(const std::string& filename) {
    std::ifstream file_tmp(filename);
    if (!file_tmp.is_open()) {
        return false;
    }
    file_tmp.close();
    return true;
}

void LoadPose(Eigen::Matrix4f& pose, const std::string& path, bool rotate) {
    pose.setIdentity();
    std::ifstream file(path);
    assert(file.is_open());
    if (file.is_open()) {
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++)
                file >> pose(i, j);
        pose.block<3,1>(0,3) *= 1000.0f;
        file.close();
    }
//        std::cout << "pose\n"<< pose << "\n";
}

DatasetLoader_ScanNet::DatasetLoader_ScanNet(std::shared_ptr<DatasetDefinitionBase> dataset):
        DatasetLoader_base(dataset){
    if(m_dataset->folder.find(".sens") == std::string::npos) {
        std::fstream f(dataset->folder+"/intrinsics.txt",std::ios::in);
        if(!f.is_open()) throw std::runtime_error("");
        float fx,fy,cx,cy;
        int width,height;
        f >> fx >> fy >> cx >> cy >> width >> height;
        m_cam_param_d.Set(width,height,fx,fy,cx,cy);
    }
}

void DatasetLoader_ScanNet::Reset() {
    frame_index = 0;
    if(m_dataset->folder.find(".sens") != std::string::npos){
        m_loader.reset(new ::ml::SensorData(m_dataset->folder));
        m_cam_param_d.Set(m_loader->m_depthWidth, m_loader->m_depthHeight,
                          m_loader->m_calibrationDepth.m_intrinsic._m00,m_loader->m_calibrationDepth.m_intrinsic._m11,
                          m_loader->m_calibrationDepth.m_intrinsic._m02,m_loader->m_calibrationDepth.m_intrinsic._m12);
        m_cam_param_rgb.Set(m_loader->m_colorWidth, m_loader->m_colorHeight,
                            m_loader->m_calibrationColor.m_intrinsic._m00,m_loader->m_calibrationColor.m_intrinsic._m11,
                            m_loader->m_calibrationColor.m_intrinsic._m02,m_loader->m_calibrationColor.m_intrinsic._m12);
        frame_index_max = m_loader->m_frames.size();
//            printf("there are %zu images.\n",frame_index_max);
    } else {
        m_loader.reset();
    }
}

bool DatasetLoader_ScanNet::Retrieve() {
    if(m_loader) if(frame_index>=frame_index_max)return false;
    bool state = true;
    state &= GetPose();
    state &= GetDepth();
    state &= GetRGB();
    if(!state) return false;
    frame_index+=m_dataset->frame_index_counter;
    return true;
}

bool DatasetLoader_ScanNet::GetDepth(){
    if(m_loader) {
        unsigned short *depthData = m_loader->decompressDepthAlloc(frame_index);
        m_d.create(m_cam_param_d.height, m_cam_param_d.width, CV_16UC1);

        for (int r = 0; r < m_cam_param_d.height; ++r)
            for (int c = 0; c < m_cam_param_d.width; ++c) {
                m_d.at<unsigned short>(r, c) =
                        depthData[r * m_cam_param_d.width + c] > m_dataset->max_depth ? 0 : depthData[
                                r * m_cam_param_d.width + c];
            }

        std::free(depthData);
        return true;
    } else {
        std::string depthFilename = GetFileName(m_dataset->folder,
                                                m_dataset->folder_depth,
                                                m_dataset->prefix_depth,
                                                m_dataset->suffix_depth,
                                                m_dataset->number_length);
        bool isExist = isFileExist(depthFilename);
        if (!isExist) {
            frame_index = 0;
            std::stringstream ss;
            ss << "cannot find file: \n"<< depthFilename;
            throw std::runtime_error(ss.str());
            return false;
        }
        m_d = cv::imread(depthFilename, -1);
        // mask depth
        for(size_t c=0;c<(size_t)m_d.cols;++c){
            for(size_t r=0;r<(size_t)m_d.rows;++r){
                if(m_d.at<unsigned short>(r,c)>m_dataset->max_depth)
                    m_d.at<unsigned short>(r,c) = 0;
            }
        }
        return true;
    }
}
bool DatasetLoader_ScanNet::GetRGB(){
    if(m_loader) {
        ::ml::vec3uc *colorData = m_loader->decompressColorAlloc(frame_index);
        m_rgb.create(m_cam_param_rgb.height, m_cam_param_rgb.width, CV_8UC3);
        for (int r = 0; r < m_cam_param_rgb.height; ++r)
            for (int c = 0; c < m_cam_param_rgb.width; ++c) {
                m_rgb.at<cv::Vec3b>(r, c)[0] = colorData[r * m_cam_param_rgb.width + c].x;
                m_rgb.at<cv::Vec3b>(r, c)[1] = colorData[r * m_cam_param_rgb.width + c].y;
                m_rgb.at<cv::Vec3b>(r, c)[2] = colorData[r * m_cam_param_rgb.width + c].z;
            }

        cv::cvtColor(m_rgb, m_rgb, cv::COLOR_RGB2BGR);
        //TODO: copy to our format
        std::free(colorData);
    } else {
        std::string colorFilename = GetFileName(m_dataset->folder,
                                                m_dataset->folder_rgb,
                                                m_dataset->prefix_rgb,
                                                m_dataset->suffix_rgb,
                                                m_dataset->number_length);
        if (isFileExist(colorFilename.c_str())) {
            m_rgb = cv::imread(colorFilename, -1);
        } else {
            return false;
        }
    }
    return true;
}
bool DatasetLoader_ScanNet::GetPose(){
    if(m_loader) {
        memcpy(m_pose.data(), m_loader->m_frames[frame_index].getCameraToWorld().matrix,
               16 * sizeof(float));
        m_pose.transposeInPlace();
        m_pose.topRightCorner<3, 1>() *= 1000;
    } else {
        auto pose_file_name_ = GetFileName(m_dataset->folder,
                                           m_dataset->folder_pose,
                                           m_dataset->prefix_pose,
                                           m_dataset->suffix_pose,
                                           m_dataset->number_length);
        if(isFileExist(pose_file_name_.c_str())) {
            LoadPose(m_pose, pose_file_name_,false);
        } else {
            return false;
        }
    }
    return true;
}

const std::string DatasetLoader_ScanNet::GetFileName(const std::string& folder,
                                                     const std::string& subfolder,
                                                     const std::string& prefix,
                                                     const std::string& suffix,
                                                     int number_length) const {
    std::stringstream filename;
    const std::string path = (folder == "/" ? "" : folder) +
                             (subfolder == "/" ? "" : subfolder) +
                             (prefix == "/" ? "" : prefix);
    if (number_length < 0)
        filename << path << (suffix == "/" ? "" : suffix);
    else
        filename << path << std::setfill('0') << std::setw(number_length) << frame_index << (suffix == "/" ? "" : suffix);
    std::string s(filename.str());
    return s;
}