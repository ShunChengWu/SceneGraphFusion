#ifndef _H_INC_INSEG_MAP_DATA_
#define _H_INC_INSEG_MAP_DATA_

#include <Eigen/Dense>
#include <opencv2/core.hpp>
// #include <opencv2/ml.hpp>

#include "camera.h"
#include "map_config.h"
#include "map_segments.h"
#include "surfel.h"

// Surfel, Map (holds list of surfel), Frame Pyramid
namespace inseg_lib {

class Map { // represents world map with list of surfels (Global Model in the paper)
public:
    Map(std::shared_ptr<inseg_lib::SegmentsInterface> segments,
        MapConfig& config);
    ~Map();
    
    bool SaveSegments(const std::string& file, const std::string& file_list, const int color_type) const;
    // save model as .ply
    bool SaveModel(const std::string& filename, const int color_type = 2,
                   Eigen::Matrix4f pose = Eigen::Matrix4f::Identity()) const;
    
    int GetSize() const;

    const std::shared_ptr<Surfel> GetSurfel(const int index) const;

    void SetColor(const int index, const cv::Vec3b& col);
    void SetLabelConfidence(const int index, const float value);
    void SetStable(const int index, bool value);
    void SetInvalid(const int index);
    void Clear();
    void Add(const std::shared_ptr<Surfel> surfel);
    void UpdateSurfel(const int index, const Eigen::Vector3f& pos, const Eigen::Vector3f& normal,
                      const float confidence, const float radius);
    void Merge(const int index, const int index2);
    // Index is the index of the surfel in the global model.
    void SetLabel(const int index, const int label);
    std::vector<std::shared_ptr<Surfel>> surfels;
    std::shared_ptr<inseg_lib::SegmentsInterface> segments_;
private:
    void WritePlyHeader(std::ofstream& fout, const int vertex) const;
    bool SaveSurfels(const std::string& filename,
                     const int color_type,
                     const std::vector<std::shared_ptr<Surfel>>& surfel_list,
                     Eigen::Matrix4f pose = Eigen::Matrix4f::Identity()) const;
};

const cv::Vec3b& CalculateLabelColor(const int16_t label);

}  // namespace inseg_lib

#endif  // INSEG_LIB_MAP_H_
