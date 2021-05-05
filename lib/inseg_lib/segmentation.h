#ifndef INSEG_LIB_SEGMENTATION_H_
#define INSEG_LIB_SEGMENTATION_H_

#include <utility>

#include <Eigen/Dense>
#include <opencv2/core/core.hpp>

#include "disjoint_set.h"

namespace inseg_lib {

// Interface class for the segmentation.
class SegmentationInterface {
public:
  virtual int Segment(const cv::Mat& point_cloud,
                      const cv::Mat& normal_map,
                      cv::Mat* label_map) = 0;
};

// Segmentation by Normal Edge.
class InSegSegmentation : public SegmentationInterface {
public:
  InSegSegmentation(const float depth_edge_threshold,
                    const float uncertainty_coefficient_depth,
                    const int min_region_size)
      : depth_edge_threshold_(depth_edge_threshold),
        uncertainty_coefficient_depth_(uncertainty_coefficient_depth),
        min_region_size_(min_region_size) {}
  int Segment(const cv::Mat& point_cloud,
              const cv::Mat& normal_map,
              cv::Mat* label_map) override;

private:
  void CalculateGeometricalEdge(const cv::Mat& point_cloud,
                                const cv::Mat& normal_map);
  void Segment(cv::Mat* label_map);
  void RegionGrowing(const cv::Mat& point_cloud,
                     const cv::Mat& normal_map, cv::Mat* label_map);
  int Relabel(cv::Mat* label_map);
  void CreateEdges(const cv::Mat& normal_edge, const cv::Mat& point_cloud,
                   const cv::Mat& normal_map);
  float CalculateEdgeWeight(const Eigen::Vector3f& point,
                            const Eigen::Vector3f& normal,
                            const Eigen::Vector3f& other_vert,
                            const Eigen::Vector3f& other_normal);
  float CalculateAngleMin(const Eigen::Vector3f& point,
                          const Eigen::Vector3f& normal,
                          const Eigen::Vector3f& other_vert,
                          const Eigen::Vector3f& other_normal, float min_dot);

  const float depth_edge_threshold_;
  const float uncertainty_coefficient_depth_;
  const int min_region_size_;
  cv::Mat normal_edge_;
  DisjointSet labels_;
};
}  // namespace inseg_lib

#endif  // INSEG_LIB_SEGMENTATION_H_
