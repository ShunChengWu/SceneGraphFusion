#ifndef _H_INC_INSEG_MAP_SEGMENTS_
#define _H_INC_INSEG_MAP_SEGMENTS_

#include <set>
#include <unordered_map>
#include <Eigen/Dense>

#include "surfel.h"
#include <iostream>
#include <memory>

namespace inseg_lib {

class SegmentsInterface {
public:
    virtual ~SegmentsInterface() { }
    virtual void Merge(const int segment_index1, const int segment_index2) = 0;
    virtual int Add(const std::shared_ptr<inseg_lib::Surfel> &surfel) = 0;
    virtual void Update(const std::shared_ptr<inseg_lib::Surfel> surfel,
                        const Eigen::Vector3f& pos, const Eigen::Vector3f& normal) = 0;
    virtual void Clear() = 0;
    virtual int UpdateLabel(const std::shared_ptr<inseg_lib::Surfel> surfel, const int label) = 0;
};

}  // namespace inseg_lib

#endif  // _H_INC_INSEG_MAP_SEGMENTS_
