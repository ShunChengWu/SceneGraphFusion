//
// Created by sc on 10/6/20.
//

#ifndef GRAPHSLAM_UTILDATAPROCESSING_H
#define GRAPHSLAM_UTILDATAPROCESSING_H

#include <algorithm>
#include <vector>
#include <string>
#include <cassert>
#include "UtilMath.h"
#include <map>
#include <set>
#include <random>
#include <iostream>
namespace PSLAM {
    class DataUtil {
    public:
        static ORUtils::VectorX<float,6> compute_bbox(const MemoryBlock2D &points){
            ORUtils::VectorX<float,6> bbox;
            bbox.Clear(0);
            for (size_t j=0;j<3;++j){
                bbox[j+0] = points.at<float>(0,j);
                bbox[j+3] = points.at<float>(0,j);
            }
            for (size_t i=1;i<points.mDims.x;++i) {
                for (size_t j=0;j<3;++j){
                    if (points.at<float>(i,j) < bbox[j+0]) {
                        bbox[j+0] = points.at<float>(i,j);
                    }
                    if (points.at<float>(i,j) > bbox[j+3]) {
                        bbox[j+3] = points.at<float>(i,j);
                    }
                }
            }
            return bbox;
        }

        static ORUtils::Vector3<float> compute_dims(const MemoryBlock2D &points){
            auto bbox = compute_bbox(points);
            ORUtils::Vector3<float> dims(0.f);
            for(size_t i=0;i<3;++i)
                dims[i] = bbox[3+i] - bbox[i];
            return dims;
        }

        static ORUtils::VectorX<float, 11> compute_descriptor(const MemoryBlock2D &points){
            ORUtils::VectorX<float, 11> descriptor;
            MemoryBlock mean, std;
            ORUtils::Vector3<float> dims;
            float volume,length;
            mean = Math::mean(points, 3);
            std  = Math::stddev(points,points.mDims.x,points.mDims.y, 3);
            dims = compute_dims(points);
            std::vector<float> dims_(3);
            for(size_t i=0;i<3;++i) dims_[i] = dims[i];
            volume = 1;
            for(size_t i=0;i<3;++i) {
                volume *= dims_[i];
            }

            length = *std::max_element(dims_.begin(),dims_.end());

            for(size_t i=0;i<3;++i){
                descriptor[i] = mean.at<float>(i);
                descriptor[i+3] = std.at<float>(i);
                descriptor[i+6] = dims[i];
            }
            descriptor[9] = volume;
            descriptor[10] = length;
            return descriptor;
        }

        template<typename T>
        static std::vector<T> Collect(T *data, int64_t *edge, size_t stride, size_t num_edge){
            std::vector<T> output(num_edge*stride);
            for(size_t i=0;i<num_edge;++i){
                memcpy(output.data()+i*stride,data+edge[i]*stride,  stride*sizeof(T));
            }
            return output;
        }
        template<typename T>
        static std::vector<T> Collect(const T *data, const int64_t *edge, size_t num_edge){
            std::vector<T> output(num_edge);
            for(size_t i=0;i<num_edge;++i){
                output.at(i) = data[edge[i]];
            }
            return output;
        }


        static MemoryBlock2D Collect(const MemoryBlock2D &data, const int64_t *edge, size_t num_edge) {
            MemoryBlock2D output(data.type(), {num_edge,data.mDims.y});
            for(size_t i=0;i<num_edge;++i) {
                for(size_t j=0;j<data.mDims.y;++j)
                    output.at<float>(i,j) = data.at<float>(edge[i],j);
            }
            return output;
        }

        template<typename T>
        static std::vector<T> IndexAggr(const T *data, const int64_t *edge, size_t stride, size_t num_edge, size_t num, int type=0){
            // type =0: add, 1: max, 2: mean
            std::vector<T> output(num * stride);
            memset(output.data(),0,sizeof(T)*output.size());
            std::map<int,size_t> counter;
            for(size_t i=0;i<num_edge;++i) {
                auto idx = edge[i];
                if(counter.find(idx) == counter.end()) counter[idx] = 0;
                for(size_t j=0;j<stride;++j) {
                    if (counter[idx] == 0) {
                        output.at(idx*stride+j) = data[i*stride+j];
                    }
                    else if(type ==1)
                        output.at(idx*stride+j) = std::max(output.at(idx*stride+j),data[i*stride+j]);
                    else
                        output.at(idx*stride+j) += data[i*stride+j];
                }
                counter[idx]++;
            }
            if (type == 2) {
                for(auto &pair:counter) {
                    for(size_t j=0;j<stride;++j) {
                        output[pair.first*stride+j] /= pair.second;
                    }
                }
            }
            return output;
        }

        template<typename T>
        static std::vector<T> Concat(const T* data1, const T* data2, size_t num,
                size_t count1, size_t count2,
                size_t stride1, size_t stride2,
                 size_t offset1=0, size_t offset2=0){
            assert(offset1<stride1);
            assert(count1<=stride1);
            assert(offset2<stride2);
            assert(count2<=stride2);
            assert(offset1+count1<=stride1);
            assert(offset2+count2<=stride2);
            std::vector<T> output ( num * (count1+count2));
            for(size_t i=0;i<num;++i) {
                memcpy(output.data()+i*(count1+count2),data1+i*(stride1)+offset1, sizeof(T)*count1);
                memcpy(output.data()+i*(count1+count2)+count1,data2+i*(stride2)+offset2, sizeof(T)*count2);
            }
            return output;
        }
        template<typename T>
        static void Relu (T* data, size_t size){
            for(size_t i=0;i<size;++i){
                data[i] = std::max(0.f, data[i]);
            }
        }

        static std::vector<std::vector<size_t>> GetMultiPrediction(const float *prob, size_t num_cls, size_t num, float threshold=0.5){
            std::vector<std::vector<size_t>> output(num);
            for(size_t i=0;i<num;++i) {
                std::vector<size_t> found;
                for(size_t j=0;j<num_cls;++j) {
                    if(prob[i*num_cls+j] < threshold) continue;
                    found.push_back(j);
                }
                output.at(i) = found;
            }
            return output;
        }

        static MemoryBlock2D compute_edge_descriptor(
                const MemoryBlock2D &data, const MemoryBlock2D &edge_index,
                bool source_to_target){
            size_t n_edge = edge_index.mDims.y;
            size_t d_edge = data.mDims.y;
            MemoryBlock2D output(data.type(), {n_edge, d_edge});
            for(size_t i=0;i<n_edge;++i){

                auto idx_i = source_to_target? edge_index.at<int64_t>(1, i):edge_index.at<int64_t>(0, i);
                auto idx_j = source_to_target? edge_index.at<int64_t>(0, i):edge_index.at<int64_t>(1, i);
                const auto &data_i = data.Row<float>(idx_i);
                const auto &data_j = data.Row<float>(idx_j);
                for(size_t j=0;j<3;++j) {
                    // centroid offset
                    output.at<float>(i,j) = data_i[j] - data_j[j];
                    // std offset
                    output.at<float>(i,j+3) = data_i[j+3] - data_j[j+3];
                    // dim log ratio
                    output.at<float>(i,j+6) = std::log( data_i[j+6] / data_j[j+6] );
                }
                // volume log ratio
                output.at<float>(i,9) = std::log (data_i[9] / data_j[9]);
                // length log ratio
                output.at<float>(i,10) = std::log (data_i[10] / data_j[10]);
            }
            return output;
        }
    };

}

#endif //GRAPHSLAM_UTILDATAPROCESSING_H
