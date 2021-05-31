//
// Created by sc on 10/9/20.
//

#ifndef GRAPHSLAM_UTILMATH_H
#define GRAPHSLAM_UTILMATH_H
#include <ORUtils/Vector.h>
#include "MemoryBlock.h"

namespace Math {
    static ORUtils::Vector3<float> sum(const ORUtils::Vector3<float> *data, size_t size) {
        ORUtils::Vector3<float> sum(0.f);
        for (size_t i = 0; i < size; ++i)
            sum += data[i];
        return sum;
    }

    static MemoryBlock sum(const MemoryBlock &data, size_t size, size_t stride, size_t dim) {
        MemoryBlock sum(data.type(), dim);
        sum.Clear();
        for (size_t i = 0; i < size; ++i) {
            for (size_t j = 0; j < dim; ++j) {
                sum.at<float>(j) += data.Get<float>()[i * stride + j];
            }
        }
        return sum;
    }

    static ORUtils::Vector3<float> mean(const ORUtils::Vector3<float> *data, size_t size) {
        return sum(data, size) / size;
    }

    static MemoryBlock mean(const MemoryBlock &data, size_t size, size_t stride, size_t dim) {
        auto result = sum(data, size, stride, dim);
        for (size_t i = 0; i < dim; ++i) result.at<float>(i) /= size;
        return result;
    }

    static MemoryBlock mean(const MemoryBlock2D &data, size_t dim = 3) {
        auto result = sum(data, data.mDims.x, data.mDims.y, dim);
        for (size_t i = 0; i < dim; ++i) result.at<float>(i) /= data.mDims.x;
        return result;
    }

    static ORUtils::Vector3<float> stddev(const ORUtils::Vector3<float> *data, size_t size) {
        auto m = mean(data, size);
        ORUtils::Vector3<float> std(0.f);
        for (size_t i = 0; i < size; ++i)
            for (size_t j = 0; j < 3; ++j)
                std[j] += pow(data[i][j] - m[j], 2);
        for (size_t j = 0; j < 3; ++j)
            std[j] = sqrt(std[j] / size);
        return std;
    }

    static MemoryBlock stddev(const MemoryBlock &data, size_t size, size_t stride, size_t dim) {
        auto m = mean(data, size, stride, dim);
        MemoryBlock result(data.type(), dim);
        result.Clear();
        for (size_t i = 0; i < size; ++i)
            for (size_t j = 0; j < dim; ++j)
                result.at<float>(j) += pow(data.at<float>(i * stride + j) - m.at<float>(j), 2);
        for (size_t j = 0; j < dim; ++j)
            result.at<float>(j) = sqrt(result.at<float>(j) / size);
        return result;
    }

    static void normalize(ORUtils::Vector3<float> *data, size_t size) {
        auto centroid = mean(data, size);
        float max_dist = 0;
        for (size_t i = 0; i < size; ++i) {
            data[i] -= centroid;
            float dist = 0;
            for (size_t j = 0; j < 3; ++j)
                dist += pow(data[i][j], 2);
            dist = sqrt(dist);
            if (dist > max_dist)max_dist = dist;
        }
        for (size_t i = 0; i < size; ++i) data[i] /= max_dist;
    }

    static void normalize(MemoryBlock2D &data, size_t dim) {
        auto centroid = mean(data, dim);
        float max_dist = 0;
        for (size_t i = 0; i < data.mDims.x; ++i) {
            for (size_t j = 0; j < dim; ++j) {
                data.at<float>(i, j) -= centroid.at<float>(j);
            }
            float dist = 0;
            for (size_t j = 0; j < dim; ++j)
                dist += pow(data.at<float>(i, j), 2);
            if (dist > max_dist)max_dist = dist;
        }
        max_dist = sqrt(max_dist);
        for (size_t i = 0; i < data.mDims.x; ++i)
            for (size_t j = 0; j < dim; ++j) data.at<float>(i, j) /= max_dist;
    }
}

#endif //GRAPHSLAM_UTILMATH_H
