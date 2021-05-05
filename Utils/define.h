#pragma once
#ifndef EIGEN_STD_VEC
#define EIGEN_STD_VEC

#include <Eigen/Core>
#define VecEigenf(x) std::vector<Eigen::Matrix<float,x,1>,Eigen::aligned_allocator<Eigen::Matrix<float,x,1>>>
#define VecEigeni(x) std::vector<Eigen::Matrix<int,x,1>,Eigen::aligned_allocator<Eigen::Matrix<int,x,1>>>
#define VecEigenuchar(x) std::vector<Eigen::Matrix<unsigned char,x,1>,Eigen::aligned_allocator<Eigen::Matrix<unsigned char,x,1>>>

#endif
