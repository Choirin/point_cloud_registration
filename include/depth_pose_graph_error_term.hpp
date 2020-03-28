#pragma once
#include <cmath>
#include <cstdio>
#include <iostream>
#include <string>

#include <Eigen/Dense>

#include <ceres/ceres.h>
#include <ceres/rotation.h>

struct DepthPoseGraphErrorTerm
{
  DepthPoseGraphErrorTerm(Eigen::Matrix<double, 3, 1> point_a, Eigen::Matrix<double, 3, 1> point_b)
      : point_a(point_a), point_b(point_b) {}

  template <typename T>
  bool operator()(const T *const t_a_ptr, const T *const q_a_ptr,
                  const T *const t_b_ptr, const T *const q_b_ptr,
                  T *residuals_ptr) const
  {
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> t_a(t_a_ptr);
    Eigen::Map<const Eigen::Quaternion<T>> q_a(q_a_ptr);

    Eigen::Map<const Eigen::Matrix<T, 3, 1>> t_b(t_b_ptr);
    Eigen::Map<const Eigen::Quaternion<T>> q_b(q_b_ptr);

    // Transform points to global coordinate
    Eigen::Matrix<T, 3, 1> t_a_g = q_a * (point_a.cast<T>()) + t_a;
    Eigen::Matrix<T, 3, 1> t_b_g = q_b * (point_b.cast<T>()) + t_b;

    // The error is the difference between the predicted and observed position.
    residuals_ptr[0] = t_a_g[0] - t_b_g[0];
    residuals_ptr[1] = t_a_g[1] - t_b_g[1];
    residuals_ptr[2] = t_a_g[2] - t_b_g[2];

    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction *Create(const Eigen::Matrix<double, 3, 1> point_a,
                                     const Eigen::Matrix<double, 3, 1> point_b)
  {
    return (new ceres::AutoDiffCostFunction<DepthPoseGraphErrorTerm, 3, 3, 4, 3, 4>(
        new DepthPoseGraphErrorTerm(point_a, point_b)));
  }
  Eigen::Matrix<double, 3, 1> point_a;
  Eigen::Matrix<double, 3, 1> point_b;
};
