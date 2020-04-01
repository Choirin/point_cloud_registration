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
    Eigen::Matrix<T, 3, 1> t_a_g = q_a * (point_a.template cast<T>()) + t_a;
    Eigen::Matrix<T, 3, 1> t_b_g = q_b * (point_b.template cast<T>()) + t_b;

    // The error is the difference between the predicted and observed position.
    Eigen::Matrix<T, 3, 1> t_diff = t_a_g - t_b_g;
    residuals_ptr[0] = t_diff[0];
    residuals_ptr[1] = t_diff[1];
    residuals_ptr[2] = t_diff[2];

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

struct DepthPoseGraphNormalErrorTerm
{
  DepthPoseGraphNormalErrorTerm(Eigen::Matrix<double, 3, 1> point_a, Eigen::Matrix<double, 3, 1> point_b,
                                Eigen::Matrix<double, 3, 1> normal_a, Eigen::Matrix<double, 3, 1> normal_b)
      : point_a(point_a), point_b(point_b), normal_a(normal_a), normal_b(normal_b) {}

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
    Eigen::Matrix<T, 3, 1> t_a_g = q_a * (point_a.template cast<T>()) + t_a;
    Eigen::Matrix<T, 3, 1> t_b_g = q_b * (point_b.template cast<T>()) + t_b;

    // The error is the difference between the predicted and observed position.
    Eigen::Matrix<T, 3, 1> t_diff = t_a_g - t_b_g;

    T residual = t_diff.transpose() * (q_b * (normal_b.template cast<T>()));
    residuals_ptr[0] = residual;

    // angle betwee normals
    // Eigen::Matrix<T, 3, 1> n_a_g = q_a * (normal_a.template cast<T>());
    // Eigen::Matrix<T, 3, 1> n_b_g = q_b * (normal_b.template cast<T>());
    // T angle = acos(n_a_g.dot(n_b_g) / (n_a_g.norm() * n_b_g.norm()));

    // residuals_ptr[0] = t_diff[0];
    // residuals_ptr[1] = t_diff[1];
    // residuals_ptr[2] = t_diff[2];
    // residuals_ptr[3] = angle;

    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction *Create(const Eigen::Matrix<double, 3, 1> point_a,
                                     const Eigen::Matrix<double, 3, 1> point_b,
                                     const Eigen::Matrix<double, 3, 1> normal_a,
                                     const Eigen::Matrix<double, 3, 1> normal_b)
  {
    return (new ceres::AutoDiffCostFunction<DepthPoseGraphNormalErrorTerm, 1, 3, 4, 3, 4>(
        new DepthPoseGraphNormalErrorTerm(point_a, point_b, normal_a, normal_b)));
  }
  Eigen::Matrix<double, 3, 1> point_a;
  Eigen::Matrix<double, 3, 1> point_b;
  Eigen::Matrix<double, 3, 1> normal_a;
  Eigen::Matrix<double, 3, 1> normal_b;
};
