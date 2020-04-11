#pragma once
#include <cmath>
#include <cstdio>
#include <iostream>
#include <string>

#include <Eigen/Dense>

#include <ceres/ceres.h>
#include <ceres/rotation.h>

struct RelativePoseErrorTerm
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  RelativePoseErrorTerm(const Eigen::Matrix<double, 3, 1> &translation_a,
                        const Eigen::Quaterniond &rotation_a,
                        const Eigen::Matrix<double, 3, 1> &translation_b,
                        const Eigen::Quaterniond &rotation_b)
  {
    relative_translation = translation_b - translation_a;
    relative_rotation = rotation_a.toRotationMatrix() * rotation_b.toRotationMatrix().transpose();
  }

  template <typename T>
  bool operator()(const T *const t_a_ptr, const T *const q_a_ptr,
                  const T *const t_b_ptr, const T *const q_b_ptr,
                  T *residuals_ptr) const
  {
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> t_a(t_a_ptr);
    Eigen::Map<const Eigen::Quaternion<T>> q_a(q_a_ptr);

    Eigen::Map<const Eigen::Matrix<T, 3, 1>> t_b(t_b_ptr);
    Eigen::Map<const Eigen::Quaternion<T>> q_b(q_b_ptr);

    // Translation error
    Eigen::Matrix<T, 3, 1> t_diff = (t_b - t_a) - (relative_translation.template cast<T>());

    // Rotation error
    Eigen::Matrix<T, 3, 3> rot_diff =
        (q_a.toRotationMatrix() * q_b.toRotationMatrix().transpose()) * (relative_rotation.template cast<T>()).transpose();
    Eigen::Matrix<T, 3, 1> angle_axes = rot_diff.eulerAngles(0, 1, 2);

    T coeff = (T)0.01;
    t_diff *= coeff;
    angle_axes *= coeff;

    residuals_ptr[0] = t_diff[0];
    residuals_ptr[1] = t_diff[1];
    residuals_ptr[2] = t_diff[2];
    residuals_ptr[3] = angle_axes[0];
    residuals_ptr[4] = angle_axes[1];
    residuals_ptr[5] = angle_axes[2];

    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction *Create(const Eigen::Matrix<double, 3, 1> &translation_a,
                                     const Eigen::Quaterniond &rotation_a,
                                     const Eigen::Matrix<double, 3, 1> &translation_b,
                                     const Eigen::Quaterniond &rotation_b)
  {
    return (new ceres::AutoDiffCostFunction<RelativePoseErrorTerm, 6, 3, 4, 3, 4>(
        new RelativePoseErrorTerm(translation_a, rotation_a, translation_b, rotation_b)));
  }
  Eigen::Matrix<double, 3, 1> relative_translation;
  Eigen::Matrix<double, 3, 3> relative_rotation;
};
