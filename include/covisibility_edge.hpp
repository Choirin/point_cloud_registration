#pragma once
#include "depth_frame.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <gflags/gflags.h>

#include <cmath>

class CovisibilityEdge
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  CovisibilityEdge(const std::shared_ptr<DepthFrame> &reference, const std::shared_ptr<DepthFrame> &target)
      : reference_(reference), target_(target)
  {
  }
  ~CovisibilityEdge() {}

  std::shared_ptr<DepthFrame> reference() { return reference_; }
  std::shared_ptr<DepthFrame> target() { return target_; }

  double relative_distance(void)
  {
    auto translation = *target_->translation() - *reference_->translation();
    return translation.norm();
  }

  double relative_angle(void)
  {
    Eigen::Matrix4d reference_pose;
    Eigen::Matrix4d target_pose;
    reference_->get_pose(reference_pose);
    target_->get_pose(target_pose);
    return acos(((reference_pose.block<3, 3>(0, 0) * target_pose.block<3, 3>(0, 0).transpose()).trace() - 1) / 2);
  }

private:
  std::shared_ptr<DepthFrame> reference_;
  std::shared_ptr<DepthFrame> target_;
};
