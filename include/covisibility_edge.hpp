#pragma once
#include "depth_frame.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <gflags/gflags.h>

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
    auto translation = target_->pose().block<3, 1>(0, 3) - reference_->pose().block<3, 1>(0, 3);
    return translation.norm();
  }

  double relative_angle(void)
  {
    auto reference_rotation = reference_->pose().block<3, 3>(0, 0);
    auto target_rotation = target_->pose().block<3, 3>(0, 0);
    return acos(((reference_rotation * target_rotation.transpose()).trace() - 1) / 2);
  }

private:
  std::shared_ptr<DepthFrame> reference_;
  std::shared_ptr<DepthFrame> target_;
};
