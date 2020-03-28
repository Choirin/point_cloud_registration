#pragma once
#include "depth_pose_graph_error_term.hpp"
#include "depth_frame.hpp"

#include <ceres/ceres.h>

#include <Eigen/Dense>

#include <cmath>
#include <cstdio>
#include <iostream>
#include <string>

void optimize_pose_graph(std::vector<std::shared_ptr<DepthFrame>>& frames)
{
  ceres::Problem problem;

  ceres::LossFunction* loss_function = NULL;
  ceres::LocalParameterization* quaternion_local_parameterization =
      new ceres::EigenQuaternionParameterization;

  for (auto frame : frames)
  {
    auto neighbors = frame->find_neighbor_frames(frames);
    auto points = frame->point_cloud()->points;
    std::cout << neighbors.size() << ", " << points.size() << std::endl;
    for (auto neighbor : neighbors)
    {
      for (auto point : points)
      {
        // transform point
        Eigen::Vector3d p(point.x, point.y, point.z);
        p = neighbor->pose().block<3, 3>(0, 0).transpose() * (frame->pose().block<3, 3>(0, 0) * p + frame->pose().block<3, 1>(0, 3) - neighbor->pose().block<3, 1>(0, 3));
        pcl::PointXYZ pt;
        pt.x = p.data()[0];
        pt.y = p.data()[1];
        pt.z = p.data()[2];

        pcl::PointXYZ closest_point;
        if (neighbor->find_closest_point(pt, closest_point))
        {
          const Eigen::Matrix<double, 3, 1> point_a = point.getVector3fMap().cast<double>();
          const Eigen::Matrix<double, 3, 1> point_b = closest_point.getVector3fMap().cast<double>();
          ceres::CostFunction *cost_function = DepthPoseGraphErrorTerm::Create(point_a, point_b);
          problem.AddResidualBlock(cost_function,
                                   loss_function,
                                   frame->mutable_translation(),
                                   frame->mutable_rotation(),
                                   neighbor->mutable_translation(),
                                   neighbor->mutable_rotation());

          problem.SetParameterization(frame->mutable_rotation(),
                                      quaternion_local_parameterization);
          problem.SetParameterization(neighbor->mutable_rotation(),
                                      quaternion_local_parameterization);
        }
      }
    }
  }

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  options.minimizer_progress_to_stdout = true;
  options.max_num_iterations = 20;
  options.num_threads = 4;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.FullReport() << "\n";
}
