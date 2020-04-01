#pragma once
#include "depth_pose_graph_error_term.hpp"
#include "depth_frame.hpp"

#include <ceres/ceres.h>

#include <Eigen/Dense>

#include <cmath>
#include <cstdio>
#include <iostream>
#include <string>

void optimize_pose_graph(std::vector<std::shared_ptr<DepthFrame>> &frames)
{
  ceres::Problem problem;

  ceres::LossFunction *loss_function = NULL;
  ceres::LocalParameterization *quaternion_local_parameterization =
      new ceres::EigenQuaternionParameterization;

  for (auto frame : frames)
  {
    auto neighbors = frame->find_neighbor_frames(frames);
    auto points = frame->point_cloud()->points;
    auto normals = frame->normals()->points;
    std::cout << "frames: " << neighbors.size() << ", "
              << "points: " << points.size() << std::endl;
    for (auto neighbor : neighbors)
    {
      Eigen::Matrix3d R_n_g = neighbor->pose().block<3, 3>(0, 0).transpose();
      auto t_g_n = neighbor->pose().block<3, 1>(0, 3);
      for (size_t i = 0; i < points.size(); ++i)
      {
        auto point = points[i];
        auto normal = normals[i];
        // transform a point to the neighbor's coordinate
        Eigen::Vector3d point_g = (frame->pose() * point.getVector4fMap().cast<double>()).head<3>();
        Eigen::Vector3d point_n = R_n_g * (point_g - t_g_n);
        pcl::PointXYZ pt(point_n.x(), point_n.y(), point_n.z());

        pcl::PointXYZ closest_point;
        pcl::Normal closes_point_normal;
        if (neighbor->find_closest_point(pt, closest_point, closes_point_normal))
        {
          const Eigen::Matrix<double, 3, 1> point_a = point.getVector3fMap().cast<double>();
          const Eigen::Matrix<double, 3, 1> point_b = closest_point.getVector3fMap().cast<double>();
          // ceres::CostFunction *cost_function = DepthPoseGraphErrorTerm::Create(point_a, point_b);
          const Eigen::Matrix<double, 3, 1> normal_a = normal.getNormalVector3fMap().cast<double>();
          const Eigen::Matrix<double, 3, 1> normal_b = closes_point_normal.getNormalVector3fMap().cast<double>();
          ceres::CostFunction *cost_function = DepthPoseGraphNormalErrorTerm::Create(point_a, point_b, normal_a, normal_b);
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
  // options.sparse_linear_algebra_library_type = ceres::EIGEN_SPARSE;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  options.minimizer_progress_to_stdout = true;
  options.max_num_iterations = 3;
  options.num_threads = 1;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.FullReport() << "\n";
}
