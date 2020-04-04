#pragma once
#include "depth_pose_graph_error_term.hpp"
#include "depth_frame.hpp"
#include "covisibility_edge.hpp"
#include "viewer.hpp"

#include <Eigen/Dense>
#include <ceres/ceres.h>
#include <gflags/gflags.h>

#include <cmath>
#include <cstdio>
#include <iostream>
#include <string>

DEFINE_bool(robustify,
            false,
            "whether to use HuberLoss for loss function");
DEFINE_double(neighbor_frame_distance_threshold,
              1.0,
              "distance threshold to select neighbor frames [m]");
DEFINE_double(neighbor_frame_angle_threshold_deg,
              30,
              "distance threshold to select neighbor frames [deg]");
DEFINE_uint32(maximum_neighbor_frames,
              6,
              "maximum number of neighbor frames");

std::vector<std::shared_ptr<DepthFrame>> find_neighbor_frames(
    const std::shared_ptr<DepthFrame> &reference, const std::vector<std::shared_ptr<DepthFrame>> &target_frames)
{
  std::vector<std::shared_ptr<CovisibilityEdge>> edges;

  for (size_t i = 0; i < target_frames.size(); ++i)
  {
    if (reference.get() == target_frames[i].get())
      continue;
    std::shared_ptr<CovisibilityEdge> edge(new CovisibilityEdge(reference, target_frames[i]));

    if (edge->relative_distance() < FLAGS_neighbor_frame_distance_threshold &&
        edge->relative_angle() < DEG2RAD(FLAGS_neighbor_frame_angle_threshold_deg))
    {
      edges.emplace_back(edge);
    }
  }

  // sort edges by distance order
  std::sort(edges.begin(), edges.end(),
            [&](const std::shared_ptr<CovisibilityEdge> a, const std::shared_ptr<CovisibilityEdge> b) {
              return a->relative_distance() < b->relative_distance();
            });

  // TODO: return edges
  std::vector<std::shared_ptr<DepthFrame>> neighbor_frames;
  for (size_t i = 0; i < std::min(edges.size(), (size_t)FLAGS_maximum_neighbor_frames); ++i)
  {
    neighbor_frames.emplace_back(edges[i]->target());
  }

  return neighbor_frames;
}

void optimize_pose_graph(std::vector<std::shared_ptr<DepthFrame>> &frames)
{
  ceres::Problem problem;

  ceres::LossFunction *loss_function = FLAGS_robustify ? new ceres::HuberLoss(1.0) : NULL;
  ceres::LocalParameterization *quaternion_local_parameterization =
      new ceres::EigenQuaternionParameterization;

  std::shared_ptr<PointCloudViewer> viewer = std::make_shared<PointCloudViewer>();

  for (auto frame : frames)
  {
    viewer->append_frame(frame);
    auto neighbors = find_neighbor_frames(frame, frames);
    auto points = frame->point_cloud()->points;
    auto normals = frame->normals()->points;
    std::cout << "frames: " << neighbors.size() << ", "
              << "points: " << points.size() << std::endl;
    for (auto neighbor : neighbors)
    {
      // viewer->append_frame(frame);
      // viewer->append_frame(neighbor);
      Eigen::Matrix3d R_n_g = neighbor->pose().block<3, 3>(0, 0).transpose();
      Eigen::Vector3d t_g_n = neighbor->pose().block<3, 1>(0, 3);
      for (size_t i = 0; i < points.size(); ++i)
      {
        auto point = points[i];
        auto normal = normals[i];
        // transform a point to the neighbor's coordinate
        Eigen::Vector3d point_g = (frame->pose() * point.getVector4fMap().cast<double>()).head<3>();
        Eigen::Vector3d point_n = R_n_g * (point_g - t_g_n);
        pcl::PointXYZ pt(point_n.x(), point_n.y(), point_n.z());

        pcl::PointXYZ closest_point;
        pcl::Normal closest_point_normal;
        if (neighbor->find_closest_point(pt, closest_point, closest_point_normal))
        {
          const Eigen::Matrix<double, 3, 1> point_a = point.getVector3fMap().cast<double>();
          const Eigen::Matrix<double, 3, 1> point_b = closest_point.getVector3fMap().cast<double>();
          // ceres::CostFunction *cost_function = DepthPoseGraphErrorTerm::Create(point_a, point_b);
          const Eigen::Matrix<double, 3, 1> normal_a = normal.getNormalVector3fMap().cast<double>();
          const Eigen::Matrix<double, 3, 1> normal_b = closest_point_normal.getNormalVector3fMap().cast<double>();
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

          auto point_b_g = (neighbor->pose() * closest_point.getVector4fMap().cast<double>()).head<3>();
          viewer->append_line(point_g, point_b_g);
        }
      }
      // viewer->clear_lines();
      // viewer->spin();
      // viewer->clear_frames();
      // viewer->clear_lines();
    }
  }

  // viewer->set_frames(frames);
  // viewer->spin();

  ceres::Solver::Options options;
  // options.sparse_linear_algebra_library_type = ceres::EIGEN_SPARSE;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  options.minimizer_progress_to_stdout = true;
  options.max_num_iterations = 20;
  options.num_threads = 4;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.FullReport() << "\n";

  viewer->clear_lines();
  viewer->spin();
}
