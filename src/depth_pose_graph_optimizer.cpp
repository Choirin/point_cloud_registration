#include "depth_pose_graph_optimizer.hpp"
#include "depth_pose_graph_error_term.hpp"
#include "relative_pose_error_term.hpp"
#include "depth_frame.hpp"
#include "covisibility_edge.hpp"
#include "point_cloud_viewer.hpp"

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
              2.0,
              "distance threshold to select neighbor frames [m]");
DEFINE_double(neighbor_frame_angle_threshold_deg,
              360,
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

    auto distance = edge->relative_distance();
    auto angle = edge->relative_angle();

    if (distance < FLAGS_neighbor_frame_distance_threshold &&
        angle < DEG2RAD(FLAGS_neighbor_frame_angle_threshold_deg))
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

void add_closest_point_residuals(ceres::Problem &problem, const std::vector<std::shared_ptr<DepthFrame>> &frames,
                                 ceres::LocalParameterization *quaternion_local_parameterization)
{
  ceres::LossFunction *loss_function = FLAGS_robustify ? new ceres::HuberLoss(1.0) : NULL;

  std::shared_ptr<PointCloudViewer> viewer = std::make_shared<PointCloudViewer>();

  for (auto frame : frames)
  {
    Eigen::Matrix4d frame_pose;
    frame->get_pose(frame_pose);
    auto neighbors = find_neighbor_frames(frame, frames);
    auto points = frame->point_cloud()->points;
    auto normals = frame->normals()->points;

    size_t number_of_edges = 0;
    for (auto neighbor : neighbors)
    {
      viewer->append_frame(frame);
      viewer->append_frame(neighbor);
      Eigen::Matrix4d neighbor_pose;
      neighbor->get_pose(neighbor_pose);
      Eigen::Matrix3d R_n_g = neighbor_pose.block<3, 3>(0, 0).transpose();
      Eigen::Vector3d t_g_n = neighbor_pose.block<3, 1>(0, 3);
      Eigen::Matrix3d R_n_f = R_n_g * frame_pose.block<3, 3>(0, 0);
      for (size_t i = 0; i < points.size(); ++i)
      {
        auto point = points[i];
        auto normal = normals[i];
        // transform a point to the neighbor's coordinate
        Eigen::Vector3d point_g = (frame_pose * point.getVector4fMap().cast<double>()).head<3>();
        Eigen::Vector3d point_n = R_n_g * (point_g - t_g_n);
        pcl::PointXYZ pt(point_n.x(), point_n.y(), point_n.z());
        Eigen::Vector3d normal_vec_n = R_n_f * normal.getNormalVector3fMap().cast<double>();
        pcl::Normal normal_n(normal_vec_n.x(), normal_vec_n.y(), normal_vec_n.z());

        pcl::PointXYZ closest_point;
        pcl::Normal closest_point_normal;
        // TODO: transform normal to neighbor's coords
        if (neighbor->find_closest_point(pt, normal_n, closest_point, closest_point_normal))
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

          number_of_edges++;

          auto point_b_g = (neighbor_pose * closest_point.getVector4fMap().cast<double>()).head<3>();
          viewer->append_line(point_g, point_b_g);
        }
      }
      // viewer->clear_lines();
      // viewer->spin();
      viewer->clear_frames();
      viewer->clear_lines();
    }
    std::cout << "frames: " << neighbors.size() << ", "
              << "points: " << number_of_edges << std::endl;
  }
}

void add_relative_pose_residuals(ceres::Problem &problem, const std::vector<std::shared_ptr<DepthFrame>> &frames,
                                 ceres::LocalParameterization *quaternion_local_parameterization)
{
  ceres::LossFunction *loss_function = NULL;

  // Add relative pose edges
  int pose_edge_count = 0;
  for (size_t i = 0; i < frames.size() - 1; ++i)
  {
    const auto &frame_a = frames[i];
    const auto &frame_b = frames[i + 1];
    if (abs(frame_a->timestamp() - frame_b->timestamp()) < 20.0)
    {
      ceres::CostFunction *cost_function =
          RelativePoseErrorTerm::Create(*frame_a->translation(), *frame_a->rotation(),
                                        *frame_b->translation(), *frame_b->rotation());
      problem.AddResidualBlock(cost_function,
                               loss_function,
                               frame_a->mutable_translation(),
                               frame_a->mutable_rotation(),
                               frame_b->mutable_translation(),
                               frame_b->mutable_rotation());

      problem.SetParameterization(frame_a->mutable_rotation(),
                                  quaternion_local_parameterization);
      problem.SetParameterization(frame_b->mutable_rotation(),
                                  quaternion_local_parameterization);

      pose_edge_count++;
    }
  }

  std::cout << "relative pose edges: " << pose_edge_count << std::endl;
}


void optimize_pose_graph(std::vector<std::shared_ptr<DepthFrame>> &frames)
{
  ceres::Problem problem;
  ceres::LocalParameterization *quaternion_local_parameterization =
      new ceres::EigenQuaternionParameterization;

  add_closest_point_residuals(problem, frames, quaternion_local_parameterization);

  add_relative_pose_residuals(problem, frames, quaternion_local_parameterization);

  ceres::Solver::Options options;
  // options.sparse_linear_algebra_library_type = ceres::EIGEN_SPARSE;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  options.minimizer_progress_to_stdout = true;
  options.max_num_iterations = 20;
  options.num_threads = 4;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.FullReport() << "\n";
}
