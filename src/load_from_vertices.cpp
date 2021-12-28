#include "depth_frame.hpp"
#include "icp_registration.hpp"
#include "depth_pose_graph_optimizer.hpp"
#include "depth_to_point_cloud.hpp"
#include "feature_map_vertices.hpp"
#include "point_cloud_viewer.hpp"

#include <gflags/gflags.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

#include <algorithm>
#include <random>
#include <experimental/filesystem>

// #define LOAD_ONLY_FIRST_FRAME 20

namespace fs = std::experimental::filesystem;

DEFINE_string(path_to_pngs,
              "/Users/kohei/Downloads/dataset_20200708/depth_image_",
              "png stored directory path");
DEFINE_string(path_to_vertices,
              "/Users/kohei/Downloads/dataset_20200708/maps/20200706_134124_492b677385c014401c460954eb005b29/feature_map/vertices",
              "png stored directory path");
DEFINE_double(timestamp_diff,
              0.05,
              "timestamp differences tolerance in seconds");

size_t get_sorted_files(const fs::path &directory, std::vector<fs::path> &file_paths)
{
  for (const auto &entry : fs::directory_iterator(directory))
  {
    if (entry.path().extension() != ".png")
      continue;
    file_paths.push_back(entry);
    std::cout << entry.path() << std::endl;
#ifdef LOAD_ONLY_FIRST_FRAME
    // // For debug purpose, load only a frame
    for (int i = 0; i < LOAD_ONLY_FIRST_FRAME; ++i)
      file_paths.push_back(entry.path());
    break;
#endif
  }
  std::sort(file_paths.begin(), file_paths.end(),
            [&](const fs::path &a, const fs::path &b) {
              return std::stod(a.stem()) < std::stod(b.stem());
            });
  return file_paths.size();
}

size_t get_vertices_files(const fs::path &directory, std::vector<fs::path> &file_paths)
{
  for (const auto &entry : fs::directory_iterator(directory))
  {
    auto file_path = entry.path() / "vertices.csv";
    if (!fs::exists(file_path))
      continue;
    file_paths.push_back(file_path);
  }
  return file_paths.size();
}

void get_keyframe_to_depth_transformation(Eigen::Matrix4d &transform)
{
  transform = Eigen::Matrix4d::Identity();
  Eigen::AngleAxisd rollAngle(DEG2RAD(0), Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitchAngle(DEG2RAD(120), Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yawAngle(DEG2RAD(-90), Eigen::Vector3d::UnitZ());
  Eigen::Quaterniond q = rollAngle * pitchAngle * yawAngle;
  transform.block<3, 3>(0, 0) = q.toRotationMatrix();
  transform.block<3, 1>(0, 3) = Eigen::Vector3d(0.0, 0.0, 0.2);
}

void add_noises(const std::vector<std::shared_ptr<DepthFrame>> &frames, double mu = 0.0, double sigma = 0.2)
{
  std::mt19937 rand_src(12345);
  for (auto frame : frames)
  {
    auto translation = frame->translation();
    auto rotation = frame->rotation();
    std::normal_distribution<double> rand_trans_dist(mu, sigma);
    Eigen::Vector3d t_dist(rand_trans_dist(rand_src), rand_trans_dist(rand_src), rand_trans_dist(rand_src));
    *translation += t_dist;

    std::normal_distribution<double> rand_angle_dist(mu, DEG2RAD(5));
    Eigen::Quaterniond q_dist;
    q_dist = Eigen::AngleAxisd(rand_angle_dist(rand_src), Eigen::Vector3d::UnitX()) *
             Eigen::AngleAxisd(rand_angle_dist(rand_src), Eigen::Vector3d::UnitY()) *
             Eigen::AngleAxisd(rand_angle_dist(rand_src), Eigen::Vector3d::UnitZ());
    *rotation *= q_dist;
  }
}

int main(int argc, char *argv[])
{
  std::shared_ptr<PointCloudViewer> viewer = std::make_shared<PointCloudViewer>();
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  std::shared_ptr<DepthToPointCloud> depth_to_cloud =
      std::make_shared<DepthToPointCloud>(224, 172,
                                          195.26491142934, 195.484689318979,
                                          111.31867165296, 86.8194913656314, 1000.0);
  std::vector<fs::path> vertices_file_paths;
  get_vertices_files(FLAGS_path_to_vertices, vertices_file_paths);
  auto feature_key_frames = FeatureMapVertices(vertices_file_paths);
  auto vertices = feature_key_frames.vertices();

  std::vector<std::shared_ptr<DepthFrame>> frames;

  std::vector<fs::path> file_paths;
  get_sorted_files(FLAGS_path_to_pngs, file_paths);
  double last_timestamp = 0;
  for (auto file_path : file_paths)
  {
    auto timestamp = std::stod(file_path.stem()) / 1.e9;
#ifndef LOAD_ONLY_FIRST_FRAME
    if (timestamp - last_timestamp < 1.0)
      continue;
#endif
    last_timestamp = timestamp;
    std::cout << file_path.stem() << std::endl;
    auto closest_vertex =
        *std::min_element(vertices.begin(), vertices.end(),
                          [&](auto const &a, auto const &b) {
                            return abs(a->timestamp - timestamp) < abs(b->timestamp - timestamp);
                          });
    if (abs(closest_vertex->timestamp - timestamp) < FLAGS_timestamp_diff)
    {
      // load point cloud from image
      cv::Mat image = cv::imread(file_path.string(), cv::IMREAD_ANYDEPTH);
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = depth_to_cloud->convert(image);
      // pose from ground truth
      Eigen::Matrix4d tf_kf_depth;
      get_keyframe_to_depth_transformation(tf_kf_depth);
      Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
      Eigen::Matrix3d R = closest_vertex->rotation.toRotationMatrix();
      pose.block<3, 3>(0, 0) = R * tf_kf_depth.block<3, 3>(0, 0);
      pose.block<3, 1>(0, 3) = pose.block<3, 3>(0, 0) * tf_kf_depth.block<3, 1>(0, 3) +
          closest_vertex->translation * 0.8565155709424058;

      std::shared_ptr<DepthFrame> frame =
          std::make_shared<DepthFrame>(closest_vertex->timestamp, cloud, pose);
      frame->filter();
      frame->compute_normal_using_unfiltered(10);
      if (frame->point_cloud()->points.size() < 50)
      {
        std::cout << frame->point_cloud()->points.size() << std::endl;
        continue;
      }
      frames.emplace_back(frame);
      viewer->append_frame(frame);
      // viewer->spin();
    }
  }

  std::cout << "frames count: " << frames.size() << std::endl;
  // add_noises(frames);

  viewer->spin();
  optimize_pose_graph(frames);
  viewer->spin();
  optimize_pose_graph(frames);
  optimize_pose_graph(frames);
  optimize_pose_graph(frames);
  optimize_pose_graph(frames);
  optimize_pose_graph(frames);
  optimize_pose_graph(frames);
  optimize_pose_graph(frames);
  viewer->spin();

  return 0;
}
