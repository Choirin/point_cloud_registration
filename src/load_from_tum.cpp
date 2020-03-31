#include "depth_frame.hpp"
#include "icp_registration.hpp"
#include "depth_pose_graph_optimizer.hpp"
#include "viewer.hpp"
#include "depth_to_point_cloud.hpp"
#include "tum_dataset.hpp"

#include <gflags/gflags.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

#include <algorithm>
#include <random>
#include <experimental/filesystem>

namespace fs = std::experimental::filesystem;

DEFINE_string(path_to_dataset,
              "/Users/kohei/data/tum/rgbd_dataset_freiburg2_pioneer_slam2",
              "png stored directory path");
DEFINE_string(path_to_org_pcd,
              "/Users/kohei/data/original.pcd",
              "output pcd file path");
DEFINE_string(path_to_pcd,
              "/Users/kohei/data/optimized.pcd",
              "output pcd file path");
DEFINE_double(timestamp_diff,
              0.05,
              "timestamp differences tolerance in seconds");

size_t get_sorted_files(const fs::path &directory, std::vector<fs::path> &file_paths)
{
  for (const auto &entry : fs::directory_iterator(directory))
  {
    if (entry.path().extension() != ".png")
      continue;
    file_paths.push_back(entry.path().string());
    // For debug purpose, load only a frame
    // for (int i = 0; i < 20; ++i)
    //   file_paths.push_back(entry.path().string());
    // break;
  }
  std::sort(file_paths.begin(), file_paths.end(),
            [&](const fs::path &a, const fs::path &b) {
              return std::stod(a.stem()) < std::stod(b.stem());
            });
  return file_paths.size();
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
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  std::shared_ptr<DepthToPointCloud> depth_to_cloud =
      std::make_shared<DepthToPointCloud>(640, 480, 525.0, 525.0, 319.5, 239.5, 5000.0);
  std::shared_ptr<PointCloudViewer> viewer = std::make_shared<PointCloudViewer>();

  fs::path dataset_path = FLAGS_path_to_dataset;
  fs::path ground_truth_path = dataset_path / "groundtruth.txt";
  fs::path depth_dir_path = dataset_path / "depth";
  auto ground_truth = tumdataset::GroundTruth(ground_truth_path.string());
  auto vertices = ground_truth.data();

  std::vector<std::shared_ptr<DepthFrame>> frames;

  std::vector<fs::path> file_paths;
  get_sorted_files(depth_dir_path, file_paths);
  double last_timestamp = 0;
  for (auto file_path : file_paths)
  {
    if (file_path.extension() != ".png")
      continue;
    auto timestamp = std::stod(file_path.stem());
    if (timestamp - last_timestamp < 2.0)
      continue;
    last_timestamp = timestamp;
    std::cout << file_path.stem() << std::endl;
    auto closest_vertex =
        *std::min_element(vertices.begin(), vertices.end(),
                          [&](const std::shared_ptr<tumdataset::Vertex> &a, const std::shared_ptr<tumdataset::Vertex> &b) {
                            return abs(a->timestamp - timestamp) < abs(b->timestamp - timestamp);
                          });
    if (abs(closest_vertex->timestamp - timestamp) < FLAGS_timestamp_diff)
    {
      // load point cloud from image
      cv::Mat image = cv::imread(file_path, cv::IMREAD_ANYDEPTH);
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = depth_to_cloud->convert(image);
      // pose from ground truth
      Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
      pose.block<3, 3>(0, 0) = closest_vertex->rotation.toRotationMatrix();
      pose.block<3, 1>(0, 3) = closest_vertex->translation;

      std::shared_ptr<DepthFrame> frame = std::make_shared<DepthFrame>(cloud, pose);
      frame->filter();
      frame->compute_normal();
      frames.emplace_back(frame);
    }
  }

  std::cout << "frames count: " << frames.size() << std::endl;
  add_noises(frames);

  auto icp = std::make_shared<ICPRegistration>(frames);
  pcl::PointCloud<pcl::PointXYZ>::Ptr merged_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  icp->merge(merged_cloud);
  viewer->view(merged_cloud);
  pcl::io::savePCDFileASCII(FLAGS_path_to_org_pcd, *merged_cloud);

  optimize_pose_graph(frames);
  optimize_pose_graph(frames);
  optimize_pose_graph(frames);
  optimize_pose_graph(frames);
  optimize_pose_graph(frames);
  optimize_pose_graph(frames);
  optimize_pose_graph(frames);
  optimize_pose_graph(frames);

  icp->merge(merged_cloud);
  viewer->view(merged_cloud);
  pcl::io::savePCDFileASCII(FLAGS_path_to_pcd, *merged_cloud);

  return 0;
}
