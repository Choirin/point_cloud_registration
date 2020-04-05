#include "depth_frame.hpp"
#include "icp_registration.hpp"
#include "depth_pose_graph_optimizer.hpp"
#include "point_cloud_viewer.hpp"
#include "eth_dataset.hpp"

#include <gflags/gflags.h>
#include <Eigen/Dense>

#include <algorithm>
#include <random>

DEFINE_string(path_to_ground_truth_csv,
              "/Users/kohei/data/pointcloud/pose_scanner_leica.csv",
              "path to vertices csv file");
DEFINE_string(base_path_to_csvs,
              "/Users/kohei/data/pointcloud",
              "csv stored directory path");

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

  std::cout << "load ground truth from " << FLAGS_path_to_ground_truth_csv << std::endl;
  auto vertices = std::make_shared<ethdata::GroundTruth>(FLAGS_path_to_ground_truth_csv);
  auto vetices_data = vertices->data();

  std::vector<std::shared_ptr<DepthFrame>> frames;

  for (auto vertex : vertices->data())
  {
    std::string path_to_point_cloud_csv = FLAGS_base_path_to_csvs + "/Hokuyo_" + std::to_string(vertex->index) + ".csv";
    std::cout << path_to_point_cloud_csv << std::endl;

    auto point_cloud_loader = ethdata::PointCloudCSV(path_to_point_cloud_csv);

    auto frame = std::make_shared<DepthFrame>(point_cloud_loader.cloud(), vertex->pose);
    frame->filter();
    frame->compute_normal();
    frames.emplace_back(frame);
  }

  std::cout << "frames count: " << frames.size() << std::endl;
  add_noises(frames);

  std::shared_ptr<PointCloudViewer> viewer = std::make_shared<PointCloudViewer>(frames);
  viewer->spin();
  optimize_pose_graph(frames);
  optimize_pose_graph(frames);
  optimize_pose_graph(frames);
  optimize_pose_graph(frames);
  optimize_pose_graph(frames);
  optimize_pose_graph(frames);
  viewer->spin();

  return 0;
}
