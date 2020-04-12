#include "depth_frame.hpp"
#include "depth_to_point_cloud.hpp"
#include "point_cloud_viewer.hpp"

#include <gflags/gflags.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

DEFINE_string(input,
              "/Users/kohei/data/tum/rgbd_dataset_freiburg2_pioneer_slam2/depth/1311877821.906634.png",
              "png file path");

int main(int argc, char *argv[])
{
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  std::shared_ptr<DepthToPointCloud> depth_to_cloud =
      std::make_shared<DepthToPointCloud>(640, 480, 525.0, 525.0, 319.5, 239.5, 5000.0);
  std::vector<std::shared_ptr<DepthFrame>> frames;

  // load point cloud from image
  cv::Mat image = cv::imread(FLAGS_input, cv::IMREAD_ANYDEPTH);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = depth_to_cloud->convert(image);
  std::shared_ptr<DepthFrame> frame = std::make_shared<DepthFrame>(cloud, Eigen::Matrix4d::Identity());
  frame->filter();
  frame->compute_normal_using_unfiltered(10);
  frames.emplace_back(frame);

  std::shared_ptr<PointCloudViewer> viewer = std::make_shared<PointCloudViewer>(frames);
  viewer->spin();

  return 0;
}
