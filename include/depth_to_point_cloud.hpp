#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>

class DepthToPointCloud
{
public:
  DepthToPointCloud(int width, int height, double fx, double fy, double cx, double cy, double factor);
  ~DepthToPointCloud() {}

  pcl::PointCloud<pcl::PointXYZ>::Ptr convert(const cv::Mat &depth_image);

private:
  int width_;
  int height_;
  double fx_;
  double fy_;
  double cx_;
  double cy_;
  double factor_;
  std::vector<double> coeff_;

  void initialize_matrix();
};
