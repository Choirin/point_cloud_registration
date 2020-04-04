#include "depth_to_point_cloud.hpp"

DepthToPointCloud::DepthToPointCloud(int width, int height, double fx, double fy, double cx, double cy, double factor)
    : width_(width), height_(height), fx_(fx), fy_(fy), cx_(cx), cy_(cy), factor_(factor)
{
  initialize_matrix();
}

pcl::PointCloud<pcl::PointXYZ>::Ptr DepthToPointCloud::convert(const cv::Mat &depth_image)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  point_cloud->width = width_;
  point_cloud->height = height_;
  point_cloud->points.resize(width_ * height_);

  auto pdepth = depth_image.ptr<unsigned short int>(0);
  double *pcoeff = coeff_.data();
  for (int i = 0; i < width_ * height_; ++i, ++pdepth)
  {
    pcl::PointXYZ &point = point_cloud->points[i];
    point.x = (*pdepth) * *(pcoeff++);
    point.y = (*pdepth) * *(pcoeff++);
    point.z = (*pdepth) * *(pcoeff++);
  }
  return point_cloud;
}

void DepthToPointCloud::initialize_matrix()
{
  coeff_.resize(3 * width_ * height_);
  double *pcoeff = coeff_.data();
  for (int v = 0; v < height_; ++v)
  {
    for (int u = 0; u < width_; ++u)
    {
      *(pcoeff++) = (u - cx_) / fx_ / factor_;
      *(pcoeff++) = (v - cy_) / fy_ / factor_;
      *(pcoeff++) = 1.0 / factor_;
    }
  }
}
