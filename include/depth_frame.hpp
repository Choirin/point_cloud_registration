#pragma once
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class DepthFrame : public std::enable_shared_from_this<DepthFrame> 
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  DepthFrame(const pcl::PointCloud<pcl::PointXYZ>::Ptr &point_cloud, const Eigen::Matrix4d &pose);
  ~DepthFrame() {}

  pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud() { return point_cloud_;}
  pcl::PointCloud<pcl::Normal>::Ptr normals() { return normals_; }

  Eigen::Vector3d *translation() { return &translation_; }
  Eigen::Quaterniond *rotation() { return &rotation_; }

  double *mutable_translation() { return translation_.data(); }
  double *mutable_rotation() { return rotation_.coeffs().data(); }

  Eigen::Matrix4d pose();

  void transformed_point_cloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
  void transformed_normals(const pcl::PointCloud<pcl::Normal>::Ptr &normals);

  void filter();

  void compute_normal(void);
  void reorient_normal_using_obsrevation_vector(void);

  bool find_closest_point(const pcl::PointXYZ &target_point, pcl::PointXYZ &closest_point, pcl::Normal &normal);

private:
  pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_;
  pcl::PointCloud<pcl::Normal>::Ptr normals_;
  Eigen::Vector3d translation_;
  Eigen::Quaterniond rotation_;
};
