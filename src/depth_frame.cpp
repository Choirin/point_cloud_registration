#include "depth_frame.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>

#include <gflags/gflags.h>

DEFINE_double(pass_through_filter_z_min,
              0.2,
              "minimum Z axis of pass through filter [m]");
DEFINE_double(pass_through_filter_z_max,
              20.0,
              "maximum Z axis of pass through filter [m]");
DEFINE_double(voxel_grid_filter_leaf_size,
              0.2,
              "leaf size of voxel grid filter [m]");
DEFINE_double(closest_point_distance_threshold,
              0.6,
              "distance threshold to select closest points [m]");

DepthFrame::DepthFrame(const double &timestamp, const pcl::PointCloud<pcl::PointXYZ>::Ptr &point_cloud, const Eigen::Matrix4d &pose)
    : timestamp_(timestamp), unfiltered_cloud_(point_cloud), point_cloud_(point_cloud)
{
  translation_ = pose.block<3, 1>(0, 3);
  rotation_ = Eigen::Quaterniond(pose.block<3, 3>(0, 0));
}

DepthFrame::DepthFrame(const pcl::PointCloud<pcl::PointXYZ>::Ptr &point_cloud, const Eigen::Matrix4d &pose)
    : unfiltered_cloud_(point_cloud), point_cloud_(point_cloud)
{
  translation_ = pose.block<3, 1>(0, 3);
  rotation_ = Eigen::Quaterniond(pose.block<3, 3>(0, 0));
}

void DepthFrame::get_pose(Eigen::Matrix4d &pose)
{
  pose = Eigen::Matrix4d::Identity();
  pose.block<3, 3>(0, 0) = rotation_.toRotationMatrix();
  pose.block<3, 1>(0, 3) = translation_;
}

void DepthFrame::transformed_point_cloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
  Eigen::Matrix4d pose;
  get_pose(pose);
  pcl::transformPointCloud(*point_cloud_, *cloud, pose);
}

void DepthFrame::transformed_normals(const pcl::PointCloud<pcl::Normal>::Ptr &normals)
{
  Eigen::Matrix4d pose;
  get_pose(pose);
  Eigen::Matrix3d rotation = pose.block<3, 3>(0, 0);
  *normals = *normals_;
  for (pcl::Normal &normal : normals->points)
  {
    Eigen::Vector3d normal_g(normal.getNormalVector3fMap().cast<double>());
    normal_g = rotation * normal_g;
    normal = pcl::Normal(normal_g.x(), normal_g.y(), normal_g.z());
  }
}

void DepthFrame::filter()
{
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(unfiltered_cloud_);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(FLAGS_pass_through_filter_z_min, FLAGS_pass_through_filter_z_max);
  pass.filter(*point_cloud_);

  // pcl::PassThrough<pcl::PointXYZ> pass_y;
  // pass_y.setInputCloud(point_cloud_);
  // pass_y.setFilterFieldName("y");
  // pass_y.setFilterLimits(0.0, 10.0);
  // pass_y.filter(*point_cloud_);

  pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
  voxel_filter.setLeafSize(FLAGS_voxel_grid_filter_leaf_size,
                           FLAGS_voxel_grid_filter_leaf_size,
                           FLAGS_voxel_grid_filter_leaf_size);
  voxel_filter.setInputCloud(point_cloud_);
  voxel_filter.filter(*point_cloud_);
}

void DepthFrame::compute_normal(const int k_nearest_neighbor)
{
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud(point_cloud_);

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
  ne.setSearchMethod(tree);
  normals_ = boost::make_shared<pcl::PointCloud<pcl::Normal>>();
  ne.setKSearch(k_nearest_neighbor);
  ne.compute(*normals_);
}

void DepthFrame::compute_normal_using_unfiltered(const int k_nearest_neighbor)
{
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(unfiltered_cloud_);
  int K = k_nearest_neighbor;
  std::vector<int> knn_searched_indices(K);
  std::vector<float> knn_searched_distances(K);

  auto size = point_cloud_->points.size();
  normals_ = boost::make_shared<pcl::PointCloud<pcl::Normal>>();
  normals_->width = size;
  normals_->height = 1;
  normals_->points.clear();
  normals_->points.resize(size);
  for (size_t i = 0; i < size; ++i)
  {
    const auto &point = point_cloud_->points[i];
    auto &normal = normals_->points[i];
    if (kdtree.nearestKSearch(point, K, knn_searched_indices, knn_searched_distances) == K)
    {
      Eigen::Vector4f plane_parameters;
      float curvature;
      pcl::computePointNormal(*unfiltered_cloud_, knn_searched_indices, plane_parameters, curvature);
      normal.normal_x = plane_parameters[0];
      normal.normal_y = plane_parameters[1];
      normal.normal_z = plane_parameters[2];
      normal.curvature = curvature;
      pcl::flipNormalTowardsViewpoint(point,
                                      0.0, 0.0, 0.0,
                                      normal.normal_x, normal.normal_y, normal.normal_z);
    }
  }
}

bool DepthFrame::find_closest_point(const pcl::PointXYZ &target_point, pcl::PointXYZ &closest_point, pcl::Normal &normal)
{
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(point_cloud_);

  int K = 1;
  std::vector<int> knn_searched_indices(K);
  std::vector<float> knn_searched_distances(K);

  auto threshold_square = pow(FLAGS_closest_point_distance_threshold, 2);
  auto size = kdtree.nearestKSearch(target_point, K, knn_searched_indices, knn_searched_distances);
  if (size > 0 && knn_searched_distances[0] < threshold_square)
  {
    closest_point = point_cloud_->points[knn_searched_indices[0]];
    normal = normals_->points[knn_searched_indices[0]];
    return true;
  }
  return false;
}

bool DepthFrame::find_closest_point(const pcl::PointXYZ &target_point, const pcl::Normal &target_normal,
                                    pcl::PointXYZ &closest_point, pcl::Normal &normal)
{
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(point_cloud_);

  int K = 10;
  std::vector<int> knn_searched_indices(K);
  std::vector<float> knn_searched_distances(K);

  auto threshold_square = pow(FLAGS_closest_point_distance_threshold, 2);
  auto size = kdtree.nearestKSearch(target_point, K, knn_searched_indices, knn_searched_distances);
  for (auto i = 0; i < size; ++i)
  {
    closest_point = point_cloud_->points[knn_searched_indices[i]];
    normal = normals_->points[knn_searched_indices[i]];
    auto normal_angle = acos(target_normal.getNormalVector3fMap().dot(normal.getNormalVector3fMap()));
    if ((knn_searched_distances[i] < threshold_square) && (normal_angle < DEG2RAD(30)))
      return true;
  }
  return false;
}
