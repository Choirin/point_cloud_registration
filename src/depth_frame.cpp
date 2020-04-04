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

DEFINE_double(voxel_grid_filter_leaf_size,
              0.25,
              "leaf size of voxel grid filter [m]");
DEFINE_double(closest_point_distance_threshold,
              0.5,
              "distance threshold to select closest points [m]");

DepthFrame::DepthFrame(const pcl::PointCloud<pcl::PointXYZ>::Ptr &point_cloud, const Eigen::Matrix4d &pose)
    : point_cloud_(point_cloud)
{
  translation_ = pose.block<3, 1>(0, 3);
  rotation_ = Eigen::Quaterniond(pose.block<3, 3>(0, 0));
}

Eigen::Matrix4d DepthFrame::pose()
{
  Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
  pose.block<3, 3>(0, 0) = rotation_.toRotationMatrix();
  pose.block<3, 1>(0, 3) = translation_;
  return pose;
}

void DepthFrame::transformed_point_cloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
  pcl::transformPointCloud(*point_cloud_, *cloud, pose());
}

void DepthFrame::transformed_normals(const pcl::PointCloud<pcl::Normal>::Ptr &normals)
{
  Eigen::Matrix3d rotation = pose().block<3, 3>(0, 0);
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
  pass.setInputCloud(point_cloud_);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.2, 20.0);
  pass.filter(*point_cloud_);

  pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
  voxel_filter.setLeafSize(FLAGS_voxel_grid_filter_leaf_size,
                           FLAGS_voxel_grid_filter_leaf_size,
                           FLAGS_voxel_grid_filter_leaf_size);
  voxel_filter.setInputCloud(point_cloud_);
  voxel_filter.filter(*point_cloud_);
}

void DepthFrame::compute_normal(void)
{
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud(point_cloud_);

  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
  ne.setSearchMethod(tree);
  normals_ = boost::make_shared<pcl::PointCloud<pcl::Normal>>();
  ne.setKSearch(5);
  ne.compute(*normals_);

  // This is already done by pcl
  // reorient_normal_using_obsrevation_vector();
}

void DepthFrame::reorient_normal_using_obsrevation_vector(void)
{
  for (size_t i = 0; i < point_cloud_->points.size(); ++i)
  {
    Eigen::Vector3d negative_observation_vector(point_cloud_->points[i].getVector3fMap().cast<double>());
    Eigen::Vector3d normal_vector(normals_->points[i].getNormalVector3fMap().cast<double>());
    auto dot_product = negative_observation_vector.dot(normal_vector);
    if (dot_product > 0)
    {
      std::cout << "reorient normal vector" << std::endl;
      normals_->points[i].getNormalVector3fMap() = -normals_->points[i].getNormalVector3fMap();
    }
  }
}

bool DepthFrame::find_closest_point(const pcl::PointXYZ &target_point, pcl::PointXYZ &closest_point, pcl::Normal &normal)
{
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(point_cloud_);

  // K nearest neighbor search
  int K = 1;

  std::vector<int> pointIdxNKNSearch(K);
  std::vector<float> pointNKNSquaredDistance(K);

  if (kdtree.nearestKSearch(target_point, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
  {
    auto threshold_square = pow(FLAGS_closest_point_distance_threshold, 2);
    if (pointNKNSquaredDistance[0] > threshold_square)
      return false;
    closest_point = point_cloud_->points[pointIdxNKNSearch[0]];
    normal = normals_->points[pointIdxNKNSearch[0]];
    return true;
  }

  return false;
}
