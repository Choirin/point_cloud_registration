#pragma once
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>

class DepthFrame
{
public:
  DepthFrame(const pcl::PointCloud<pcl::PointXYZ>::Ptr& point_cloud, Eigen::Matrix4d pose)
      : point_cloud_(point_cloud)
  {
    translation_ = pose.block<3, 1>(0, 3);
    rotation_ = Eigen::Quaterniond(pose.block<3, 3>(0, 0));
    // pcl::io::savePCDFileASCII("/home/kohei/map.pcd", *point_cloud_);
    // exit(-1);
  }
  ~DepthFrame() {}

  double* mutable_translation() { return translation_.data(); }
  double* mutable_rotation() { return rotation_.coeffs().data(); }

  Eigen::Matrix4d pose()
  {
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    pose.block<3, 3>(0, 0) = rotation_.toRotationMatrix();
    pose.block<3, 1>(0, 3) = translation_;
    return pose;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud()
  {
    return point_cloud_;
  }

  void transformed_point_cloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
  {
    pcl::transformPointCloud(*point_cloud_, *cloud, pose());
  }

  void filter()
  {
    // pcl::PassThrough<pcl::PointXYZ> pass;
    // pass.setInputCloud(point_cloud_);
    // pass.setFilterFieldName("z");
    // pass.setFilterLimits(0.2, 1.8);
    // //pass.setFilterLimitsNegative (true);
    // pass.filter(*point_cloud_);

    pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
    // approximate_voxel_filter.setLeafSize(0.05, 0.05, 0.05);
    approximate_voxel_filter.setLeafSize(0.5, 0.5, 0.5);
    approximate_voxel_filter.setInputCloud(point_cloud_);
    approximate_voxel_filter.filter(*point_cloud_);
  }

  void compute_normal()
  {
    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(point_cloud_);

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);

    // Output datasets
    cloud_normals_ = boost::make_shared<pcl::PointCloud<pcl::Normal>>();

    // Use all neighbors in a sphere of radius 3cm
    ne.setRadiusSearch(0.03);

    // Compute the features
    ne.compute(*cloud_normals_);
  }

  std::vector<std::shared_ptr<DepthFrame>> find_neighbor_frames(const std::vector<std::shared_ptr<DepthFrame>> &frames)
  {
    std::vector<bool> adjacency;
    std::vector<std::shared_ptr<DepthFrame>> neighbor_frames;

    auto size = frames.size();
    adjacency.resize(size);
    for (size_t i = 0; i < size; ++i)
    {
      if (this == frames[i].get())
        continue;
      auto translation_norm = (pose().block<3, 1>(0, 3) - frames[i]->pose().block<3, 1>(0, 3)).norm();
      auto angle = acos(((pose().block<3, 3>(0, 0) * frames[i]->pose().block<3, 3>(0, 0).transpose()).trace() - 1) / 2);

      // std::cout << " d: " << translation_norm << " theta: " << RAD2DEG(angle) << std::endl;

      // if (translation_norm < 0.5 && angle < DEG2RAD(30))
      if (translation_norm < 1.0)
      {
        adjacency[i] = true;
        neighbor_frames.emplace_back(frames[i]);
      }
    }

    return neighbor_frames;
  }

  bool find_closest_point(const pcl::PointXYZ &target_point, pcl::PointXYZ &closest_point)
  {
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(point_cloud_);

    // K nearest neighbor search
    int K = 10;

    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);

    if (kdtree.nearestKSearch(target_point, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
    {
      if (pointNKNSquaredDistance[0] > (0.2 * 0.2)) return false;
      closest_point = point_cloud_->points[pointIdxNKNSearch[0]];
      return true;
    }

    return false;
  }

private:
  pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_;
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_;
  Eigen::Vector3d translation_;
  Eigen::Quaterniond rotation_;

};
