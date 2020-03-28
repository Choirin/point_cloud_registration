#pragma once
#include "depth_frame.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>

class ICPRegistration
{
public:
  ICPRegistration(std::vector<std::shared_ptr<DepthFrame>> frames)
    : frames_(frames)
  {
  }
  ~ICPRegistration() {}

  void merge(const pcl::PointCloud<pcl::PointXYZ>::Ptr& merged_cloud)
  {
    for (auto frame : frames_)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>());
      frame->transformed_point_cloud(point_cloud);
      if (merged_cloud->points.size() == 0)
        *merged_cloud = *point_cloud;
      else
      {
        *merged_cloud = *merged_cloud + *point_cloud;
        continue;
        Eigen::Matrix4d refinement_matrix = icp(point_cloud, merged_cloud);
        // Eigen::Matrix4f refinement_matrix = ndt(input_cloud, pointcloud_map);
        // std::cout << refinement_matrix << std::endl;

        pcl::PointCloud<pcl::PointXYZ>::Ptr refined_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*point_cloud, *refined_cloud, refinement_matrix);

        *merged_cloud = *merged_cloud + *refined_cloud;
      }
    }

    pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
    approximate_voxel_filter.setLeafSize(0.01, 0.01, 0.01);
    approximate_voxel_filter.setInputCloud(merged_cloud);
    approximate_voxel_filter.filter(*merged_cloud);
    std::cout << "Voxel grid filtered cloud contains " << merged_cloud->size()
              << " data points" << std::endl;
  }

private:
  std::vector<std::shared_ptr<DepthFrame>> frames_;

  Eigen::Matrix4d icp(const pcl::PointCloud<pcl::PointXYZ>::Ptr input, const pcl::PointCloud<pcl::PointXYZ>::Ptr target)
  {
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

    // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
    icp.setMaxCorrespondenceDistance(0.05);
    // Set the maximum number of iterations (criterion 1)
    icp.setMaximumIterations(50);
    // Set the transformation epsilon (criterion 2)
    icp.setTransformationEpsilon(1e-8);
    // Set the euclidean distance difference epsilon (criterion 3)
    icp.setEuclideanFitnessEpsilon(1);

    icp.setInputCloud(input);
    icp.setInputTarget(target);

    pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(Final);

    Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
    transformation_matrix = icp.getFinalTransformation().cast<double>();

    // Check translation norm and rotation
    auto translation_norm = transformation_matrix.block<3, 1>(0, 3).norm();
    // https://en.wikipedia.org/wiki/Rotation_matrix#Determining_the_angle
    auto angle = acos((transformation_matrix.block<3, 3>(0, 0).trace() - 1) / 2);
    // TODO: check outside motion of yaw axis
    std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore()
              << " d: " << translation_norm << " theta: " << RAD2DEG(angle) << std::endl;

    if (icp.hasConverged() && icp.getFitnessScore() < 0.01 && translation_norm < 0.3 && angle < DEG2RAD(30))
    {
      std::cout << "refine transformation" << std::endl;
      return transformation_matrix;
    }

    std::cout << "" << std::endl;
    return Eigen::Matrix4d::Identity();
  }

};
