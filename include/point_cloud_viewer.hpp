#pragma once
#include "depth_frame.hpp"

#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

#include <iostream>
#include <thread>
#include <memory>

class PointCloudViewer
{
public:
  PointCloudViewer(const std::vector<std::shared_ptr<DepthFrame>> &frames)
      : frames_(frames)
  {
  }
  PointCloudViewer() {}
  ~PointCloudViewer() {}

  void set_frames(const std::vector<std::shared_ptr<DepthFrame>> &frames)
  {
    frames_ = frames;
  }
  void append_frame(const std::shared_ptr<DepthFrame> &frame)
  {
    frames_.emplace_back(frame);
  }
  void append_line(const Eigen::Vector3d &point_a, const Eigen::Vector3d &point_b);
  void clear_frames(void) { frames_.clear(); }
  void clear_lines(void) { lines_.clear(); }

  void spin(void);

private:
  pcl::visualization::PCLVisualizer::Ptr visualizer_;
  std::vector<std::shared_ptr<DepthFrame>> frames_;
  std::vector<std::tuple<pcl::PointXYZ, pcl::PointXYZ>> lines_;

  void intialize_visualizer_(void);

  void add_line(const std::tuple<pcl::PointXYZ, pcl::PointXYZ> &line);
  void add_point_cloud(const std::shared_ptr<DepthFrame> &frame);
  void add_normals(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const pcl::PointCloud<pcl::Normal>::Ptr &normals);

  std::string pointer_to_string_id(const void *ptr);
  std::string unique_id(void);
  void set_unique_color_(const std::string &id);
};
