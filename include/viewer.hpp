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
  PointCloudViewer() {}
  ~PointCloudViewer() {}

  void view(const std::shared_ptr<DepthFrame>& frame)
  {
    view(frame->point_cloud());
  }

  void view(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
  {
    pcl::visualization::PCLVisualizer::Ptr viewer;
    viewer = simpleVis(cloud);
    viewer->spin();
    // while (!viewer->wasStopped())
    // {
    //   viewer->spinOnce(100);
    //   std::this_thread::sleep_for(std::chrono::milliseconds(100));
    // }
  }

private:
  pcl::visualization::PCLVisualizer::Ptr
  simpleVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
  {
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    return viewer;
  }
};
