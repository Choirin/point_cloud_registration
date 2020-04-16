#pragma once
#include "depth_frame.hpp"

#include <pcl/common/common_headers.h>
#include <pangolin/pangolin.h>

#include <iostream>
#include <thread>
#include <memory>

class GlCloud
{
public:
  GlCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
      : number_of_points(cloud->size()), stride(sizeof(pcl::PointXYZ))
      , r_(1.0), g_(1.0), b_(1.0)
  {
    glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, cloud->points.size() * stride, cloud->points.data(), GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
  }

  virtual ~GlCloud()
  {
    glDeleteBuffers(1, &vbo);
  }

  void set_color(const float r, const float g, const float b)
  {
    r_ = r;
    g_ = g;
    b_ = b;
  }

  void draw()
  {
    glColor3f(r_, g_, b_);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glVertexPointer(3, GL_FLOAT, stride, 0);
    glEnableClientState(GL_VERTEX_ARRAY);
    glDrawArrays(GL_POINTS, 0, number_of_points);
    glDisableClientState(GL_VERTEX_ARRAY);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
  }

private:
  const int number_of_points;
  const int stride;
  GLuint vbo;

  float r_;
  float g_;
  float b_;
};

class PointCloudViewer
{
public:
  PointCloudViewer(const std::vector<std::shared_ptr<DepthFrame>> &frames)
      : frames_(frames)
  {
  }
  PointCloudViewer()
  {
  }
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
  std::shared_ptr<pangolin::OpenGlRenderState> s_cam_;
  std::shared_ptr<pangolin::View *> d_cam_;
  std::shared_ptr<pangolin::Handler3D> handler_;

  std::vector<std::shared_ptr<DepthFrame>> frames_;
  std::vector<std::tuple<pcl::PointXYZ, pcl::PointXYZ>> lines_;

  std::vector<std::shared_ptr<GlCloud>> gl_clouds_;

  void intialize_visualizer_(void);

  void add_line(const std::tuple<pcl::PointXYZ, pcl::PointXYZ> &line);
  void add_point_cloud(const std::shared_ptr<DepthFrame> &frame);
  void add_normals(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const pcl::PointCloud<pcl::Normal>::Ptr &normals);

  void get_unique_color_(float &r, float &g, float &b);
};
