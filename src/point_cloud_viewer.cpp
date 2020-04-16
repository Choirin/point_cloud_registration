#include "point_cloud_viewer.hpp"

#include <pangolin/gl/gl.h>
#include <pangolin/gl/gldraw.h>

void PointCloudViewer::append_line(const Eigen::Vector3d &point_a, const Eigen::Vector3d &point_b)
{
  pcl::PointXYZ pt_a(point_a.x(), point_a.y(), point_a.z());
  pcl::PointXYZ pt_b(point_b.x(), point_b.y(), point_b.z());
  lines_.emplace_back(std::make_tuple(pt_a, pt_b));
}

void PointCloudViewer::spin(void)
{
  intialize_visualizer_();
  for (auto frame : frames_)
    add_point_cloud(frame);
  for (auto line : lines_)
    add_line(line);

  while (!pangolin::ShouldQuit())
  {
    // Clear screen and activate view to render into
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    (*d_cam_)->Activate(*s_cam_);

    for (auto gl_cloud : gl_clouds_)
      gl_cloud->draw();

    // Swap frames and Process Events
    pangolin::FinishFrame();
  }
  pangolin::DestroyWindow("Main");
}

void PointCloudViewer::intialize_visualizer_(void)
{
  pangolin::CreateWindowAndBind("Main", 640, 480);
  glEnable(GL_DEPTH_TEST);

  // Define Projection and initial ModelView matrix
  s_cam_ = std::make_shared<pangolin::OpenGlRenderState>(
      pangolin::ProjectionMatrix(640, 480, 420, 420, 320, 240, 0.2, 100),
      pangolin::ModelViewLookAt(-2, 2, -2, 0, 0, 0, pangolin::AxisY));

  // Create Interactive View in window
  handler_ = std::make_shared<pangolin::Handler3D>(*s_cam_);
  d_cam_ = std::make_shared<pangolin::View *>(
      &pangolin::CreateDisplay()
           .SetBounds(0.0, 1.0, 0.0, 1.0, -640.0f / 480.0f)
           .SetHandler(handler_.get()));
}

void PointCloudViewer::add_line(const std::tuple<pcl::PointXYZ, pcl::PointXYZ> &line)
{
  // visualizer_->addLine<pcl::PointXYZ>(std::get<0>(line), std::get<1>(line), id);
}

void PointCloudViewer::add_point_cloud(const std::shared_ptr<DepthFrame> &frame)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  frame->transformed_point_cloud(cloud);

  auto gl_cloud = std::make_shared<GlCloud>(cloud);
  float r, g, b;
  get_unique_color_(r, g, b);
  gl_cloud->set_color(r, g, b);
  gl_clouds_.emplace_back(gl_cloud);

  // pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
  // frame->transformed_normals(normals);
  // // normals = frame->normals(); // overwrite with original
  // add_normals(cloud, normals);
}

void PointCloudViewer::add_normals(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const pcl::PointCloud<pcl::Normal>::Ptr &normals)
{
  for (size_t i = 0; i < cloud->points.size(); ++i)
  {
    pcl::PointXYZ normal_vertex;
    normal_vertex.getVector3fMap() = cloud->points[i].getVector3fMap() + normals->points[i].getNormalVector3fMap();
    // visualizer_->addLine<pcl::PointXYZ>(cloud->points[i], normal_vertex, id);
  }
}

void PointCloudViewer::get_unique_color_(float &r, float &g, float& b)
{
  r = std::rand() / float(RAND_MAX);
  g = std::rand() / float(RAND_MAX);
  b = std::rand() / float(RAND_MAX);
}
