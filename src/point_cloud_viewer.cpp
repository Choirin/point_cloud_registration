#include "point_cloud_viewer.hpp"

void PointCloudViewer::append_line(const Eigen::Vector3d &point_a, const Eigen::Vector3d &point_b)
{
  auto id = unique_id();
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
  visualizer_->spin();
  visualizer_->close();
}

void PointCloudViewer::intialize_visualizer_(void)
{
  visualizer_ = boost::make_shared<pcl::visualization::PCLVisualizer>("PointCloudViewer");
  visualizer_->setBackgroundColor(0, 0, 0);
  visualizer_->addCoordinateSystem(1.0);
  visualizer_->initCameraParameters();
}

void PointCloudViewer::add_line(const std::tuple<pcl::PointXYZ, pcl::PointXYZ> &line)
{
  auto id = unique_id();
  visualizer_->addLine<pcl::PointXYZ>(std::get<0>(line), std::get<1>(line), id);
}

void PointCloudViewer::add_point_cloud(const std::shared_ptr<DepthFrame> &frame)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  std::string id = pointer_to_string_id(frame.get());
  frame->transformed_point_cloud(cloud);
  // cloud = frame->point_cloud(); // overwrite with original
  visualizer_->addPointCloud<pcl::PointXYZ>(cloud, id);
  visualizer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, id);
  set_unique_color_(id);

  // pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
  // frame->transformed_normals(normals);
  // // normals = frame->normals(); // overwrite with original
  // add_normals(cloud, normals);
}

void PointCloudViewer::add_normals(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const pcl::PointCloud<pcl::Normal>::Ptr &normals)
{
  for (size_t i = 0; i < cloud->points.size(); ++i)
  {
    auto id = unique_id();
    pcl::PointXYZ normal_vertex;
    normal_vertex.getVector3fMap() = cloud->points[i].getVector3fMap() + normals->points[i].getNormalVector3fMap();
    visualizer_->addLine<pcl::PointXYZ>(cloud->points[i], normal_vertex, id);
  }
}

std::string PointCloudViewer::pointer_to_string_id(const void *ptr)
{
  std::stringstream stream;
  stream << "0x"
         << std::setfill('0') << std::setw(sizeof(void *) * 2)
         << std::hex << ptr;
  return stream.str();
}

std::string PointCloudViewer::unique_id(void)
{
  std::stringstream stream;
  stream << "0x"
         << std::setfill('0') << std::setw(sizeof(unsigned long int) * 2)
         << std::hex << std::rand();
  return stream.str();
}

void PointCloudViewer::set_unique_color_(const std::string &id)
{
  double r = (std::rand() % (100 + 1)) / 100.0;
  double g = (std::rand() % (100 + 1)) / 100.0;
  double b = (std::rand() % (100 + 1)) / 100.0;
  visualizer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, r, g, b, id, 0);
}
