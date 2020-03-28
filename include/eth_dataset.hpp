#pragma once
#include <gflags/gflags.h>
#include <Eigen/Dense>

#include <algorithm>
#include <random>
#include <fstream>

namespace ethdata
{
typedef struct Vertex_
{
  Vertex_(long int index_, double timestamp_, Eigen::Matrix4d pose_)
      : index(index_), timestamp(timestamp_), pose(pose_)
  {
  }
  long int index;
  double timestamp;
  Eigen::Matrix4d pose;
} Vertex;

class GroundTruth
{
public:
  GroundTruth(std::string path_to_csv)
    : number_of_elements_in_a_column_(18)
  {
    std::fstream filestream(path_to_csv);

    if (!filestream.is_open())
      return;

    std::string buffer;
    while (getline(filestream, buffer))
    {
      std::vector<std::string> record;
      std::istringstream streambuffer(buffer);
      std::string token;

      while (getline(streambuffer, token, ','))
      {
        record.push_back(token);
      }

      if (record.size() == number_of_elements_in_a_column_)
      {
        try
        {
          std::uniform_real_distribution<double> unif(-0.5, 0.5);
          std::default_random_engine re;
          double sx = unif(re);
          double sy = unif(re);
          double sz = unif(re);
          Eigen::Matrix4d pose;
          pose << std::stod(record[2]), std::stod(record[3]), std::stod(record[4]), std::stod(record[5]) + sx,
              std::stod(record[6]), std::stod(record[7]), std::stod(record[8]), std::stod(record[9]) + sy,
              std::stod(record[10]), std::stod(record[11]), std::stod(record[12]), std::stod(record[13]) + sz,
              std::stod(record[14]), std::stod(record[15]), std::stod(record[16]), std::stod(record[17]);
          std::shared_ptr<Vertex> vertex = std::make_shared<Vertex>(
              std::stol(record[0]), std::stod(record[1]), pose);
          data_.push_back(vertex);
        }
        catch (const std::exception &e)
        {
          // std::cerr << e.what() << std::endl;
          // std::cerr << "skip this line" << std::endl;
        }
      }
    }
  }
  ~GroundTruth() {}

  std::vector<std::shared_ptr<Vertex>> data()
  {
    return data_;
  }

private:
  size_t number_of_elements_in_a_column_;
  std::vector<std::shared_ptr<Vertex>> data_;
};

class PointCloudCSV
{
public:
  PointCloudCSV(std::string path_to_csv)
    : number_of_elements_in_a_column_(7)
  {
    cloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    std::fstream filestream(path_to_csv);

    if (!filestream.is_open())
      return;

    std::string buffer;
    while (getline(filestream, buffer))
    {
      std::vector<std::string> record;
      std::istringstream streambuffer(buffer);
      std::string token;

      while (getline(streambuffer, token, ','))
      {
        record.push_back(token);
      }

      if (record.size() == number_of_elements_in_a_column_)
      {
        try
        {
          pcl::PointXYZ point;
          point.x = std::stod(record[1]);
          point.y = std::stod(record[2]);
          point.z = std::stod(record[3]);
          cloud_->points.emplace_back(point);
        }
        catch (const std::exception &e)
        {
          // std::cerr << e.what() << std::endl;
          // std::cerr << "skip this line" << std::endl;
        }
      }
    }
    cloud_->width = cloud_->points.size();
    cloud_->height = 0;
    cloud_->is_dense = false;
  }
  ~PointCloudCSV() {}

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud()
  {
    return cloud_;
  }

private:
  size_t number_of_elements_in_a_column_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
};
}
