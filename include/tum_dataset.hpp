#pragma once
#include <gflags/gflags.h>
#include <Eigen/Dense>

#include <algorithm>
#include <random>
#include <iostream>
#include <fstream>

namespace tumdataset
{
typedef struct Vertex_
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Vertex_(double timestamp_, Eigen::Vector3d translation_, Eigen::Quaterniond rotation_)
      : timestamp(timestamp_), translation(translation_), rotation(rotation_)
  {
  }
  double timestamp;
  Eigen::Vector3d translation;
  Eigen::Quaterniond rotation;
} Vertex;

class GroundTruth
{
public:
  GroundTruth(std::string path_to_csv)
    : number_of_elements_in_a_column_(8)
  {
    std::cout << path_to_csv << "¥n";
    std::fstream filestream(path_to_csv);

    if (!filestream.is_open())
      return;

    std::string buffer;
    while (getline(filestream, buffer))
    {
      std::vector<std::string> record;
      std::istringstream streambuffer(buffer);
      std::string token;

      while (getline(streambuffer, token, ' '))
      {
        record.push_back(token);
      }

      if (record.size() == number_of_elements_in_a_column_)
      {
        try
        {
          std::shared_ptr<Vertex> vertex = std::make_shared<Vertex>(
              std::stod(record[0]),
              Eigen::Vector3d(std::stod(record[1]), std::stod(record[2]), std::stod(record[3])),
              // Caution: w, x, y, z
              Eigen::Quaterniond(std::stod(record[7]), std::stod(record[4]), std::stod(record[5]), std::stod(record[6])));
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
}
