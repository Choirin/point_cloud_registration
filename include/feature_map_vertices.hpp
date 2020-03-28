#pragma once
#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <memory>

#include <Eigen/Dense>

// vertex index, timestamp [ns], position x [m], position y [m], position z [m], quaternion x, quaternion y, quaternion z, quaternion w
// velocity x [m/s], velocity y [m/s], velocity z [m/s]
// acc bias x [m/s^2], acc bias y [m/s^2], acc bias z [m/s^2]
// gyro bias x [rad/s], gyro bias y [rad/s], gyro bias z [rad/s]

#define NUMBER_OF_ELEMENTS_IN_A_VERTEX 18

typedef struct Vertex_
{
  Vertex_(long int index_, double timestamp_, Eigen::Vector3d position_, Eigen::Quaterniond rotation_,
          Eigen::Vector3d velocity_, Eigen::Vector3d acc_bias_, Eigen::Vector3d gyro_bias_)
      : index(index_), timestamp(timestamp_), position(position_), rotation(rotation_), velocity(velocity_), acc_bias(acc_bias_), gyro_bias(gyro_bias_)
  {
  }
  long int index;
  double timestamp;
  Eigen::Vector3d position;
  Eigen::Quaterniond rotation;
  Eigen::Vector3d velocity;
  Eigen::Vector3d acc_bias;
  Eigen::Vector3d gyro_bias;
} Vertex;

class FeatureMapVertices
{
public:
  FeatureMapVertices(std::string path_to_csv)
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

      if (record.size() == NUMBER_OF_ELEMENTS_IN_A_VERTEX)
      {
        try
        {
          std::shared_ptr<Vertex> vertex = std::make_shared<Vertex>(
              std::stol(record[0]),
              std::stod(record[1]) / 1.0e9,
              Eigen::Vector3d(std::stod(record[2]), std::stod(record[3]), std::stod(record[4])),
              // Caution: w, x, y, z
              Eigen::Quaterniond(std::stod(record[8]), std::stod(record[5]), std::stod(record[6]), std::stod(record[7])),
              Eigen::Vector3d(std::stod(record[9]), std::stod(record[10]), std::stod(record[11])),
              Eigen::Vector3d(std::stod(record[12]), std::stod(record[13]), std::stod(record[14])),
              Eigen::Vector3d(std::stod(record[15]), std::stod(record[16]), std::stod(record[17])));
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
  ~FeatureMapVertices() {}

  std::vector<std::shared_ptr<Vertex>> data()
  {
    return data_;
  }

  bool get_pose(double timestamp, Eigen::Vector3d &position, Eigen::Quaterniond &rotation)
  {
    std::shared_ptr<Vertex> before = nullptr;
    std::shared_ptr<Vertex> after = nullptr;
    for (auto vertex : data_)
    {
      if (vertex->timestamp <= timestamp)
        before = vertex;
      else if (timestamp <= vertex->timestamp)
        after = vertex;
      if (before != nullptr && after != nullptr)
        break;
    }
    if (before == nullptr || after == nullptr)
      return false;
    auto t = (timestamp - before->timestamp) / (after->timestamp - before->timestamp);
    position = before->position * (1 - t) + after->position * t;
    rotation = before->rotation.slerp(t, after->rotation);
    return true;
  }

  void print(void)
  {
    std::cout << table_.size() << std::endl;
    for (auto vertex : data_)
    {
      std::cout << vertex->index << ", ";
      std::cout.precision(20);
      std::cout << vertex->timestamp << ", ";
      std::cout << vertex->position(0) << ", "
                << vertex->position(1) << ", "
                << vertex->position(2) << ", ";
      std::cout << std::endl;
    }
  }

private:
  std::vector<std::vector<double>> table_;
  std::vector<std::shared_ptr<Vertex>> data_;
};
