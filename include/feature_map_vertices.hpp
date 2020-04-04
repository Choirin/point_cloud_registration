#pragma once
#include <iostream>
#include <string>
#include <vector>
#include <memory>

#include <Eigen/Dense>

// vertex index, timestamp [ns], position x [m], position y [m], position z [m], quaternion x, quaternion y, quaternion z, quaternion w
// velocity x [m/s], velocity y [m/s], velocity z [m/s]
// acc bias x [m/s^2], acc bias y [m/s^2], acc bias z [m/s^2]
// gyro bias x [rad/s], gyro bias y [rad/s], gyro bias z [rad/s]

#define NUMBER_OF_ELEMENTS_IN_A_VERTEX 18

typedef struct Vertex_
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
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
  FeatureMapVertices(std::string path_to_csv);
  ~FeatureMapVertices() {}

  std::vector<std::shared_ptr<Vertex>> data() { return data_; }

  bool get_pose(double timestamp, Eigen::Vector3d &position, Eigen::Quaterniond &rotation);
  void print(void);

private:
  std::vector<std::vector<double>> table_;
  std::vector<std::shared_ptr<Vertex>> data_;
};
