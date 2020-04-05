#pragma once
#include <Eigen/Dense>

#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <experimental/filesystem>

namespace fs = std::experimental::filesystem;

// vertex index, timestamp [ns], translation x [m], translation y [m], translation z [m], quaternion x, quaternion y, quaternion z, quaternion w
// velocity x [m/s], velocity y [m/s], velocity z [m/s]
// acc bias x [m/s^2], acc bias y [m/s^2], acc bias z [m/s^2]
// gyro bias x [rad/s], gyro bias y [rad/s], gyro bias z [rad/s]

#define NUMBER_OF_ELEMENTS_IN_A_VERTEX 18

typedef struct Vertex_
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Vertex_(long int index_, double timestamp_, Eigen::Vector3d translation_, Eigen::Quaterniond rotation_,
          Eigen::Vector3d velocity_, Eigen::Vector3d acc_bias_, Eigen::Vector3d gyro_bias_)
      : index(index_), timestamp(timestamp_), translation(translation_), rotation(rotation_), velocity(velocity_), acc_bias(acc_bias_), gyro_bias(gyro_bias_)
  {
  }
  long int index;
  double timestamp;
  Eigen::Vector3d translation;
  Eigen::Quaterniond rotation;
  Eigen::Vector3d velocity;
  Eigen::Vector3d acc_bias;
  Eigen::Vector3d gyro_bias;
} Vertex;

class FeatureMapVertices
{
public:
  FeatureMapVertices(const fs::path &file_path);
  FeatureMapVertices(const std::vector<fs::path> &file_paths);
  ~FeatureMapVertices() {}

  std::vector<std::shared_ptr<Vertex>> vertices() { return vertices_; }

  bool get_pose(double timestamp, Eigen::Vector3d &translation, Eigen::Quaterniond &rotation);
  void print(void);

private:
  std::vector<std::shared_ptr<Vertex>> vertices_;

  size_t load_from_csv(const fs::path &file_path);
  bool vertex_from_string_vector(const std::vector<std::string> &record, std::shared_ptr<Vertex> &vertex);
  void sort_by_timestamp(void);
};
