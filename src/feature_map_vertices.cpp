#include "feature_map_vertices.hpp"

#include <Eigen/Dense>

#include <fstream>
#include <sstream>

FeatureMapVertices::FeatureMapVertices(std::string path_to_csv)
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

bool FeatureMapVertices::get_pose(double timestamp, Eigen::Vector3d &position, Eigen::Quaterniond &rotation)
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

void FeatureMapVertices::print(void)
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
