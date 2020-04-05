#include "feature_map_vertices.hpp"

#include <Eigen/Dense>

#include <fstream>
#include <sstream>

#define NANOSECONDS_IN_A_SECOND 1.0e9

FeatureMapVertices::FeatureMapVertices(const fs::path &file_path)
{
  load_from_csv(file_path);
  sort_by_timestamp();
}

FeatureMapVertices::FeatureMapVertices(const std::vector<fs::path> &file_paths)
{
  for (auto file_path : file_paths)
  {
    load_from_csv(file_path);
  }
  sort_by_timestamp();
}

bool FeatureMapVertices::get_pose(double timestamp, Eigen::Vector3d &translation, Eigen::Quaterniond &rotation)
{
  std::shared_ptr<Vertex> before = nullptr;
  std::shared_ptr<Vertex> after = nullptr;
  for (auto vertex : vertices_)
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
  translation = before->translation * (1 - t) + after->translation * t;
  rotation = before->rotation.slerp(t, after->rotation);
  return true;
}

void FeatureMapVertices::print(void)
{
  for (auto vertex : vertices_)
  {
    std::cout << vertex->index << ", ";
    std::cout.precision(20);
    std::cout << vertex->timestamp << ", ";
    std::cout << vertex->translation(0) << ", "
              << vertex->translation(1) << ", "
              << vertex->translation(2) << ", ";
    std::cout << std::endl;
  }
}

size_t FeatureMapVertices::load_from_csv(const fs::path &file_path)
{
  std::fstream filestream(file_path.string());

  if (!filestream.is_open())
    return 0;

  std::string buffer;
  while (getline(filestream, buffer))
  {
    std::vector<std::string> record;
    std::istringstream streambuffer(buffer);
    std::string token;

    while (getline(streambuffer, token, ','))
      record.emplace_back(token);

    std::shared_ptr<Vertex> vertex;
    if (record.size() == NUMBER_OF_ELEMENTS_IN_A_VERTEX &&
        vertex_from_string_vector(record, vertex))
      vertices_.emplace_back(vertex);
  }
  return vertices_.size();
}

bool FeatureMapVertices::vertex_from_string_vector(const std::vector<std::string> &record, std::shared_ptr<Vertex> &vertex)
{
  try
  {
    vertex = std::make_shared<Vertex>(
        std::stol(record[0]),
        std::stod(record[1]) / NANOSECONDS_IN_A_SECOND,
        Eigen::Vector3d(std::stod(record[2]), std::stod(record[3]), std::stod(record[4])),
        // Caution: w, x, y, z
        Eigen::Quaterniond(std::stod(record[8]), std::stod(record[5]), std::stod(record[6]), std::stod(record[7])),
        Eigen::Vector3d(std::stod(record[9]), std::stod(record[10]), std::stod(record[11])),
        Eigen::Vector3d(std::stod(record[12]), std::stod(record[13]), std::stod(record[14])),
        Eigen::Vector3d(std::stod(record[15]), std::stod(record[16]), std::stod(record[17])));
    return true;
  }
  catch (const std::exception &e)
  {
    // std::cerr << e.what() << std::endl;
    // std::cerr << "skip this line" << std::endl;
    return false;
  }
}

void FeatureMapVertices::sort_by_timestamp(void)
{
  std::sort(vertices_.begin(), vertices_.end(),
            [&](auto const &a, auto const &b) {
              return a->timestamp < b->timestamp;
            });
}
