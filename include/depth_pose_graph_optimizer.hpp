#pragma once
#include "depth_frame.hpp"

#include <memory>
#include <vector>

void optimize_pose_graph(std::vector<std::shared_ptr<DepthFrame>> &frames);
