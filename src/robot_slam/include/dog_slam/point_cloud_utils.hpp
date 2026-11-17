#pragma once

#include "dog_slam/loop_closure_types.hpp"

namespace dog_slam
{

Cloud::Ptr voxelized(const Cloud::ConstPtr & input, double leaf_size);

}  // namespace dog_slam
