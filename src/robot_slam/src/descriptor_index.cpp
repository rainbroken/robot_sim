#include "dog_slam/descriptor_index.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <numeric>
#include <queue>
#include <stdexcept>
#include <utility>

namespace dog_slam
{

struct DescriptorIndex::Node
{
  std::size_t descriptor_index = 0;
  Eigen::Index axis = 0;
  std::unique_ptr<Node> left;
  std::unique_ptr<Node> right;
};

DescriptorIndex::DescriptorIndex(std::size_t rebuild_interval)
: rebuild_interval_(std::max<std::size_t>(1, rebuild_interval))
{
}

DescriptorIndex::~DescriptorIndex() = default;

void DescriptorIndex::clear()
{
  descriptors_.clear();
  indexed_size_ = 0;
  dimension_ = 0;
  root_.reset();
}

void DescriptorIndex::add(const Eigen::VectorXd & descriptor)
{
  if (descriptor.size() <= 0) {
    throw std::invalid_argument("descriptor index cannot store an empty descriptor");
  }
  if (dimension_ == 0) {
    dimension_ = descriptor.size();
  } else if (descriptor.size() != dimension_) {
    throw std::invalid_argument("descriptor index dimension changed");
  }
  descriptors_.push_back(descriptor);
  if (!root_ || descriptors_.size() - indexed_size_ >= rebuild_interval_) {
    rebuild();
  }
}

std::unique_ptr<DescriptorIndex::Node> DescriptorIndex::build(
  std::vector<std::size_t>::iterator begin,
  std::vector<std::size_t>::iterator end, std::size_t depth)
{
  if (begin == end) {
    return nullptr;
  }
  const Eigen::Index axis = static_cast<Eigen::Index>(depth % dimension_);
  auto middle = begin + (end - begin) / 2;
  std::nth_element(
    begin, middle, end,
    [this, axis](std::size_t left, std::size_t right) {
      return descriptors_[left](axis) < descriptors_[right](axis);
    });
  auto node = std::make_unique<Node>();
  node->descriptor_index = *middle;
  node->axis = axis;
  node->left = build(begin, middle, depth + 1);
  node->right = build(middle + 1, end, depth + 1);
  return node;
}

void DescriptorIndex::rebuild()
{
  indexed_size_ = descriptors_.size();
  std::vector<std::size_t> indices(indexed_size_);
  std::iota(indices.begin(), indices.end(), 0);
  root_ = build(indices.begin(), indices.end(), 0);
}

std::vector<std::size_t> DescriptorIndex::nearest(
  const Eigen::VectorXd & query, std::size_t eligible_count, std::size_t count) const
{
  if (count == 0 || descriptors_.empty() || query.size() != dimension_) {
    return {};
  }
  eligible_count = std::min(eligible_count, descriptors_.size());
  count = std::min(count, eligible_count);
  if (count == 0) {
    return {};
  }

  using Neighbor = std::pair<double, std::size_t>;
  std::priority_queue<Neighbor> neighbors;
  const auto consider = [&](std::size_t index) {
      if (index >= eligible_count) {
        return;
      }
      const double distance = (descriptors_[index] - query).squaredNorm();
      if (neighbors.size() < count) {
        neighbors.emplace(distance, index);
      } else if (distance < neighbors.top().first ||
        (distance == neighbors.top().first && index < neighbors.top().second))
      {
        neighbors.pop();
        neighbors.emplace(distance, index);
      }
    };
  const std::function<void(const Node *)> search = [&](const Node * node) {
      if (!node) {
        return;
      }
      const double difference =
        query(node->axis) - descriptors_[node->descriptor_index](node->axis);
      const Node * near = difference <= 0.0F ? node->left.get() : node->right.get();
      const Node * far = difference <= 0.0F ? node->right.get() : node->left.get();
      search(near);
      consider(node->descriptor_index);
      if (neighbors.size() < count || difference * difference <= neighbors.top().first) {
        search(far);
      }
    };
  search(root_.get());
  for (std::size_t index = indexed_size_; index < eligible_count; ++index) {
    consider(index);
  }

  std::vector<Neighbor> ordered;
  ordered.reserve(neighbors.size());
  while (!neighbors.empty()) {
    ordered.push_back(neighbors.top());
    neighbors.pop();
  }
  std::sort(
    ordered.begin(), ordered.end(),
    [](const Neighbor & left, const Neighbor & right) {
      return left.first < right.first ||
      (left.first == right.first && left.second < right.second);
    });
  std::vector<std::size_t> result;
  result.reserve(ordered.size());
  for (const auto & entry : ordered) {
    result.push_back(entry.second);
  }
  return result;
}

std::size_t DescriptorIndex::size() const
{
  return descriptors_.size();
}

}  // namespace dog_slam
