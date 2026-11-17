#pragma once

#include <cstddef>
#include <memory>
#include <vector>

#include <Eigen/Core>

namespace dog_slam
{

// Exact, incrementally maintained KD-tree for normalized SOLiD range descriptors.
// A small unindexed tail is scanned linearly and periodically folded into a
// balanced tree, avoiding an O(history) scan on every keyframe without changing
// nearest-neighbour results.
class DescriptorIndex
{
public:
  explicit DescriptorIndex(std::size_t rebuild_interval = 64);
  ~DescriptorIndex();

  void clear();
  void add(const Eigen::VectorXd & descriptor);
  std::vector<std::size_t> nearest(
    const Eigen::VectorXd & query, std::size_t eligible_count, std::size_t count) const;
  std::size_t size() const;

private:
  struct Node;

  std::unique_ptr<Node> build(
    std::vector<std::size_t>::iterator begin,
    std::vector<std::size_t>::iterator end, std::size_t depth);
  void rebuild();

  std::size_t rebuild_interval_;
  Eigen::Index dimension_ = 0;
  std::vector<Eigen::VectorXd> descriptors_;
  std::size_t indexed_size_ = 0;
  std::unique_ptr<Node> root_;
};

}  // namespace dog_slam
