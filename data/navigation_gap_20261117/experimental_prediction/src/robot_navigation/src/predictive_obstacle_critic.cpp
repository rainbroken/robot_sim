#include "dog_nav/predictive_obstacle_critic.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include "dwb_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace dog_nav
{
namespace
{
// Fit compact round returns when possible. A visible-arc centroid shifts as
// the robot passes an obstacle, even if the obstacle has no velocity in x.
// Using it directly can mistake changing viewpoint for an approaching person.
bool fitCircle(const PredictiveObstacleCritic::Cluster & points,
  double & x, double & y, double & radius)
{
  if (points.size() < 5) {return false;}
  double xx = 0, xy = 0, yy = 0, xr = 0, yr = 0;
  for (const auto & p : points) {
    const double u = p.x - x, v = p.y - y, r2 = u * u + v * v;
    xx += u * u; xy += u * v; yy += v * v;
    xr += u * r2 / 2; yr += v * r2 / 2;
  }
  const double det = xx * yy - xy * xy;
  if (det < 1e-8) {return false;}
  const double cx = x + (xr * yy - yr * xy) / det;
  const double cy = y + (yr * xx - xr * xy) / det;
  double r = 0, error = 0;
  for (const auto & p : points) {r += std::hypot(p.x - cx, p.y - cy);}
  r /= points.size();
  if (r < .08 || r > .45 || std::hypot(cx - x, cy - y) > .30) {return false;}
  for (const auto & p : points) {
    const double residual = std::hypot(p.x - cx, p.y - cy) - r;
    error += residual * residual;
  }
  const double rms = std::sqrt(error / points.size());
  if (rms > .04) {return false;}
  x = cx; y = cy; radius = r + rms;
  return true;
}

double footprintDistance(double x, double y,
  const PredictiveObstacleCritic::Cluster & footprint)
{
  bool inside = false;
  double distance = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0, j = footprint.size() - 1; i < footprint.size(); j = i++) {
    const auto & a = footprint[j]; const auto & b = footprint[i];
    if ((a.y > y) != (b.y > y) && x < (b.x - a.x) * (y - a.y) / (b.y - a.y) + a.x) {
      inside = !inside;
    }
    const double dx = b.x - a.x, dy = b.y - a.y;
    const double length_sq = dx * dx + dy * dy;
    const double t = length_sq > 0 ? std::clamp(
      ((x - a.x) * dx + (y - a.y) * dy) / length_sq, 0.0, 1.0) : 0;
    distance = std::min(distance, std::hypot(x - a.x - t * dx, y - a.y - t * dy));
  }
  return inside ? 0.0 : distance;
}
}  // namespace

void PredictiveObstacleCritic::onInit()
{
  auto node = node_.lock();
  if (!node) {throw std::runtime_error("Predictive obstacle node expired");}
  clock_ = node->get_clock();
  const auto get = [&](const std::string & name, double value) {
      const std::string key = dwb_plugin_name_ + "." + name_ + "." + name;
      nav2_util::declare_parameter_if_not_declared(node, key, rclcpp::ParameterValue(value));
      const double result = node->get_parameter(key).as_double();
      if (!std::isfinite(result) || result <= 0) {throw std::runtime_error(key + " must be positive");}
      return result;
    };
  horizon_ = get("prediction_horizon", 1.0);
  margin_ = get("prediction_margin", .04);
  min_speed_ = get("minimum_speed", .08);
  max_speed_ = get("maximum_speed", .8);
  max_cluster_extent_ = get("maximum_cluster_extent", .8);
  association_distance_ = get("association_distance", .5);
  if (min_speed_ >= max_speed_) {throw std::runtime_error("Invalid predictive speed range");}
}

void PredictiveObstacleCritic::updateTracks(const std::vector<Cluster> & clusters, double now)
{
  const double elapsed = now - last_update_;
  if (last_update_ < 0 || elapsed <= 0 || elapsed > 1.0) {tracks_.clear();}
  std::vector<bool> used(tracks_.size(), false);
  std::vector<Track> next;
  for (const auto & points : clusters) {
    if (points.empty()) {continue;}
    Track track;
    track.points = points;
    for (const auto & p : points) {track.x += p.x; track.y += p.y;}
    track.x /= points.size(); track.y /= points.size();
    double extent_radius = .15;
    for (const auto & p : points) {
      extent_radius = std::max(extent_radius, std::hypot(p.x - track.x, p.y - track.y));
    }
    track.radius = extent_radius;
    fitCircle(points, track.x, track.y, track.radius);
    double closest = association_distance_;
    std::size_t matched = tracks_.size();
    for (std::size_t i = 0; i < tracks_.size(); ++i) {
      if (used[i]) {continue;}
      const auto & previous = tracks_[i];
      const double distance = std::hypot(track.x - previous.x - previous.vx * elapsed,
        track.y - previous.y - previous.vy * elapsed);
      if (distance < closest) {closest = distance; matched = i;}
    }
    if (matched < tracks_.size() && elapsed > 1e-4) {
      used[matched] = true;
      const auto & previous = tracks_[matched];
      track.last_motion = previous.last_motion;
      track.history = previous.history;
      geometry_msgs::msg::Point observation; observation.x = track.x; observation.y = track.y;
      track.history.emplace_back(now, observation);
      track.history.erase(std::remove_if(track.history.begin(), track.history.end(),
          [now](const auto & p) {return p.first < now - .6;}), track.history.end());
      double mean_t = 0, mean_x = 0, mean_y = 0;
      for (const auto & h : track.history) {
        mean_t += h.first; mean_x += h.second.x; mean_y += h.second.y;
      }
      mean_t /= track.history.size(); mean_x /= track.history.size(); mean_y /= track.history.size();
      double variance = 0, covariance_x = 0, covariance_y = 0;
      for (const auto & h : track.history) {
        const double t = h.first - mean_t;
        variance += t * t;
        covariance_x += t * (h.second.x - mean_x); covariance_y += t * (h.second.y - mean_y);
      }
      const double vx = variance > 1e-8 ? covariance_x / variance : 0;
      const double vy = variance > 1e-8 ? covariance_y / variance : 0;
      // Reject implausible associations instead of extrapolating a jumped track.
      if (std::hypot(vx, vy) <= max_speed_ * 1.5) {
        track.vx = .5 * previous.vx + .5 * vx;
        track.vy = .5 * previous.vy + .5 * vy;
        const double speed = std::hypot(track.vx, track.vy);
        if (speed > max_speed_) {track.vx *= max_speed_ / speed; track.vy *= max_speed_ / speed;}
        if (speed >= min_speed_) {
          track.moving_updates = previous.moving_updates + 1;
          track.last_motion = now;
        } else if (previous.moving_updates >= 2 && now - previous.last_motion < 2.0) {
          // Keep a recently moving object's clearance envelope through a brief
          // pause / reversal instead of immediately driving into its corridor.
          track.moving_updates = previous.moving_updates;
        }
      }
    }
    if (track.history.empty()) {
      geometry_msgs::msg::Point observation; observation.x = track.x; observation.y = track.y;
      track.history.emplace_back(now, observation);
    }
    next.push_back(std::move(track));
  }
  tracks_ = std::move(next);
  last_update_ = now;
  prediction_age_ = 0;
}

bool PredictiveObstacleCritic::prepare(
  const geometry_msgs::msg::Pose2D & pose, const nav_2d_msgs::msg::Twist2D &,
  const geometry_msgs::msg::Pose2D &, const nav_2d_msgs::msg::Path2D &)
{
  footprint_ = costmap_ros_->getRobotFootprint();
  const double now = clock_->now().seconds();
  prediction_age_ = std::max(0.0, now - last_update_);
  if (last_update_ >= 0 && now >= last_update_ && prediction_age_ < .09) {return true;}
  const auto * map = costmap_ros_->getCostmap();
  const int width = map->getSizeInCellsX(), height = map->getSizeInCellsY();
  std::vector<bool> visited(width * height, false);
  std::vector<Cluster> clusters;
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const auto index = y * width + x;
      if (visited[index] || map->getCost(x, y) != 254) {continue;}
      visited[index] = true;
      std::vector<int> queue{index};
      Cluster cluster;
      double min_x = std::numeric_limits<double>::infinity(), min_y = min_x;
      double max_x = -min_x, max_y = -min_x;
      for (std::size_t i = 0; i < queue.size(); ++i) {
        const int cx = queue[i] % width, cy = queue[i] / width;
        geometry_msgs::msg::Point point;
        map->mapToWorld(cx, cy, point.x, point.y);
        cluster.push_back(point);
        min_x = std::min(min_x, point.x); max_x = std::max(max_x, point.x);
        min_y = std::min(min_y, point.y); max_y = std::max(max_y, point.y);
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dx = -1; dx <= 1; ++dx) {
            const int nx = cx + dx, ny = cy + dy;
            if (nx < 0 || ny < 0 || nx >= width || ny >= height) {continue;}
            const auto n = ny * width + nx;
            if (!visited[n] && map->getCost(nx, ny) == 254) {
              visited[n] = true; queue.push_back(n);
            }
          }
        }
      }
      // Long walls remain in ordinary footprint collision checking; only
      // compact clusters with repeated motion evidence receive extrapolation.
      if (cluster.size() >= 3 && max_x - min_x <= max_cluster_extent_ &&
        max_y - min_y <= max_cluster_extent_ &&
        std::hypot((max_x + min_x) / 2 - pose.x, (max_y + min_y) / 2 - pose.y) <= 2.5)
      {
        clusters.push_back(std::move(cluster));
      }
    }
  }
  updateTracks(clusters, now);
  return true;
}

double PredictiveObstacleCritic::scoreTrajectory(const dwb_msgs::msg::Trajectory2D & trajectory)
{
  if (footprint_.size() < 3) {throw dwb_core::IllegalTrajectoryException(name_, "Empty footprint");}
  if (std::none_of(tracks_.begin(), tracks_.end(),
      [](const Track & track) {return track.moving_updates >= 2;})) {return 0;}
  const auto count = std::min(trajectory.poses.size(), trajectory.time_offsets.size());
  if (count == 0) {throw dwb_core::IllegalTrajectoryException(name_, "Missing trajectory timing");}
  // DWB distance-based discretization can contain only two timestamps for a
  // stopped robot. Resample TIME so an approaching obstacle cannot disappear
  // between those endpoints, even when the candidate command is zero.
  std::vector<std::pair<geometry_msgs::msg::Pose2D, double>> samples;
  double minimum_clearance = std::numeric_limits<double>::infinity();
  const double end = std::min(horizon_, rclcpp::Duration(trajectory.time_offsets[count - 1]).seconds());
  std::size_t index = 0;
  for (double time = 0; time <= end + 1e-6; time += .1) {
    while (index + 1 < count &&
      rclcpp::Duration(trajectory.time_offsets[index + 1]).seconds() < time) {++index;}
    auto pose = trajectory.poses[index];
    if (index + 1 < count) {
      const double before = rclcpp::Duration(trajectory.time_offsets[index]).seconds();
      const double after = rclcpp::Duration(trajectory.time_offsets[index + 1]).seconds();
      const double f = after > before ? std::clamp((time - before) / (after - before), 0.0, 1.0) : 0;
      const auto & next = trajectory.poses[index + 1];
      pose.x += f * (next.x - pose.x); pose.y += f * (next.y - pose.y);
      pose.theta += f * std::atan2(std::sin(next.theta - pose.theta), std::cos(next.theta - pose.theta));
    }
    samples.emplace_back(pose, time);
  }
  for (const auto & track : tracks_) {
    if (track.moving_updates < 2) {continue;}
    for (const auto & sample : samples) {
      const double time = sample.second;
      const auto & pose = sample.first;
      const double c = std::cos(pose.theta), s = std::sin(pose.theta);
      {
        const double dx = track.x + track.vx * (time + prediction_age_) - pose.x;
        const double dy = track.y + track.vy * (time + prediction_age_) - pose.y;
        if (std::abs(dx) > 1.5 || std::abs(dy) > 1.5) {continue;}
        const double clearance = footprintDistance(dx * c + dy * s, -dx * s + dy * c, footprint_) - track.radius;
        minimum_clearance = std::min(minimum_clearance, clearance);
        if (clearance <= margin_) {
          throw dwb_core::IllegalTrajectoryException(name_, "Predicted moving obstacle intersection");
        }
      }
    }
  }
  // A soft preference acts BEFORE intersection becomes unavoidable. Hard
  // collision rejection alone can brake too late for a returning pedestrian.
  return std::max(0.0, .35 - minimum_clearance);
}
}  // namespace dog_nav

PLUGINLIB_EXPORT_CLASS(dog_nav::PredictiveObstacleCritic, dwb_core::TrajectoryCritic)
