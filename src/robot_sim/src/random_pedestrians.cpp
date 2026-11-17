// Local, offline humanoids with real ray/physics collisions, independent of ROS navigation.
#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <sdf/sdf.hh>

#include <algorithm>
#include <cmath>
#include <functional>
#include <random>
#include <sstream>
#include <string>
#include <vector>
#include "pedestrian_clearance.hpp"

namespace robot_sim
{
class RandomPedestrians : public gazebo::WorldPlugin
{
  struct Person
  {
    std::string name;
    gazebo::physics::ModelPtr model;
    ignition::math::Vector2d position, goal;
    double speed{0.5}, wait_until{0}, distance{0};
  };
  gazebo::physics::WorldPtr world_;
  gazebo::event::ConnectionPtr update_;
  std::mt19937 rng_;
  std::vector<ignition::math::AxisAlignedBox> obstacles_;
  std::vector<Person> people_;
  double min_x_{-5.5}, max_x_{5.5}, min_y_{-5.5}, max_y_{5.5};
  double last_time_{-1}, max_speed_{0.7};
  unsigned int count_{3};
  bool initialized_{false};
  static constexpr double clearance_ = 0.34;

  double Random(double lo, double hi)
  {
    return std::uniform_real_distribution<double>(lo, hi)(rng_);
  }

  bool Free(const ignition::math::Vector2d & p) const
  {
    if (p.X() < min_x_ || p.X() > max_x_ || p.Y() < min_y_ || p.Y() > max_y_)
      return false;
    for (const auto & b : obstacles_)
      if (p.X() >= b.Min().X() - clearance_ && p.X() <= b.Max().X() + clearance_ &&
          p.Y() >= b.Min().Y() - clearance_ && p.Y() <= b.Max().Y() + clearance_)
        return false;
    return true;
  }

  bool SegmentFree(const ignition::math::Vector2d & a,
                   const ignition::math::Vector2d & b) const
  {
    const int steps = std::max(1, static_cast<int>(std::ceil(a.Distance(b) / 0.08)));
    for (int i = 0; i <= steps; ++i)
      if (!Free(a + (b - a) * (static_cast<double>(i) / steps))) return false;
    return true;
  }

  std::string Model(const Person & p, unsigned int index) const
  {
    const char * colors[] = {"0.15 0.35 0.85 1", "0.85 0.22 0.12 1", "0.15 0.65 0.25 1"};
    std::ostringstream s;
    s << "<sdf version='1.6'><model name='" << p.name << "'><static>true</static><pose>"
      << p.position.X() << " " << p.position.Y() << " 0 0 0 0</pose>";
    const auto part = [&s](const std::string & name, const std::string & pose,
                          const std::string & geometry, const std::string & color) {
      s << "<link name='" << name << "'><pose>" << pose << "</pose>"
        << "<collision name='collision'><geometry>" << geometry << "</geometry></collision>"
        << "<visual name='visual'><geometry>" << geometry << "</geometry><material><ambient>"
        << color << "</ambient><diffuse>" << color << "</diffuse></material></visual></link>";
    };
    part("torso", "0 0 1.12 0 0 0", "<box><size>0.30 0.42 0.64</size></box>", colors[index % 3]);
    part("head", "0 0 1.60 0 0 0", "<sphere><radius>0.15</radius></sphere>", "0.85 0.65 0.45 1");
    part("left_leg", "0 0.115 0.40 0 0 0", "<cylinder><radius>0.075</radius><length>0.72</length></cylinder>", "0.12 0.14 0.20 1");
    part("right_leg", "0 -0.115 0.40 0 0 0", "<cylinder><radius>0.075</radius><length>0.72</length></cylinder>", "0.12 0.14 0.20 1");
    part("left_arm", "0 0.28 1.10 0 0 0", "<cylinder><radius>0.06</radius><length>0.60</length></cylinder>", colors[index % 3]);
    part("right_arm", "0 -0.28 1.10 0 0 0", "<cylinder><radius>0.06</radius><length>0.60</length></cylinder>", colors[index % 3]);
    s << "</model></sdf>";
    return s.str();
  }

  void Initialize()
  {
    // Cache each collision, not model bounds: the house is a single model including its floor.
    for (const auto & model : world_->Models())
    {
      if (!model->IsStatic()) continue;
      for (const auto & link : model->GetLinks())
        for (const auto & collision : link->GetCollisions())
        {
          const auto b = collision->BoundingBox();
          if (b.Max().Z() <= 0.08 && b.XLength() > 3 && b.YLength() > 3)
          {
            min_x_ = b.Min().X() + 0.5; max_x_ = b.Max().X() - 0.5;
            min_y_ = b.Min().Y() + 0.5; max_y_ = b.Max().Y() - 0.5;
          }
          // Ignore floor and high door lintels, retain walls and low furniture.
          if (b.Max().Z() > 0.10 && b.Min().Z() < 1.75) obstacles_.push_back(b);
        }
    }
    for (unsigned int i = 0; i < count_; ++i)
    {
      Person p;
      p.name = "dynamic_pedestrian_" + std::to_string(i + 1);
      bool found = false;
      for (int attempt = 0; attempt < 2000; ++attempt)
      {
        // First two start in the main circulation space in both bundled worlds.
        p.position = attempt == 0 && i < 2 ?
          ignition::math::Vector2d(i == 0 ? 3.0 : -2.0, 0.4) :
          ignition::math::Vector2d(Random(min_x_, max_x_), Random(min_y_, max_y_));
        if (!Free(p.position) || p.position.Length() < 1.2) continue;
        found = true;
        for (const auto & other : people_)
          if (p.position.Distance(other.position) < 0.9) found = false;
        if (found) break;
      }
      if (!found)
      {
        gzwarn << "[pedestrians] No free spawn space for " << p.name << "\n";
        break;
      }
      p.goal = p.position;
      p.speed = Random(0.6 * max_speed_, max_speed_);
      world_->InsertModelString(Model(p, i));
      people_.push_back(p);
    }
    gzmsg << "[pedestrians] Requested " << people_.size()
          << " collision-enabled humanoids; speed <= " << max_speed_ << " m/s\n";
    initialized_ = true;
  }

  void Update(const gazebo::common::UpdateInfo & info)
  {
    const double now = info.simTime.Double();
    if (!initialized_) { Initialize(); last_time_ = now; return; }
    if (now < last_time_)
    {
      for (auto & p : people_) { p.goal = p.position; p.wait_until = now; }
      last_time_ = now;
      return;
    }
    const double dt = now - last_time_;
    if (dt < 0.04) return;  // 25 Hz in simulation time; pausing Gazebo pauses pedestrians.
    last_time_ = now;
    const auto robot = world_->ModelByName("robot");
    for (auto & p : people_)
    {
      if (!p.model) p.model = world_->ModelByName(p.name);
      if (!p.model || now < p.wait_until) continue;
      if (p.position.Distance(p.goal) < 0.12)
      {
        bool found = false;
        for (int attempt = 0; attempt < 100; ++attempt)
        {
          const ignition::math::Vector2d goal(
            p.position.X() + Random(-3.5, 3.5), p.position.Y() + Random(-3.5, 3.5));
          if (goal.Distance(p.position) > 0.7 && SegmentFree(p.position, goal))
          { p.goal = goal; found = true; break; }
        }
        p.wait_until = now + Random(0.3, found ? 1.5 : 0.6);
        continue;
      }
      auto direction = p.goal - p.position;
      direction.Normalize();
      const auto next = p.position + direction * std::min(p.speed * std::min(dt, 0.1), p.position.Distance(p.goal));
      bool blocked = !SegmentFree(p.position, next);
      for (const auto & other : people_)
        if (other.name != p.name && next.Distance(other.position) < 0.70) blocked = true;
      if (robot)
      {
        const auto r = robot->WorldPose().Pos();
        const ignition::math::Vector2d robot_position(r.X(), r.Y());
        if (robotBlocksPedestrian(
            p.position.Distance(robot_position), next.Distance(robot_position))) blocked = true;
      }
      if (blocked) { p.goal = p.position; p.wait_until = now + Random(0.3, 0.8); continue; }
      p.distance += next.Distance(p.position);
      p.position = next;
      const double yaw = std::atan2(direction.Y(), direction.X());
      p.model->SetWorldPose({next.X(), next.Y(), 0, 0, 0, yaw});
      // Small leg swings preserve ground clearance; collision shapes follow the moving links.
      const double swing = 0.22 * std::sin(p.distance * 9.0);
      p.model->GetLink("left_leg")->SetRelativePose({0, 0.115, 0.40, 0, swing, 0});
      p.model->GetLink("right_leg")->SetRelativePose({0, -0.115, 0.40, 0, -swing, 0});
    }
  }

public:
  void Load(gazebo::physics::WorldPtr world, sdf::ElementPtr sdf) override
  {
    world_ = world;
    const auto count = sdf->Get<unsigned int>("count", 3).first;
    const auto seed = sdf->Get<unsigned int>("seed", 42).first;
    max_speed_ = sdf->Get<double>("max_speed", 0.7).first;
    if (count > 20 || !std::isfinite(max_speed_) || max_speed_ <= 0 || max_speed_ > 2.0)
    { gzerr << "[pedestrians] Invalid count (0..20) or max_speed (0..2 m/s)\n"; return; }
    count_ = count;
    rng_.seed(seed);
    update_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&RandomPedestrians::Update, this, std::placeholders::_1));
  }
};
GZ_REGISTER_WORLD_PLUGIN(RandomPedestrians)
}  // namespace robot_sim
