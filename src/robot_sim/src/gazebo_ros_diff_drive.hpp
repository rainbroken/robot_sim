// Workspace-local wrapper for the BSD-licensed differential-drive implementation.
#pragma once

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Model.hh>
#include <memory>

namespace robot_sim
{
class GazeboRosDiffDrivePrivate;

class GazeboRosDiffDrive : public gazebo::ModelPlugin
{
public:
  GazeboRosDiffDrive();
  ~GazeboRosDiffDrive() override;

protected:
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;
  void Reset() override;

private:
  std::unique_ptr<GazeboRosDiffDrivePrivate> impl_;
};
}  // namespace robot_sim
