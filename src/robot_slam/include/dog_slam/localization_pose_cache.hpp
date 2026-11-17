#pragma once

#include <cstdint>
#include <filesystem>
#include <string>

#include <Eigen/Geometry>

namespace dog_slam
{

struct CachedLocalizationPose
{
  std::string map_id;
  std::string map_path;
  std::string map_frame;
  std::string base_frame;
  std::int64_t stamp_nanoseconds{0};
  Eigen::Isometry3d map_from_base = Eigen::Isometry3d::Identity();

  bool matches(
    const std::string & id, const std::string & path,
    const std::string & frame, const std::string & base) const;
};

// Versioned text, full precision pose; throws on malformed input or I/O errors.
CachedLocalizationPose readLocalizationPose(const std::filesystem::path & path);
// Sync the temporary file, atomically rename it, then sync its directory.
void writeLocalizationPose(
  const std::filesystem::path & path, const CachedLocalizationPose & pose);

}  // namespace dog_slam
