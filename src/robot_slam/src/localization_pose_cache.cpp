#include "dog_slam/localization_pose_cache.hpp"

#include <cerrno>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <system_error>
#include <vector>

#include <fcntl.h>
#include <unistd.h>

namespace dog_slam
{
namespace
{
void validate(const CachedLocalizationPose & pose)
{
  if (pose.map_id.empty() || pose.map_path.empty() || pose.map_frame.empty() ||
    pose.base_frame.empty() || pose.stamp_nanoseconds <= 0 ||
    !pose.map_from_base.matrix().allFinite() ||
    !pose.map_from_base.linear().transpose().isApprox(pose.map_from_base.linear().inverse()) ||
    std::abs(pose.map_from_base.linear().determinant() - 1.0) > 1e-6)
  {
    throw std::runtime_error("invalid cached localization pose");
  }
}

void ioError(const char * operation)
{
  throw std::system_error(errno, std::generic_category(), operation);
}
}  // namespace

bool CachedLocalizationPose::matches(
  const std::string & id, const std::string & path,
  const std::string & frame, const std::string & base) const
{
  return map_id == id && map_path == path && map_frame == frame && base_frame == base;
}

CachedLocalizationPose readLocalizationPose(const std::filesystem::path & path)
{
  std::ifstream input(path);
  CachedLocalizationPose pose;
  std::string magic;
  int version = 0;
  Eigen::Vector3d position;
  double x, y, z, w;
  if (!(input >> magic >> version) || magic != "DOG_LOCALIZATION_POSE" || version != 1 ||
    !(input >> std::quoted(pose.map_id) >> std::quoted(pose.map_path) >>
    std::quoted(pose.map_frame) >> std::quoted(pose.base_frame) >> pose.stamp_nanoseconds >>
    position.x() >> position.y() >> position.z() >> x >> y >> z >> w))
  {
    throw std::runtime_error("unreadable or incomplete localization pose cache");
  }
  Eigen::Quaterniond rotation(w, x, y, z);
  if (!rotation.coeffs().allFinite() || std::abs(rotation.norm() - 1.0) > 1e-6) {
    throw std::runtime_error("invalid cached localization quaternion");
  }
  input >> std::ws;
  if (!input.eof()) {
    throw std::runtime_error("unexpected data in localization pose cache");
  }
  pose.map_from_base.linear() = rotation.normalized().toRotationMatrix();
  pose.map_from_base.translation() = position;
  validate(pose);
  return pose;
}

void writeLocalizationPose(
  const std::filesystem::path & path, const CachedLocalizationPose & pose)
{
  validate(pose);
  const auto directory = path.has_parent_path() ? path.parent_path() : std::filesystem::path(".");
  std::filesystem::create_directories(directory);
  const Eigen::Quaterniond q(pose.map_from_base.linear());
  const auto & p = pose.map_from_base.translation();
  std::ostringstream output;
  output << std::setprecision(17) << "DOG_LOCALIZATION_POSE 1\n" <<
    std::quoted(pose.map_id) << '\n' << std::quoted(pose.map_path) << '\n' <<
    std::quoted(pose.map_frame) << ' ' << std::quoted(pose.base_frame) << '\n' <<
    pose.stamp_nanoseconds << '\n' << p.x() << ' ' << p.y() << ' ' << p.z() << '\n' <<
    q.x() << ' ' << q.y() << ' ' << q.z() << ' ' << q.w() << '\n';
  const auto data = output.str();
  const std::string pattern = path.string() + ".XXXXXX";
  std::vector<char> temporary(pattern.begin(), pattern.end());
  temporary.push_back('\0');
  int fd = ::mkstemp(temporary.data());
  if (fd < 0) {
    ioError("create localization pose temporary file");
  }
  try {
    std::size_t written = 0;
    while (written < data.size()) {
      const auto count = ::write(fd, data.data() + written, data.size() - written);
      if (count < 0 && errno == EINTR) {
        continue;
      }
      if (count <= 0) {
        ioError("write localization pose");
      }
      written += static_cast<std::size_t>(count);
    }
    if (::fsync(fd) != 0) {
      ioError("sync localization pose");
    }
    const int result = ::close(fd);
    fd = -1;
    if (result != 0) {
      ioError("close localization pose");
    }
    std::filesystem::rename(temporary.data(), path);
    fd = ::open(directory.c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC);
    if (fd < 0 || ::fsync(fd) != 0) {
      ioError("sync localization pose directory");
    }
    ::close(fd);
  } catch (...) {
    if (fd >= 0) {
      ::close(fd);
    }
    ::unlink(temporary.data());
    throw;
  }
}

}  // namespace dog_slam
