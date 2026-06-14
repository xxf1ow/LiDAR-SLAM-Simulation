#include "gicp_localization/prior_map.hpp"

#include <stdexcept>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

namespace gicp_localization {

std::vector<Eigen::Vector4f> loadPriorMapPcd(const std::string& path) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  if (pcl::io::loadPCDFile(path, cloud) < 0) {
    throw std::runtime_error("failed to load prior map PCD: " + path);
  }
  if (cloud.empty()) {
    throw std::runtime_error("prior map PCD is empty: " + path);
  }
  std::vector<Eigen::Vector4f> pts;
  pts.reserve(cloud.size());
  for (const auto& p : cloud.points) {
    pts.emplace_back(p.x, p.y, p.z, 1.0f);
  }
  return pts;
}

}  // namespace gicp_localization
