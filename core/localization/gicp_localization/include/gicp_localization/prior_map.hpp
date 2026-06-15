#pragma once
#include <string>
#include <vector>
#include <Eigen/Core>

namespace gicp_localization {

// 加载 PCD 为齐次点 (w=1)。失败或空抛 std::runtime_error。
std::vector<Eigen::Vector4f> loadPriorMapPcd(const std::string& path);

}  // namespace gicp_localization
