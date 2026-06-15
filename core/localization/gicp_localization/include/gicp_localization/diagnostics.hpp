#pragma once
#include <cstddef>
#include <memory>

namespace rclcpp { class Node; }  // 前向声明，OFF 构建不依赖 rclcpp 头

namespace gicp_localization {

struct GicpMetrics {
  double fitness = 0.0;
  double mean_residual = 0.0;
  std::size_t num_inliers = 0;
  std::size_t num_source = 0;
  int iterations = 0;
  bool converged = false;
  double correction_delta_trans = 0.0;
  double correction_delta_rot = 0.0;
  bool accepted = false;
};

class Diagnostics {
public:
  virtual ~Diagnostics() = default;
  virtual void report(const GicpMetrics& m) = 0;
};

// 在 diagnostics_real.cpp(ON) 或 diagnostics_null.cpp(OFF) 中定义，由 CMake 选编。
std::unique_ptr<Diagnostics> make_diagnostics(rclcpp::Node* node);

}  // namespace gicp_localization
