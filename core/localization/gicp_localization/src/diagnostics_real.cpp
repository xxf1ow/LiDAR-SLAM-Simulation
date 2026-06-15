#include "gicp_localization/diagnostics.hpp"

#include <rclcpp/rclcpp.hpp>
#include "gicp_localization/msg/gicp_diagnostics.hpp"

namespace gicp_localization {
namespace {
class RealDiagnostics : public Diagnostics {
public:
  explicit RealDiagnostics(rclcpp::Node* node) : node_(node) {
    pub_ = node_->create_publisher<msg::GicpDiagnostics>("~/diagnostics", 10);
  }
  void report(const GicpMetrics& m) override {
    msg::GicpDiagnostics out;
    out.stamp = node_->now();
    out.fitness = m.fitness;
    out.mean_residual = m.mean_residual;
    out.num_inliers = static_cast<uint32_t>(m.num_inliers);
    out.num_source = static_cast<uint32_t>(m.num_source);
    out.iterations = m.iterations;
    out.converged = m.converged;
    out.correction_delta_trans = m.correction_delta_trans;
    out.correction_delta_rot = m.correction_delta_rot;
    out.accepted = m.accepted;
    pub_->publish(out);
  }
private:
  rclcpp::Node* node_;
  rclcpp::Publisher<msg::GicpDiagnostics>::SharedPtr pub_;
};
}  // namespace

std::unique_ptr<Diagnostics> make_diagnostics(rclcpp::Node* node) {
  return std::make_unique<RealDiagnostics>(node);
}
}  // namespace gicp_localization
