#include "gicp_localization/diagnostics.hpp"

namespace gicp_localization {
namespace {
class NullDiagnostics : public Diagnostics {
public:
  void report(const GicpMetrics&) override {}
};
}  // namespace

std::unique_ptr<Diagnostics> make_diagnostics(rclcpp::Node*) {
  return std::make_unique<NullDiagnostics>();
}
}  // namespace gicp_localization
