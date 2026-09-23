#ifndef XX_HTTP_ACTIONS_H_
#define XX_HTTP_ACTIONS_H_

#include "robot_web_ui/web_types.h"
#include <nlohmann/json.hpp>
#include <string>

namespace robot_web_ui
{
/** Expected HTTP outcomes, including validation and unavailable dependencies. */
struct ApiReply {
    int status;
    nlohmann::json body;
};

/** Internal HTTP substitution interface, borrowed from WebUiNode and never installed.
 * Calls are thread-safe. Mode calls wait at most one second for service confirmation.
 * ROS publication may block in the transport, without holding the node's shared-state mutex.
 * Parking calls perform synchronous I/O and return 503 when another parking operation is active.
 * Binary snapshots retain immutable ownership independently of the node. Stop all callers before
 * destroying the node; asynchronous ROS callbacks run on its single-threaded executor.
 */
class HttpActions {
public:
    virtual ~HttpActions() = default;
    [[nodiscard]] virtual nlohmann::json navigation_state() const = 0;
    [[nodiscard]] virtual nlohmann::json assistant_state() const = 0;
    /** Returns the latest tracking snapshot, or {"available":false} before one arrives. */
    [[nodiscard]] virtual nlohmann::json tracking_state() const = 0;
    /** Publishes finite numeric x/y in base_footprint; returns 400 for invalid input,
     * 503 without a target subscriber, or 202 after publication.
     */
    [[nodiscard]] virtual ApiReply publish_tracking_target(const nlohmann::json &payload) = 0;
    /** Returns shared immutable bytes, or null for an unknown or unavailable layer. */
    [[nodiscard]] virtual BinarySnapshotPtr navigation_asset(const std::string &name) const = 0;
    /** Publishes a direction at 0–100 percent; nonzero commands require manual mode.
     * Checks and returns mode as observed before publication.
     */
    [[nodiscard]] virtual ApiReply manual_command(const std::string &direction, double speed_percent) = 0;
    /** Returns 200 on confirmation, 202 while unconfirmed, or 503 on service unavailability/rejection. */
    [[nodiscard]] virtual ApiReply takeover_manual() = 0;
    /** Uses the same confirmation and timeout rules as takeover_manual. */
    [[nodiscard]] virtual ApiReply resume_automatic() = 0;
    /** Validates map pose/revision and reserves publication against active goals and other initial poses.
     * The reservation remains active through transport completion or failure; errors are 400/409/503.
     */
    [[nodiscard]] virtual ApiReply publish_initial_pose(const nlohmann::json &payload) = 0;
    /** Returns 202 after reserving and submitting a goal; acceptance and result arrive asynchronously. */
    [[nodiscard]] virtual ApiReply send_navigation_goal(const nlohmann::json &payload) = 0;
    /** Returns 202 after requesting cancellation of the accepted current goal, or 409/503. */
    [[nodiscard]] virtual ApiReply cancel_navigation() = 0;
    [[nodiscard]] virtual ApiReply list_parking_points() = 0;
    [[nodiscard]] virtual ApiReply save_parking_point(const std::string &name) = 0;
    [[nodiscard]] virtual ApiReply navigate_parking_point(const std::string &name) = 0;
    [[nodiscard]] virtual ApiReply delete_parking_point(const std::string &name) = 0;
};
} // namespace robot_web_ui

#endif // XX_HTTP_ACTIONS_H_
