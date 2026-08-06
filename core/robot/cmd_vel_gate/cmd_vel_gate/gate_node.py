import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String
from std_srvs.srv import Trigger

from cmd_vel_gate.gate_logic import GateState, Mode


SOURCE_TIMEOUT = 0.5
ZERO_PERIOD = 0.05


class CmdVelGate(Node):
    def __init__(self) -> None:
        super().__init__("cmd_vel_gate")
        self._state = GateState()
        self._publisher = self.create_publisher(TwistStamped, "/cmd_vel", 1)
        mode_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._mode_publisher = self.create_publisher(
            String,
            "/cmd_vel_gate/mode",
            mode_qos,
        )
        self._publish_mode(Mode.AUTOMATIC)
        self._automatic_subscription = self.create_subscription(
            TwistStamped,
            "/cmd_vel_auto",
            self._automatic_callback,
            1,
        )
        self._manual_subscription = self.create_subscription(
            TwistStamped,
            "/cmd_vel_manual",
            self._manual_callback,
            1,
        )
        self._manual_service = self.create_service(
            Trigger,
            "/cmd_vel_gate/takeover_manual",
            self._takeover_manual,
        )
        self._automatic_service = self.create_service(
            Trigger,
            "/cmd_vel_gate/resume_automatic",
            self._resume_automatic,
        )
        self._zero_timer = self.create_timer(ZERO_PERIOD, self._on_timer)

    def _automatic_callback(self, message: TwistStamped) -> None:
        self._forward(Mode.AUTOMATIC, message)

    def _manual_callback(self, message: TwistStamped) -> None:
        self._forward(Mode.MANUAL, message)

    def _now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds / 1_000_000_000.0

    def _forward(self, source: Mode, message: TwistStamped) -> None:
        if self._state.accept(source, self._now_seconds()):
            output = self._new_output()
            output.twist = message.twist
            self._publisher.publish(output)

    def _new_output(self) -> TwistStamped:
        output = TwistStamped()
        output.header.stamp = self.get_clock().now().to_msg()
        output.header.frame_id = "base_link"
        return output

    def _publish_zero(self) -> None:
        self._publisher.publish(self._new_output())

    def _publish_mode(self, mode: Mode) -> None:
        self._mode_publisher.publish(String(data=mode.value))

    def _on_timer(self) -> None:
        if self._state.selected_source_is_stale(
            self._now_seconds(),
            SOURCE_TIMEOUT,
        ):
            self._publish_zero()

    def _switch(self, target: Mode, response):
        self._state.stop()
        self._publish_zero()
        self._state.select(target)
        self._publish_mode(target)
        response.success = True
        response.message = target.value
        return response

    def _takeover_manual(self, _request, response):
        return self._switch(Mode.MANUAL, response)

    def _resume_automatic(self, _request, response):
        return self._switch(Mode.AUTOMATIC, response)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdVelGate()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
