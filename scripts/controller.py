#!/usr/bin/env python3

"""ROS 2 bridge that acknowledges actions dispatched by the ISL runtime."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import actions


ISL_ACTION_TOPIC = "/isl_send"
ISL_CONFIRMATION_TOPIC = "/isl_receive"


class ISLController(Node):
    """Receive an ISL action and confirm that it has been handled."""

    def __init__(self):
        super().__init__("isl_controller")

        self._confirmation_publisher = self.create_publisher(
            String,
            ISL_CONFIRMATION_TOPIC,
            10,
        )
        self._action_subscription = self.create_subscription(
            String,
            ISL_ACTION_TOPIC,
            self._action_callback,
            10,
        )

        self.get_logger().info(
            f"Waiting for ISL actions on {ISL_ACTION_TOPIC}"
        )

    def _action_callback(self, message: String) -> None:
        """Handle one action and echo it to ISL as its confirmation."""
        action = message.data.strip()
        if not action:
            self.get_logger().warning("Ignoring an empty ISL action")
            return

        self.get_logger().info(f"Received ISL action: {action}")

        symbol, separator, arguments = action.partition("(")
        symbol = symbol.strip()
        if not separator or not symbol or not arguments.endswith(")"):
            self.get_logger().error(f"Invalid ISL action: {action}")
            return

        parameters_text = arguments[:-1].strip()
        parameters = (
            [parameter.strip() for parameter in parameters_text.split(",")]
            if parameters_text
            else []
        )

        if symbol == "moveTo":
            actions.moveTo(*parameters)
        else:
            self.get_logger().error(f"Unknown ISL action: {symbol}")

    def _action_finished_callback(self, action: String) -> None:
        """Once an action has finished, send back to ISL"""
        confirmation = String()
        confirmation.data = action
        self._confirmation_publisher.publish(confirmation)
        self.get_logger().info(f"Confirmed ISL action: {action}")


def main(args=None):
    rclpy.init(args=args)
    node = ISLController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
