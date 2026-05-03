#!/usr/bin/env python3
import argparse
import time

import rclpy
from geometry_msgs.msg import Twist


def publish_twist(node, publisher, linear_x, angular_z, duration, rate_hz):
    msg = Twist()
    msg.linear.x = linear_x
    msg.angular.z = angular_z

    period = 1.0 / rate_hz
    deadline = time.monotonic() + duration

    node.get_logger().info(
        f"Publishing linear.x={linear_x:.3f}, angular.z={angular_z:.3f} "
        f"for {duration:.2f}s"
    )

    while rclpy.ok() and time.monotonic() < deadline:
        publisher.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.0)
        time.sleep(period)


def publish_stop(publisher, repeats=10, delay=0.05):
    stop = Twist()
    for _ in range(repeats):
        publisher.publish(stop)
        time.sleep(delay)


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Send an obvious direct motor command to diffbot_base_controller. "
            "This bypasses joystick and twist_mux."
        )
    )
    parser.add_argument(
        "--namespace",
        default="robot_5",
        help="Robot namespace without leading slash (default: robot_5)",
    )
    parser.add_argument(
        "--linear",
        type=float,
        default=0.08,
        help="Forward linear velocity in m/s (default: 0.08)",
    )
    parser.add_argument(
        "--angular",
        type=float,
        default=0.0,
        help="Angular velocity in rad/s (default: 0.0)",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=1.0,
        help="Command duration in seconds (default: 1.0)",
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=20.0,
        help="Publish rate in Hz (default: 20.0)",
    )
    parser.add_argument(
        "--topic",
        default=None,
        help=(
            "Override command topic. By default uses "
            "/<namespace>/diffbot_base_controller/cmd_vel_unstamped"
        ),
    )

    args = parser.parse_args()
    namespace = args.namespace.strip("/")
    topic = args.topic or f"/{namespace}/diffbot_base_controller/cmd_vel_unstamped"

    rclpy.init()
    node = rclpy.create_node("motor_console_test")
    publisher = node.create_publisher(Twist, topic, 10)

    node.get_logger().warn(
        "Robot may move now. Keep it lifted or ready to stop. "
        f"Publishing directly to {topic}"
    )

    # Give DDS discovery a short moment so the first command is not dropped.
    time.sleep(0.5)

    try:
        publish_twist(
            node,
            publisher,
            args.linear,
            args.angular,
            args.duration,
            args.rate,
        )
    finally:
        node.get_logger().info("Sending stop command")
        publish_stop(publisher)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
