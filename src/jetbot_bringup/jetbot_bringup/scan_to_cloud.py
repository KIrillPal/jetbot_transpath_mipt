#!/usr/bin/env python3

from laser_geometry import LaserProjection
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2


class ScanToCloudNode(Node):
    """Convert LaserScan to PointCloud2 for KISS-ICP."""

    def __init__(self) -> None:
        super().__init__("scan_to_cloud")
        self.declare_parameter("scan_topic", "scan")
        self.declare_parameter("cloud_topic", "scan/points")
        self.declare_parameter("target_frame", "")

        self.scan_topic = self.get_parameter("scan_topic").value
        self.cloud_topic = self.get_parameter("cloud_topic").value
        self.target_frame = self.get_parameter("target_frame").value

        self.projector = LaserProjection()
        self.cloud_pub = self.create_publisher(PointCloud2, self.cloud_topic, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, self.scan_topic, self._scan_callback, qos_profile_sensor_data
        )
        self.get_logger().info(
            f"Converting LaserScan '{self.scan_topic}' -> PointCloud2 '{self.cloud_topic}'"
        )

    def _scan_callback(self, scan: LaserScan) -> None:
        cloud = self.projector.projectLaser(scan)
        if self.target_frame:
            cloud.header.frame_id = self.target_frame
        self.cloud_pub.publish(cloud)


def main() -> None:
    rclpy.init()
    node = ScanToCloudNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
