#!/usr/bin/env python3
"""
Robot navigation bridge node.
Bridges between coordinator commands and Nav2 navigation stack.
"""

from typing import Optional
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Bool
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus


class RobotNavBridge(Node):
    """
    Navigation bridge between coordinator and Nav2.
    
    Responsibilities:
    - Subscribe to goal commands from coordinator
    - Send NavigateToPose action to Nav2
    - Publish goal completion status back to coordinator
    """
    
    def __init__(self):
        """Initialize robot navigation bridge."""
        super().__init__('robot_nav_bridge')
        
        # Declare parameters
        self.declare_parameter('robot_namespace', 'robot_2')
        
        # Get parameters
        robot_namespace = self.get_parameter('robot_namespace').value
        
        # Extract robot_id from namespace (e.g., "robot_2" -> "2")
        self.robot_id = robot_namespace.split('_')[-1]
        self.robot_namespace = robot_namespace
        
        # State tracking
        self.current_goal_handle: Optional[NavigateToPose.Goal] = None
        self.is_navigating = False
        
        # Callback groups: ReentrantCallbackGroup for action callbacks to avoid deadlock
        # when action result arrives while processing feedback or new goal
        self.action_callback_group = ReentrantCallbackGroup()
        # MutuallyExclusiveCallbackGroup for goal subscriber to serialize incoming goals
        self.goal_callback_group = MutuallyExclusiveCallbackGroup()
        
        # QoS profile for topics (match coordinator settings)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Create subscriber for goal commands
        goal_topic = f'/robot_{self.robot_id}/nav/goal'
        self.goal_sub = self.create_subscription(
            Pose,
            goal_topic,
            self._goal_callback,
            qos_profile,
            callback_group=self.goal_callback_group
        )
        
        # Create publisher for goal completion
        goal_reached_topic = f'/robot_{self.robot_id}/nav/goal_reached'
        self.goal_reached_pub = self.create_publisher(
            Bool,
            goal_reached_topic,
            qos_profile
        )
        
        # Create action client for Nav2
        action_name = f'/{self.robot_namespace}/navigate_to_pose'
        self.nav_action_client = ActionClient(
            self,
            NavigateToPose,
            action_name,
            callback_group=self.action_callback_group
        )
        
        self.get_logger().info(f"Robot Nav Bridge initialized for {robot_namespace}")
        self.get_logger().info(f"Subscribing to: {goal_topic}")
        self.get_logger().info(f"Publishing to: {goal_reached_topic}")
        self.get_logger().info(f"Action client: {action_name}")
    
    def _goal_callback(self, msg: Pose) -> None:
        """
        Handle incoming goal from coordinator.
        
        Args:
            msg: Goal pose from coordinator
        """
        if self.is_navigating:
            self.get_logger().warn(
                "Already navigating to a goal. Ignoring new goal request."
            )
            return
        
        self.get_logger().info(f"Received new goal: ({msg.position.x}, {msg.position.y})")
        
        # Wait for action server with retry (Nav2 may take time to start)
        max_retries = 3
        retry_timeout = 10.0
        for attempt in range(max_retries):
            if self.nav_action_client.wait_for_server(timeout_sec=retry_timeout):
                break
            self.get_logger().warn(
                f"NavigateToPose action server not available (attempt {attempt + 1}/{max_retries})"
            )
        else:
            self.get_logger().error(
                f"NavigateToPose action server not available after {max_retries} attempts!"
            )
            self._publish_goal_reached(False)
            return
        
        # Convert Pose to PoseStamped for action
        goal_pose = self._create_pose_stamped(msg)
        
        # Send goal to Nav2
        self._send_navigation_goal(goal_pose)
    
    def _create_pose_stamped(self, pose: Pose) -> PoseStamped:
        """
        Convert Pose to PoseStamped with frame and timestamp.
        
        Args:
            pose: Input pose
            
        Returns:
            PoseStamped with proper frame_id and timestamp
        """
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = f'robot_{self.robot_id}/map'
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose = pose
        
        return pose_stamped
    
    def _send_navigation_goal(self, goal_pose: PoseStamped) -> None:
        """
        Send NavigateToPose action goal to Nav2.
        
        Args:
            goal_pose: Target pose for navigation
        """
        # Create action goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        
        self.get_logger().info("Sending goal to Nav2...")
        self.is_navigating = True
        
        # Send goal with result callback only
        send_goal_future = self.nav_action_client.send_goal_async(goal_msg)
        
        send_goal_future.add_done_callback(self._goal_response_callback)
    
    def _goal_response_callback(self, future) -> None:
        """
        Handle goal acceptance/rejection from Nav2.
        
        Args:
            future: Future containing goal handle
        """
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error("Goal was rejected by Nav2!")
            self.is_navigating = False
            self._publish_goal_reached(False)
            return
        
        self.get_logger().info("Goal accepted by Nav2")
        self.current_goal_handle = goal_handle
        
        # Get result asynchronously
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)
    
    def _result_callback(self, future) -> None:
        """
        Handle result from NavigateToPose action.
        
        Called after BT completes (including all recoveries).
        
        Args:
            future: Future containing action result
        """
        try:
            result = future.result()
            status = result.status
            
            # Check if navigation succeeded
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info("Navigation succeeded!")
                self._publish_goal_reached(True)
            else:
                # Handle all failure cases: ABORTED, CANCELED, etc.
                self.get_logger().error(f"Navigation failed with status: {status}")
                self._publish_goal_reached(False)
            
        except Exception as e:
            self.get_logger().error(f"Exception in result callback: {e}")
            self._publish_goal_reached(False)
        
        finally:
            # Reset state
            self.is_navigating = False
            self.current_goal_handle = None
    
    def _publish_goal_reached(self, success: bool) -> None:
        """
        Publish goal completion status to coordinator.
        
        Args:
            success: True if goal reached, False otherwise
        """
        msg = Bool()
        msg.data = success
        self.goal_reached_pub.publish(msg)
        
        self.get_logger().info(f"Published goal_reached: {success}")


def main(args=None):
    """Main entry point for robot navigation bridge."""
    rclpy.init(args=args)
    
    try:
        bridge = RobotNavBridge()
        
        # Use MultiThreadedExecutor to handle action callbacks concurrently
        # with goal subscriber without blocking
        executor = MultiThreadedExecutor()
        executor.add_node(bridge)
        
        try:
            executor.spin()
        finally:
            executor.shutdown()
            bridge.destroy_node()
    
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error in robot_nav_bridge: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
