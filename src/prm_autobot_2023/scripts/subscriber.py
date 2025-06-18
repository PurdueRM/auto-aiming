#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import Bool, Int16, String
import time
import math

NAV_STATUS = {
    'NOT_STARTED': 'NOT_STARTED',
    'NAVIGATING': 'NAVIGATING',
    'IDLING': 'IDLING'
}


class Navigator(Node):
    def __init__(self):
        super().__init__('navigator')
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
        self.nav_status_pub = self.create_publisher(String, 'nav_status', 10)
        self.feedback = None
        self.goal_handle = None
        self.result_future = None
        self.initial_pose_received = False
        self.goal_sent_time = None

        qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=QoSReliabilityPolicy.RELIABLE, history=QoSHistoryPolicy.KEEP_LAST)
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self._amcl_pose_cb, qos)

    def _amcl_pose_cb(self, msg):
        self.initial_pose_received = True

    def set_initial_pose(self, pose: PoseWithCovarianceStamped):
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        self.get_logger().info('Setting initial pose')
        self.initial_pose_pub.publish(pose)

    def wait_for_initial_pose(self):
        while not self.initial_pose_received:
            self.get_logger().info('Waiting for amcl_pose...')
            rclpy.spin_once(self, timeout_sec=0.5)

    def send_goal(self, pose: PoseStamped):
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('NavigateToPose action server not available!')
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        self.get_logger().info(f'Sending goal to: {pose.pose.position}')
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.get_logger().warn('Goal rejected')
            return False

        self.result_future = self.goal_handle.get_result_async()
        self.goal_sent_time = time.time()
        return True

    def is_navigating(self):
        if not self.result_future:
            return False
        rclpy.spin_until_future_complete(self, self.result_future, timeout_sec=0.1)
        return not self.result_future.done()

    def cancel_goal(self):
        if self.goal_handle:
            cancel_future = self.goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel_future)
            self.get_logger().info("Canceled current goal")
            self.goal_handle = None
            self.result_future = None
            self.goal_sent_time = None
            time.sleep(1)

    def time_since_goal_sent(self):
        if self.goal_sent_time:
            return time.time() - self.goal_sent_time
        return None

    def publish_nav_status(self, status):
        self.nav_status_pub.publish(String(data=status))

####################################
### Pose Scheduler State Machine ###
####################################

class PoseSchedulerStateMachine(Node):
    GOAL_TIMEOUT = 30  # seconds

    def __init__(self, navigator: Navigator):
        super().__init__('pose_scheduler_sm')
        self.navigator = navigator

        self.match_started = False
        self.start_time = None
        self.low_health = False

        self.state = 'IDLE'
        self.current_pose_name = None
        self.current_pose_start_time = None

        self.named_poses = {
            "CENTER": self._make_pose([2.9, 1.0]),
            "FAR_WALL": self._make_pose([-0.583, 1.25]),
            "HOME": self._make_pose([-0.2, 0.0, 90]),
            "HALF_METER_FORWARD": self._make_pose([0.5, 0.0]),
            "HEAL": self._make_pose([0.7, -2.9, 90]),

            "FAR_RIGHT": self._make_pose([4.5, -8.0]),
            "INTERSECT": self._make_pose([3.29, 0.0]),
            "FAR": self._make_pose([8.0, 2.5]),
            "FURTHER": self._make_pose([14.5, 3.0]),
            "HALFWAY_RIGHT": self._make_pose([3.7, -3.5]),
        }

        self.pose_queue = {
            2: "INTERSECT",
            10: "FAR_RIGHT",
            20: "HALFWAY_RIGHT",
            30: "INTERSECT",
            40: "HOME",
            60: "FAR",
            70: "HOME"
        }

        self.override_pose_name = "HEAL"

        # Subscribers
        self.create_subscription(Bool, 'match_start', self._match_cb, 10)
        self.create_subscription(Int16, 'health', self._health_cb, 10)

        # Timer
        self.create_timer(0.5, self._tick)

    def _make_pose(self, lst):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = lst[0]
        pose.pose.position.y = lst[1]
        if len(lst) >= 3:
            yaw = lst[2] * math.pi / 180  # Convert degrees to radians
            pose.pose.orientation.z = math.sin(yaw / 2)
            pose.pose.orientation.w = math.cos(yaw / 2)
        else:
            pose.pose.orientation.w = 1.0  # No rotation
        return pose

    def _match_cb(self, msg):
        if msg.data and not self.match_started:
            self.get_logger().info("Match started")
            self.match_started = True
            self.start_time = time.time()

    def _health_cb(self, msg):
        self.low_health = msg.data < 300

    def _tick(self):
        if not self.match_started:
            self.get_logger().info("Publishing (Match not started)")
            self.navigator.publish_nav_status(NAV_STATUS['NOT_STARTED'])
            return

        if self.navigator.is_navigating():
            self.navigator.publish_nav_status(NAV_STATUS['NAVIGATING'])
        else:
            self.navigator.publish_nav_status(NAV_STATUS['IDLING'])

        if self.state == 'IDLE':
            if self.match_started:
                self.get_logger().info("Transitioning to NAVIGATING state")
                self.state = 'NAVIGATING'
                self.current_pose_start_time = None  # Reset
                self._send_next_pose()

        elif self.state == 'NAVIGATING':
            if self.low_health:
                self.get_logger().info("Low health detected! Switching to OVERRIDE state")
                self.navigator.cancel_goal()
                self.state = 'OVERRIDE'
                self._send_override_pose()
                return

            # Check for goal timeout
            if self._goal_timed_out():
                self.get_logger().warn("Goal timed out, cancelling and sending next")
                self.navigator.cancel_goal()
                self._send_next_pose()
                return

            # Check if it's time to send next pose
            if self._should_send_next_pose():
                self.get_logger().info("Time to send next scheduled pose")
                self.navigator.cancel_goal()
                self._send_next_pose()
                return

            # If navigation finished and no more poses, transition to DONE
            if not self.navigator.is_navigating() and not self.pose_queue:
                self.get_logger().info("All poses sent and navigation complete")
                self.state = 'DONE'
                return

        elif self.state == 'OVERRIDE':
            if not self.low_health:
                self.get_logger().info("Health recovered, returning to NAVIGATING")
                self.navigator.cancel_goal()
                self.state = 'NAVIGATING'
                self._send_next_pose()
                return

        elif self.state == 'DONE':
            self.navigator.publish_nav_status(NAV_STATUS['IDLING'])
            # Could add restart logic here if desired

        else:
            self.get_logger().warn(f"Unknown state: {self.state}")

    def _goal_timed_out(self):
        if self.current_pose_start_time is None:
            return False
        return (time.time() - self.current_pose_start_time) > self.GOAL_TIMEOUT

    def _should_send_next_pose(self):
        if not self.pose_queue:
            return False
        elapsed = time.time() - self.start_time
        next_time = min(self.pose_queue.keys())
        return elapsed >= next_time

    def _send_next_pose(self):
        if not self.pose_queue:
            self.get_logger().info("Pose queue empty, no next pose to send")
            return

        elapsed = time.time() - self.start_time
        # Find the next pose to send based on elapsed time
        next_times = sorted(t for t in self.pose_queue if t <= elapsed)
        if not next_times:
            return  # Not yet time for next pose

        next_time = next_times[0]
        pose_name = self.pose_queue.pop(next_time)

        if pose_name not in self.named_poses:
            self.get_logger().warn(f"Pose '{pose_name}' not found in named poses")
            return

        self.current_pose_name = pose_name
        pose = self.named_poses[pose_name]
        sent = self.navigator.send_goal(pose)
        if sent:
            self.get_logger().info(f"Sent pose '{pose_name}' scheduled at t={next_time}s")
            self.current_pose_start_time = time.time()
        else:
            self.get_logger().warn(f"Failed to send pose '{pose_name}'")

    def _send_override_pose(self):
        pose = self.named_poses[self.override_pose_name]
        sent = self.navigator.send_goal(pose)
        if sent:
            self.get_logger().info(f"Sent OVERRIDE pose '{self.override_pose_name}'")
            self.current_pose_start_time = time.time()
        else:
            self.get_logger().warn(f"Failed to send OVERRIDE pose '{self.override_pose_name}'")


def main(args=None):
    rclpy.init(args=args)

    navigator = Navigator()
    navigator.wait_for_initial_pose()

    scheduler = PoseSchedulerStateMachine(navigator)

    executor = MultiThreadedExecutor()
    executor.add_node(navigator)
    executor.add_node(scheduler)

    executor.spin()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
