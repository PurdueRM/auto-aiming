#!/usr/bin/env python3

# ======================================================================
# CHANGE LEGEND
#   [NEW]      = line/block added (did not exist in your original)
#   [CHANGED]  = line/block that existed but was modified
#   [REMOVED]  = something deleted from your original (comment only, no code)
# Anything without a tag is unchanged from your original file.
# ======================================================================

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from action_msgs.msg import GoalStatus  # [NEW] used by Navigator.goal_succeeded()
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

    # [NEW] goal_succeeded() method (whole block below)
    def goal_succeeded(self):
        """True only if the most recent goal finished with SUCCEEDED."""
        if not self.result_future or not self.result_future.done():
            return False
        return self.result_future.result().status == GoalStatus.STATUS_SUCCEEDED
    # [NEW] end of goal_succeeded()

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
    # [NEW] class docstring describing the new states
    """
    States:
      WAITING - match not started, do nothing
      CENTER  - match running, go to / hold CENTER_ZONE
      HEALING - health is low, go to HOME and stay until healed, then back to CENTER
    """
    # [REMOVED] GOAL_TIMEOUT = 30  (old goal timeout; replaced by retry-on-failure)
    LOW_HEALTH_THRESHOLD = 150     # [CHANGED] comment added only; value unchanged
    HEALED_THRESHOLD = 370         # [NEW] was hardcoded as "370" inside _tick
    RETRY_DELAY = 1.0              # [NEW] seconds between retries if a goal fails

    CENTER_POSE = "CENTER_ZONE"    # [NEW]
    HEAL_POSE = "HOME"             # [NEW] replaces self.override_pose_name

    def __init__(self, navigator: Navigator):
        super().__init__('pose_scheduler_sm')
        self.navigator = navigator

        self.match_started = False
        # [REMOVED] self.start_time, self.low_health (now a property), self.override_sent
        self.health = None  # [CHANGED] replaces self.new_health

        self.state = 'WAITING'  # [CHANGED] was 'IDLE'
        # [REMOVED] self.current_pose_name, self.current_pose_start_time
        self.target_pose_name = None  # [NEW] pose we're currently trying to reach
        self.last_attempt_time = 0.0  # [NEW] used for retry delay

        # CHANGE BASED ON MAP
        self.named_poses = {
            "HOME": self._make_pose([4.57, 7.92]),
            "CENTER_ZONE": self._make_pose([8.47, 8.56])
        }

        # [REMOVED] self.pose_queue = {5: "CENTER_ZONE", 15: "HOME", ...}  (timed schedule)
        # [REMOVED] self.override_pose_name = "HOME"  (now HEAL_POSE constant above)

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
            # [REMOVED] self.start_time = time.time()  (no longer needed without a schedule)

            # # a little bit of messing around
            # # Republish initial pose at x:0.0, y:0.0, z:0.0, yaw:1.6
            # initial_pose = PoseWithCovarianceStamped()
            # initial_pose.header.frame_id = 'map'
            # initial_pose.pose.pose.position.x = 6.0
            # initial_pose.pose.pose.position.y = 7.2
            # initial_pose.pose.pose.position.z = 0.0
            # # Convert yaw to quaternion
            # yaw = -0.5
            # initial_pose.pose.pose.orientation.z = math.sin(yaw / 2)
            # initial_pose.pose.pose.orientation.w = math.cos(yaw / 2)
            # self.navigator.set_initial_pose(initial_pose)

    def _health_cb(self, msg):
        self.health = msg.data  # [CHANGED] now only stores health; low_health is computed below

    # [NEW] low_health and healed properties (whole block below)
    @property
    def low_health(self):
        return self.health is not None and self.health < self.LOW_HEALTH_THRESHOLD

    @property
    def healed(self):
        return self.health is not None and self.health >= self.HEALED_THRESHOLD
    # [NEW] end of properties

    # [CHANGED] _tick() fully rewritten (everything through the end of the method)
    def _tick(self):
        # Match not started: do nothing (only report status)
        if not self.match_started:
            self.navigator.publish_nav_status(NAV_STATUS['NOT_STARTED'])
            return

        navigating = self.navigator.is_navigating()
        self.navigator.publish_nav_status(
            NAV_STATUS['NAVIGATING'] if navigating else NAV_STATUS['IDLING'])

        # --- State transitions ---
        if self.state == 'WAITING':
            self._enter_state('CENTER', self.CENTER_POSE)
            return

        if self.state == 'CENTER' and self.low_health:
            self.get_logger().warn(f"Low health ({self.health}), heading HOME to heal")
            self._enter_state('HEALING', self.HEAL_POSE)
            return

        if self.state == 'HEALING' and self.healed:
            self.get_logger().info(f"Healed ({self.health}), returning to center zone")
            self._enter_state('CENTER', self.CENTER_POSE)
            return

        # --- Steady state: make sure we reach the target, retry if the goal failed ---
        if (not navigating
                and not self.navigator.goal_succeeded()
                and time.time() - self.last_attempt_time > self.RETRY_DELAY):
            self.get_logger().warn(f"Goal to '{self.target_pose_name}' not active/succeeded, retrying")
            self._send_target()
    # [CHANGED] end of _tick()

    # [REMOVED] _goal_timed_out(), _should_send_next_pose(), _send_next_pose()

    # [NEW] _enter_state() and _send_target() (whole block below)
    def _enter_state(self, state, pose_name):
        self.state = state
        self.target_pose_name = pose_name
        if self.navigator.is_navigating():
            self.navigator.cancel_goal()
        self._send_target()

    def _send_target(self):
        self.last_attempt_time = time.time()
        pose = self.named_poses[self.target_pose_name]
        if self.navigator.send_goal(pose):
            self.get_logger().info(f"[{self.state}] Sent goal '{self.target_pose_name}'")
        else:
            self.get_logger().warn(f"[{self.state}] Failed to send goal '{self.target_pose_name}'")
    # [NEW] end of _enter_state() / _send_target()


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