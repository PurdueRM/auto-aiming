#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import Bool, Int16, String
from action_msgs.msg import GoalStatus
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

    def goal_succeeded(self):
        if not self.result_future or not self.result_future.done():
            return False
        return self.result_future.result().status == GoalStatus.STATUS_SUCCEEDED

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
    # GOAL_TIMEOUT = 30  # seconds
    LOW_HEALTH_THRESHOLD = 150
    HEALED_THRESHOLD = 370
    RETRY_DELAY = 1.0

    def __init__(self, navigator: Navigator):
        super().__init__('pose_scheduler_sm')
        self.navigator = navigator

        self.match_started = False
        # self.start_time = None
        # self.low_health = False
        # self.override_sent = False

        self.state = 'WAITING'
        #self.current_pose_name = None
        #self.current_pose_start_time = None
        self.target_pose_name = None
        self.last_attempt_time = 0.0

        self.enemy_is_red = None
        self.heatlh = None

        # self.new_health = None

        # CHANGE BASED ON MAP
        self.named_poses = {
            "RED_HOME": self._make_pose([-5.7,-2.99]),
            "BLUE_HOME": self._make_pose([1.87, -1.4]),
            "CENTER_ZONE": self._make_pose([-1.81, -2.08])
        }
        
        self.home_pose_name = "BLUE_HOME"

        # Subscribers
        self.create_subscription(Bool, 'match_start', self._match_cb, 10)
        self.create_subscription(Int16, 'health', self._health_cb, 10)
        self.create_subscription(String, 'target_robot_color', self._enemy_color_cb, 10)

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
            # self.start_time = time.time()

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

    @property
    def low_health(self):
        return self.health is not None and self.health < self.LOW_HEALTH_THRESHOLD

    @property
    def healed(self):
        return self.health is not None and self.health >= self.HEALED_THRESHOLD
    
    def _health_cb(self, msg):
        self.health = msg.data
        # self.low_health = self.new_health < self.LOW_HEALTH_THRESHOLD

    def _enemy_color_cb(self, msg):
        self.enemy_is_red = (msg.data == 1)
        # if self.enemy_is_red:
        #    self.home_pose_name = "BLUE_HOME"
        # else:
        #    self.home_pose_name = "RED_HOME"

    def wait_for_enemy_color(self):
        while self.enemy_is_red is None and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

        if self.enemy_is_red:
            self.home_pose_name = "BLUE_HOME"
        else:
            self.home_pose_name = "RED_HOME"
            
    def _tick(self):
        if not self.match_started:
            self.get_logger().info("Publishing (Match not started)")
            self.navigator.publish_nav_status(NAV_STATUS['NOT_STARTED'])
            return

        navigating = self.navigator.is_navigating()
        if navigating:
            self.navigator.publish_nav_status(NAV_STATUS['NAVIGATING'])
        else:
            self.navigator.publish_nav_status(NAV_STATUS['IDLING'])

        # self.get_logger().info(f'Current nav time: [{time.time() - self.start_time}] sec')

        if self.state == 'WAITING':
            self._enter_state('CENTER', "CENTER_ZONE")
            return

        if self.state == 'CENTER' and self.low_health:
            self.get_logger().warn(f"Low health ({self.health}), heading HOME to heal")
            self._enter_state('HEALING', self.home_pose_name)
            return

        if self.state == 'HEALING' and self.healed:
            self.get_logger().info(f"Healed ({self.health}), returning to center zone")
            self._enter_state('CENTER', "CENTER_ZONE")
            return
            
        if (not navigating
                and not self.navigator.goal_succeeded()
                and time.time() - self.last_attempt_time > self.RETRY_DELAY):
            self.get_logger().warn(f"Goal to '{self.target_pose_name}' not active/succeeded, retrying")
            self._send_target()
        # Handle low health override
        # if self.low_health:
        #     if self.state != 'OVERRIDE':
        #         self.get_logger().warn("Entering LOW HEALTH OVERRIDE")
        #         self.navigator.cancel_goal()
        #         self.state = 'OVERRIDE'
        #         self.override_sent = False

        #     if not self.override_sent:
        #         pose = self.named_poses[self.home_pose_name]
        #         sent = self.navigator.send_goal(pose)
        #         if sent:
        #             self.get_logger().info(f"Sent override HOME pose due to low health")
        #             self.override_sent = True
        #     return
        # else:
        #     pose = self.named_poses[self.named_poses[CENTER_ZONE]]
        #     self.navigator.send_goal(pose)
        #     return


        # # If we just exited override
        # if self.state == 'OVERRIDE' and not self.low_health and self.new_health > 370:
        #     self.get_logger().info("Exiting LOW HEALTH OVERRIDE, resuming schedule")
        #     self.state = 'NAVIGATING'
        #     self.current_pose_start_time = None
        #     self.navigator.cancel_goal()
        #     pose = self.named_poses[self.named_poses[CENTER_ZONE]]
        #     self.navigator.send_goal(pose)
        #     return

        # # Normal state machine
        # if self.state == 'IDLE':
        #     if self.match_started:
        #         self.get_logger().info("Transitioning to NAVIGATING state")
        #         self.state = 'NAVIGATING'
        #         self.current_pose_start_time = None
        #         self._send_next_pose()

        # elif self.state == 'NAVIGATING':
        #     # if self._goal_timed_out():
        #     #     self.get_logger().warn("Goal timed out, cancelling and sending next")
        #     #     self.navigator.cancel_goal()
        #     #     self._send_next_pose()
        #     #     return

        #     # if self._should_send_next_pose():
        #     #     self.get_logger().info("Time to send next scheduled pose")
        #     #     self.navigator.cancel_goal()
        #     #     self._send_next_pose()
        #     #     return

        #     # if not self.navigator.is_navigating() and not self.pose_queue:
        #     #     self.get_logger().info("All poses sent and navigation complete")
        #     #     self.state = 'DONE'
        #     #     return

        # elif self.state == 'DONE':
        #     self.navigator.publish_nav_status(NAV_STATUS['IDLING'])

        # else:
        #     self.get_logger().warn(f"Unknown state: {self.state}")

    # def _goal_timed_out(self):
    #     if self.current_pose_start_time is None:
    #         return False
    #     return (time.time() - self.current_pose_start_time) > self.GOAL_TIMEOUT

    # def _should_send_next_pose(self):
    #     if not self.pose_queue:
    #         return False
    #     elapsed = time.time() - self.start_time
    #     next_time = min(self.pose_queue.keys())
    #     return elapsed >= next_time

    # def _send_next_pose(self):
    #     if not self.pose_queue:
    #         self.get_logger().info("Pose queue empty, no next pose to send")
    #         return

    #     elapsed = time.time() - self.start_time
    #     next_times = sorted(t for t in self.pose_queue if t <= elapsed)
    #     if not next_times:
    #         return

    #     next_time = next_times[0]
    #     pose_name = self.pose_queue.pop(next_time)

    #     if pose_name not in self.named_poses:
    #         self.get_logger().warn(f"Pose '{pose_name}' not found in named poses")
    #         return

    #     self.current_pose_name = pose_name
    #     pose = self.named_poses[pose_name]
    #     sent = self.navigator.send_goal(pose)
    #     if sent:
    #         self.get_logger().info(f"Sent pose '{pose_name}' scheduled at t={next_time}s")
    #         self.current_pose_start_time = time.time()
    #     else:
    #         self.get_logger().warn(f"Failed to send pose '{pose_name}'")

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

def main(args=None):
    rclpy.init(args=args)

    navigator = Navigator()
    navigator.wait_for_initial_pose()

    scheduler = PoseSchedulerStateMachine(navigator)
    scheduler.wait_for_enemy_color()

    executor = MultiThreadedExecutor()
    executor.add_node(navigator)
    executor.add_node(scheduler)

    executor.spin()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
