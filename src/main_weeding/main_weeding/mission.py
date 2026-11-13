"""Stop-and-go weeding mission.

Repeats: drive `step_distance` metres straight (measured on /diff_cont/odom), stop,
run one RemoveWeed cycle on the arm (/arm/remove_weed), and drive on once the arm
reports a result.

Velocity goes out on /cmd_vel, twist_mux's lowest-priority input, so holding the
joystick's enable button always overrides the mission. While the arm works, zero
velocity is published so the rover holds still (RemoveWeed requires that).

An ARM_FAULT result, or the arm server rejecting goals, ends the mission with the
rover stopped -- driving on with the arm in an unknown pose is not safe.
"""

import math

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from robot_arm_interfaces.action import RemoveWeed


OUTCOME_NAMES = {
    RemoveWeed.Result.EXTRACTED: 'EXTRACTED',
    RemoveWeed.Result.NO_WEED_FOUND: 'NO_WEED_FOUND',
    RemoveWeed.Result.LOST_TARGET: 'LOST_TARGET',
    RemoveWeed.Result.UNREACHABLE: 'UNREACHABLE',
    RemoveWeed.Result.PLUNGE_BLOCKED: 'PLUNGE_BLOCKED',
    RemoveWeed.Result.ARM_FAULT: 'ARM_FAULT',
    RemoveWeed.Result.CANCELED: 'CANCELED',
}

# States
WAIT_READY = 'WAIT_READY'
DRIVING = 'DRIVING'
SETTLING = 'SETTLING'
ARM_BUSY = 'ARM_BUSY'
DONE = 'DONE'


class Mission(Node):

    def __init__(self):
        super().__init__('mission')

        self.step_distance = self.declare_parameter('step_distance', 1.0).value  # [m]
        self.speed = self.declare_parameter('speed', 0.2).value                  # [m/s], negative drives backwards
        self.settle_time = self.declare_parameter('settle_time', 1.0).value      # [s] standstill before the arm starts
        self.search_timeout = self.declare_parameter('search_timeout', 0.0).value  # [s], 0 = arm's default
        self.max_stops = self.declare_parameter('max_stops', 0).value            # 0 = unlimited
        self.odom_timeout = self.declare_parameter('odom_timeout', 0.5).value    # [s]
        self.rate = self.declare_parameter('rate', 20.0).value                   # [Hz], must beat twist_mux's 0.5 s timeout

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/diff_cont/odom', self.odom_callback, 10)
        self.arm_client = ActionClient(self, RemoveWeed, '/arm/remove_weed')

        self.state = WAIT_READY
        self.pose = None             # latest (x, y) from odom
        self.last_odom_time = None
        self.start_pose = None       # where the current drive leg began
        self.state_since = self.now()
        self.stops_done = 0
        self.rejections = 0
        self.goal_handle = None
        self.last_feedback = None
        self.last_wait_log = None

        self.timer = self.create_timer(1.0 / self.rate, self.step)
        self.get_logger().info(
            f"Mission: drive {self.step_distance:.2f} m at {self.speed:.2f} m/s, then run RemoveWeed; "
            f"stops: {'unlimited' if self.max_stops == 0 else self.max_stops}")

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def set_state(self, state):
        self.get_logger().info(f"{self.state} -> {state}")
        self.state = state
        self.state_since = self.now()

    def odom_callback(self, msg):
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.last_odom_time = self.now()

    def odom_fresh(self):
        return self.last_odom_time is not None and self.now() - self.last_odom_time < self.odom_timeout

    def publish_velocity(self, linear_x):
        msg = Twist()
        msg.linear.x = float(linear_x)
        self.cmd_pub.publish(msg)

    # ---------------- State machine ----------------

    def step(self):
        if self.state == WAIT_READY:
            odom_ok = self.odom_fresh()
            arm_ok = self.arm_client.server_is_ready()
            if odom_ok and arm_ok:
                self.get_logger().info("Odometry and arm action server are up. Starting.")
                self.start_leg()
            elif self.last_wait_log is None or self.now() - self.last_wait_log > 5.0:
                self.last_wait_log = self.now()
                self.get_logger().info(
                    f"Waiting for: {'' if odom_ok else '/diff_cont/odom '}{'' if arm_ok else '/arm/remove_weed'}")

        elif self.state == DRIVING:
            if not self.odom_fresh():
                # No odometry means no idea how far we went: hold still until it comes back.
                self.publish_velocity(0.0)
                self.get_logger().warn("Odometry lost, holding still.", throttle_duration_sec=2.0)
                return
            travelled = math.hypot(self.pose[0] - self.start_pose[0], self.pose[1] - self.start_pose[1])
            if travelled >= self.step_distance:
                self.publish_velocity(0.0)
                self.get_logger().info(f"Travelled {travelled:.2f} m, stopping.")
                self.set_state(SETTLING)
            else:
                self.publish_velocity(self.speed)

        elif self.state == SETTLING:
            self.publish_velocity(0.0)
            if self.now() - self.state_since >= self.settle_time:
                self.send_arm_goal()

        elif self.state == ARM_BUSY:
            self.publish_velocity(0.0)

        # DONE: publish nothing; twist_mux times out and diff_cont stops the wheels.

    def start_leg(self):
        if self.max_stops and self.stops_done >= self.max_stops:
            self.publish_velocity(0.0)
            self.get_logger().info(f"Mission complete after {self.stops_done} stop(s).")
            self.set_state(DONE)
            return
        self.start_pose = self.pose
        self.set_state(DRIVING)

    def finish(self, reason):
        self.publish_velocity(0.0)
        self.get_logger().error(f"Mission stopped: {reason}")
        self.set_state(DONE)

    # ---------------- Arm action ----------------

    def send_arm_goal(self):
        if not self.arm_client.server_is_ready():
            self.finish("arm action server /arm/remove_weed disappeared")
            return
        goal = RemoveWeed.Goal()
        goal.search_timeout_s = float(self.search_timeout)
        self.last_feedback = None
        self.set_state(ARM_BUSY)
        self.get_logger().info(f"Stop {self.stops_done + 1}: sending RemoveWeed goal.")
        future = self.arm_client.send_goal_async(goal, feedback_callback=self.feedback_callback)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.rejections += 1
            if self.rejections >= 3:
                self.finish("arm rejected the RemoveWeed goal 3 times (is a cycle already running?)")
            else:
                self.get_logger().warn("Arm rejected the goal, retrying.")
                self.set_state(SETTLING)
            return
        self.rejections = 0
        self.goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        state = feedback_msg.feedback.state
        if state != self.last_feedback:
            self.last_feedback = state
            self.get_logger().info(f"Arm: {state}")

    def result_callback(self, future):
        self.goal_handle = None
        result = future.result().result
        outcome = OUTCOME_NAMES.get(result.outcome, str(result.outcome))
        self.stops_done += 1
        self.get_logger().info(f"Arm finished: {outcome} ({result.message})")

        if result.outcome == RemoveWeed.Result.ARM_FAULT:
            self.finish("arm reported ARM_FAULT")
        elif result.outcome == RemoveWeed.Result.CANCELED:
            self.finish("arm goal was canceled")
        else:
            self.start_leg()

    def shutdown(self):
        self.publish_velocity(0.0)
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()


def main(args=None):
    rclpy.init(args=args)
    node = Mission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
