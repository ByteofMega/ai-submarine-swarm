#!/usr/bin/env python3
"""
Helm: the crew member that steers one submarine.

It is told a target depth and keeps the sub there. It also keeps the sub
pointing the way it was pointing at the start, and standing still.
(Based on the "reflex" layer of Nemo.)

Start it for sub1:
    python3 scripts/helm.py --ros-args -r __ns:=/sub1

Give it a target depth of 5 m (depth is positive downwards):
    ros2 topic pub --once /sub1/helm/target_depth std_msgs/msg/Float64 "{data: 5.0}"

All topic names below are relative, so the same program works for /sub2, /sub3, ...
"""
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64

RATE = 20.0            # control steps per second
MIN_DEPTH = 0.3        # shallowest allowed target (m)
MIN_ALTITUDE = 1.5     # never go closer than this to the seabed (m)
ODOM_TIMEOUT = 0.5     # if no position arrives for this long, stop the thrusters (s)
REAR_ARM = 0.15        # rear thrusters sit 15 cm left/right of the centre line (m)


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def wrap(a):
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


class Helm(Node):
    def __init__(self):
        super().__init__('helm')
        # --- what the sensors say
        self.depth = None          # metres below the surface
        self.vz = 0.0              # vertical speed, up is positive (m/s)
        self.yaw = 0.0             # direction the nose points (rad)
        self.yaw_rate = 0.0
        self.surge = 0.0           # forward speed (m/s)
        self.altitude = None       # height above the seabed (m), None = out of range
        self.last_odom = None      # time of the last position message

        # --- what we are asked to do
        self.target_depth = None   # set from the first position, then by helm/target_depth
        self.target_yaw = None
        self.target_speed = 0.0
        self.depth_i = 0.0         # memory term of the depth controller
        self.steps = 0

        self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        self.create_subscription(LaserScan, 'altimeter', self.on_altimeter, 10)
        self.create_subscription(Float64, 'helm/target_depth', self.on_target_depth, 10)
        self.pub = {name: self.create_publisher(Float64, 'thrust/' + name, 10)
                    for name in ('rear_left', 'rear_right', 'vert_front', 'vert_back')}
        self.create_timer(1.0 / RATE, self.step)
        self.get_logger().info('helm ready, waiting for position on "odom"')

    # ----------------------------------------------------------- messages in
    def on_odom(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        self.depth = -p.z
        self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.vz = m.twist.twist.linear.z
        self.surge = m.twist.twist.linear.x
        self.yaw_rate = m.twist.twist.angular.z
        self.last_odom = self.get_clock().now()
        if self.target_depth is None:          # first message: hold where we are
            self.target_depth = max(self.depth, MIN_DEPTH)
            self.target_yaw = self.yaw
            self.get_logger().info('position received, holding depth %.2f m' % self.target_depth)

    def on_altimeter(self, m):
        r = m.ranges[0] if m.ranges else float('inf')
        self.altitude = None if (math.isinf(r) or math.isnan(r)) else r + 0.2   # beam starts 20 cm below centre

    def on_target_depth(self, m):
        self.target_depth = max(float(m.data), MIN_DEPTH)
        self.depth_i = 0.0
        self.get_logger().info('new target depth: %.2f m' % self.target_depth)

    # ------------------------------------------------------------ thrusters
    def send(self, surge_force, yaw_torque, heave_force):
        """Turn 'push forward / turn / push up' into four thruster forces (newtons)."""
        diff = clamp(yaw_torque / (2 * REAR_ARM), -12.0, 12.0)
        fwd = clamp(surge_force / 2, -22.0, 22.0)
        up = clamp(heave_force / 2, -32.0, 32.0)
        forces = {'rear_left': fwd - diff, 'rear_right': fwd + diff,
                  'vert_front': up, 'vert_back': up}
        for name, f in forces.items():
            self.pub[name].publish(Float64(data=float(f)))

    # --------------------------------------------------------- control step
    def step(self):
        if self.target_depth is None:
            return
        age = (self.get_clock().now() - self.last_odom).nanoseconds / 1e9
        if age > ODOM_TIMEOUT:                 # safety: never steer blind
            self.send(0.0, 0.0, 0.0)
            self.get_logger().warn('no position for %.1f s, thrusters off' % age,
                                   throttle_duration_sec=2.0)
            return

        # safety: stay above the seabed (only when the altimeter can see it)
        target = self.target_depth
        if self.altitude is not None:
            target = min(target, self.depth + self.altitude - MIN_ALTITUDE)

        # depth: positive error = too deep = push up
        err = self.depth - target
        if abs(err) < 1.0:
            self.depth_i = clamp(self.depth_i + err / RATE, -3.0, 3.0)
        heave = 60.0 * clamp(err, -1.2, 1.2) - 80.0 * self.vz + 5.0 * self.depth_i

        # heading and speed: keep pointing the same way, stand still
        err_yaw = wrap(self.target_yaw - self.yaw)
        yaw_torque = 7.0 * err_yaw - 6.0 * self.yaw_rate
        surge = 45.0 * (self.target_speed - self.surge)

        self.send(surge, yaw_torque, heave)

        self.steps += 1
        if self.steps % int(RATE) == 0:        # one log line per second
            alt = 'none' if self.altitude is None else '%.1f' % self.altitude
            self.get_logger().info(
                'depth=%6.2f target=%6.2f error=%+5.2f vz=%+5.2f heave=%+6.1fN alt=%s' %
                (self.depth, target, err, self.vz, heave, alt))


def main():
    rclpy.init()
    node = Helm()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
