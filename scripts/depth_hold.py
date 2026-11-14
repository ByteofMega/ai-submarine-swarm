#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from tf2_msgs.msg import TFMessage


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class DepthHold(Node):
    def __init__(self):
        super().__init__('depth_hold')
        self.declare_parameter('target_z', -4.0)
        self.declare_parameter('thrust', -31.0)
        self.z = None
        self.prev_z = None
        self.vz = 0.0
        self.dt = 0.05
        self.n = 0
        self.create_subscription(TFMessage, '/tethys/poses', self.on_pose, 10)
        self.pub_fin = self.create_publisher(Float64, '/tethys/cmd/fins_h', 10)
        self.pub_thr = self.create_publisher(Float64, '/tethys/cmd/thrust', 10)
        self.create_timer(self.dt, self.step)

    def on_pose(self, msg):
        if msg.transforms:
            self.z = msg.transforms[0].transform.translation.z

    def step(self):
        if self.z is None:
            return
        if self.prev_z is not None:
            raw = (self.z - self.prev_z) / self.dt
            self.vz = 0.7 * self.vz + 0.3 * raw
        self.prev_z = self.z
        tz = self.get_parameter('target_z').value
        vz_des = clamp(0.3 * (tz - self.z), -0.8, 0.8)
        fin = clamp(-0.7 * (vz_des - self.vz), -0.7, 0.7)
        self.pub_fin.publish(Float64(data=float(fin)))
        thr = float(self.get_parameter('thrust').value)
        self.pub_thr.publish(Float64(data=thr))
        self.n += 1
        if self.n % 20 == 0:
            self.get_logger().info(
                f'z={self.z:6.2f} target={tz:6.2f} vz={self.vz:5.2f} fin={fin:5.2f}')


def main():
    rclpy.init()
    node = DepthHold()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.pub_fin.publish(Float64(data=0.0))
    node.pub_thr.publish(Float64(data=0.0))
    node.destroy_node()


if __name__ == '__main__':
    main()
