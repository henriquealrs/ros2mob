import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from transforms3d.euler import euler2quat, quat2euler
import numpy as np
import math

class State:
    def __init__(self):
        self.state = np.zeros(3, dtype=np.float32)
        self.velocity = np.zeros(2)

    def update_state(self, dt: float) -> None:
        dx = dt * np.array([self.velocity[0] * np.cos(self.state[2]),
                           self.velocity[0] * np.sin(self.state[2]), self.velocity[1]])
        self.state = self.state + dx


class SimulateBot(Node):
    def __init__(self):
        super().__init__('simulate_bot')
        self.get_logger().info('Simulation Node Started')

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.lidar_pub = self.create_publisher(LaserScan, '/scan', 10)

        # Subscriber to velocity command
        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # Internal state
        self.state = State()

        # Timer for updates
        self.last_time = self.get_clock().now()
        self.timer = self.create_timer(0.1, self.update)

    def cmd_vel_callback(self, msg: Twist):
        self.vel = np.array([[msg.linear], [msg.angular]])

    def update(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.state.update_state(dt)
        self.publish_odometry(now)
        self.publish_fake_lidar(now)

    def publish_odometry(self, now: Time):
        odo = Odometry()
        odo.header.stamp = now.to_msg()
        odo.header.frame_id = 'odom'
        odo.child_frame_id = 'base_link'
        self.get_logger().info(f"----\n{self.state.state}\n------")
        odo.pose.pose.position.x = self.state.state[0]
        odo.pose.pose.position.y = self.state.state[1]
        q = euler2quat(0, 0, self.state.state[2])
        odo.pose.pose.orientation.w = q[0]
        odo.pose.pose.orientation.x = q[1]
        odo.pose.pose.orientation.y = q[2]
        odo.pose.pose.orientation.z = q[3]
        self.odom_pub.publish(odo)

    def publish_fake_lidar(self, now: Time):
        scan = LaserScan()
        scan.header.frame_id = 'scan'
        scan.header.stamp = now.to_msg()
        scan.angle_min = -math.pi
        scan.angle_max = -math.pi
        scan.angle_increment = math.pi/180.0
        scan.range_min = 0.1
        scan.range_max = 10.0
        scan.ranges = [self.fake_range(angle) for angle in range(360)]
        self.lidar_pub.publish(scan)

    def fake_range(self, angle_deg):
        # Fake wall at 5m unless facing directly "behind", where it’s 2m
        if 170 < angle_deg < 190:
            return 2.0
        return 5.0

def main(args=None):
    rclpy.init(args=args)
    node = SimulateBot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
