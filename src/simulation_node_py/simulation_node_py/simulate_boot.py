import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from transforms3d.euler import euler2quat, quat2euler
import numpy as np
import math
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
# from scipy.spatial.transform import Rotation


class State:
    def __init__(self):
        self.pose = np.zeros(3, dtype=np.float32)
        self.velocity = np.zeros(2)
        pass


class Simulation:
    def __init__(self):
        self.accel = np.zeros(2)
        self.state = State()

    def update_state(self, dt: float) -> None:
        pose = self.state.pose
        vel = self.state.velocity
        # rot_mat = Rotation.from_euler('z', [pose[2]]).as_matrix()[:2, :2]
        d_long_v = self.accel[0] * dt
        d_long = dt * (vel[0] + 0.5 * d_long_v)

        d_ang_v = self.accel[1] * dt
        d_ang = dt * (vel[0] + 0.5 * d_ang_v)

        vel[0] += d_long_v
        vel[1] += d_ang_v
        pose[:2] += d_long * np.array([np.cos(pose[2]), np.sin(pose[2])])
        pose[2] += d_ang


    def set_command_accel(self, lin_acc: float, ang_acc: float) -> None:
        self.accel[0] = lin_acc
        self.accel[1] = ang_acc


class SimulateBot(Node):
    def __init__(self):
        super().__init__('simulate_bot')
        # self.get_logger().info('Simulation Node Started')

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.lidar_pub = self.create_publisher(LaserScan, 'scan', 10)

        # Subscriber to velocity command
        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_acc', self.cmd_acc_callback, 10)

        # Internal state
        self.sim = Simulation()

        # Timer for updates
        self.last_time = self.get_clock().now()
        self.timer = self.create_timer(0.1, self.update)
        self.tf_broadcaster = TransformBroadcaster(self)


    def cmd_acc_callback(self, msg: Twist):
        self.get_logger().info(f"Received twist {msg.linear} {msg.angular}")

    def update(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.sim.update_state(dt)
        self.publish_odometry(now)
        self.publish_fake_lidar(now)
        self.send_to_baselink(now)

    def send_to_baselink(self, now: Time):
        tf = TransformStamped()
        tf.header.stamp = now.to_msg()
        tf.header.frame_id = "odom"
        tf.child_frame_id = "base_link"
        pose = self.sim.state.pose
        tf.transform.translation.x = float(pose[0])
        tf.transform.translation.y = float(pose[1])
        tf.transform.translation.z = 0.0
        q = euler2quat(0, 0, float(pose[2]))
        tf.transform.rotation.x = q[0]
        tf.transform.rotation.y = q[1]
        tf.transform.rotation.z = q[2]
        tf.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(tf)

    def publish_odometry(self, now: Time):
        odo = Odometry()
        odo.header.stamp = now.to_msg()
        odo.header.frame_id = 'odom'
        odo.child_frame_id = 'base_link'
        pose = self.sim.state.pose
        self.get_logger().info(f"----\n{pose}\n------")
        odo.pose.pose.position.x = float(pose[0])
        odo.pose.pose.position.y = float(pose[1])
        q = euler2quat(0, 0, float(pose[2]))
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
