import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from transforms3d.euler import euler2quat, quat2euler

class SimulateBot(Node):
    def __init__(self):
        super().__init__('simulate_bot')
        self.get_logger().info('Simulation Node Started')

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.lidar_pub = self.create_publisher(LaserScan, '/scan', 10)

        # Subscriber to velocity command
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # Internal state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vx = 0.0
        self.vtheta = 0.0

        # Timer for updates
        self.last_time = self.get_clock().now()
        self.timer = self.create_timer(0.1, self.update)
