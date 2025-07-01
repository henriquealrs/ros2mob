from simulation_node_py.simulate_boot import SimulateBot
import rclpy
from rclpy import Node
from geometry_msgs.msg import Twist
import numpy as np
import pytest

class TestNode(Node):
    def __init__(self):
        super().__init__("test_sim")
        self.pub = self.create_publisher(Twist, '/cmd_acc', 10)
        self.simulation = SimulateBot()

    def publish(self, vel: np.ndarray) -> None:
        twist = Twist()
        twist.linear.x = vel[0]
        twist.angular.z = vel[1]
        self.pub.publish(twist)


