#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String

class DecisionNode(Node):

    def __init__(self):
        super().__init__('decision_node')

        # Internal state variables
        self.speed = None
        self.distance = None
        self.last_decision = None

        # Subscribers
        self.speed_sub = self.create_subscription(
            Float32,
            '/speed_info',
            self.speed_callback,
            10)

        self.distance_sub = self.create_subscription(
            Float32,
            '/fused_distance',
            self.distance_callback,
            10)

        # Publisher
        self.brake_pub = self.create_publisher(
            String,
            '/brake_command',
            10)

        # Timer - Evaluate every 0.1 sec (10Hz)
        self.timer = self.create_timer(0.1, self.evaluate_decision)

        self.get_logger().info("🚗 DecisionNode started and running...")

    def speed_callback(self, msg):
        self.speed = msg.data
        self.get_logger().debug(f"Speed updated: {self.speed} km/h")

    def distance_callback(self, msg):
        self.distance = msg.data
        self.get_logger().debug(f"Distance updated: {self.distance} meters")

    def evaluate_decision(self):
        if self.speed is None or self.distance is None:
            # Wait until both values are received at least once
            return

        decision = self.calculate_decision(self.speed, self.distance)

        if decision != self.last_decision:
            self.publish_decision(decision)
            self.last_decision = decision

    def calculate_decision(self, speed, distance):
        """
        Simple rule-based decision logic.
        Adjust thresholds as needed based on real vehicle data.
        """
        if distance < 5.0 and speed > 30.0:
            return "brake_hard"
        elif distance < 10.0:
            return "brake_soft"
        else:
            return "no_brake"

    def publish_decision(self, decision):
        msg = String()
        msg.data = decision
        self.brake_pub.publish(msg)
        self.get_logger().info(f"Decision published: {decision}")

def main(args=None):
    rclpy.init(args=args)
    node = DecisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
