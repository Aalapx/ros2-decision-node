import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class DecisionNode(Node):
    def __init__(self):
        super().__init__('decision_node')
        self.subscription_speed = self.create_subscription(Float32, '/speed_info', self.speed_callback, 10)
        self.subscription_distance = self.create_subscription(Float32, '/fused_distance', self.distance_callback, 10)
        self.publisher_ = self.create_publisher(Float32, '/brake_command', 10)

        self.speed = None
        self.distance = None
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz

    def speed_callback(self, msg):
        self.speed = msg.data

    def distance_callback(self, msg):
        self.distance = msg.data

    def timer_callback(self):
        if self.speed is None or self.distance is None:
            return
        
        brake_intensity = self.calculate_brake_intensity(self.speed, self.distance)
        self.publisher_.publish(Float32(data=brake_intensity))
        self.get_logger().info(f"Published brake intensity: {brake_intensity:.2f}")

    def calculate_brake_intensity(self, speed, distance):
        # Simple linear mapping — you can improve this later with better models
        if distance >= 20.0:
            return 0.0  # No brake needed

        if distance <= 5.0:
            return 1.0  # Full emergency brake

        # Linearly scale intensity between 0.0 (at 20m) and 1.0 (at 5m)
        intensity = (20.0 - distance) / 15.0  # (20 - 5 = 15)
        intensity = min(max(intensity, 0.0), 1.0)  # Clamp between 0 and 1

        # Optionally scale with speed (e.g., more brake at higher speed)
        if speed > 30.0:
            intensity *= 1.2
        elif speed < 10.0:
            intensity *= 0.7

        return min(intensity, 1.0)
        
def main(args=None):
    rclpy.init(args=args)
    node = DecisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
