import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math
import os

class AIPlanner(Node):
    def __init__(self):
        super().__init__('ai_planner')
        self.use_policy = self.declare_parameter('use_policy', False).get_parameter_value().bool_value
        self.max_lin = self.declare_parameter('max_linear', 0.6).get_parameter_value().double_value
        self.max_ang = self.declare_parameter('max_angular', 1.2).get_parameter_value().double_value
        self.safe_dist = self.declare_parameter('safe_distance', 0.6).get_parameter_value().double_value
        self.scan = None
        # Sub to Nav2's cmd_vel and Lidar
        self.sub_cmd = self.create_subscription(Twist, '/nav_cmd_vel', self.on_nav_cmd, 10)
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.on_scan, 10)
        # Publish to robot's actual cmd_vel
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('AIPlanner started (policy=%s)' % self.use_policy)

    def on_scan(self, msg: LaserScan):
        self.scan = msg

    def on_nav_cmd(self, msg: Twist):
        # Fallback heuristic: if obstacle within safe_dist in front arc, reduce lin and steer away
        out = Twist()
        out.linear.x = max(0.0, min(self.max_lin, msg.linear.x))
        out.angular.z = max(-self.max_ang, min(self.max_ang, msg.angular.z))
        if self.scan is not None:
            rngs = self.scan.ranges
            if len(rngs) > 0:
                n = len(rngs)
                # consider +- 20 degrees front arc
                arc = 20 * math.pi/180.0
                idx_center = n // 2
                span = max(1, int(arc / (self.scan.angle_increment + 1e-6)))
                front = [r for r in rngs[idx_center-span: idx_center+span] if r > 0.01]
                if front:
                    min_front = min(front)
                    if min_front < self.safe_dist:
                        # slow down and turn away
                        out.linear.x *= max(0.0, (min_front / self.safe_dist))
                        # choose turn direction using left vs right distances
                        left = sum([r for r in rngs[idx_center: idx_center+span] if r > 0.01]) / (span+1e-6)
                        right = sum([r for r in rngs[idx_center-span: idx_center] if r > 0.01]) / (span+1e-6)
                        out.angular.z += 0.8 * (1.0 if left > right else -1.0)
                        out.angular.z = max(-self.max_ang, min(self.max_ang, out.angular.z))
        self.pub_cmd.publish(out)

def main():
    rclpy.init()
    node = AIPlanner()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
