#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import math

class RightSensor(Node):
    def __init__(self):
        super().__init__('dist_direita')
        self.scan_sub = self.create_subscription(
            LaserScan, 
            '/scan', 
            self.scan_callback, 
            qos_profile_sensor_data
        )
        self.obstacle_pub = self.create_publisher(Bool, '/right_obstacle', 10)
        self.safe_distance = 0.5  # meters

    def scan_callback(self, scan):
        # Check right side (260-280 degrees)
        right_ranges = [r for r in scan.ranges[260:280] if math.isfinite(r)]
        if right_ranges and min(right_ranges) < self.safe_distance:
            self.obstacle_pub.publish(Bool(data=True))
        else:
            self.obstacle_pub.publish(Bool(data=False))

def main():
    rclpy.init()
    node = RightSensor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()