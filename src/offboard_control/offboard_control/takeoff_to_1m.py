#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import time

class TakeoffTo1m(Node):
    def __init__(self):
        super().__init__('takeoff_to_1m')

        self.pub = self.create_publisher(
            PoseStamped,
            '/mavros/setpoint_position/local',
            10
        )

        self.timer = self.create_timer(0.05, self.timer_cb)  # 20 Hz

        self.start_time = time.time()
        self.target_height = 1.0  # meters

    def timer_cb(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()

        # Hold x,y at 0,0 and climb to 1.0m
        msg.pose.position.x = 0.0
        msg.pose.position.y = 0.0
        msg.pose.position.z = self.target_height

        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TakeoffTo1m()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
