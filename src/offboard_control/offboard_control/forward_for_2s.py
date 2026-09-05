#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class ForwardFor2s(Node):
    def __init__(self):
        super().__init__('forward_for_2s')

        self.pub = self.create_publisher(
            Twist,
            '/mavros/setpoint_velocity/cmd_vel_unstamped',
            10
        )

        self.timer = self.create_timer(0.05, self.timer_cb)  # 20 Hz
        self.start_time = time.time()

    def timer_cb(self):
        msg = Twist()

        # Move forward for 2 seconds
        if time.time() - self.start_time < 2.0:
            msg.linear.x = 0.2
        else:
            msg.linear.x = 0.0  # hover

        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ForwardFor2s()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
