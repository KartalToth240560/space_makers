#!/usr/bin/env python3
"""
G1 Joint Freezer - ROS2
Reads current position and holds it
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import threading


class Freeze(Node):
    def __init__(self):
        super().__init__('freeze')
        
        # Publisher - sends commands
        self.pub = self.create_publisher(JointState, '/joint_command', 10)
        
        # Subscriber - reads current positions
        self.sub = self.create_subscription(
            JointState, '/joint_states', self.state_callback, 10
        )
        
        # Timer - publish at 50 Hz
        self.timer = self.create_timer(0.02, self.hold)
        
        # State
        self.frozen_names = []
        self.frozen_positions = []
        self.is_frozen = False
        
        self.get_logger().info('Freeze node ready')
        self.get_logger().info('Waiting for /joint_states...')
    
    def state_callback(self, msg):
        """When we receive joint states, capture them."""
        if not self.is_frozen:
            self.frozen_names = list(msg.name)
            self.frozen_positions = list(msg.position)
            self.is_frozen = True
            self.get_logger().info(f'FROZEN! Holding {len(self.frozen_names)} joints')
    
    def hold(self):
        """Continuously publish frozen positions."""
        if self.is_frozen and self.frozen_names:
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = self.frozen_names
            msg.position = self.frozen_positions
            self.pub.publish(msg)


def main():
    rclpy.init()
    node = Freeze()
    
    print("\n" + "="*50)
    print("G1 JOINT FREEZER")
    print("="*50)
    print("Will freeze joints as soon as /joint_states")
    print("publishes. Press Ctrl+C to release.")
    print("="*50 + "\n")
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n[RELEASED] Joints are free now.")
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()