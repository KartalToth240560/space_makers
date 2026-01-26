#!/usr/bin/env python3
"""
Read G1 Joint Positions from ROS2
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class PositionReader(Node):
    def __init__(self):
        super().__init__('position_reader')
        
        self.sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.callback,
            10
        )
        
        self.get_logger().info('Listening for joint positions on /joint_states')
        self.get_logger().info('Press Ctrl+C to stop\n')
    
    def callback(self, msg):
        print("\n" + "="*60)
        print("JOINT POSITIONS")
        print("="*60)
        
        for name, pos in zip(msg.name, msg.position):
            print(f"  {name:35s}: {pos:8.4f} rad ({pos*57.2958:8.2f} deg)")
        
        print("="*60)


def main():
    rclpy.init()
    node = PositionReader()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()