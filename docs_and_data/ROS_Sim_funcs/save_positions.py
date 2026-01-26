#!/usr/bin/env python3
"""
Save G1 Joint Positions to CSV
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import csv
import time
from datetime import datetime


class SavePositions(Node):
    def __init__(self):
        super().__init__('save_positions')
        
        self.sub = self.create_subscription(
            JointState, '/joint_states', self.state_callback, 10
        )
        
        self.current_names = []
        self.current_positions = []
        self.has_data = False
        
        self.get_logger().info('Waiting for /joint_states...')
    
    def state_callback(self, msg):
        self.current_names = list(msg.name)
        self.current_positions = list(msg.position)
        self.has_data = True
    
    def save_to_csv(self, filename=None):
        """Save current positions to CSV."""
        if not self.has_data:
            print("[ERROR] No joint data received!")
            return None
        
        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"g1_positions_{timestamp}.csv"
        
        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['joint_name', 'position_rad', 'position_deg'])
            
            for name, pos in zip(self.current_names, self.current_positions):
                pos_deg = pos * 57.2958  # Convert to degrees
                writer.writerow([name, f"{pos:.6f}", f"{pos_deg:.2f}"])
        
        print(f"[SAVED] {filename}")
        print(f"        {len(self.current_names)} joints saved")
        return filename


def main():
    rclpy.init()
    node = SavePositions()
    
    import threading
    spin_thread = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    spin_thread.start()
    
    print("\n" + "="*50)
    print("G1 POSITION SAVER")
    print("="*50)
    print("Commands:")
    print("  ENTER = Save current position to CSV")
    print("  p     = Print current positions")
    print("  q     = Quit")
    print("="*50 + "\n")
    
    time.sleep(1)  # Wait for data
    
    if not node.has_data:
        print("[WARNING] No /joint_states data yet!")
        print("Make sure Isaac Sim is publishing joint states.\n")
    
    try:
        while True:
            cmd = input(">>> ").strip().lower()
            
            if cmd == '':
                node.save_to_csv()
            
            elif cmd == 'p':
                if node.has_data:
                    print("\n[CURRENT POSITIONS]")
                    for name, pos in zip(node.current_names, node.current_positions):
                        print(f"  {name}: {pos:.4f} rad ({pos*57.2958:.2f} deg)")
                else:
                    print("[NO DATA]")
            
            elif cmd == 'q':
                break
    
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()