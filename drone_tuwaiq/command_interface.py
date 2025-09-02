#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys

class CommandInterface(Node):
    def __init__(self):
        super().__init__('command_interface')
        
        # Publisher to send commands
        self.command_pub = self.create_publisher(String, '/drone/command', 10)
        
        self.get_logger().info('🎮 Drone Command Interface Started')
        self.print_help()
        
    def print_help(self):
        print("\n" + "="*60)
        print("🚁 DRONE CONTROL COMMANDS")
        print("="*60)
        print("Basic Commands:")
        print("  arm              - Arm the drone")
        print("  disarm           - Disarm the drone")
        print("  takeoff [alt]    - Takeoff to altitude (default: 2.0m)")
        print("  land             - Land the drone")
        print("")
        print("Movement Commands (with optional distance in meters):")
        print("  up [distance]       - Move up (default: 1.0m)")
        print("  down [distance]     - Move down (default: 1.0m)")
        print("  forward [distance]  - Move forward (default: 1.0m)")
        print("  backward [distance] - Move backward (default: 1.0m)")
        print("  left [distance]     - Move left (default: 1.0m)")
        print("  right [distance]    - Move right (default: 1.0m)")
        print("")
        print("Rotation Commands (with optional degrees):")
        print("  rotate_left [deg]   - Rotate left (default: 90°)")
        print("  rotate_right [deg]  - Rotate right (default: 90°)")
        print("")
        print("Control Commands:")
        print("  stop             - Stop all movement")
        print("")
        print("Special:")
        print("  help             - Show this help")
        print("  quit             - Exit the interface")
        print("="*60)
        print("Examples:")
        print("  takeoff 3.0      - Takeoff to 3 meters")
        print("  forward 2.5      - Move forward 2.5 meters")
        print("  rotate_left 45   - Rotate left 45 degrees")
        print("="*60)
        print("Type a command and press Enter...")
        print()
        
    def send_command(self, command):
        msg = String()
        msg.data = command
        self.command_pub.publish(msg)
        self.get_logger().info(f'Sent command: {command}')
        
    def run_interactive(self):
        while rclpy.ok():
            try:
                command = input("🎮 Enter command: ").strip().lower()
                
                if command == 'quit' or command == 'exit':
                    print("👋 Goodbye!")
                    break
                elif command == 'help':
                    self.print_help()
                elif command == '':
                    continue
                else:
                    self.send_command(command)
                    
            except KeyboardInterrupt:
                print("\n👋 Goodbye!")
                break
            except EOFError:
                print("\n👋 Goodbye!")
                break

def main(args=None):
    rclpy.init(args=args)
    
    # Check if command was passed as argument
    if len(sys.argv) > 1:
        command_interface = CommandInterface()
        command = ' '.join(sys.argv[1:]).lower()
        command_interface.send_command(command)
        command_interface.destroy_node()
        rclpy.shutdown()
        return
    
    # Interactive mode
    command_interface = CommandInterface()
    
    try:
        command_interface.run_interactive()
    finally:
        command_interface.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()