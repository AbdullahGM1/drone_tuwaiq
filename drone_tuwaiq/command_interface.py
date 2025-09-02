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
        print("\n" + "="*50)
        print("🚁 DRONE CONTROL COMMANDS")
        print("="*50)
        print("Basic Commands:")
        print("  arm      - Arm the drone")
        print("  disarm   - Disarm the drone")
        print("  takeoff  - Takeoff to hover altitude")
        print("  land     - Land the drone")
        print("")
        print("Movement Commands:")
        print("  up       - Move up")
        print("  down     - Move down")
        print("  forward  - Move forward")
        print("  backward - Move backward")
        print("  left     - Move left")
        print("  right    - Move right")
        print("  stop     - Stop all movement")
        print("")
        print("Special:")
        print("  help     - Show this help")
        print("  quit     - Exit the interface")
        print("="*50)
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