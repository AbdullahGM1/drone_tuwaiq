#!/usr/bin/env python3
"""
Drone CLI Command Sender
Sends commands to the drone controller via socket connection.
Usage: python drone_cli.py [command] [arguments]
"""

import socket
import sys
import json
import argparse
import time

class DroneCommandSender:
    def __init__(self, host='localhost', port=9999):
        self.host = host
        self.port = port
        self.socket = None
    
    def connect(self):
        """Establish connection to drone controller"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(10)  # 10 second timeout
            self.socket.connect((self.host, self.port))
            print(f"✓ Connected to drone controller at {self.host}:{self.port}")
            return True
        except Exception as e:
            print(f"✗ Failed to connect to drone controller: {e}")
            return False
    
    def send_command(self, command_data):
        """Send command to drone controller"""
        try:
            if not self.socket:
                if not self.connect():
                    return False
            
            # Send command as JSON
            message = json.dumps(command_data) + '\n'
            self.socket.send(message.encode('utf-8'))
            
            # Wait for response
            response = self.socket.recv(1024).decode('utf-8').strip()
            response_data = json.loads(response)
            
            if response_data.get('status') == 'success':
                print(f"✓ Command executed: {response_data.get('message', 'Success')}")
                return True
            else:
                print(f"✗ Command failed: {response_data.get('message', 'Unknown error')}")
                return False
                
        except Exception as e:
            print(f"✗ Error sending command: {e}")
            return False
    
    def close(self):
        """Close socket connection"""
        if self.socket:
            self.socket.close()
            self.socket = None

def main():
    parser = argparse.ArgumentParser(description='Send commands to PX4 drone')
    parser.add_argument('--host', default='localhost', help='Drone controller host (default: localhost)')
    parser.add_argument('--port', type=int, default=9999, help='Drone controller port (default: 9999)')
    
    subparsers = parser.add_subparsers(dest='command', help='Available commands')
    
    # Takeoff command
    takeoff_parser = subparsers.add_parser('takeoff', help='Takeoff to specified altitude')
    takeoff_parser.add_argument('altitude', type=float, help='Takeoff altitude in meters')
    
    # Land command
    land_parser = subparsers.add_parser('land', help='Land the drone')
    
    # Move forward command
    move_parser = subparsers.add_parser('move', help='Move drone forward')
    move_parser.add_argument('distance', type=float, help='Distance to move forward in meters')
    move_parser.add_argument('--speed', type=float, default=2.0, help='Movement speed in m/s (default: 2.0)')
    
    # Status command
    status_parser = subparsers.add_parser('status', help='Get drone status')
    
    # Arm command
    arm_parser = subparsers.add_parser('arm', help='Arm the drone')
    
    # Disarm command
    disarm_parser = subparsers.add_parser('disarm', help='Disarm the drone')
    
    args = parser.parse_args()
    
    if not args.command:
        parser.print_help()
        return
    
    # Create command sender
    sender = DroneCommandSender(args.host, args.port)
    
    try:
        # Prepare command data based on user input
        if args.command == 'takeoff':
            command_data = {
                'action': 'takeoff',
                'altitude': args.altitude
            }
            print(f"Sending takeoff command (altitude: {args.altitude}m)...")
            
        elif args.command == 'land':
            command_data = {
                'action': 'land'
            }
            print("Sending land command...")
            
        elif args.command == 'move':
            command_data = {
                'action': 'move_forward',
                'distance': args.distance,
                'speed': args.speed
            }
            print(f"Sending move forward command (distance: {args.distance}m, speed: {args.speed}m/s)...")
            
        elif args.command == 'status':
            command_data = {
                'action': 'status'
            }
            print("Requesting drone status...")
            
        elif args.command == 'arm':
            command_data = {
                'action': 'arm'
            }
            print("Sending arm command...")
            
        elif args.command == 'disarm':
            command_data = {
                'action': 'disarm'
            }
            print("Sending disarm command...")
        
        # Send the command
        success = sender.send_command(command_data)
        
        if not success:
            sys.exit(1)
            
    except KeyboardInterrupt:
        print("\nOperation cancelled by user")
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)
    finally:
        sender.close()

if __name__ == '__main__':
    main()