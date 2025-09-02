#!/usr/bin/env python3
"""
Interactive Drone CLI Command Sender
Sends commands to the drone controller via socket connection.
Usage: python3 drone_cli.py
"""

import socket
import sys
import json
import time

class DroneCommandSender:
    def __init__(self, host='localhost', port=9999):
        self.host = host
        self.port = port
        self.socket = None
        self.connected = False
    
    def connect(self):
        """Establish connection to drone controller"""
        try:
            if self.socket:
                self.socket.close()
            
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(180)  # 3 minute timeout for long operations
            self.socket.connect((self.host, self.port))
            print(f"✓ Connected to drone controller at {self.host}:{self.port}")
            self.connected = True
            return True
        except Exception as e:
            print(f"✗ Failed to connect to drone controller: {e}")
            self.connected = False
            return False
    
    def send_command(self, command_data):
        """Send command to drone controller"""
        try:
            if not self.connected:
                if not self.connect():
                    return False
            
            # Show different messages for different commands
            action = command_data.get('action', 'unknown')
            if action == 'goto':
                print("⏳ Executing goto command... (this may take up to 2 minutes)")
            elif action == 'takeoff':
                print("⏳ Executing takeoff... (this may take up to 1 minute)")
            
            # Send command as JSON
            message = json.dumps(command_data) + '\n'
            self.socket.send(message.encode('utf-8'))
            
            # Wait for response with longer timeout for complex operations
            response = self.socket.recv(8192).decode('utf-8').strip()
            response_data = json.loads(response)
            
            if response_data.get('status') == 'success':
                message = response_data.get('message', 'Success')
                print(f"✓ {message}")
                
                # Print additional data if available
                if 'data' in response_data:
                    self._print_status_data(response_data['data'])
                return True
            else:
                print(f"✗ Command failed: {response_data.get('message', 'Unknown error')}")
                return False
                
        except socket.timeout:
            print(f"✗ Command timed out - this usually means the drone is still executing the command")
            print("  You can check status or try reconnecting")
            self.connected = False
            return False
        except Exception as e:
            print(f"✗ Error sending command: {e}")
            self.connected = False
            return False
    
    def _print_status_data(self, data):
        """Pretty print status data"""
        print("--- Drone Status ---")
        print(f"Connected: {'Yes' if data.get('connected') else 'No'}")
        print(f"Armed: {'Yes' if data.get('armed') else 'No'}")
        print(f"Altitude: {data.get('altitude', 0.0):.2f}m")
        print(f"Flight Mode: {data.get('flight_mode', 'Unknown')}")
        
        position = data.get('position')
        if position:
            print(f"Position: Lat={position.get('lat', 0.0):.6f}°, Lon={position.get('lon', 0.0):.6f}°, Alt={position.get('alt', 0.0):.2f}m")
        
        print(f"Takeoff Position Set: {'Yes' if data.get('takeoff_position_set') else 'No'}")
        
        battery = data.get('battery')
        if battery:
            print(f"Battery: {battery.get('remaining', 0)}% ({battery.get('voltage', 0.0):.1f}V)")
        
        gps = data.get('gps')
        if gps:
            print(f"GPS: {gps.get('satellites', 0)} satellites, {gps.get('fix_type', 'Unknown')} fix")
        
        print("-------------------")
    
    def close(self):
        """Close socket connection"""
        if self.socket:
            self.socket.close()
            self.socket = None
        self.connected = False

def print_help():
    """Print available commands"""
    print("\n=== Available Commands ===")
    print("arm                    - Arm the drone")
    print("disarm                 - Disarm the drone")
    print("takeoff <altitude>     - Takeoff to specified altitude (meters)")
    print("land                   - Land the drone")
    print("move <distance>        - Move forward by distance (meters)")
    print("goto <x> <y> <z>       - Go to specific position (x,y,z in meters)")
    print("status                 - Get drone status")
    print("help                   - Show this help message")
    print("exit/quit              - Exit the program")
    print("==========================\n")

def parse_command(command_line):
    """Parse command line input"""
    parts = command_line.strip().split()
    if not parts:
        return None
    
    command = parts[0].lower()
    args = parts[1:]
    
    if command in ['exit', 'quit']:
        return {'action': 'exit'}
    
    elif command == 'help':
        return {'action': 'help'}
    
    elif command == 'arm':
        return {'action': 'arm'}
    
    elif command == 'disarm':
        return {'action': 'disarm'}
    
    elif command == 'status':
        return {'action': 'status'}
    
    elif command == 'land':
        return {'action': 'land'}
    
    elif command == 'takeoff':
        if len(args) != 1:
            print("✗ Usage: takeoff <altitude>")
            return None
        try:
            altitude = float(args[0])
            return {'action': 'takeoff', 'altitude': altitude}
        except ValueError:
            print("✗ Altitude must be a number")
            return None
    
    elif command == 'move':
        if len(args) < 1:
            print("✗ Usage: move <distance> [speed]")
            return None
        try:
            distance = float(args[0])
            speed = float(args[1]) if len(args) > 1 else 2.0
            return {'action': 'move_forward', 'distance': distance, 'speed': speed}
        except ValueError:
            print("✗ Distance and speed must be numbers")
            return None
    
    elif command == 'goto':
        if len(args) != 3:
            print("✗ Usage: goto <x> <y> <z>")
            return None
        try:
            x = float(args[0])
            y = float(args[1])
            z = float(args[2])
            return {'action': 'goto', 'x': x, 'y': y, 'z': z}
        except ValueError:
            print("✗ Coordinates must be numbers")
            return None
    
    else:
        print(f"✗ Unknown command: {command}")
        return None

def main():
    """Main interactive loop"""
    print("🚁 Interactive Drone CLI")
    print("Type 'help' for available commands or 'exit' to quit")
    
    sender = DroneCommandSender()
    
    # Try initial connection
    print("\nAttempting to connect to drone controller...")
    sender.connect()
    
    try:
        while True:
            try:
                # Get user input
                user_input = input("\ndrone> ").strip()
                
                if not user_input:
                    continue
                
                # Parse command
                command_data = parse_command(user_input)
                
                if not command_data:
                    continue
                
                # Handle special commands
                if command_data['action'] == 'exit':
                    print("Goodbye! 👋")
                    break
                
                elif command_data['action'] == 'help':
                    print_help()
                    continue
                
                # Send command to drone controller
                print(f"Sending command: {command_data['action']}...")
                success = sender.send_command(command_data)
                
                if not success and not sender.connected:
                    print("Connection lost. Attempting to reconnect...")
                    sender.connect()
            
            except EOFError:
                print("\nGoodbye! 👋")
                break
            except KeyboardInterrupt:
                print("\nGoodbye! 👋")
                break
                
    except Exception as e:
        print(f"Error: {e}")
    finally:
        sender.close()

if __name__ == '__main__':
    main()