#!/usr/bin/env python3
"""
Drone Controller using MAVSDK
Receives commands via socket and controls PX4 drone using MAVSDK.
"""

import asyncio
import socket
import json
import threading
import signal
import sys
from mavsdk import System
from mavsdk.offboard import PositionNedYaw, VelocityBodyYawspeed
from mavsdk.mission import MissionItem, MissionPlan
import math

class DroneController:
    def __init__(self, connection_string="udpin://0.0.0.0:14540", port=9999):
        self.connection_string = connection_string
        self.port = port
        self.drone = System()
        self.server_socket = None
        self.running = False
        self.current_position = None
        self.current_altitude = None
        self.is_armed = False
        self.is_connected = False
        self.main_loop = None  # Store reference to main event loop
        
    async def connect_drone(self):
        """Connect to the drone via MAVSDK"""
        try:
            print(f"Connecting to drone at {self.connection_string}...")
            await self.drone.connect(system_address=self.connection_string)
            
            print("Waiting for drone connection...")
            async for state in self.drone.core.connection_state():
                if state.is_connected:
                    print("✓ Drone connected!")
                    self.is_connected = True
                    break
            
            # Start telemetry monitoring
            await self._start_telemetry_monitoring()
            return True
            
        except Exception as e:
            print(f"✗ Failed to connect to drone: {e}")
            return False
    
    async def _start_telemetry_monitoring(self):
        """Start monitoring drone telemetry"""
        # Monitor position
        asyncio.create_task(self._monitor_position())
        # Monitor armed state
        asyncio.create_task(self._monitor_armed_state())
    
    async def _monitor_position(self):
        """Monitor drone position"""
        async for position in self.drone.telemetry.position():
            self.current_position = position
            self.current_altitude = position.relative_altitude_m
    
    async def _monitor_armed_state(self):
        """Monitor drone armed state"""
        async for armed in self.drone.telemetry.armed():
            self.is_armed = armed
    
    async def arm_drone(self):
        """Arm the drone"""
        try:
            print("Arming drone...")
            await self.drone.action.arm()
            return {"status": "success", "message": "Drone armed successfully"}
        except Exception as e:
            return {"status": "error", "message": f"Failed to arm drone: {e}"}
    
    async def disarm_drone(self):
        """Disarm the drone"""
        try:
            print("Disarming drone...")
            await self.drone.action.disarm()
            return {"status": "success", "message": "Drone disarmed successfully"}
        except Exception as e:
            return {"status": "error", "message": f"Failed to disarm drone: {e}"}
    
    async def takeoff(self, altitude):
        """Takeoff to specified altitude"""
        try:
            print(f"Taking off to {altitude}m...")
            
            # Arm the drone first if not armed
            if not self.is_armed:
                await self.drone.action.arm()
                await asyncio.sleep(1)
            
            # Set takeoff altitude
            await self.drone.action.set_takeoff_altitude(altitude)
            
            # Takeoff
            await self.drone.action.takeoff()
            
            # Wait for takeoff to complete
            await self._wait_for_altitude(altitude * 0.9)  # Wait until 90% of target altitude
            
            return {"status": "success", "message": f"Takeoff completed to {altitude}m"}
            
        except Exception as e:
            return {"status": "error", "message": f"Takeoff failed: {e}"}
    
    async def land(self):
        """Land the drone"""
        try:
            print("Landing drone...")
            await self.drone.action.land()
            return {"status": "success", "message": "Landing initiated"}
        except Exception as e:
            return {"status": "error", "message": f"Landing failed: {e}"}
    
    async def move_forward(self, distance, speed=2.0):
        """Move drone forward by specified distance"""
        try:
            if not self.current_position:
                return {"status": "error", "message": "No position data available"}
            
            print(f"Moving forward {distance}m at {speed}m/s...")
            
            # Start offboard mode
            await self.drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))
            await self.drone.offboard.start()
            
            # Calculate movement duration
            duration = distance / speed
            
            # Move forward using body frame velocity
            await self.drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(speed, 0.0, 0.0, 0.0)
            )
            
            # Wait for movement to complete
            await asyncio.sleep(duration)
            
            # Stop movement
            await self.drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
            )
            
            # Hold position
            if self.current_position:
                await self.drone.offboard.set_position_ned(
                    PositionNedYaw(0.0, 0.0, -self.current_altitude, 0.0)
                )
            
            await asyncio.sleep(1)  # Brief hold
            await self.drone.offboard.stop()
            
            return {"status": "success", "message": f"Moved forward {distance}m"}
            
        except Exception as e:
            try:
                await self.drone.offboard.stop()
            except:
                pass
            return {"status": "error", "message": f"Move forward failed: {e}"}
    
    async def get_status(self):
        """Get drone status"""
        try:
            status_info = {
                "connected": self.is_connected,
                "armed": self.is_armed,
                "altitude": self.current_altitude if self.current_altitude else 0.0,
                "position": {
                    "lat": self.current_position.latitude_deg if self.current_position else 0.0,
                    "lon": self.current_position.longitude_deg if self.current_position else 0.0,
                    "alt": self.current_position.relative_altitude_m if self.current_position else 0.0
                } if self.current_position else None
            }
            
            return {
                "status": "success", 
                "message": "Status retrieved",
                "data": status_info
            }
            
        except Exception as e:
            return {"status": "error", "message": f"Failed to get status: {e}"}
    
    async def _wait_for_altitude(self, target_altitude, timeout=30):
        """Wait for drone to reach target altitude"""
        start_time = asyncio.get_event_loop().time()
        
        while True:
            if self.current_altitude and self.current_altitude >= target_altitude:
                break
                
            if asyncio.get_event_loop().time() - start_time > timeout:
                raise Exception(f"Timeout waiting for altitude {target_altitude}m")
                
            await asyncio.sleep(0.5)
    
    async def handle_command(self, command_data):
        """Handle incoming command"""
        action = command_data.get('action')
        
        if action == 'takeoff':
            altitude = command_data.get('altitude', 5.0)
            return await self.takeoff(altitude)
            
        elif action == 'land':
            return await self.land()
            
        elif action == 'move_forward':
            distance = command_data.get('distance', 1.0)
            speed = command_data.get('speed', 2.0)
            return await self.move_forward(distance, speed)
            
        elif action == 'status':
            return await self.get_status()
            
        elif action == 'arm':
            return await self.arm_drone()
            
        elif action == 'disarm':
            return await self.disarm_drone()
            
        else:
            return {"status": "error", "message": f"Unknown command: {action}"}
    
    def start_server(self):
        """Start the command server"""
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind(('localhost', self.port))
            self.server_socket.listen(5)
            self.running = True
            
            print(f"✓ Command server listening on port {self.port}")
            
            while self.running:
                try:
                    client_socket, address = self.server_socket.accept()
                    print(f"Client connected from {address}")
                    
                    # Handle client in separate thread
                    client_thread = threading.Thread(
                        target=self._handle_client,
                        args=(client_socket,)
                    )
                    client_thread.daemon = True
                    client_thread.start()
                    
                except Exception as e:
                    if self.running:
                        print(f"Server error: {e}")
                        
        except Exception as e:
            print(f"Failed to start server: {e}")
    
    def _handle_client(self, client_socket):
        """Handle client connection"""
        try:
            # Receive command
            data = client_socket.recv(1024).decode('utf-8').strip()
            if not data:
                return
                
            command_data = json.loads(data)
            print(f"Received command: {command_data}")
            
            # Execute command in main event loop using asyncio.run_coroutine_threadsafe
            if self.main_loop:
                future = asyncio.run_coroutine_threadsafe(
                    self.handle_command(command_data), 
                    self.main_loop
                )
                response = future.result(timeout=30)  # 30 second timeout
            else:
                response = {"status": "error", "message": "Main event loop not available"}
            
            # Send response
            response_json = json.dumps(response) + '\n'
            client_socket.send(response_json.encode('utf-8'))
            
        except Exception as e:
            error_response = {"status": "error", "message": f"Command handling error: {e}"}
            try:
                client_socket.send(json.dumps(error_response).encode('utf-8'))
            except:
                pass
        finally:
            client_socket.close()
    
    def stop_server(self):
        """Stop the command server"""
        self.running = False
        if self.server_socket:
            self.server_socket.close()
            print("Command server stopped")

async def main():
    """Main function"""
    controller = DroneController()
    
    # Store reference to the main event loop
    controller.main_loop = asyncio.get_event_loop()
    
    # Set up signal handler for graceful shutdown
    def signal_handler(signum, frame):
        print("\nShutting down...")
        controller.stop_server()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # Connect to drone
    connected = await controller.connect_drone()
    if not connected:
        print("Failed to connect to drone. Exiting...")
        sys.exit(1)
    
    # Start command server in separate thread
    server_thread = threading.Thread(target=controller.start_server)
    server_thread.daemon = True
    server_thread.start()
    
    print("Drone controller ready. Press Ctrl+C to exit.")
    
    # Keep main thread alive
    try:
        while True:
            await asyncio.sleep(1)
    except KeyboardInterrupt:
        print("\nShutting down...")
        controller.stop_server()

if __name__ == '__main__':
    asyncio.run(main())