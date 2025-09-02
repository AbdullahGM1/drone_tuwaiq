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
        self.takeoff_position = None  # Store takeoff position for relative positioning
        
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
            
            # Store takeoff position for relative positioning
            if self.current_position:
                self.takeoff_position = {
                    'lat': self.current_position.latitude_deg,
                    'lon': self.current_position.longitude_deg,
                    'alt': self.current_position.absolute_altitude_m
                }
                print(f"Takeoff position stored: {self.takeoff_position}")
            
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
            
            # Clear takeoff position after landing
            self.takeoff_position = None
            
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
    
    async def goto_position(self, x, y, z):
        """Go to specific position (x, y, z) in local NED frame relative to takeoff point"""
        try:
            if not self.current_position:
                return {"status": "error", "message": "No position data available"}
            
            if not self.takeoff_position:
                return {"status": "error", "message": "No takeoff position reference. Please takeoff first."}
            
            print(f"Going to position: x={x}m, y={y}m, z={z}m (NED frame relative to takeoff)...")
            print(f"Current altitude: {self.current_altitude}m")
            
            # Calculate target position in NED frame
            target_z_ned = -abs(z)  # Convert altitude to NED down coordinate
            print(f"Target NED position: N={x}m, E={y}m, D={target_z_ned}m")
            
            # Use velocity-based approach for more reliable movement
            return await self._goto_with_velocity(x, y, z)
            
        except Exception as e:
            error_msg = f"Goto position failed: {e}"
            print(f"Error in goto_position: {error_msg}")
            return {"status": "error", "message": error_msg}
    
    async def _goto_with_velocity(self, target_x, target_y, target_z):
        """Move to position using velocity control for more reliable movement"""
        try:
            print("Starting velocity-based goto...")
            
            # Start offboard mode with current position
            current_z_ned = -self.current_altitude if self.current_altitude else -1.0
            print(f"Starting offboard mode at current NED position (0, 0, {current_z_ned})")
            
            await self.drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, current_z_ned, 0.0))
            await self.drone.offboard.start()
            await asyncio.sleep(2)  # Give offboard mode time to engage
            
            # Check if we need to move in altitude first
            altitude_diff = abs(self.current_altitude - target_z)
            if altitude_diff > 0.5:  # If altitude difference is significant
                print(f"Moving to target altitude: {target_z}m...")
                await self._move_to_altitude(target_z)
                await asyncio.sleep(2)
            
            # Now move horizontally if needed
            horizontal_distance = (target_x**2 + target_y**2)**0.5
            if horizontal_distance > 0.5:  # If horizontal movement is needed
                print(f"Moving horizontally: {horizontal_distance:.2f}m...")
                await self._move_horizontally(target_x, target_y, target_z)
            
            # Final position hold
            target_z_ned = -abs(target_z)
            print(f"Final position hold at: N={target_x}m, E={target_y}m, D={target_z_ned}m")
            await self.drone.offboard.set_position_ned(PositionNedYaw(target_x, target_y, target_z_ned, 0.0))
            await asyncio.sleep(3)
            
            print("Stopping offboard mode...")
            await self.drone.offboard.stop()
            
            return {"status": "success", "message": f"Successfully reached position x={target_x}m, y={target_y}m, z={target_z}m"}
            
        except Exception as e:
            try:
                await self.drone.offboard.stop()
            except:
                pass
            raise e
    
    async def _move_to_altitude(self, target_altitude):
        """Move to target altitude using velocity control"""
        max_speed = 2.0  # m/s
        tolerance = 0.5  # meters
        
        for _ in range(30):  # Max 30 seconds
            if not self.current_altitude:
                await asyncio.sleep(1)
                continue
                
            altitude_error = target_altitude - self.current_altitude
            
            if abs(altitude_error) < tolerance:
                print(f"Reached target altitude: {self.current_altitude:.2f}m")
                break
            
            # Calculate velocity
            velocity_z = max(-max_speed, min(max_speed, altitude_error * 0.5))
            
            print(f"Altitude: {self.current_altitude:.2f}m -> {target_altitude}m, vel_z: {velocity_z:.2f}m/s")
            
            # Send velocity command (NED: positive Z is down, so negate for up movement)
            await self.drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(0.0, 0.0, -velocity_z, 0.0)
            )
            
            await asyncio.sleep(1)
    
    async def _move_horizontally(self, target_x, target_y, target_z):
        """Move horizontally to target position"""
        max_speed = 2.0  # m/s
        tolerance = 1.0  # meters
        current_x, current_y = 0.0, 0.0  # Assuming we start at origin
        
        for _ in range(60):  # Max 60 seconds for horizontal movement
            # Calculate errors
            error_x = target_x - current_x
            error_y = target_y - current_y
            distance = (error_x**2 + error_y**2)**0.5
            
            if distance < tolerance:
                print(f"Reached horizontal target position")
                break
            
            # Calculate velocities (proportional control)
            if distance > 0.1:  # Avoid division by zero
                velocity_x = max(-max_speed, min(max_speed, error_x * 0.5))
                velocity_y = max(-max_speed, min(max_speed, error_y * 0.5))
            else:
                velocity_x = velocity_y = 0.0
            
            print(f"Horizontal movement: distance={distance:.2f}m, vel_x={velocity_x:.2f}, vel_y={velocity_y:.2f}")
            
            # Send velocity command in body frame
            await self.drone.offboard.set_velocity_body(
                VelocityBodyYawspeed(velocity_x, velocity_y, 0.0, 0.0)
            )
            
            # Update estimated position (rough estimation)
            current_x += velocity_x * 1.0  # 1 second update
            current_y += velocity_y * 1.0
            
            await asyncio.sleep(1)
    
    async def get_status(self):
        """Get drone status"""
        try:
            # Get additional telemetry data
            flight_mode = None
            battery_info = None
            gps_info = None
            
            try:
                # Try to get flight mode (may not be available in all setups)
                async for mode in self.drone.telemetry.flight_mode():
                    flight_mode = str(mode)
                    break
            except:
                flight_mode = "Unknown"
            
            try:
                # Try to get battery info (may not be available in SITL)
                async for battery in self.drone.telemetry.battery():
                    battery_info = {
                        "remaining": battery.remaining_percent,
                        "voltage": battery.voltage_v
                    }
                    break
            except:
                battery_info = None
            
            try:
                # Try to get GPS info
                async for gps in self.drone.telemetry.gps_info():
                    gps_info = {
                        "satellites": gps.num_satellites,
                        "fix_type": str(gps.fix_type)
                    }
                    break
            except:
                gps_info = None
            
            status_info = {
                "connected": self.is_connected,
                "armed": self.is_armed,
                "altitude": self.current_altitude if self.current_altitude else 0.0,
                "flight_mode": flight_mode,
                "position": {
                    "lat": self.current_position.latitude_deg if self.current_position else 0.0,
                    "lon": self.current_position.longitude_deg if self.current_position else 0.0,
                    "alt": self.current_position.relative_altitude_m if self.current_position else 0.0
                } if self.current_position else None,
                "takeoff_position_set": self.takeoff_position is not None,
                "battery": battery_info,
                "gps": gps_info
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
    
    async def _wait_for_position(self, target_x, target_y, target_z, timeout=90, tolerance=2.0):
        """Wait for drone to reach target position"""
        start_time = asyncio.get_event_loop().time()
        
        print(f"Waiting for drone to reach position (tolerance: {tolerance}m, timeout: {timeout}s)...")
        
        last_alt = None
        stable_count = 0
        
        while True:
            if not self.current_position:
                await asyncio.sleep(0.5)
                continue
            
            # Get current altitude
            current_alt = self.current_altitude if self.current_altitude else 0.0
            
            # Check altitude difference (main check for now)
            alt_diff = abs(current_alt - target_z)
            
            # Print progress every few seconds or when altitude changes significantly
            if last_alt is None or abs(current_alt - last_alt) > 0.5:
                print(f"Current altitude: {current_alt:.2f}m, Target: {target_z}m, Diff: {alt_diff:.2f}m")
                last_alt = current_alt
            
            # Check if we're close enough
            if alt_diff <= tolerance:
                stable_count += 1
                if stable_count >= 3:  # Require stable position for 3 checks
                    print(f"Position reached! (altitude within {tolerance}m tolerance)")
                    break
            else:
                stable_count = 0
                
            # Check timeout
            elapsed = asyncio.get_event_loop().time() - start_time
            if elapsed > timeout:
                print(f"Timeout after {elapsed:.1f}s - Current alt: {current_alt:.2f}m, Target: {target_z}m")
                raise Exception(f"Timeout waiting for position x={target_x}, y={target_y}, z={target_z}")
                
            await asyncio.sleep(1.0)
    
    async def handle_command(self, command_data):
        """Handle incoming command"""
        action = command_data.get('action')
        
        print(f"Processing command: {action}")
        
        if action == 'takeoff':
            altitude = command_data.get('altitude', 5.0)
            return await self.takeoff(altitude)
            
        elif action == 'land':
            return await self.land()
            
        elif action == 'move_forward':
            distance = command_data.get('distance', 1.0)
            speed = command_data.get('speed', 2.0)
            return await self.move_forward(distance, speed)
            
        elif action == 'goto':
            x = command_data.get('x', 0.0)
            y = command_data.get('y', 0.0)
            z = command_data.get('z', 5.0)
            print(f"Goto command received: x={x}, y={y}, z={z}")
            return await self.goto_position(x, y, z)
            
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
                    self.server_socket.settimeout(1.0)  # Allow periodic checks
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
                    except socket.timeout:
                        continue  # Continue to check if running
                        
                except Exception as e:
                    if self.running:
                        print(f"Server error: {e}")
                        
        except Exception as e:
            print(f"Failed to start server: {e}")
    
    def _handle_client(self, client_socket):
        """Handle client connection"""
        try:
            # Set longer timeout for client socket to handle complex operations
            client_socket.settimeout(150.0)  # 2.5 minutes timeout
            
            # Receive command
            data = client_socket.recv(4096).decode('utf-8').strip()
            if not data:
                return
                
            command_data = json.loads(data)
            print(f"Received command from client: {command_data}")
            
            # Execute command in main event loop using asyncio.run_coroutine_threadsafe
            if self.main_loop:
                # Determine timeout based on command type
                command_timeout = 30.0  # Default timeout
                if command_data.get('action') == 'goto':
                    command_timeout = 120.0  # 2 minutes for goto commands
                elif command_data.get('action') == 'takeoff':
                    command_timeout = 60.0   # 1 minute for takeoff
                
                future = asyncio.run_coroutine_threadsafe(
                    self.handle_command(command_data), 
                    self.main_loop
                )
                response = future.result(timeout=command_timeout)
            else:
                response = {"status": "error", "message": "Main event loop not available"}
            
            # Send response
            response_json = json.dumps(response, indent=2) + '\n'
            client_socket.send(response_json.encode('utf-8'))
            print(f"Sent response: {response.get('message', 'Unknown')}")
            
        except socket.timeout:
            print("Client connection timed out during command execution")
            error_response = {"status": "error", "message": "Command timed out"}
            try:
                response_json = json.dumps(error_response) + '\n'
                client_socket.send(response_json.encode('utf-8'))
            except:
                pass
        except Exception as e:
            print(f"Error handling client: {e}")
            error_response = {"status": "error", "message": f"Command handling error: {e}"}
            try:
                response_json = json.dumps(error_response) + '\n'
                client_socket.send(response_json.encode('utf-8'))
            except:
                pass
        finally:
            try:
                client_socket.close()
            except:
                pass
    
    def stop_server(self):
        """Stop the command server"""
        print("Stopping command server...")
        self.running = False
        if self.server_socket:
            try:
                self.server_socket.close()
            except:
                pass
            print("Command server stopped")

async def main():
    """Main function"""
    print("🚁 Drone Controller Starting...")
    controller = DroneController()
    
    # Store reference to the main event loop
    controller.main_loop = asyncio.get_event_loop()
    
    # Set up signal handler for graceful shutdown
    def signal_handler(signum, frame):
        print("\nShutting down gracefully...")
        controller.stop_server()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # Connect to drone
    print("Connecting to drone...")
    connected = await controller.connect_drone()
    if not connected:
        print("Failed to connect to drone. Exiting...")
        sys.exit(1)
    
    # Start command server in separate thread
    server_thread = threading.Thread(target=controller.start_server)
    server_thread.daemon = True
    server_thread.start()
    
    print("✓ Drone controller ready!")
    print("✓ Waiting for CLI commands...")
    print("Press Ctrl+C to exit.")
    
    # Keep main thread alive
    try:
        while True:
            await asyncio.sleep(1)
    except KeyboardInterrupt:
        print("\nShutting down...")
        controller.stop_server()

if __name__ == '__main__':
    asyncio.run(main())