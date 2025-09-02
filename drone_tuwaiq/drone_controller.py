#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from px4_msgs.msg import (
    VehicleCommand,
    VehicleControlMode,
    VehicleStatus,
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleAttitudeSetpoint,
    VehicleRatesSetpoint
)
import math
import threading
import time
import numpy as np

class DroneController(Node):
    def __init__(self):
        super().__init__('drone_controller')
        
        # Publishers for PX4 commands
        self.vehicle_command_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        self.offboard_control_mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.trajectory_setpoint_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.vehicle_rates_setpoint_pub = self.create_publisher(VehicleRatesSetpoint, '/fmu/in/vehicle_rates_setpoint', 10)
        
        # Subscribers
        self.vehicle_status_sub = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, 10)
        self.vehicle_control_mode_sub = self.create_subscription(VehicleControlMode, '/fmu/out/vehicle_control_mode', self.vehicle_control_mode_callback, 10)
        self.command_sub = self.create_subscription(String, '/drone/command', self.command_callback, 10)
        
        # Timer for control loop
        self.control_timer = self.create_timer(0.1, self.control_loop)
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        # State variables
        self.vehicle_status = VehicleStatus()
        self.vehicle_control_mode = VehicleControlMode()
        self.movement_active = False
        self.movement_lock = threading.Lock()
        self.offboard_enabled = False
        self.command_id = 0
        
        # Movement parameters
        self.default_velocity = 2.0  # m/s
        self.default_distance = 1.0  # meters
        self.rotation_speed = 0.5    # rad/s
        self.current_setpoint = TrajectorySetpoint()
        
        # Initialize trajectory setpoint
        self.current_setpoint.position = [float('nan'), float('nan'), float('nan')]
        self.current_setpoint.velocity = [0.0, 0.0, 0.0]
        self.current_setpoint.acceleration = [0.0, 0.0, 0.0]
        self.current_setpoint.jerk = [0.0, 0.0, 0.0]
        self.current_setpoint.yaw = float('nan')
        self.current_setpoint.yawspeed = 0.0
        
        self.get_logger().info('Drone Controller Node Started')
        self.get_logger().info('Available commands: arm, disarm, takeoff, land, up <distance>, down <distance>')
        self.get_logger().info('                   forward <distance>, backward <distance>, left <distance>, right <distance>')
        self.get_logger().info('                   rotate_left <degrees>, rotate_right <degrees>, stop')
        
        self.drone_state = "IDLE"
        
    def vehicle_status_callback(self, msg):
        self.vehicle_status = msg
        
    def vehicle_control_mode_callback(self, msg):
        self.vehicle_control_mode = msg
        
    def command_callback(self, msg):
        command_parts = msg.data.lower().strip().split()
        command = command_parts[0]
        self.get_logger().info(f'Received command: {msg.data}')
        
        if command == 'arm':
            self.arm_drone()
        elif command == 'disarm':
            self.disarm_drone()
        elif command == 'takeoff':
            altitude = float(command_parts[1]) if len(command_parts) > 1 else 2.0
            self.takeoff(altitude)
        elif command == 'land':
            self.land()
        elif command == 'up':
            distance = float(command_parts[1]) if len(command_parts) > 1 else self.default_distance
            self.move_with_distance('up', distance)
        elif command == 'down':
            distance = float(command_parts[1]) if len(command_parts) > 1 else self.default_distance
            self.move_with_distance('down', distance)
        elif command == 'forward':
            distance = float(command_parts[1]) if len(command_parts) > 1 else self.default_distance
            self.move_with_distance('forward', distance)
        elif command == 'backward':
            distance = float(command_parts[1]) if len(command_parts) > 1 else self.default_distance
            self.move_with_distance('backward', distance)
        elif command == 'left':
            distance = float(command_parts[1]) if len(command_parts) > 1 else self.default_distance
            self.move_with_distance('left', distance)
        elif command == 'right':
            distance = float(command_parts[1]) if len(command_parts) > 1 else self.default_distance
            self.move_with_distance('right', distance)
        elif command == 'rotate_left':
            degrees = float(command_parts[1]) if len(command_parts) > 1 else 90.0
            self.rotate(-math.radians(degrees))
        elif command == 'rotate_right':
            degrees = float(command_parts[1]) if len(command_parts) > 1 else 90.0
            self.rotate(math.radians(degrees))
        elif command == 'stop':
            self.stop()
        else:
            self.get_logger().warn(f'Unknown command: {command}')
    
    def arm_drone(self):
        self.get_logger().info('🔓 Arming drone...')
        
        # Send arm command
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        msg.param1 = 1.0  # 1 to arm, 0 to disarm
        msg.param2 = 0.0
        msg.param3 = 0.0
        msg.param4 = 0.0
        msg.param5 = 0.0
        msg.param6 = 0.0
        msg.param7 = 0.0
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        
        self.vehicle_command_pub.publish(msg)
        self.get_logger().info('✅ Arm command sent')
        self.drone_state = "ARMING"
    
    def disarm_drone(self):
        self.get_logger().info('🔒 Disarming drone...')
        
        # Send disarm command
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        msg.param1 = 0.0  # 0 to disarm
        msg.param2 = 0.0
        msg.param3 = 0.0
        msg.param4 = 0.0
        msg.param5 = 0.0
        msg.param6 = 0.0
        msg.param7 = 0.0
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        
        self.vehicle_command_pub.publish(msg)
        self.get_logger().info('✅ Disarm command sent')
        self.drone_state = "DISARMING"
    
    def takeoff(self, altitude=2.0):
        self.get_logger().info(f'🚁 Taking off to {altitude}m...')
        
        # Enable offboard mode first
        self.enable_offboard_mode()
        
        # Send takeoff command
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.command = VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF
        msg.param1 = 0.0  # pitch
        msg.param2 = 0.0  # empty
        msg.param3 = 0.0  # empty
        msg.param4 = 0.0  # yaw angle
        msg.param5 = 0.0  # latitude
        msg.param6 = 0.0  # longitude
        msg.param7 = altitude  # altitude
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        
        self.vehicle_command_pub.publish(msg)
        self.get_logger().info('✅ Takeoff command sent')
        self.drone_state = "TAKEOFF"
    
    def land(self):
        self.get_logger().info('🛬 Landing...')
        
        # Send land command
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.command = VehicleCommand.VEHICLE_CMD_NAV_LAND
        msg.param1 = 0.0  # abort altitude
        msg.param2 = 0.0  # land mode
        msg.param3 = 0.0  # empty
        msg.param4 = 0.0  # yaw angle
        msg.param5 = 0.0  # latitude
        msg.param6 = 0.0  # longitude
        msg.param7 = 0.0  # altitude
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        
        self.vehicle_command_pub.publish(msg)
        self.get_logger().info('✅ Landing command sent')
        self.drone_state = "LANDING"
    
    def enable_offboard_mode(self):
        # Set offboard control mode
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        offboard_msg.position = True
        offboard_msg.velocity = True
        offboard_msg.acceleration = False
        offboard_msg.attitude = False
        offboard_msg.body_rate = False
        
        self.offboard_control_mode_pub.publish(offboard_msg)
        
        # Send mode change command to OFFBOARD
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        msg.param1 = 1.0  # main mode
        msg.param2 = 6.0  # OFFBOARD mode
        msg.param3 = 0.0
        msg.param4 = 0.0
        msg.param5 = 0.0
        msg.param6 = 0.0
        msg.param7 = 0.0
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        
        self.vehicle_command_pub.publish(msg)
        self.offboard_enabled = True
        
    def move_with_distance(self, direction, distance):
        with self.movement_lock:
            if self.movement_active:
                self.get_logger().warn('Movement already in progress')
                return
                
            self.movement_active = True
            
        self.get_logger().info(f'Moving {direction} for {distance}m')
        
        # Enable offboard mode if not already enabled
        if not self.offboard_enabled:
            self.enable_offboard_mode()
        
        # Calculate movement time based on distance and velocity
        move_time = distance / self.default_velocity
        
        # Set velocity based on direction (NED frame)
        if direction == 'up':
            self.current_setpoint.velocity = [0.0, 0.0, -self.default_velocity]  # NED frame: negative Z is up
        elif direction == 'down':
            self.current_setpoint.velocity = [0.0, 0.0, self.default_velocity]   # NED frame: positive Z is down
        elif direction == 'forward':
            self.current_setpoint.velocity = [self.default_velocity, 0.0, 0.0]   # NED frame: positive X is forward
        elif direction == 'backward':
            self.current_setpoint.velocity = [-self.default_velocity, 0.0, 0.0]  # NED frame: negative X is backward
        elif direction == 'left':
            self.current_setpoint.velocity = [0.0, -self.default_velocity, 0.0]  # NED frame: negative Y is left
        elif direction == 'right':
            self.current_setpoint.velocity = [0.0, self.default_velocity, 0.0]   # NED frame: positive Y is right
            
        # Start movement in separate thread
        movement_thread = threading.Thread(target=self._execute_movement, args=(move_time,))
        movement_thread.start()
    
    def _execute_movement(self, duration):
        start_time = time.time()
        while time.time() - start_time < duration and self.movement_active:
            # Publish trajectory setpoint
            self.current_setpoint.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            self.trajectory_setpoint_pub.publish(self.current_setpoint)
            time.sleep(0.1)
            
        # Stop movement
        self.stop()
        with self.movement_lock:
            self.movement_active = False
            
    def rotate(self, angle_radians):
        with self.movement_lock:
            if self.movement_active:
                self.get_logger().warn('Movement already in progress')
                return
                
            self.movement_active = True
            
        direction = "left" if angle_radians < 0 else "right"
        degrees = abs(math.degrees(angle_radians))
        self.get_logger().info(f'Rotating {direction} {degrees:.1f} degrees')
        
        # Enable offboard mode if not already enabled
        if not self.offboard_enabled:
            self.enable_offboard_mode()
        
        # Calculate rotation time
        rotation_time = abs(angle_radians) / self.rotation_speed
        
        # Set yaw rate for rotation
        self.current_setpoint.velocity = [0.0, 0.0, 0.0]  # No linear velocity
        self.current_setpoint.yawspeed = self.rotation_speed if angle_radians > 0 else -self.rotation_speed
        
        # Start rotation in separate thread
        rotation_thread = threading.Thread(target=self._execute_movement, args=(rotation_time,))
        rotation_thread.start()
    
    def stop(self):
        self.get_logger().info('⏹️ Stopping...')
        
        # Reset velocities to zero
        self.current_setpoint.velocity = [0.0, 0.0, 0.0]
        self.current_setpoint.yawspeed = 0.0
        
        with self.movement_lock:
            self.movement_active = False
            
        if self.drone_state not in ["LANDING", "LANDED"]:
            self.drone_state = "HOVERING"
    
    def control_loop(self):
        # Publish offboard control mode and trajectory setpoints to maintain OFFBOARD mode
        if self.offboard_enabled:
            # Publish offboard control mode
            offboard_msg = OffboardControlMode()
            offboard_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            offboard_msg.position = True
            offboard_msg.velocity = True
            offboard_msg.acceleration = False
            offboard_msg.attitude = False
            offboard_msg.body_rate = False
            self.offboard_control_mode_pub.publish(offboard_msg)
            
            # Publish trajectory setpoint
            self.current_setpoint.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            self.trajectory_setpoint_pub.publish(self.current_setpoint)
    
    def publish_status(self):
        armed_state = "Armed" if self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED else "Disarmed"
        nav_state_names = {
            VehicleStatus.NAVIGATION_STATE_MANUAL: "Manual",
            VehicleStatus.NAVIGATION_STATE_ALTCTL: "Altitude Control",
            VehicleStatus.NAVIGATION_STATE_POSCTL: "Position Control",
            VehicleStatus.NAVIGATION_STATE_AUTO_MISSION: "Mission",
            VehicleStatus.NAVIGATION_STATE_AUTO_LOITER: "Loiter",
            VehicleStatus.NAVIGATION_STATE_AUTO_RTL: "Return to Launch",
            VehicleStatus.NAVIGATION_STATE_OFFBOARD: "Offboard",
            VehicleStatus.NAVIGATION_STATE_STAB: "Stabilized",
            VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF: "Auto Takeoff",
            VehicleStatus.NAVIGATION_STATE_AUTO_LAND: "Auto Land"
        }
        nav_state = nav_state_names.get(self.vehicle_status.nav_state, f"Unknown({self.vehicle_status.nav_state})")
        
        self.get_logger().info(f'Status: {self.drone_state} | {armed_state} | Nav: {nav_state} | Offboard: {self.offboard_enabled}')

def main(args=None):
    rclpy.init(args=args)
    
    drone_controller = DroneController()
    
    try:
        rclpy.spin(drone_controller)
    except KeyboardInterrupt:
        drone_controller.get_logger().info('Drone Controller shutting down...')
    finally:
        drone_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()