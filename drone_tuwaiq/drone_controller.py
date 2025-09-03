#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
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
        
        # Configure QoS for PX4 compatibility
        # PX4 uses BEST_EFFORT reliability and VOLATILE durability
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1  # Reduce depth to minimize buffer issues
        )
        
        # Standard QoS for regular ROS2 topics
        standard_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publishers for PX4 commands (use PX4 QoS)
        self.vehicle_command_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', px4_qos)
        self.offboard_control_mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', px4_qos)
        self.trajectory_setpoint_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', px4_qos)
        self.vehicle_rates_setpoint_pub = self.create_publisher(VehicleRatesSetpoint, '/fmu/in/vehicle_rates_setpoint', px4_qos)
        
        # Don't subscribe to status topics due to RTPS buffer issues
        # We'll use commands without feedback for now
        # self.vehicle_status_sub = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, px4_qos)
        # self.vehicle_control_mode_sub = self.create_subscription(VehicleControlMode, '/fmu/out/vehicle_control_mode', self.vehicle_control_mode_callback, px4_qos)
        
        # Subscriber for commands (use standard QoS)
        self.command_sub = self.create_subscription(String, '/drone/command', self.command_callback, standard_qos)
        
        # Timer for control loop
        self.control_timer = self.create_timer(0.1, self.control_loop)
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        # State variables (simplified - no real-time feedback)
        self.movement_active = False
        self.movement_lock = threading.Lock()
        self.offboard_enabled = False
        self.offboard_setpoint_counter = 0
        self.command_id = 0
        self.simulated_armed_state = False
        self.simulated_nav_state = "Manual"
        
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
        
    # Removed callback functions due to RTPS buffer issues
    # Using simplified state management instead
        
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
        
        # First set to manual mode
        self.set_manual_mode()
        
        # Start publishing offboard control mode and setpoints before arming
        self.offboard_enabled = True
        self.offboard_setpoint_counter = 0
        
        # Wait for a few setpoint publications before arming (one-shot timer)
        self.arm_timer = self.create_timer(2.0, self._delayed_arm)
        self.arm_timer_called = False
        self.drone_state = "PREPARING_ARM"
        
    def set_manual_mode(self):
        # Set to manual mode first (required by some PX4 configurations)
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        msg.param1 = 1.0  # main mode
        msg.param2 = 1.0  # MANUAL mode
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
        self.get_logger().info('Set to manual mode')
        
    def _delayed_arm(self):
        # Ensure this only runs once
        if self.arm_timer_called:
            return
        self.arm_timer_called = True
        
        # Destroy the timer to prevent repeated calls
        if hasattr(self, 'arm_timer'):
            self.arm_timer.destroy()
        
        # Send arm command after publishing setpoints for 2 seconds
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        msg.param1 = 1.0  # 1 to arm, 0 to disarm
        msg.param2 = 0.0  # Don't force arm since pre-flight checks are now passing
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
        
        # Simulate arm success after 1 second (since we can't get feedback)
        self.create_timer(1.0, self._simulate_arm_success)
    
    def _simulate_arm_success(self):
        self.simulated_armed_state = True
        self.get_logger().info('🎉 DRONE ARMED SUCCESSFULLY! (simulated)')
        self.drone_state = "ARMED"
    
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
        self.simulated_armed_state = False
    
    def takeoff(self, altitude=2.0):
        self.get_logger().info(f'🚁 Taking off to {altitude}m...')
        
        # Check if drone is armed (simplified check)
        if not self.simulated_armed_state:
            self.get_logger().warn('Drone must be armed before takeoff. Please arm first.')
            return
        
        # Enable offboard mode first
        self.enable_offboard_mode()
        
        # Set takeoff setpoint (hover at altitude)
        self.current_setpoint.position = [0.0, 0.0, -altitude]  # NED frame: negative Z for up
        self.current_setpoint.velocity = [0.0, 0.0, 0.0]
        self.current_setpoint.yaw = float('nan')  # Keep current yaw
        
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
        # Just enable the publishing of offboard control mode and setpoints
        # The actual mode switch will happen after enough setpoints are published
        if not self.offboard_enabled:
            self.offboard_enabled = True
            self.offboard_setpoint_counter = 0
            self.get_logger().info('Started publishing offboard setpoints')
    
    def switch_to_offboard_mode(self):
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
        self.get_logger().info('✅ Offboard mode switch command sent')
        self.simulated_nav_state = "Offboard"
        
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
            
            # Count setpoint publications for offboard mode activation
            self.offboard_setpoint_counter += 1
            
            # Switch to offboard mode after publishing enough setpoints
            if self.offboard_setpoint_counter == 10:
                self.switch_to_offboard_mode()
    
    def publish_status(self):
        # Simplified status without real-time feedback
        armed_state = "Armed" if self.simulated_armed_state else "Disarmed"
        nav_state = self.simulated_nav_state
        
        # Add setpoint counter info when preparing for offboard
        extra_info = ""
        if self.offboard_enabled and self.offboard_setpoint_counter < 10:
            extra_info = f" | Setpoints: {self.offboard_setpoint_counter}/10"
        
        self.get_logger().info(f'Status: {self.drone_state} | {armed_state} | Nav: {nav_state} | Offboard: {self.offboard_enabled}{extra_info}')

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