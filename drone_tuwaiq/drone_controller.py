#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, String
from geometry_msgs.msg import Twist, PoseStamped
import time

class DroneController(Node):
    def __init__(self):
        super().__init__('drone_controller')
        
        # Publishers for drone commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/fmu/in/setpoint_velocity', 10)
        self.arm_pub = self.create_publisher(Empty, '/drone/arm', 10)
        self.disarm_pub = self.create_publisher(Empty, '/drone/disarm', 10)
        self.takeoff_pub = self.create_publisher(Empty, '/drone/takeoff', 10)
        self.land_pub = self.create_publisher(Empty, '/drone/land', 10)
        
        # Subscriber for commands
        self.command_sub = self.create_subscription(
            String, 
            '/drone/command', 
            self.command_callback, 
            10
        )
        
        # Timer for status updates
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        self.get_logger().info('Drone Controller Node Started')
        self.get_logger().info('Available commands: arm, disarm, takeoff, land, up, down, forward, backward, left, right, stop')
        
        self.current_state = "IDLE"
        
    def command_callback(self, msg):
        command = msg.data.lower().strip()
        self.get_logger().info(f'Received command: {command}')
        
        if command == 'arm':
            self.arm_drone()
        elif command == 'disarm':
            self.disarm_drone()
        elif command == 'takeoff':
            self.takeoff()
        elif command == 'land':
            self.land()
        elif command == 'up':
            self.move_up()
        elif command == 'down':
            self.move_down()
        elif command == 'forward':
            self.move_forward()
        elif command == 'backward':
            self.move_backward()
        elif command == 'left':
            self.move_left()
        elif command == 'right':
            self.move_right()
        elif command == 'stop':
            self.stop()
        else:
            self.get_logger().warn(f'Unknown command: {command}')
    
    def arm_drone(self):
        self.get_logger().info('🔓 Arming drone...')
        self.arm_pub.publish(Empty())
        self.current_state = "ARMED"
    
    def disarm_drone(self):
        self.get_logger().info('🔒 Disarming drone...')
        self.disarm_pub.publish(Empty())
        self.current_state = "DISARMED"
    
    def takeoff(self):
        self.get_logger().info('🚁 Taking off...')
        self.takeoff_pub.publish(Empty())
        self.current_state = "TAKEOFF"
        
        # Send upward velocity for takeoff
        vel_msg = Twist()
        vel_msg.linear.z = 2.0  # 2 m/s upward
        for i in range(10):  # Send for 1 second
            self.cmd_vel_pub.publish(vel_msg)
            time.sleep(0.1)
        self.stop()
        self.current_state = "HOVERING"
    
    def land(self):
        self.get_logger().info('🛬 Landing...')
        self.land_pub.publish(Empty())
        self.current_state = "LANDING"
        
        # Send downward velocity for landing
        vel_msg = Twist()
        vel_msg.linear.z = -1.0  # 1 m/s downward
        for i in range(20):  # Send for 2 seconds
            self.cmd_vel_pub.publish(vel_msg)
            time.sleep(0.1)
        self.stop()
        self.current_state = "LANDED"
    
    def move_up(self):
        self.get_logger().info('⬆️ Moving up...')
        vel_msg = Twist()
        vel_msg.linear.z = 1.0
        self.cmd_vel_pub.publish(vel_msg)
    
    def move_down(self):
        self.get_logger().info('⬇️ Moving down...')
        vel_msg = Twist()
        vel_msg.linear.z = -1.0
        self.cmd_vel_pub.publish(vel_msg)
    
    def move_forward(self):
        self.get_logger().info('➡️ Moving forward...')
        vel_msg = Twist()
        vel_msg.linear.x = 1.0
        self.cmd_vel_pub.publish(vel_msg)
    
    def move_backward(self):
        self.get_logger().info('⬅️ Moving backward...')
        vel_msg = Twist()
        vel_msg.linear.x = -1.0
        self.cmd_vel_pub.publish(vel_msg)
    
    def move_left(self):
        self.get_logger().info('⬅️ Moving left...')
        vel_msg = Twist()
        vel_msg.linear.y = 1.0
        self.cmd_vel_pub.publish(vel_msg)
    
    def move_right(self):
        self.get_logger().info('➡️ Moving right...')
        vel_msg = Twist()
        vel_msg.linear.y = -1.0
        self.cmd_vel_pub.publish(vel_msg)
    
    def stop(self):
        self.get_logger().info('⏹️ Stopping...')
        vel_msg = Twist()  # All zeros
        self.cmd_vel_pub.publish(vel_msg)
        if self.current_state not in ["LANDING", "LANDED"]:
            self.current_state = "HOVERING"
    
    def publish_status(self):
        self.get_logger().info(f'Status: {self.current_state}')

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