#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from yolo_msgs.msg import DetectionArray
import cv2
from cv_bridge import CvBridge


class GimbalYoloNode(Node):
    """
    ROS2 node that subscribes to drone gimbal camera and republishes 
    images for YOLO detection processing.
    """
    
    def __init__(self):
        super().__init__('gimbal_yolo_node')
        
        # Initialize CV Bridge
        self.cv_bridge = CvBridge()
        
        # QoS Profile for image subscription (best effort for camera feeds)
        image_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )
        
        # Subscriber to gimbal camera
        self.image_subscription = self.create_subscription(
            Image,
            '/drone/gimbal_camera',
            self.image_callback,
            image_qos_profile
        )
        
        
        # Subscriber to YOLO detections
        self.detection_subscription = self.create_subscription(
            DetectionArray,
            'yolo/detections',
            self.detection_callback,
            10
        )
        
        # Publisher for processed detections (with drone-specific processing)
        self.processed_detections_publisher = self.create_publisher(
            DetectionArray,
            '/drone/gimbal_detections_yolo',
            10
        )
        
        self.get_logger().info('Gimbal YOLO Node initialized')
        self.get_logger().info('Subscribing to: /drone/gimbal_camera')
        self.get_logger().info('YOLO reads directly from: /drone/gimbal_camera')
        self.get_logger().info('Publishing detections to: /drone/gimbal_detections_yolo')
    
    def image_callback(self, msg: Image):
        """
        Callback for gimbal camera images.
        Republishes images to YOLO node for processing.
        """
        try:
            # Log image reception (optional, remove if too verbose)
            self.get_logger().debug(
                f'Received image: {msg.width}x{msg.height}, encoding: {msg.encoding}'
            )
            
            # YOLO now reads directly from /drone/gimbal_camera
            pass
            
        except Exception as e:
            self.get_logger().error(f'Error processing gimbal image: {str(e)}')
    
    def detection_callback(self, msg: DetectionArray):
        """
        Callback for YOLO detections.
        Process and republish detections with drone-specific information.
        """
        try:
            # Log detection results
            num_detections = len(msg.detections)
            if num_detections > 0:
                self.get_logger().info(f'Received {num_detections} detections')
                
                # Log each detection
                for i, detection in enumerate(msg.detections):
                    self.get_logger().info(
                        f'Detection {i+1}: {detection.class_name} '
                        f'(confidence: {detection.score:.2f})'
                    )
            
            # Add custom drone-specific processing here if needed
            # For now, just republish the detections
            processed_msg = msg  # Could add custom processing here
            
            # Republish detections
            self.processed_detections_publisher.publish(processed_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing detections: {str(e)}')
    
    def visualize_detections(self, image_msg: Image, detections: DetectionArray):
        """
        Optional method to visualize detections on the image.
        Call this method if you want to publish annotated images.
        """
        try:
            # Convert ROS image to OpenCV
            cv_image = self.cv_bridge.imgmsg_to_cv2(image_msg, 'bgr8')
            
            # Draw bounding boxes and labels
            for detection in detections.detections:
                if detection.bbox:
                    # Extract bounding box coordinates from yolo_msgs format
                    center_x = int(detection.bbox.center.position.x)
                    center_y = int(detection.bbox.center.position.y)
                    width = int(detection.bbox.size.x)
                    height = int(detection.bbox.size.y)
                    
                    # Calculate corner points
                    x1 = center_x - width // 2
                    y1 = center_y - height // 2
                    x2 = center_x + width // 2
                    y2 = center_y + height // 2
                    
                    # Draw bounding box
                    cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    
                    # Draw label using yolo_msgs format
                    label = f'{detection.class_name}: {detection.score:.2f}'
                    cv2.putText(cv_image, label, (x1, y1 - 10), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # Convert back to ROS image and return
            annotated_msg = self.cv_bridge.cv2_to_imgmsg(cv_image, 'bgr8')
            annotated_msg.header = image_msg.header
            return annotated_msg
            
        except Exception as e:
            self.get_logger().error(f'Error visualizing detections: {str(e)}')
            return image_msg


def main(args=None):
    rclpy.init(args=args)
    
    gimbal_yolo_node = GimbalYoloNode()
    
    try:
        rclpy.spin(gimbal_yolo_node)
    except KeyboardInterrupt:
        pass
    finally:
        gimbal_yolo_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()