#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from yolo_msgs.msg import DetectionArray
import cv2
from cv_bridge import CvBridge
import math
from collections import defaultdict
import time


class GimbalYoloNode(Node):
    """
    ROS2 node that subscribes to drone gimbal camera and republishes 
    images for YOLO detection processing.
    """
    
    def __init__(self):
        super().__init__('gimbal_yolo_node')
        
        # Initialize CV Bridge
        self.cv_bridge = CvBridge()
        
        # Person tracking variables
        self.tracked_persons = {}  # {person_id: {'bbox': bbox, 'last_seen': timestamp, 'confidence': confidence}}
        self.person_id_counter = 0
        self.total_unique_persons = 0
        self.bbox_similarity_threshold = 50.0  # pixels
        self.person_timeout = 5.0  # seconds - remove person if not seen for this long
        
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
        self.get_logger().info('Person tracking enabled - counting unique individuals')
    
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
            current_time = time.time()
            
            # Filter person detections
            person_detections = [d for d in msg.detections if d.class_name == 'person']
            
            if person_detections:
                # Track persons and update count
                self.track_persons(person_detections, current_time)
                
                # Clean up old persons
                self.cleanup_old_persons(current_time)
                
                # Log current person count
                active_persons = len([p for p in self.tracked_persons.values() 
                                    if current_time - p['last_seen'] < self.person_timeout])
                
                self.get_logger().info(
                    f'👥 PERSON COUNT: {active_persons} currently visible | '
                    f'{self.total_unique_persons} total unique persons detected'
                )
                
                # Log all detections
                for i, detection in enumerate(msg.detections):
                    if detection.class_name == 'person':
                        person_id = self.get_person_id_from_detection(detection)
                        self.get_logger().info(
                            f'Person #{person_id}: confidence {detection.score:.2f}'
                        )
                    else:
                        self.get_logger().info(
                            f'{detection.class_name}: confidence {detection.score:.2f}'
                        )
            else:
                # Clean up old persons even if no new detections
                self.cleanup_old_persons(current_time)
                
                if len(msg.detections) > 0:
                    self.get_logger().info(
                        f'Received {len(msg.detections)} non-person detections'
                    )
            
            # Republish detections
            self.processed_detections_publisher.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing detections: {str(e)}')
    
    def calculate_bbox_distance(self, bbox1, bbox2):
        """
        Calculate distance between two bounding boxes using center points.
        """
        center1_x = bbox1['center_x']
        center1_y = bbox1['center_y']
        center2_x = bbox2['center_x']
        center2_y = bbox2['center_y']
        
        return math.sqrt((center1_x - center2_x)**2 + (center1_y - center2_y)**2)
    
    def extract_bbox_info(self, detection):
        """
        Extract bounding box information from detection.
        """
        return {
            'center_x': detection.bbox.center.position.x,
            'center_y': detection.bbox.center.position.y,
            'width': detection.bbox.size.x,
            'height': detection.bbox.size.y
        }
    
    def get_person_id_from_detection(self, detection):
        """
        Find or assign person ID for a detection.
        """
        bbox_info = self.extract_bbox_info(detection)
        
        # Try to match with existing persons
        for person_id, person_data in self.tracked_persons.items():
            distance = self.calculate_bbox_distance(bbox_info, person_data['bbox'])
            if distance < self.bbox_similarity_threshold:
                return person_id
        
        # If no match found, this is a new person
        return None
    
    def track_persons(self, person_detections, current_time):
        """
        Track persons and assign unique IDs.
        """
        for detection in person_detections:
            bbox_info = self.extract_bbox_info(detection)
            person_id = None
            
            # Try to match with existing persons
            best_match_id = None
            best_distance = float('inf')
            
            for existing_id, person_data in self.tracked_persons.items():
                distance = self.calculate_bbox_distance(bbox_info, person_data['bbox'])
                if distance < self.bbox_similarity_threshold and distance < best_distance:
                    best_distance = distance
                    best_match_id = existing_id
            
            if best_match_id is not None:
                # Update existing person
                person_id = best_match_id
                self.tracked_persons[person_id].update({
                    'bbox': bbox_info,
                    'last_seen': current_time,
                    'confidence': detection.score
                })
            else:
                # New person detected
                self.person_id_counter += 1
                self.total_unique_persons += 1
                person_id = self.person_id_counter
                
                self.tracked_persons[person_id] = {
                    'bbox': bbox_info,
                    'last_seen': current_time,
                    'confidence': detection.score
                }
                
                self.get_logger().info(
                    f'🆕 NEW PERSON DETECTED! Assigned ID #{person_id} '
                    f'(Total unique persons: {self.total_unique_persons})'
                )
    
    def cleanup_old_persons(self, current_time):
        """
        Remove persons that haven't been seen for a while.
        """
        persons_to_remove = []
        
        for person_id, person_data in self.tracked_persons.items():
            if current_time - person_data['last_seen'] > self.person_timeout:
                persons_to_remove.append(person_id)
        
        for person_id in persons_to_remove:
            del self.tracked_persons[person_id]
            self.get_logger().info(f'👋 Person #{person_id} left the scene')
    
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