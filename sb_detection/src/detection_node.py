#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Strawberry detection ROS node.

This node integrates computer vision, deep learning, and coordinate transformation
to detect and localize strawberries in 3D space from RGB-D camera input.

Author: Gesem Gudiño, Andres Montes de Oca, Gerardo Flores
Date: April 16, 2020
License: GPLv3
"""

import sys
from typing import Optional

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
import message_filters

# Local imports
from config import DetectionConfig
from vision_utils import (
    ColorFilterProcessor, CoordinateTransformer, DepthValidator,
    BoundingBoxProcessor, VisualizationHelper
)
from classifier import StrawberryClassifier


class StrawberryDetectionNode:
    """
    ROS node for real-time strawberry detection and 3D localization.
    
    This node subscribes to synchronized RGB, depth, and odometry data,
    detects strawberries using computer vision and CNN classification,
    and publishes 3D positions in the world frame.
    """
    
    def __init__(self):
        """Initialize the strawberry detection node."""
        rospy.init_node('strawberry_detection_node', anonymous=False)
        rospy.loginfo("Initializing Strawberry Detection Node...")
        
        # Load configuration
        self.config = DetectionConfig()
        
        # Initialize components
        self._initialize_processors()
        self._initialize_classifier()
        self._initialize_ros_interfaces()
        
        rospy.loginfo("Strawberry Detection Node initialized successfully")
    
    def _initialize_processors(self) -> None:
        """Initialize computer vision processors."""
        self.bridge = CvBridge()
        
        self.color_filter = ColorFilterProcessor(
            a_threshold=self.config.lab_a_threshold,
            kernel_size=self.config.morphology_kernel_size,
            dilation_iterations=self.config.morphology_iterations
        )
        
        self.coord_transformer = CoordinateTransformer(
            focal_length=self.config.focal_length
        )
        
        self.depth_validator = DepthValidator()
        
        self.bbox_processor = BoundingBoxProcessor(
            padding=self.config.bounding_box_padding
        )
        
        rospy.logdebug("Vision processors initialized")
    
    def _initialize_classifier(self) -> None:
        """Initialize CNN classifier."""
        try:
            self.classifier = StrawberryClassifier(
                model_path=self.config.model_path,
                input_size=self.config.cnn_input_size,
                enable_gpu_growth=self.config.enable_gpu_growth
            )
        except (FileNotFoundError, ValueError) as e:
            rospy.logfatal(f"Failed to initialize classifier: {e}")
            sys.exit(1)
    
    def _initialize_ros_interfaces(self) -> None:
        """Initialize ROS publishers and subscribers."""
        # Publishers
        self.debug_image_pub = rospy.Publisher(
            self.config.debug_image_topic, Image, queue_size=10
        )
        
        self.detection_pub = rospy.Publisher(
            self.config.detection_topic, PoseStamped, queue_size=10
        )
        
        # Synchronized subscribers
        rgb_sub = message_filters.Subscriber(
            self.config.rgb_topic, Image, 
            queue_size=1, buff_size=2**24
        )
        
        depth_sub = message_filters.Subscriber(
            self.config.depth_topic, Image
        )
        
        odom_sub = message_filters.Subscriber(
            self.config.odom_topic, Odometry
        )
        
        # Time synchronizer
        self.sync = message_filters.TimeSynchronizer(
            [rgb_sub, depth_sub, odom_sub], queue_size=10
        )
        self.sync.registerCallback(self._synchronized_callback)
        
        rospy.loginfo("ROS interfaces initialized")
        rospy.loginfo(f"Subscribed to: {self.config.rgb_topic}")
        rospy.loginfo(f"Subscribed to: {self.config.depth_topic}")
        rospy.loginfo(f"Subscribed to: {self.config.odom_topic}")
    
    def _synchronized_callback(self, rgb_msg: Image, depth_msg: Image, 
                               odom_msg: Odometry) -> None:
        """
        Process synchronized RGB-D-Odometry data.
        
        Args:
            rgb_msg: RGB image message
            depth_msg: Depth image message
            odom_msg: Odometry message with robot pose
        """
        try:
            # Convert ROS messages to OpenCV format
            rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough")
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge conversion error: {e}")
            return
        
        # Process frame
        detections = self._process_frame(rgb_image, depth_image, odom_msg)
        
        # Publish results
        self._publish_detections(detections, odom_msg)
        
        # Publish debug visualization
        if self.config.publish_debug_image:
            self._publish_debug_image(rgb_image, rgb_msg.header)
    
    def _process_frame(self, rgb_image: np.ndarray, depth_image: np.ndarray,
                      odom_msg: Odometry) -> list:
        """
        Process a single frame to detect strawberries.
        
        Args:
            rgb_image: RGB image from camera
            depth_image: Depth image from camera
            odom_msg: Robot odometry
            
        Returns:
            List of detection dictionaries
        """
        detections = []
        
        # Get image dimensions
        height, width = rgb_image.shape[:2]
        
        # Apply color filtering to find candidates
        contours, a_channel_3d = self.color_filter.filter_strawberries(rgb_image)
        
        rospy.logdebug(f"Found {len(contours)} candidate regions")
        
        # Process each detected contour
        for contour in contours:
            detection = self._process_contour(
                contour, rgb_image, a_channel_3d, depth_image, 
                width, height, odom_msg
            )
            
            if detection is not None:
                detections.append(detection)
        
        return detections
    
    def _process_contour(self, contour: np.ndarray, rgb_image: np.ndarray,
                        a_channel_3d: np.ndarray, depth_image: np.ndarray,
                        width: int, height: int, odom_msg: Odometry) -> Optional[dict]:
        """
        Process a single contour for strawberry detection.
        
        Args:
            contour: OpenCV contour
            rgb_image: Original RGB image (for visualization)
            a_channel_3d: 3-channel A-channel image (for classification)
            depth_image: Depth image
            width: Image width
            height: Image height
            odom_msg: Robot odometry
            
        Returns:
            Detection dictionary or None if not valid
        """
        # Get padded bounding box
        bbox = self.bbox_processor.get_padded_bounding_box(
            contour, rgb_image.shape
        )
        x, y, w, h = bbox
        
        # Validate bounding box
        if x + w > width:
            rospy.logdebug(f"Bounding box exceeds image width: {x + w} > {width}")
            return None
        
        # Classify with CNN
        label, confidence = self.classifier.predict(a_channel_3d, bbox)
        
        # Filter by confidence threshold
        if confidence < self.config.confidence_threshold:
            rospy.logdebug(f"Low confidence detection: {confidence:.3f}")
            return None
        
        # Calculate centroid
        centroid = self.bbox_processor.get_centroid(bbox)
        
        # Check if in central region
        in_central_region = self.bbox_processor.is_in_central_region(
            centroid, width, self.config.central_region_ratio
        )
        
        # Draw visualization on RGB image
        VisualizationHelper.draw_detection(
            rgb_image, bbox, label, confidence, in_central_region
        )
        
        # Only process detections in central region for 3D localization
        if not in_central_region:
            return None
        
        # Get depth at centroid
        depth = self.depth_validator.get_depth_at_point(depth_image, centroid)
        
        if depth == 0.0:
            rospy.logwarn(f"Invalid depth at centroid {centroid}")
            return None
        
        # Transform to 3D coordinates
        camera_coords = self.coord_transformer.pixel_to_camera_frame(
            centroid, height, width, depth
        )
        
        # Transform to world coordinates
        world_coords = self._camera_to_world_frame(camera_coords, odom_msg)
        
        return {
            'label': label,
            'confidence': confidence,
            'centroid': centroid,
            'bbox': bbox,
            'camera_coords': camera_coords,
            'world_coords': world_coords
        }
    
    def _camera_to_world_frame(self, camera_coords: np.ndarray, 
                               odom_msg: Odometry) -> np.ndarray:
        """
        Transform coordinates from camera frame to world frame.
        
        Args:
            camera_coords: 3D coordinates in camera frame
            odom_msg: Robot odometry for pose information
            
        Returns:
            3D coordinates in world frame
        """
        rover_position = odom_msg.pose.pose.position
        
        world_coords = np.array([
            camera_coords[0] + rover_position.x,
            camera_coords[2] + rover_position.y,  # Note: coordinate axis swap
            camera_coords[1] + rover_position.z
        ])
        
        return world_coords
    
    def _publish_detections(self, detections: list, odom_msg: Odometry) -> None:
        """
        Publish detection results.
        
        Args:
            detections: List of detection dictionaries
            odom_msg: Odometry message for timestamp
        """
        for detection in detections:
            pose_msg = PoseStamped()
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.header.frame_id = "world"
            
            world_coords = detection['world_coords']
            pose_msg.pose.position.x = world_coords[0]
            pose_msg.pose.position.y = world_coords[1]
            pose_msg.pose.position.z = world_coords[2]
            
            self.detection_pub.publish(pose_msg)
            
            rospy.loginfo(
                f"Detected strawberry at ({world_coords[0]:.2f}, "
                f"{world_coords[1]:.2f}, {world_coords[2]:.2f}) "
                f"with confidence {detection['confidence']:.3f}"
            )
    
    def _publish_debug_image(self, image: np.ndarray, header) -> None:
        """
        Publish debug visualization image.
        
        Args:
            image: Annotated image
            header: Original image header for timestamp
        """
        # Resize for visualization
        resized = cv2.resize(image, 
                           (self.config.debug_image_size, 
                            self.config.debug_image_size))
        
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(resized, "bgr8")
            debug_msg.header = header
            self.debug_image_pub.publish(debug_msg)
        except CvBridgeError as e:
            rospy.logerr(f"Failed to publish debug image: {e}")
    
    def run(self) -> None:
        """Run the node (blocking)."""
        rospy.loginfo("Strawberry detection node running...")
        rospy.spin()


def main():
    """Main entry point for the node."""
    try:
        node = StrawberryDetectionNode()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Strawberry detection node shutting down")
    except Exception as e:
        rospy.logfatal(f"Fatal error in detection node: {e}")
        sys.exit(1)


if __name__ == '__main__':
    main()
