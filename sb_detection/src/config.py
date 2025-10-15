#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Configuration management for strawberry detection system.

This module centralizes all configuration parameters using ROS parameter server
with fallback defaults, following best practices for ROS node configuration.

Author: Gesem Gudiño
License: GPLv3
"""

import rospy
from typing import Dict, Any


class DetectionConfig:
    """Configuration container for strawberry detection parameters."""
    
    def __init__(self):
        """Initialize configuration from ROS parameter server with defaults."""
        # Camera parameters
        self.focal_length = rospy.get_param('~focal_length', 1396.91)
        self.image_width = rospy.get_param('~image_width', 720)
        self.image_height = rospy.get_param('~image_height', 480)
        
        # Detection parameters
        self.model_path = rospy.get_param('~model_path', '')
        self.confidence_threshold = rospy.get_param('~confidence_threshold', 0.98)
        self.cnn_input_size = rospy.get_param('~cnn_input_size', 128)
        
        # Color filtering parameters (LAB color space)
        self.lab_a_threshold = rospy.get_param('~lab_a_threshold', 165)
        self.morphology_kernel_size = rospy.get_param('~morphology_kernel_size', 15)
        self.morphology_iterations = rospy.get_param('~morphology_iterations', 1)
        
        # Region of interest parameters
        self.central_region_ratio = rospy.get_param('~central_region_ratio', 0.05)  # 5% on each side
        self.bounding_box_padding = rospy.get_param('~bounding_box_padding', 5)
        
        # ROS topic names
        self.rgb_topic = rospy.get_param('~rgb_topic', '/zedm/zed_node/left/image_rect_color')
        self.depth_topic = rospy.get_param('~depth_topic', '/zedm/zed_node/depth/depth_registered')
        self.odom_topic = rospy.get_param('~odom_topic', '/zedm/zed_node/odom')
        self.debug_image_topic = rospy.get_param('~debug_image_topic', 'debug_image')
        self.detection_topic = rospy.get_param('~detection_topic', 'sb_coordinates')
        
        # Visualization parameters
        self.publish_debug_image = rospy.get_param('~publish_debug_image', True)
        self.debug_image_size = rospy.get_param('~debug_image_size', 480)
        
        # GPU configuration
        self.enable_gpu_growth = rospy.get_param('~enable_gpu_growth', True)
        
        self._validate()
        self._log_configuration()
    
    def _validate(self):
        """Validate configuration parameters."""
        if not self.model_path:
            raise ValueError("Model path cannot be empty. Set ~model_path parameter.")
        
        if self.confidence_threshold < 0.0 or self.confidence_threshold > 1.0:
            raise ValueError(f"Confidence threshold must be in [0, 1], got {self.confidence_threshold}")
        
        if self.focal_length <= 0:
            raise ValueError(f"Focal length must be positive, got {self.focal_length}")
        
        if self.cnn_input_size <= 0:
            raise ValueError(f"CNN input size must be positive, got {self.cnn_input_size}")
    
    def _log_configuration(self):
        """Log current configuration for debugging."""
        rospy.loginfo("=== Strawberry Detection Configuration ===")
        rospy.loginfo(f"Model path: {self.model_path}")
        rospy.loginfo(f"Focal length: {self.focal_length}")
        rospy.loginfo(f"Confidence threshold: {self.confidence_threshold}")
        rospy.loginfo(f"LAB A-channel threshold: {self.lab_a_threshold}")
        rospy.loginfo(f"Central region ratio: {self.central_region_ratio}")
        rospy.loginfo(f"RGB topic: {self.rgb_topic}")
        rospy.loginfo(f"Depth topic: {self.depth_topic}")
        rospy.loginfo("=========================================")


class PlotterConfig:
    """Configuration container for visualization parameters."""
    
    def __init__(self):
        """Initialize plotter configuration from ROS parameter server."""
        # Marker parameters
        self.marker_scale = rospy.get_param('~marker_scale', 0.05)
        self.marker_color_r = rospy.get_param('~marker_color_r', 1.0)
        self.marker_color_g = rospy.get_param('~marker_color_g', 0.0)
        self.marker_color_b = rospy.get_param('~marker_color_b', 0.0)
        self.marker_alpha = rospy.get_param('~marker_alpha', 1.0)
        
        # Coordinate transformation offsets (camera calibration)
        self.offset_x = rospy.get_param('~offset_x', 0.545)  # 0.21 + 0.335
        self.offset_y = rospy.get_param('~offset_y', -0.495)
        self.offset_z = rospy.get_param('~offset_z', 0.39)
        
        # Frame IDs
        self.world_frame = rospy.get_param('~world_frame', 'world')
        
        # ROS topics
        self.detection_topic = rospy.get_param('~detection_topic', '/sb_coordinates')
        self.marker_topic = rospy.get_param('~marker_topic', 'markers')
        
        self._log_configuration()
    
    def _log_configuration(self):
        """Log current configuration for debugging."""
        rospy.loginfo("=== Strawberry Plotter Configuration ===")
        rospy.loginfo(f"Marker scale: {self.marker_scale}")
        rospy.loginfo(f"World frame: {self.world_frame}")
        rospy.loginfo(f"Coordinate offsets: ({self.offset_x}, {self.offset_y}, {self.offset_z})")
        rospy.loginfo(f"Detection topic: {self.detection_topic}")
        rospy.loginfo(f"Marker topic: {self.marker_topic}")
        rospy.loginfo("=======================================")
