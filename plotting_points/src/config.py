#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Configuration management for plotting_points package.

Author: Gesem Gudiño
License: GPLv3
"""

import rospy


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
