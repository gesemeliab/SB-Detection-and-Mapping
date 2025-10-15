#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Strawberry visualization ROS node.

This node subscribes to detected strawberry positions and publishes
RViz markers for 3D visualization of the strawberry map.

Author: Gesem Gudiño
Date: April 16, 2020
License: GPLv3
"""

import sys
from typing import List

import rospy
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray

# Local imports
from config import PlotterConfig


class StrawberryPlotter:
    """
    ROS node for visualizing detected strawberries in RViz.
    
    This node maintains a persistent map of all detected strawberries
    and publishes MarkerArray messages for visualization.
    """
    
    def __init__(self):
        """Initialize the strawberry plotter node."""
        rospy.init_node('strawberry_plotter_node', anonymous=False)
        rospy.loginfo("Initializing Strawberry Plotter Node...")
        
        # Load configuration
        self.config = PlotterConfig()
        
        # Internal state
        self.markers: List[Marker] = []
        self.next_marker_id = 0
        
        # Initialize ROS interfaces
        self._initialize_ros_interfaces()
        
        rospy.loginfo("Strawberry Plotter Node initialized successfully")
    
    def _initialize_ros_interfaces(self) -> None:
        """Initialize ROS publishers and subscribers."""
        # Publisher for marker array
        self.marker_pub = rospy.Publisher(
            self.config.marker_topic, MarkerArray, queue_size=100
        )
        
        # Subscriber for detections
        self.detection_sub = rospy.Subscriber(
            self.config.detection_topic, PoseStamped, 
            self._detection_callback
        )
        
        rospy.loginfo(f"Subscribed to: {self.config.detection_topic}")
        rospy.loginfo(f"Publishing markers to: {self.config.marker_topic}")
    
    def _detection_callback(self, detection_msg: PoseStamped) -> None:
        """
        Process detected strawberry position and create marker.
        
        Args:
            detection_msg: PoseStamped message with strawberry position
        """
        # Apply calibration offsets
        corrected_position = self._apply_calibration_offsets(detection_msg)
        
        # Create marker
        marker = self._create_marker(corrected_position)
        
        # Add to collection
        self.markers.append(marker)
        self.next_marker_id += 1
        
        # Publish updated marker array
        self._publish_markers()
        
        rospy.loginfo(
            f"Added strawberry marker at ({corrected_position.x:.2f}, "
            f"{corrected_position.y:.2f}, {corrected_position.z:.2f})"
        )
    
    def _apply_calibration_offsets(self, detection_msg: PoseStamped):
        """
        Apply calibration offsets to detection position.
        
        These offsets account for camera mounting position and calibration
        errors between the coordinate systems.
        
        Args:
            detection_msg: Original detection message
            
        Returns:
            Corrected position with offsets applied
        """
        position = detection_msg.pose.position
        
        # Create a copy to avoid modifying original
        from geometry_msgs.msg import Point
        corrected = Point()
        corrected.x = position.x + self.config.offset_x
        corrected.y = position.y + self.config.offset_y
        corrected.z = position.z + self.config.offset_z
        
        return corrected
    
    def _create_marker(self, position) -> Marker:
        """
        Create RViz marker for a strawberry detection.
        
        Args:
            position: Corrected 3D position
            
        Returns:
            Marker message configured for visualization
        """
        marker = Marker()
        
        # Header
        marker.header.frame_id = self.config.world_frame
        marker.header.stamp = rospy.Time.now()
        
        # Marker properties
        marker.ns = "strawberries"
        marker.id = self.next_marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        # Position
        marker.pose.position = position
        marker.pose.orientation.w = 1.0
        
        # Scale
        marker.scale.x = self.config.marker_scale
        marker.scale.y = self.config.marker_scale
        marker.scale.z = self.config.marker_scale
        
        # Color (red for strawberries)
        marker.color.r = self.config.marker_color_r
        marker.color.g = self.config.marker_color_g
        marker.color.b = self.config.marker_color_b
        marker.color.a = self.config.marker_alpha
        
        # Lifetime (0 = forever)
        marker.lifetime = rospy.Duration(0)
        
        return marker
    
    def _publish_markers(self) -> None:
        """Publish all markers as a MarkerArray."""
        marker_array = MarkerArray()
        marker_array.markers = self.markers
        
        self.marker_pub.publish(marker_array)
        
        rospy.logdebug(f"Published {len(self.markers)} markers")
    
    def clear_markers(self) -> None:
        """Clear all markers from the visualization."""
        # Publish DELETE action for all markers
        delete_array = MarkerArray()
        for marker in self.markers:
            delete_marker = Marker()
            delete_marker.header.frame_id = self.config.world_frame
            delete_marker.ns = "strawberries"
            delete_marker.id = marker.id
            delete_marker.action = Marker.DELETE
            delete_array.markers.append(delete_marker)
        
        self.marker_pub.publish(delete_array)
        
        # Clear internal state
        self.markers.clear()
        self.next_marker_id = 0
        
        rospy.loginfo("Cleared all strawberry markers")
    
    def run(self) -> None:
        """Run the node (blocking)."""
        rospy.loginfo("Strawberry plotter node running...")
        rospy.spin()


def main():
    """Main entry point for the node."""
    try:
        node = StrawberryPlotter()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Strawberry plotter node shutting down")
    except Exception as e:
        rospy.logfatal(f"Fatal error in plotter node: {e}")
        sys.exit(1)


if __name__ == '__main__':
    main()
