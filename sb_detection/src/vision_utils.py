#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Computer vision utilities for strawberry detection.

This module contains pure computer vision functions for image processing,
filtering, and coordinate transformations, following functional programming
principles for testability.

Author: Gesem Gudiño
License: GPLv3
"""

import math
import cv2
import numpy as np
from typing import Tuple, List, Optional
import rospy


class DepthValidator:
    """Utility class for validating and processing depth information."""
    
    @staticmethod
    def get_depth_at_point(depth_image: np.ndarray, 
                          point: Tuple[int, int]) -> float:
        """
        Extract depth value at specified pixel coordinates.
        
        Args:
            depth_image: Depth image array from stereo camera
            point: Tuple of (x, y) pixel coordinates
            
        Returns:
            Depth value in meters, or 0.0 if invalid
        """
        if point is None:
            rospy.logwarn("Depth requested for None point")
            return 0.0
        
        x, y = point
        
        # Validate coordinates
        if not (0 <= y < depth_image.shape[0] and 0 <= x < depth_image.shape[1]):
            rospy.logwarn(f"Point ({x}, {y}) outside image bounds")
            return 0.0
        
        depth = float(depth_image[y, x])
        
        # Handle invalid depth values
        if math.isinf(depth) or math.isnan(depth):
            rospy.logdebug(f"Invalid depth at ({x}, {y}): {depth}")
            return 0.0
        
        return depth


class ColorFilterProcessor:
    """Processes images using LAB color space filtering for strawberry detection."""
    
    def __init__(self, a_threshold: int = 165, 
                 kernel_size: int = 15,
                 dilation_iterations: int = 1):
        """
        Initialize color filter processor.
        
        Args:
            a_threshold: Threshold value for LAB A-channel (red-green axis)
            kernel_size: Size of morphological operation kernel
            dilation_iterations: Number of dilation iterations
        """
        self.a_threshold = a_threshold
        self.kernel = np.ones((kernel_size, kernel_size), np.uint8)
        self.dilation_iterations = dilation_iterations
    
    def filter_strawberries(self, rgb_image: np.ndarray) -> Tuple[List, np.ndarray]:
        """
        Apply LAB color space filtering to detect strawberry candidates.
        
        Args:
            rgb_image: Input RGB image (BGR format from OpenCV)
            
        Returns:
            Tuple of (contours, a_channel_3d):
                - contours: List of detected contours
                - a_channel_3d: 3-channel A-channel image for visualization
        """
        # Convert to LAB color space
        lab_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2LAB)
        
        # Extract A-channel (red-green axis)
        a_channel = lab_image[:, :, 1]
        
        # Binarize based on threshold
        _, binary = cv2.threshold(a_channel, self.a_threshold, 255, 
                                 cv2.THRESH_BINARY)
        
        # Apply morphological operations to remove noise
        filtered = cv2.morphologyEx(binary, cv2.MORPH_OPEN, self.kernel)
        filtered = cv2.dilate(filtered, self.kernel, 
                             iterations=self.dilation_iterations)
        
        # Find contours
        contours, _ = cv2.findContours(filtered, cv2.RETR_LIST, 
                                      cv2.CHAIN_APPROX_SIMPLE)
        
        # Create 3-channel A-channel image for CNN input
        a_channel_3d = np.stack([a_channel] * 3, axis=2)
        
        return contours, a_channel_3d


class CoordinateTransformer:
    """Handles coordinate transformations from pixel to camera frame."""
    
    def __init__(self, focal_length: float):
        """
        Initialize coordinate transformer.
        
        Args:
            focal_length: Camera focal length in pixels
        """
        self.focal_length = focal_length
    
    def pixel_to_camera_frame(self, 
                             pixel_coords: Tuple[float, float],
                             image_height: int,
                             image_width: int,
                             depth: float) -> np.ndarray:
        """
        Transform pixel coordinates to 3D camera frame coordinates.
        
        Uses pinhole camera model for back-projection.
        
        Args:
            pixel_coords: (x, y) pixel coordinates
            image_height: Image height in pixels
            image_width: Image width in pixels
            depth: Depth value in meters
            
        Returns:
            3D coordinates in camera frame [x, y, z] in meters
        """
        if depth == 0.0:
            rospy.logwarn("Zero depth provided for coordinate transformation")
            return np.zeros(3)
        
        # Calculate principal point (image center)
        cx = image_width / 2.0
        cy = image_height / 2.0
        
        # Construct transformation matrix
        # Using pinhole camera model with depth
        transform_matrix = np.array([
            [depth / self.focal_length, 0, 0, -(cx * depth) / self.focal_length],
            [0, -depth / self.focal_length, 0, (cy * depth) / self.focal_length],
            [0, 0, depth, 0]
        ])
        
        # Convert pixel coordinates to homogeneous coordinates
        pixel_x, pixel_y = pixel_coords
        homogeneous_coords = np.array([pixel_x, pixel_y, 1, 1])
        
        # Apply transformation
        camera_coords = transform_matrix.dot(homogeneous_coords)
        
        return camera_coords


class BoundingBoxProcessor:
    """Processes bounding boxes for detected contours."""
    
    def __init__(self, padding: int = 5):
        """
        Initialize bounding box processor.
        
        Args:
            padding: Pixels to add around detected contours
        """
        self.padding = padding
    
    def get_padded_bounding_box(self, 
                               contour: np.ndarray,
                               image_shape: Tuple[int, int]) -> Tuple[int, int, int, int]:
        """
        Get padded bounding box for a contour.
        
        Args:
            contour: Contour from cv2.findContours
            image_shape: (height, width) of image for boundary checking
            
        Returns:
            Tuple of (x, y, w, h) for bounding box
        """
        x, y, w, h = cv2.boundingRect(contour)
        
        # Apply padding
        x = max(0, x - self.padding)
        y = max(0, y - self.padding)
        w = w + 2 * self.padding
        h = h + 2 * self.padding
        
        # Ensure we don't exceed image boundaries
        height, width = image_shape[:2]
        if x + w > width:
            w = width - x
        if y + h > height:
            h = height - y
        
        return x, y, w, h
    
    @staticmethod
    def get_centroid(bbox: Tuple[int, int, int, int]) -> Tuple[int, int]:
        """
        Calculate centroid of bounding box.
        
        Args:
            bbox: Tuple of (x, y, w, h)
            
        Returns:
            Tuple of (cx, cy) centroid coordinates
        """
        x, y, w, h = bbox
        cx = int(x + w / 2)
        cy = int(y + h / 2)
        return cx, cy
    
    @staticmethod
    def is_in_central_region(centroid: Tuple[int, int],
                            image_width: int,
                            central_ratio: float = 0.05) -> bool:
        """
        Check if centroid is within central region of image.
        
        Args:
            centroid: (x, y) centroid coordinates
            image_width: Width of image
            central_ratio: Ratio of image width for central region (each side)
            
        Returns:
            True if centroid is in central region
        """
        cx, _ = centroid
        center = image_width / 2
        margin = image_width * central_ratio
        
        return (center - margin) < cx < (center + margin)


class VisualizationHelper:
    """Helper class for drawing detection results on images."""
    
    # Color constants (BGR format)
    COLOR_GREEN = (0, 255, 0)  # Detection in central region
    COLOR_RED = (0, 0, 255)    # Detection outside central region
    COLOR_BLUE = (255, 0, 0)   # Low confidence
    
    @staticmethod
    def draw_detection(image: np.ndarray,
                      bbox: Tuple[int, int, int, int],
                      label: int,
                      confidence: float,
                      in_central_region: bool) -> None:
        """
        Draw detection result on image (in-place).
        
        Args:
            image: Image to draw on
            bbox: Bounding box (x, y, w, h)
            label: Classification label (0=red, 1=green)
            confidence: Classification confidence
            in_central_region: Whether detection is in central ROI
        """
        x, y, w, h = bbox
        
        # Choose color based on region
        color = VisualizationHelper.COLOR_GREEN if in_central_region else VisualizationHelper.COLOR_BLUE
        
        # Draw bounding box
        cv2.rectangle(image, (x, y), (x + w, y + h), color, 2)
        
        # Draw label text
        label_text = "Green" if label == 1 else f"acc: {confidence:.2f}"
        text_pos = (x, y - 10) if y > 20 else (x, y + h + 20)
        cv2.putText(image, label_text, text_pos, 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.75, color, 2, cv2.LINE_AA)
