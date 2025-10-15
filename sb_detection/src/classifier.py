#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
CNN-based strawberry classifier.

This module encapsulates the deep learning model for strawberry classification,
providing a clean interface with proper resource management.

Author: Gesem Gudiño
License: GPLv3
"""

import cv2
import numpy as np
from typing import Tuple
import rospy

# TensorFlow imports with GPU configuration
import tensorflow as tf
from tensorflow.keras.models import load_model


class StrawberryClassifier:
    """
    Deep learning classifier for red/green strawberry detection.
    
    This class encapsulates the CNN model and provides a clean interface
    for prediction with proper resource management and error handling.
    """
    
    # Class constants
    LABEL_RED = 0
    LABEL_GREEN = 1
    
    def __init__(self, model_path: str, input_size: int = 128, 
                 enable_gpu_growth: bool = True):
        """
        Initialize strawberry classifier.
        
        Args:
            model_path: Path to trained Keras model file
            input_size: Input image size for CNN (assumes square images)
            enable_gpu_growth: Whether to enable GPU memory growth
            
        Raises:
            FileNotFoundError: If model file doesn't exist
            ValueError: If model cannot be loaded
        """
        self.input_size = input_size
        self.model = None
        
        # Configure GPU memory growth
        if enable_gpu_growth:
            self._configure_gpu()
        
        # Load model
        self._load_model(model_path)
        
        rospy.loginfo(f"Strawberry classifier initialized with model: {model_path}")
    
    def _configure_gpu(self) -> None:
        """Configure TensorFlow GPU settings to prevent memory allocation issues."""
        try:
            gpus = tf.config.experimental.list_physical_devices('GPU')
            if gpus:
                for gpu in gpus:
                    tf.config.experimental.set_memory_growth(gpu, True)
                rospy.loginfo(f"Configured {len(gpus)} GPU(s) with memory growth")
            else:
                rospy.logwarn("No GPUs detected, using CPU")
        except RuntimeError as e:
            rospy.logerr(f"GPU configuration failed: {e}")
    
    def _load_model(self, model_path: str) -> None:
        """
        Load Keras model from file.
        
        Args:
            model_path: Path to model file
            
        Raises:
            FileNotFoundError: If model file doesn't exist
            ValueError: If model cannot be loaded
        """
        try:
            self.model = load_model(model_path)
            rospy.loginfo("Model loaded successfully")
        except FileNotFoundError:
            raise FileNotFoundError(f"Model file not found: {model_path}")
        except Exception as e:
            raise ValueError(f"Failed to load model: {e}")
    
    def predict(self, image: np.ndarray, 
               bbox: Tuple[int, int, int, int]) -> Tuple[int, float]:
        """
        Classify strawberry in bounding box.
        
        Args:
            image: Input image (already in A-channel 3D format)
            bbox: Bounding box (x, y, w, h) to classify
            
        Returns:
            Tuple of (label, confidence):
                - label: 0 for red, 1 for green
                - confidence: Classification confidence [0, 1]
        """
        # Extract and preprocess crop
        crop = self._preprocess_crop(image, bbox)
        
        # Run inference
        probabilities = self.model.predict(crop, batch_size=1, verbose=0)
        
        # Extract prediction
        label = int(probabilities.argmax(axis=1)[0])
        confidence = float(probabilities[0][label])
        
        rospy.logdebug(f"Classification: label={label}, confidence={confidence:.3f}")
        
        return label, confidence
    
    def _preprocess_crop(self, image: np.ndarray, 
                        bbox: Tuple[int, int, int, int]) -> np.ndarray:
        """
        Extract and preprocess image crop for CNN input.
        
        Args:
            image: Source image
            bbox: Bounding box (x, y, w, h)
            
        Returns:
            Preprocessed crop ready for CNN input
        """
        x, y, w, h = bbox
        
        # Extract crop
        crop = image[y:y+h, x:x+w].copy()
        
        # Resize to CNN input size
        crop = cv2.resize(crop, (self.input_size, self.input_size))
        
        # Normalize to [0, 1]
        crop = crop.astype(np.float32) / 255.0
        
        # Add batch dimension
        crop = np.expand_dims(crop, axis=0)
        
        return crop
    
    def __del__(self):
        """Cleanup resources on deletion."""
        if self.model is not None:
            del self.model
            rospy.logdebug("Classifier model cleaned up")
