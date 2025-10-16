# SB-Detection-and-Mapping (SBDM)

[![ROS](https://img.shields.io/badge/ROS-Melodic%20%7C%20Noetic-blue)](http://wiki.ros.org/)
[![Python](https://img.shields.io/badge/Python-3.6%2B-brightgreen)](https://www.python.org/)
[![C++](https://img.shields.io/badge/C%2B%2B-11-orange)](https://isocpp.org/)
[![License](https://img.shields.io/badge/License-GPLv3-red)](./LICENSE)
[![Code Quality](https://img.shields.io/badge/Code%20Quality-Production%20Ready-success)](./ARCHITECTURE.md)

> **Robotic system for real-time strawberry detection and 3D mapping using computer vision and deep learning**

SBDM is a professional robotic perception system that combines LAB color space filtering, CNN classification, and stereo vision to autonomously detect and spatially map strawberries in agricultural environments. Built with clean architecture principles and comprehensive error handling.

**Authors:** Gesem Gudiño, Andres Montes de Oca, and Gerardo Flores  
**Institution:** Centro de Investigaciones en Optica   
**License:** [GPLv3](./LICENSE)

---

## Table of Contents

- [Overview](#-overview)
- [Features](#-features)
- [System Requirements](#-system-requirements)
- [Installation](#-installation)
- [Quick Start](#-quick-start)
- [Configuration](#-configuration)
- [Architecture](#-architecture)
- [Dataset](#-dataset)
- [Usage Examples](#-usage-examples)
- [Troubleshooting](#-troubleshooting)
- [Documentation](#-documentation)
- [Contributing](#-contributing)
- [Citation](#-citation)

---

## Overview

SBDM implements an end-to-end pipeline for agricultural robotics:

1. **Perception**: RGB-D data acquisition from ZED stereo camera
2. **Detection**: LAB color space filtering + CNN classification
3. **Localization**: Pixel-to-world coordinate transformation using stereo depth
4. **Mapping**: Persistent 3D visualization of detected strawberries
5. **Navigation**: TF frame broadcasting for robot localization

**Use Cases:**
- Precision agriculture and yield estimation
- Autonomous harvesting systems
- Agricultural robotics research
- Computer vision in unstructured environments

---

## Features

### Core Capabilities
- ✅ **Real-time Detection**: 15-30 Hz processing rate with GPU acceleration
- ✅ **3D Localization**: Stereo depth + pinhole camera model for world-frame mapping
- ✅ **High Accuracy**: CNN classifier with 98% confidence threshold
- ✅ **Robust Filtering**: LAB color space segmentation with morphological operations

---

## System Requirements

### Hardware
- **Camera**: ZED or ZED Mini stereo camera
- **Compute**: 
  - CPU: Intel i5 or better (i7 recommended)
  - RAM: 8GB minimum (16GB recommended)
  - GPU: NVIDIA GPU with CUDA support (optional but recommended for CNN inference)
- **Robot Platform**: Any mobile robot with odometry

### Software
- **OS**: Ubuntu 18.04 (Melodic) or Ubuntu 20.04 (Noetic)
- **ROS**: Melodic or Noetic
- **Python**: 3.6+
- **CUDA**: 10.1+ (optional, for GPU acceleration)

---

## Installation


---

## Quick Start### Minimal Start (Single Command)

```bash
# 1. Connect ZED camera
# 2. Launch complete system
roslaunch sb_detection detection_pipeline.launch \
    model_path:=$HOME/sbdm_ws/src/SB-Detection-and-Mapping/models/strawberry_classifier.h5
```

### Verify System is Running

```bash
# Check active nodes
rosnode list
# Expected: /tf_broadcaster, /strawberry_detector, /strawberry_plotter

# Check published topics
rostopic list | grep -E "(sb_coordinates|debug_image|markers)"

# Monitor detection rate
rostopic hz /sb_coordinates

# View debug image
rqt_image_view /debug_image
```

---

## Configuration

All system parameters are configurable via ROS parameter server without code modifications.

### Detection Node Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `~model_path` | string | **required** | Path to trained Keras model (.h5 file) |
| `~confidence_threshold` | float | 0.98 | Minimum confidence for valid detection [0-1] |
| `~focal_length` | float | 1396.91 | Camera focal length in pixels |
| `~lab_a_threshold` | int | 165 | LAB A-channel threshold for color filtering |
| `~morphology_kernel_size` | int | 15 | Morphological operation kernel size |
| `~central_region_ratio` | float | 0.05 | Central ROI width ratio (5% each side) |
| `~publish_debug_image` | bool | true | Enable debug visualization |
| `~debug_image_size` | int | 480 | Debug image output size (pixels) |

### Plotter Node Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `~marker_scale` | float | 0.05 | Size of RViz markers (meters) |
| `~marker_color_r` | float | 1.0 | Marker red component [0-1] |
| `~marker_color_g` | float | 0.0 | Marker green component [0-1] |
| `~marker_color_b` | float | 0.0 | Marker blue component [0-1] |
| `~offset_x` | float | 0.545 | X-axis calibration offset (meters) |
| `~offset_y` | float | -0.495 | Y-axis calibration offset (meters) |
| `~offset_z` | float | 0.39 | Z-axis calibration offset (meters) |

### TF Broadcaster Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `~rover_height` | float | 0.13 | Rover base height above ground (meters) |
| `~camera_offset_x` | float | 0.0 | Camera X offset from rover base (meters) |
| `~camera_offset_y` | float | -0.21 | Camera Y offset from rover base (meters) |
| `~camera_offset_z` | float | 0.26 | Camera Z offset from rover base (meters) |
| `~world_frame` | string | "world" | World coordinate frame name |
| `~rover_frame` | string | "rover" | Rover coordinate frame name |
| `~camera_frame` | string | "camera" | Camera coordinate frame name |

### Example: Custom Configuration

```bash
roslaunch sb_detection detection_pipeline.launch \
    model_path:=/path/to/model.h5 \
    confidence_threshold:=0.95 \
    lab_a_threshold:=160 \
    focal_length:=1400.0 \
    marker_scale:=0.08 \
    publish_debug_image:=false
```

### Runtime Parameter Changes

```bash
# Change confidence threshold without restarting
rosparam set /strawberry_detector/confidence_threshold 0.95

# Enable debug logging
rosservice call /strawberry_detector/set_logger_level "logger: 'rosout' level: 'debug'"
```

---

## Architecture

---

## 📊 Dataset

### HDF5 Preprocessed Dataset

**Location**: `database/h5/strawberries.h5`

**Specifications**:
- **Images**: 520 labeled samples
- **Resolution**: 128×128×3
- **Color Space**: LAB (A-channel)
- **Normalization**: Pixel values in [0, 1]
- **Classes**: Red strawberries (0), Green strawberries (1)

### Raw Image Dataset

**Location**: `database/images/`

#### Complete Images
- **Path**: `database/images/complete/`
- **Count**: 88+ high-resolution images
- **Source**: Natural farmland environments
- **Format**: JPEG, RGB color space
- **Size**: Variable (typically 1920×1080 or higher)

#### Cropped Images
- **Path**: `database/images/cropped/`
- **Content**: Extracted strawberry patches
- **Format**: JPEG, RGB color space
- **Usage**: Training data augmentation

### Using the Dataset

```python
# Load HDF5 dataset
import h5py
import numpy as np

with h5py.File('database/h5/strawberries.h5', 'r') as f:
    images = np.array(f['images'])
    labels = np.array(f['labels'])
    
print(f"Dataset shape: {images.shape}")  # (520, 128, 128, 3)
print(f"Labels shape: {labels.shape}")    # (520,)
```
---

## 📜 License

This project is licensed under the GNU General Public License v3.0 - see the [LICENSE](./LICENSE) file for details.

```
Copyright (C) 2020-2025 Gesem Gudiño, Andres Montes de Oca, Gerardo Flores

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
```

---

<div align="center">

**⭐ If you find this project useful, please consider giving it a star! ⭐**

Made with ❤️ for agricultural robotics research

</div>


