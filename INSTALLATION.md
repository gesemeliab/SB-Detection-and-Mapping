# 📦 Package Installation & Verification Guide

## Quick Installation Checklist

- [ ] ROS Melodic or Noetic installed
- [ ] ZED SDK installed
- [ ] ZED ROS wrapper installed
- [ ] Python dependencies installed
- [ ] Repository cloned and built
- [ ] Model file downloaded/trained
- [ ] System tested and verified

---

## Detailed Installation Steps

### 1. ROS Installation

#### Follow the official ROS instalation guides for Ubuntu 20.04 (ROS Noetic) or Ubuntu 18.04 (ROS Melodic)

### 2. ZED SDK Installation

```bash
# Download from https://www.stereolabs.com/developers/release/
# Choose your ZED camera model and Ubuntu version

# For ZED Mini on Ubuntu 20.04 (example)
wget https://download.stereolabs.com/zedsdk/3.8/cu118/ubuntu20
chmod +x ubuntu20
./ubuntu20

# Follow installation prompts
# When asked, install CUDA if you have an NVIDIA GPU
```

### 3. ZED ROS Wrapper

```bash
# Create workspace if not exists
mkdir -p ~/zed_ws/src
cd ~/zed_ws/src

# Clone ZED wrapper
git clone --recursive https://github.com/stereolabs/zed-ros-wrapper.git

# Build
cd ~/zed_ws
rosdep install --from-paths src --ignore-src -r -y
catkin_make -DCMAKE_BUILD_TYPE=Release

# Source
echo "source ~/zed_ws/devel/setup.bash" >> ~/.bashrc
source ~/zed_ws/devel/setup.bash
```

### 4. Additional ROS Packages

```bash
# For ROS Noetic
sudo apt install \
    ros-noetic-tf \
    ros-noetic-cv-bridge \
    ros-noetic-image-transport \
    ros-noetic-message-filters \
    ros-noetic-visualization-msgs

# Optional: For trajectory visualization
sudo apt install ros-noetic-hector-trajectory-server

# For ROS Melodic, replace 'noetic' with 'melodic'
```

### 5. Python Dependencies

```bash
# Update pip
python3 -m pip install --upgrade pip

# Install from requirements file
cd ~/sbdm_ws/src/SB-Detection-and-Mapping
pip3 install -r requirements.txt

# Or install minimal dependencies
pip3 install tensorflow>=2.4.0 keras>=2.4.0 opencv-python>=4.5.0 numpy>=1.19.0
```

### 6. Clone and Build Repository

```bash
# Create workspace
mkdir -p ~/sbdm_ws/src
cd ~/sbdm_ws/src

# Clone repository
git clone https://github.com/gesemeliab/SB-Detection-and-Mapping.git

# Install ROS dependencies
cd ~/sbdm_ws
rosdep install --from-paths src --ignore-src -r -y

# Build
catkin_make

# Source workspace
source devel/setup.bash
echo "source ~/sbdm_ws/devel/setup.bash" >> ~/.bashrc
```

### 7. Model Setup

```bash
# Create models directory
mkdir -p ~/sbdm_ws/src/SB-Detection-and-Mapping/models

# Option A: Use existing model
# Copy your trained model to the models directory
cp /path/to/your/model.h5 ~/sbdm_ws/src/SB-Detection-and-Mapping/models/

# Option B: Train new model using provided dataset
# See database/h5/strawberries.h5 for training data
```

---

## Verification

### Check ROS Installation
```bash
rosversion -d
# Expected output: noetic or melodic
```

### Check ZED Camera
```bash
# Launch ROS wrapper
roslaunch zed_wrapper zedm.launch
# Should see camera topics published
```

### Check Package Installation
```bash
# Find packages
rospack find sb_detection
rospack find plotting_points
rospack find broadcaster

# Check executables
which detection_node.py
which plotter_node.py
which tf_broadcaster_node

# List available launch files
roslaunch sb_detection <TAB><TAB>
```

### Check Python Dependencies
```bash
# Test TensorFlow
python3 -c "import tensorflow as tf; print(tf.__version__)"

# Test GPU support (if available)
python3 -c "import tensorflow as tf; print('GPU Available:', len(tf.config.list_physical_devices('GPU')) > 0)"

# Test OpenCV
python3 -c "import cv2; print(cv2.__version__)"

# Test NumPy
python3 -c "import numpy as np; print(np.__version__)"
```

### Test System
```bash
# Quick test (dry run without camera)
roscore &
sleep 2
rosrun sb_detection detection_node.py _model_path:=/path/to/model.h5

# Full system test
roslaunch sb_detection detection_pipeline.launch \
    model_path:=$HOME/sbdm_ws/src/SB-Detection-and-Mapping/models/strawberry_classifier.h5

# Check topics
rostopic list
# Should see: /sb_coordinates, /debug_image, /strawberry_markers

# Monitor output
rostopic echo /sb_coordinates
```
---

## Post-Installation

### Configure Your Model Path

Edit the launch file or pass as argument:
```bash
roslaunch sb_detection detection_pipeline.launch \
    model_path:=$HOME/sbdm_ws/src/SB-Detection-and-Mapping/models/your_model.h5
```

### Calibrate Camera Offsets

If your camera mounting is different:
```xml
<!-- Edit launch file -->
<param name="camera_offset_x" value="0.0" />
<param name="camera_offset_y" value="-0.21" />
<param name="camera_offset_z" value="0.26" />
```