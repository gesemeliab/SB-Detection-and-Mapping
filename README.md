# SB-Detection-and-Mapping (SBDM)
SBDM: An image processing + Deeplearning approach to detect and map strawberries with a robotic platform 

**Authors:** Gesem Gudiño, Andres Montes de Oca, and Gerardo Flores.

# License
SBDM released under a [GPLv3 license] (https://github.com/gesemeliab/SB-Detection-and-Mapping/blob/main/LICENSE)

# Install

## Prerequisites

The necessary prerequisites are found in hector_slam -> (https://github.com/tu-darmstadt-ros-pkg/hector_slam) and  zed_ros_wrapper ->(https://github.com/stereolabs/zed-ros-wrapper):

- OpenCV
- Keras
- ROS
- zed_ros_wrapper
- hector_slam

## Building SBDM

Change the name of the catkin workspace to yours.

```
mkdir sbdm_ws; cd sbdm_ws; mkdir src; cd src
git clone https://github.com/gesemeliab/SB-Detection-and-Mapping.git
cd ..
catkin_make
```
After compilation pleas do:

source ~/sbdm_ws/devel/setup.bash

# Run

To run this code you first have to have connected your ZED stereo camera with its respective SDK. You can find it in (https://www.stereolabs.com/developers/release/)

Next you will need to launch the zed_ros_wrapper.

ZED camera:

    roslaunch zed_wrapper zed.launch
   
ZED Mini camera:

    roslaunch zed_wrapper zedm.launch

To start the broadcaster node
    
    roslaunch broadcaster rover.launch

Then run the detector node

    rosrun sb_detection SBDetection.py
    
Finally run the plotter node

    rosrun plotting_points sb_plotter.py

To visualize the rovers trajectory you may also run the hector_trajectory_server

    rosrun hector_trajectory_server hector_trajectory_server

# Data Base

You can find the data base used for this work in (https://github.com/gesemeliab/SB-Detection-and-Mapping/tree/main/database) where you will find two directories.

 ## h5
 
 An already created database for training purpose, this file contains 520 labeled images of green and red strawberries with shape (128,128,3), they are in the A channel of the CIE LAB color space and pixels are between 0 and 1.
 
 ## images
 
 In images directory you will find another two sub directories which are the complete natural RGB images taken in farmlands and also the already cropped images of only red and green strawberries, aslo in RGB.
 






   
