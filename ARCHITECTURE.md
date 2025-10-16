# System Architecture

## High-Level Overview

```
┌─────────────────────────────────────────────────────────────────┐
│                         HARDWARE LAYER                          │
│  ┌──────────────────┐              ┌────────────────┐          │
│  │  ZED Stereo      │              │  Mobile Robot  │          │
│  │  Camera          │              │  Platform      │          │
│  │  - RGB Stream    │              │  - Odometry    │          │
│  │  - Depth Stream  │              │  - Locomotion  │          │
│  └────────┬─────────┘              └────────┬───────┘          │
└───────────┼────────────────────────────────┼──────────────────┘
            │                                │
            ▼                                ▼
┌─────────────────────────────────────────────────────────────────┐
│                      ROS MIDDLEWARE LAYER                        │
│                                                                  │
│  Topics:                                                         │
│   /zedm/zed_node/left/image_rect_color      (sensor_msgs/Image) │
│   /zedm/zed_node/depth/depth_registered     (sensor_msgs/Image) │
│   /zedm/zed_node/odom                       (nav_msgs/Odometry) │
│   /sb_coordinates                         (geometry_msgs/PoseStamped) │
│   /debug_image                            (sensor_msgs/Image) │
│   /strawberry_markers                     (visualization_msgs/MarkerArray) │
│                                                                  │
│  TF Frames:  world → rover → camera                            │
└─────────────────────────────────────────────────────────────────┘
            │
            ▼
┌─────────────────────────────────────────────────────────────────┐
│                    APPLICATION LAYER (3 Nodes)                   │
│                                                                  │
│  ┌──────────────────┐  ┌──────────────────┐  ┌──────────────┐ │
│  │  TF Broadcaster  │  │   Strawberry     │  │  Strawberry  │ │
│  │     (C++)        │  │   Detector       │  │   Plotter    │ │
│  │                  │  │   (Python)       │  │   (Python)   │ │
│  │  - Publishes     │  │                  │  │              │ │
│  │    world→rover   │  │  - Image filter  │  │  - Receives  │ │
│  │  - Publishes     │  │  - CNN classify  │  │    detections│ │
│  │    rover→camera  │  │  - 3D transform  │  │  - Creates   │ │
│  │                  │  │  - Publishes     │  │    markers   │ │
│  │  Subscribes:     │  │    coordinates   │  │  - Publishes │ │
│  │   • odom         │  │                  │  │    to RViz   │ │
│  │                  │  │  Subscribes:     │  │              │ │
│  │                  │  │   • RGB image    │  │  Subscribes: │ │
│  │                  │  │   • Depth image  │  │   • sb_coord │ │
│  │                  │  │   • Odometry     │  │              │ │
│  └──────────────────┘  └──────────────────┘  └──────────────┘ │
└─────────────────────────────────────────────────────────────────┘
            │
            ▼
┌─────────────────────────────────────────────────────────────────┐
│                     VISUALIZATION LAYER                          │
│                                                                  │
│                         ┌────────┐                              │
│                         │  RViz  │                              │
│                         │        │                              │
│                         │ • TF   │                              │
│                         │ • Map  │                              │
│                         │ • Markers                             │
│                         │ • Robot                               │
│                         └────────┘                              │
└─────────────────────────────────────────────────────────────────┘
```

## Detection Node Internal Architecture

```
┌────────────────────────────────────────────────────────────────┐
│                  StrawberryDetectionNode                       │
│                                                                 │
│  ┌─────────────────────────────────────────────────────────┐  │
│  │                   Configuration                          │  │
│  │  • DetectionConfig (ROS params → validated values)      │  │
│  └─────────────────────────────────────────────────────────┘  │
│                              │                                  │
│  ┌───────────────────────────┼─────────────────────────────┐  │
│  │        Processing Pipeline Components                    │  │
│  │                           │                              │  │
│  │  ┌────────────────────────▼───────────────────────┐    │  │
│  │  │      ColorFilterProcessor                       │    │  │
│  │  │  • LAB color space conversion                   │    │  │
│  │  │  • A-channel thresholding                       │    │  │
│  │  │  • Morphological operations                     │    │  │
│  │  │  • Contour detection                            │    │  │
│  │  └─────────────────────────┬───────────────────────┘    │  │
│  │                             │                             │  │
│  │  ┌──────────────────────────▼──────────────────────┐    │  │
│  │  │      BoundingBoxProcessor                        │    │  │
│  │  │  • Bounding box extraction                       │    │  │
│  │  │  • Padding and boundary checking                 │    │  │
│  │  │  • Centroid calculation                          │    │  │
│  │  │  • Central region validation                     │    │  │
│  │  └─────────────────────────┬───────────────────────┘    │  │
│  │                             │                             │  │
│  │  ┌──────────────────────────▼──────────────────────┐    │  │
│  │  │      StrawberryClassifier                        │    │  │
│  │  │  • CNN model loading                             │    │  │
│  │  │  • Image preprocessing                           │    │  │
│  │  │  • Inference (red/green classification)          │    │  │
│  │  │  • Confidence scoring                            │    │  │
│  │  └─────────────────────────┬───────────────────────┘    │  │
│  │                             │                             │  │
│  │  ┌──────────────────────────▼──────────────────────┐    │  │
│  │  │      DepthValidator                              │    │  │
│  │  │  • Depth extraction from stereo                  │    │  │
│  │  │  • Invalid value handling (inf/nan)              │    │  │
│  │  │  • Boundary validation                           │    │  │
│  │  └─────────────────────────┬───────────────────────┘    │  │
│  │                             │                             │  │
│  │  ┌──────────────────────────▼──────────────────────┐    │  │
│  │  │      CoordinateTransformer                       │    │  │
│  │  │  • Pixel → Camera frame transform               │    │  │
│  │  │  • Pinhole camera model                          │    │  │
│  │  │  • Camera → World frame transform               │    │  │
│  │  └─────────────────────────┬───────────────────────┘    │  │
│  │                             │                             │  │
│  │  ┌──────────────────────────▼──────────────────────┐    │  │
│  │  │      VisualizationHelper                         │    │  │
│  │  │  • Bounding box drawing                          │    │  │
│  │  │  • Label annotation                              │    │  │
│  │  │  • Color coding by region/confidence             │    │  │
│  │  └──────────────────────────────────────────────────┘    │  │
│  └───────────────────────────────────────────────────────┘  │
│                                                               │
│  ┌───────────────────────────────────────────────────────┐  │
│  │                  ROS Interfaces                        │  │
│  │                                                         │  │
│  │  Subscribers:                    Publishers:           │  │
│  │   • RGB (synchronized)            • Detections         │  │
│  │   • Depth (synchronized)          • Debug images       │  │
│  │   • Odometry (synchronized)                            │  │
│  └───────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────┘
```

## Data Flow

```
RGB Image ──┐
            │
Depth Image ├─→ Time Synchronizer ──→ Detection Callback
            │                              │
Odometry ───┘                              │
                                           ▼
                                    Color Filtering
                                           │
                                           ▼
                                    For Each Contour:
                                           │
                                           ├─→ Extract BBox
                                           │
                                           ├─→ CNN Classification
                                           │      │
                                           │      ├─→ Confidence < 0.98? → Skip
                                           │      │
                                           │      └─→ Confidence ≥ 0.98
                                           │             │
                                           ├─→ In Central Region?
                                           │      │
                                           │      ├─→ No → Draw blue box, Skip
                                           │      │
                                           │      └─→ Yes
                                           │             │
                                           ├─→ Get Depth at Centroid
                                           │      │
                                           │      ├─→ Invalid depth? → Skip
                                           │      │
                                           │      └─→ Valid depth
                                           │             │
                                           ├─→ Pixel → Camera coords
                                           │
                                           ├─→ Camera → World coords
                                           │
                                           └─→ Publish PoseStamped
                                                  │
                                                  ▼
                                           Plotter Node
                                                  │
                                                  ├─→ Apply calibration offset
                                                  │
                                                  ├─→ Create marker
                                                  │
                                                  └─→ Publish MarkerArray
                                                         │
                                                         ▼
                                                      RViz
```
## Configuration Flow

```
Launch File Parameters
         │
         ▼
ROS Parameter Server
         │
         ├─→ DetectionConfig.__init__()
         │      │
         │      ├─→ Load parameters
         │      ├─→ Set defaults
         │      ├─→ Validate values
         │      └─→ Log configuration
         │
         ├─→ PlotterConfig.__init__()
         │      │
         │      └─→ (same pattern)
         │
         └─→ BroadcasterConfig (C++)
                │
                └─→ (same pattern)
```
