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

## Class Diagram (Detection Node)

```
┌─────────────────────────┐
│   DetectionConfig       │
│─────────────────────────│
│ + focal_length          │
│ + confidence_threshold  │
│ + lab_a_threshold       │
│ + topics, params...     │
│─────────────────────────│
│ + _validate()           │
│ + _log_configuration()  │
└─────────────────────────┘
           ▲
           │ uses
           │
┌──────────┴──────────────────────────────┐
│   StrawberryDetectionNode                │
│──────────────────────────────────────────│
│ - config: DetectionConfig                │
│ - color_filter: ColorFilterProcessor     │
│ - classifier: StrawberryClassifier       │
│ - coord_transformer: CoordinateTransformer│
│ - depth_validator: DepthValidator        │
│ - bbox_processor: BoundingBoxProcessor   │
│──────────────────────────────────────────│
│ + __init__()                             │
│ - _initialize_processors()               │
│ - _initialize_classifier()               │
│ - _initialize_ros_interfaces()           │
│ - _synchronized_callback()               │
│ - _process_frame()                       │
│ - _process_contour()                     │
│ - _publish_detections()                  │
│ + run()                                  │
└──────────────────────────────────────────┘
           │
           │ depends on
           │
    ┌──────┴──────┐
    │             │
    ▼             ▼
┌──────────┐  ┌──────────────┐
│ColorFilter│  │Classifier    │
│Processor  │  │              │
│──────────│  │──────────────│
│+ filter_ │  │+ predict()   │
│strawberries│ │+ _preprocess│
└──────────┘  └──────────────┘
```

## Deployment Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    Deployment View                       │
│                                                           │
│  ┌────────────────────────────────────────────────────┐ │
│  │         detection_pipeline.launch                   │ │
│  │                                                      │ │
│  │  ┌──────────────┐  ┌──────────────┐  ┌──────────┐ │ │
│  │  │ Parameters   │  │   Includes   │  │  Nodes   │ │ │
│  │  │              │  │              │  │          │ │ │
│  │  │ • model_path │  │ • zed.launch │  │ • TF     │ │ │
│  │  │ • confidence │  │   (optional) │  │ • Detect │ │ │
│  │  │ • focal_len  │  │              │  │ • Plot   │ │ │
│  │  │ • topics     │  │              │  │ • RViz   │ │ │
│  │  │ • thresholds │  │              │  │ (optional│ │ │
│  │  └──────────────┘  └──────────────┘  └──────────┘ │ │
│  └────────────────────────────────────────────────────┘ │
│                                                           │
│  Single command: roslaunch sb_detection detection_...   │
└─────────────────────────────────────────────────────────┘
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

## Error Handling Flow

```
Input Data
    │
    ├─→ Validation Layer
    │      │
    │      ├─→ Valid? → Continue
    │      │
    │      └─→ Invalid? → Log warning → Return default/skip
    │
    ├─→ Processing Layer
    │      │
    │      ├─→ Try: Process
    │      │
    │      └─→ Except: Log error → Graceful degradation
    │
    └─→ Output Layer
           │
           ├─→ Verify output
           │
           └─→ Publish with timestamp
```

## Key Design Patterns

1. **Strategy Pattern**: Interchangeable processing components
2. **Facade Pattern**: Simple interface to complex subsystems
3. **Factory Pattern**: Configuration object creation
4. **Observer Pattern**: ROS pub/sub architecture
5. **Singleton Pattern**: ROS node instances
6. **Dependency Injection**: Configuration passed to components

## Scalability Considerations

- **Horizontal**: Multiple detector nodes with different cameras
- **Vertical**: GPU acceleration for CNN inference
- **Modular**: Replace CNN with different model easily
- **Configurable**: Adapt to different cameras via parameters
- **Extensible**: Add new filters/transforms without core changes

---

This architecture demonstrates production-ready design suitable for:
- Research publications
- Commercial deployment
- Team collaboration
- Long-term maintenance
- Future extensions
