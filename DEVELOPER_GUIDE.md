# Developer Guide

## Code Organization

The refactored codebase follows a modular, object-oriented architecture with clear separation of concerns.

### Directory Structure

```
SB-Detection-and-Mapping/
├── broadcaster/              # C++ TF broadcasting node
│   ├── src/
│   │   ├── broadcaster.cpp          # Legacy broadcaster (kept for compatibility)
│   │   └── tf_broadcaster_node.cpp  # Refactored OOP broadcaster
│   ├── launch/
│   └── CMakeLists.txt
│
├── sb_detection/            # Python detection node
│   ├── src/
│   │   ├── config.py               # Configuration management
│   │   ├── vision_utils.py         # CV utilities (pure functions)
│   │   ├── classifier.py           # CNN model wrapper
│   │   ├── detection_node.py       # Main ROS node
│   │   ├── SBDetection.py          # Legacy node (kept for reference)
│   │   └── strawberry.py           # Legacy utilities
│   ├── launch/
│   │   └── detection_pipeline.launch  # Complete system launch
│   ├── setup.py
│   └── CMakeLists.txt
│
├── plotting_points/         # Python visualization node
│   ├── src/
│   │   ├── config.py               # Plotter configuration
│   │   ├── plotter_node.py         # Refactored plotter
│   │   └── sb_plotter.py           # Legacy plotter
│   ├── setup.py
│   └── CMakeLists.txt
│
├── database/                # Training data
│   ├── h5/                  # HDF5 datasets
│   └── images/              # Raw images
│
├── requirements.txt         # Python dependencies
├── README.md               # Main documentation
└── REFACTORING.md          # This refactoring summary
```

## Coding Standards

### Python (PEP8 Compliant)

#### Naming Conventions
- **Functions/Methods**: `snake_case`
- **Classes**: `PascalCase`
- **Constants**: `UPPER_CASE`
- **Private members**: `_leading_underscore`

#### Type Hints
Always use type hints for function signatures:
```python
def process_image(image: np.ndarray, threshold: float) -> Tuple[List, np.ndarray]:
    """Process image with given threshold."""
    pass
```

#### Docstrings
Use Google-style docstrings:
```python
def function_name(param1: type, param2: type) -> return_type:
    """
    Brief description of function.
    
    More detailed description if needed.
    
    Args:
        param1: Description of param1
        param2: Description of param2
        
    Returns:
        Description of return value
        
    Raises:
        ExceptionType: When this exception is raised
    """
    pass
```

#### Imports
Order imports as follows:
1. Standard library
2. Third-party packages
3. ROS packages
4. Local modules

```python
import sys
from typing import List, Tuple

import numpy as np
import cv2

import rospy
from sensor_msgs.msg import Image

from config import DetectionConfig
```

### C++ (Google C++ Style Guide)

#### Naming Conventions
- **Classes**: `PascalCase`
- **Functions**: `camelCase`
- **Variables**: `snake_case`
- **Member variables**: `snake_case_`
- **Constants**: `kPascalCase`

#### Documentation
Use Doxygen format:
```cpp
/**
 * @brief Brief description
 * 
 * Detailed description if needed.
 * 
 * @param param1 Description of param1
 * @param param2 Description of param2
 * @return Description of return value
 */
ReturnType functionName(Type1 param1, Type2 param2);
```

## Architecture Patterns

### Configuration Pattern

All configuration parameters are managed through ROS parameter server with defaults:

```python
class MyConfig:
    def __init__(self):
        self.param = rospy.get_param('~param_name', default_value)
        self._validate()
        self._log_configuration()
```

**Benefits**:
- Runtime configurability
- Clear defaults
- Validation on startup
- Self-documenting via logs

### Processor Pattern

Stateless processing classes for reusable operations:

```python
class ImageProcessor:
    def __init__(self, config_params):
        """Initialize with configuration."""
        self.threshold = config_params
    
    def process(self, input_data):
        """Pure processing function."""
        # No side effects
        return result
```

**Benefits**:
- Easy to test
- Reusable
- Composable
- Thread-safe (if truly stateless)

### Node Pattern

ROS nodes follow this structure:

```python
class MyNode:
    def __init__(self):
        rospy.init_node('my_node')
        self._load_config()
        self._initialize_processors()
        self._initialize_ros_interfaces()
    
    def _callback(self, msg):
        """Process incoming message."""
        pass
    
    def run(self):
        """Start node (blocking)."""
        rospy.spin()
```

## Adding New Features

### Adding a New Vision Filter

1. Create new class in `vision_utils.py`:
```python
class NewFilter:
    """Description of new filter."""
    
    def __init__(self, param1: float):
        self.param1 = param1
    
    def filter(self, image: np.ndarray) -> np.ndarray:
        """Apply filter to image."""
        # Implementation
        return filtered_image
```

2. Add configuration in `config.py`:
```python
class DetectionConfig:
    def __init__(self):
        # ... existing params ...
        self.new_filter_param = rospy.get_param('~new_filter_param', 1.0)
```

3. Integrate in `detection_node.py`:
```python
class StrawberryDetectionNode:
    def _initialize_processors(self):
        # ... existing processors ...
        self.new_filter = NewFilter(self.config.new_filter_param)
    
    def _process_frame(self, image):
        filtered = self.new_filter.filter(image)
        # ... continue processing ...
```

4. Update launch file with new parameter

### Adding a New Detection Output

1. Define new publisher in `detection_node.py`:
```python
self.new_output_pub = rospy.Publisher(
    self.config.new_output_topic, NewMsgType, queue_size=10
)
```

2. Publish in processing pipeline:
```python
def _publish_detections(self, detections):
    # ... existing publishing ...
    new_msg = self._create_new_message(detections)
    self.new_output_pub.publish(new_msg)
```

## Testing

### Unit Tests

Create tests in `test/` directory:

```python
#!/usr/bin/env python
import unittest
import numpy as np
from vision_utils import ColorFilterProcessor

class TestColorFilter(unittest.TestCase):
    def setUp(self):
        self.processor = ColorFilterProcessor(threshold=165)
    
    def test_filter_returns_contours(self):
        # Create test image
        test_image = np.zeros((480, 640, 3), dtype=np.uint8)
        # ... add test content ...
        
        contours, processed = self.processor.filter_strawberries(test_image)
        
        self.assertIsInstance(contours, list)
        self.assertEqual(processed.shape[2], 3)

if __name__ == '__main__':
    unittest.main()
```

Run tests:
```bash
cd ~/sbdm_ws
catkin_make run_tests
```

### Integration Tests

Test with bag files:
```bash
# Record test data
rosbag record -O test_data.bag /zedm/zed_node/left/image_rect_color /zedm/zed_node/depth/depth_registered /zedm/zed_node/odom

# Play back for testing
roslaunch sb_detection detection_pipeline.launch model_path:=/path/to/model.h5
rosbag play test_data.bag
```

## Debugging

### Enable Debug Logging

Set ROS log level:
```bash
rosservice call /strawberry_detector/set_logger_level "logger: 'rosout'
level: 'debug'"
```

Or in launch file:
```xml
<node pkg="sb_detection" type="detection_node.py" name="strawberry_detector" output="screen">
    <env name="ROSCONSOLE_CONFIG_FILE" value="$(find sb_detection)/config/debug.conf"/>
</node>
```

### Visualize Debug Images

```bash
rqt_image_view /debug_image
```

### Monitor Detection Performance

```bash
rostopic hz /sb_coordinates  # Detection rate
rostopic echo /sb_coordinates  # See detections
```

### Check TF Transforms

```bash
rosrun tf view_frames  # Generate PDF of TF tree
rosrun tf tf_echo world camera  # Monitor specific transform
```

## Performance Profiling

### Python Profiling

Add to your code:
```python
import cProfile
import pstats

def profile_callback():
    profiler = cProfile.Profile()
    profiler.enable()
    
    # ... your code ...
    
    profiler.disable()
    stats = pstats.Stats(profiler)
    stats.sort_stats('cumulative')
    stats.print_stats(20)  # Top 20 functions
```

### Memory Profiling

```bash
pip install memory_profiler
python -m memory_profiler detection_node.py
```

## Best Practices

### DO:
- ✅ Use type hints
- ✅ Write comprehensive docstrings
- ✅ Log at appropriate levels
- ✅ Validate inputs
- ✅ Handle errors gracefully
- ✅ Use configuration files
- ✅ Keep functions small and focused
- ✅ Write tests for new features

### DON'T:
- ❌ Use global mutable state
- ❌ Hardcode magic numbers
- ❌ Use print() for logging
- ❌ Ignore exceptions
- ❌ Mix concerns in one function
- ❌ Commit commented-out code
- ❌ Leave TODOs without issues

## Git Workflow

### Branching Strategy
- `main`: Stable, production-ready code
- `develop`: Integration branch
- `feature/xxx`: New features
- `bugfix/xxx`: Bug fixes
- `refactor/xxx`: Code improvements

### Commit Messages
Follow conventional commits:
```
feat(detection): add new color space filter
fix(plotter): correct coordinate transformation offset
refactor(broadcaster): convert to OOP pattern
docs(readme): update installation instructions
test(vision): add unit tests for depth validation
```

### Pull Request Checklist
- [ ] Code follows style guide
- [ ] All tests pass
- [ ] Documentation updated
- [ ] No linting errors
- [ ] Commit messages are clear
- [ ] Branch is up-to-date with main

## Deployment

### Building the Workspace

```bash
cd ~/sbdm_ws
catkin_make
source devel/setup.bash
```

### Running the System

```bash
# Start ZED camera
roslaunch zed_wrapper zedm.launch

# Start detection pipeline
roslaunch sb_detection detection_pipeline.launch model_path:=/path/to/your/model.h5

# Optional: Start trajectory visualization
rosrun hector_trajectory_server hector_trajectory_server

# Optional: Launch RViz
roslaunch sb_detection detection_pipeline.launch rviz:=true
```

### Troubleshooting

**Model not found**:
```bash
# Check model path parameter
rosparam get /strawberry_detector/model_path

# Set correct path
rosparam set /strawberry_detector/model_path /full/path/to/model.h5
```

**No detections**:
- Check camera topics: `rostopic list | grep zedm`
- Verify image stream: `rqt_image_view`
- Lower confidence threshold temporarily
- Check debug image for filtering results

**TF errors**:
```bash
# Check if broadcaster is running
rosnode list | grep broadcaster

# View TF tree
rosrun rqt_tf_tree rqt_tf_tree
```

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes following this guide
4. Add tests
5. Update documentation
6. Submit a pull request

## Resources

- [ROS Best Practices](http://wiki.ros.org/BestPractices)
- [PEP 8 Style Guide](https://pep8.org/)
- [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html)
- [Python Type Hints](https://docs.python.org/3/library/typing.html)
- [ROS Launch Files](http://wiki.ros.org/roslaunch/XML)
