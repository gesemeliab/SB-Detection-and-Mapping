# Migration Guide: Legacy to Refactored Code

## Overview

This guide helps you transition from the legacy codebase to the refactored, production-ready version while maintaining full backward compatibility.

## Quick Start

### Option 1: Use Refactored Code (Recommended)

```bash
# Use the new unified launch file
roslaunch sb_detection detection_pipeline.launch \
    model_path:=/path/to/your/model.h5 \
    confidence_threshold:=0.98 \
    rviz:=true
```

### Option 2: Continue Using Legacy Code

The original files are preserved:
- `sb_detection/src/SBDetection.py` (legacy detection node)
- `sb_detection/src/strawberry.py` (legacy utilities)
- `plotting_points/src/sb_plotter.py` (legacy plotter)
- `broadcaster/src/broadcaster.cpp` (legacy broadcaster)

## Side-by-Side Comparison

### Running Detection Node

**Legacy**:
```bash
# Must set model path in code before running
rosrun sb_detection SBDetection.py
```

**Refactored**:
```bash
# Model path as parameter - no code changes needed
roslaunch sb_detection detection_pipeline.launch model_path:=/path/to/model.h5
```

### Running Plotter

**Legacy**:
```bash
rosrun plotting_points sb_plotter.py
```

**Refactored**:
```bash
# Included in detection_pipeline.launch, or run standalone:
rosrun plotting_points plotter_node.py
```

### Running Broadcaster

**Legacy**:
```bash
roslaunch broadcaster rover.launch
```

**Refactored**:
```bash
# Included in detection_pipeline.launch, or uses new node:
rosrun broadcaster tf_broadcaster_node
```

## Key Differences

### Configuration

| Aspect | Legacy | Refactored |
|--------|--------|------------|
| Model path | Hardcoded in Python file | ROS parameter |
| Confidence threshold | Hardcoded (0.98) | Configurable parameter |
| Focal length | Global variable | Configuration class |
| Topics | Hardcoded strings | Configurable parameters |

### Code Organization

| Aspect | Legacy | Refactored |
|--------|--------|------------|
| Structure | Monolithic script | Modular classes |
| Configuration | Magic numbers | Config classes |
| Error handling | Minimal | Comprehensive |
| Logging | print() statements | ROS logging |
| Documentation | Sparse comments | Full docstrings |

### Functionality

**100% Feature Parity** - No functional changes:
- ✅ Same detection algorithm (LAB color space + CNN)
- ✅ Same coordinate transformations
- ✅ Same visualization markers
- ✅ Same TF broadcasting
- ✅ Same ROS message types

## Migrating Your Setup

### Step 1: Update Your Model Path

**Before** (editing Python file):
```python
PATH_TO_MODEL = "/home/user/models/my_model.h5"
```

**After** (launch file or command line):
```xml
<arg name="model_path" default="/home/user/models/my_model.h5" />
```

### Step 2: Update Launch Files

**Before**:
```xml
<launch>
  <node pkg="broadcaster" type="broadcaster" args="/rover" name="rover_tf_broadcaster" />
  <!-- Then run detection and plotter manually -->
</launch>
```

**After**:
```xml
<launch>
  <!-- Everything in one launch file -->
  <include file="$(find sb_detection)/launch/detection_pipeline.launch">
    <arg name="model_path" value="/path/to/model.h5" />
  </include>
</launch>
```

### Step 3: Update Parameter Access

If you've customized parameters:

**Before**:
```python
# Edit in code
f = 1396.91
if acc >= 0.98:
```

**After**:
```bash
# Set via ROS parameters
rosparam set /strawberry_detector/focal_length 1396.91
rosparam set /strawberry_detector/confidence_threshold 0.98
```

### Step 4: Update Node Names

**Before**:
```bash
rosnode list
# /detector_node
# /strawberry_plt
```

**After**:
```bash
rosnode list
# /strawberry_detector
# /strawberry_plotter
# /tf_broadcaster
```

Update any scripts or configs that reference these nodes.

## Configuration Parameters Reference

### Detection Node Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `~model_path` | string | "" | Path to Keras model file (required) |
| `~confidence_threshold` | float | 0.98 | Minimum confidence for detection |
| `~focal_length` | float | 1396.91 | Camera focal length (pixels) |
| `~lab_a_threshold` | int | 165 | LAB A-channel threshold |
| `~central_region_ratio` | float | 0.05 | Central ROI width ratio |
| `~publish_debug_image` | bool | true | Enable debug visualization |

### Plotter Node Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `~marker_scale` | float | 0.05 | Size of visualization markers |
| `~marker_color_r` | float | 1.0 | Red component [0-1] |
| `~offset_x` | float | 0.545 | X-axis calibration offset |
| `~offset_y` | float | -0.495 | Y-axis calibration offset |
| `~offset_z` | float | 0.39 | Z-axis calibration offset |

### Broadcaster Node Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `~rover_height` | float | 0.13 | Rover base height (meters) |
| `~camera_offset_x` | float | 0.0 | Camera X offset from rover |
| `~camera_offset_y` | float | -0.21 | Camera Y offset from rover |
| `~camera_offset_z` | float | 0.26 | Camera Z offset from rover |

## Testing Your Migration

### 1. Verify Node Startup

```bash
# Launch refactored system
roslaunch sb_detection detection_pipeline.launch model_path:=/path/to/model.h5

# Check all nodes are running
rosnode list

# Should see:
# /tf_broadcaster
# /strawberry_detector
# /strawberry_plotter
```

### 2. Verify Topics

```bash
# Check published topics
rostopic list | grep -E "(sb_coordinates|debug_image|markers)"

# Should see:
# /sb_coordinates
# /debug_image
# /strawberry_markers (or /markers depending on config)
```

### 3. Verify TF Frames

```bash
# Check transform tree
rosrun tf view_frames

# Should generate PDF showing:
# world -> rover -> camera
```

### 4. Compare Detection Output

Run both versions side-by-side with same bag file:

```bash
# Record test data
rosbag record -O test.bag /zedm/zed_node/left/image_rect_color \
                          /zedm/zed_node/depth/depth_registered \
                          /zedm/zed_node/odom

# Test legacy (in one terminal)
rosrun sb_detection SBDetection.py
rosbag play test.bag
rostopic echo /sb_coordinates > legacy_output.txt

# Test refactored (in another terminal after stopping legacy)
roslaunch sb_detection detection_pipeline.launch model_path:=/path/to/model.h5
rosbag play test.bag
rostopic echo /sb_coordinates > refactored_output.txt

# Compare outputs (should be identical)
diff legacy_output.txt refactored_output.txt
```

## Troubleshooting Migration Issues

### Issue: "Model file not found"

**Cause**: Empty or incorrect model path

**Solution**:
```bash
# Check parameter
rosparam get /strawberry_detector/model_path

# Set correct path
roslaunch sb_detection detection_pipeline.launch \
    model_path:=/full/absolute/path/to/model.h5
```

### Issue: "No module named config"

**Cause**: Python path not set correctly

**Solution**:
```bash
# Rebuild workspace
cd ~/sbdm_ws
catkin_make
source devel/setup.bash
```

### Issue: Different detection results

**Possible causes**:
1. Different confidence thresholds
2. Different preprocessing parameters
3. Different model files

**Solution**:
```bash
# Match all parameters to legacy values
roslaunch sb_detection detection_pipeline.launch \
    model_path:=/path/to/model.h5 \
    confidence_threshold:=0.98 \
    lab_a_threshold:=165 \
    focal_length:=1396.91
```

### Issue: TF lookup errors

**Cause**: Frame names changed or broadcaster not running

**Solution**:
```bash
# Check TF tree
rosrun rqt_tf_tree rqt_tf_tree

# Verify broadcaster is running
rosnode info /tf_broadcaster

# Check frame names match your system
rosparam set /tf_broadcaster/rover_frame your_rover_name
```

## Gradual Migration Strategy

You can migrate incrementally:

### Phase 1: Broadcaster Only
```bash
# Use new broadcaster with legacy detection/plotter
rosrun broadcaster tf_broadcaster_node
rosrun sb_detection SBDetection.py
rosrun plotting_points sb_plotter.py
```

### Phase 2: Detection + Broadcaster
```bash
# Use new detection with legacy plotter
roslaunch sb_detection detection_pipeline.launch model_path:=/path/to/model.h5
# Kill plotter from launch, run legacy:
rosrun plotting_points sb_plotter.py
```

### Phase 3: Full Migration
```bash
# Use complete refactored pipeline
roslaunch sb_detection detection_pipeline.launch model_path:=/path/to/model.h5
```

## Benefits After Migration

✅ **No Code Edits Required** - Configure via parameters

✅ **Better Error Messages** - Clear logs when something fails

✅ **Easier Debugging** - Debug level logging available

✅ **Faster Iteration** - Change parameters without recompiling

✅ **Better Documentation** - Self-documenting code

✅ **Easier Maintenance** - Modular, tested components

✅ **Professional Quality** - Production-ready codebase

## Rollback Plan

If you need to revert to legacy code:

```bash
# All legacy files are preserved:
rosrun sb_detection SBDetection.py
rosrun plotting_points sb_plotter.py
roslaunch broadcaster rover.launch
```

Nothing is deleted - you can always go back!

## Getting Help

If you encounter issues during migration:

1. Check the logs: `rosnode list` then `rosnode info /node_name`
2. Verify parameters: `rosparam list | grep strawberry`
3. Compare with legacy: Run both versions with same input
4. Consult [DEVELOPER_GUIDE.md](./DEVELOPER_GUIDE.md)
5. Open an issue on GitHub with:
   - ROS version
   - Error messages
   - Parameter values used
   - Steps to reproduce

## Success Checklist

- [ ] All nodes start without errors
- [ ] TF tree shows correct frame hierarchy
- [ ] Detection topics publish at expected rate
- [ ] Markers appear in RViz
- [ ] Debug images show correct filtering
- [ ] Detection accuracy matches legacy version
- [ ] Parameters can be changed without code edits
- [ ] System can be launched with single command

## Next Steps

After successful migration:

1. Remove legacy code (optional - keep for reference)
2. Add custom configurations for your robot
3. Tune parameters for your specific use case
4. Write tests for your custom features
5. Contribute improvements back to the project

---

**Questions?** See [DEVELOPER_GUIDE.md](./DEVELOPER_GUIDE.md) or open an issue!
