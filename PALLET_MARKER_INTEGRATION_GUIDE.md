# Pallet Marker Integration Guide

## Overview

The pallet marker package provides visual markers for tracking pallets through their lifecycle in the RGV (Robot Guided Vehicle) system. This guide explains how to integrate the pallet marker node with your main RGV launch file.

## Package Structure

```
src/demonstrations/rmf_demos/pallet_marker/
├── pallet_marker/
│   ├── __init__.py
│   ├── multi_pallet_marker_node_rgv.py    # Main pallet marker node
│   ├── tf_simulator.py
│   └── test_dispenser_subscription.py
├── launch/
│   └── pallet_marker.launch.py            # Dedicated launch file
├── setup.py                               # Package configuration
└── package.xml                            # Package dependencies
```

## Dependencies

The pallet marker package requires these ROS 2 packages:
- `rclpy`
- `tf2_ros`
- `visualization_msgs`
- `rmf_dispenser_msgs`
- `rmf_fleet_msgs`
- `rmf_ingestor_msgs`

## Integration Methods

### Method 1: Using the Dedicated Launch File (Recommended)

Create a new launch file that includes the pallet marker:

```python
#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        # Include your main RGV launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('rmf_demos'),
                    'launch',
                    'rgv.launch.xml'
                ])
            ])
        ),
        
        # Include the pallet marker launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('pallet_marker'),
                    'launch',
                    'pallet_marker.launch.py'
                ])
            ])
        ),
    ])
```

### Method 2: Direct Node Inclusion (Already Implemented)

The pallet marker node is already included in the main RGV launch file (`src/demonstrations/rmf_demos/rmf_demos/launch/rgv.launch.xml`):

```xml
<!-- Pallet marker node -->
<node
  pkg="pallet_marker"
  exec="multi_pallet_marker_node_rgv"
  name="multi_pallet_marker_node_rgv"
  output="screen"
/>

<!-- Static transform publisher -->
<node
  pkg="tf2_ros"
  exec="static_transform_publisher"
  name="static_transform_publisher_map_to_world"
  args="0 0 0 0 0 0 map world"
  output="screen"
/>
```

### Method 3: Manual Node Addition

If you want to add the pallet marker node manually to any launch file:

```xml
<!-- For XML launch files -->
<node
  pkg="pallet_marker"
  exec="multi_pallet_marker_node_rgv"
  name="multi_pallet_marker_node_rgv"
  output="screen"
  parameters="[{'use_sim_time': true}]"
/>
```

```python
# For Python launch files
Node(
    package='pallet_marker',
    executable='multi_pallet_marker_node_rgv',
    name='multi_pallet_marker_node_rgv',
    output='screen',
    parameters=[{'use_sim_time': True}],
)
```

## Building and Installation

1. **Build the package:**
   ```bash
   cd /home/ubuntu/rgv_layout
   colcon build --packages-select pallet_marker
   ```

2. **Source the workspace:**
   ```bash
   source install/setup.bash
   ```

## Launching the System

### Option 1: Launch with Gazebo Classic (Recommended for simulation)
```bash
ros2 launch rmf_demos_gz_classic rgv.launch.xml
```

### Option 2: Launch without simulation
```bash
ros2 launch rmf_demos rgv.launch.xml
```

### Option 3: Launch pallet marker separately
```bash
ros2 launch pallet_marker pallet_marker.launch.py
```

## How the Pallet Marker Works

### 1. **Pallet States**
- **State 0**: At pickup location (dark blue)
- **State 1**: Attaching to robot (dark blue)
- **State 2**: Carried by robot (follows robot position)
- **State 3**: At dropoff location (dark blue)
- **State 4**: Dropped (purple, stays at final location)

### 2. **Message Subscriptions**
- `/dispenser_requests`: Maps pickup requests to pallet locations
- `/fleet_states`: Tracks robot positions and task assignments
- `/fleet_markers`: Gets exact robot positions for marker positioning
- `/ingestor_requests`: Maps dropoff requests to dropoff locations

### 3. **Published Topics**
- `/pallet_markers`: Visual markers for pallets (MarkerArray)

### 4. **Pickup Locations**
The system tracks these pickup locations:
- `pick1`: (53.1872, -13.1755)
- `pick2`: (55.0161, -22.0214)
- `pick3`: (44.0161, -31.4271)
- `pick4`: (42.87, -42.2885)
- `pick5`: (59.9429, -18.1396)
- `pick6`: (59.9429, -13.4368)

### 5. **Dropoff Locations**
Multiple dropoff locations per zone:
- `drop1`: 2 locations
- `drop2`: 2 locations
- `drop3`: 2 locations
- `drop4`: 2 locations
- `drop5`: 2 locations
- `drop6`: 2 locations
- `drop8`: 2 locations

## Configuration

### Customizing Pallet Locations
Edit the `self.pallets` list in `multi_pallet_marker_node_rgv.py`:

```python
self.pallets = [
    {
        'pickup': (x, y),           # Pickup coordinates
        'dropoff': (x, y),          # Default dropoff coordinates
        'state': 0,                 # Initial state
        'target_guid': 'pick1',     # Pickup location identifier
        # ... other fields
    },
    # ... more pallets
]
```

### Customizing Dropoff Locations
Edit the `self.dropoff_locations` dictionary:

```python
self.dropoff_locations = {
    'drop1': [(x1, y1), (x2, y2)],  # Multiple locations per zone
    'drop2': [(x1, y1), (x2, y2)],
    # ... more dropoff zones
}
```

## Troubleshooting

### Common Issues

1. **Markers not appearing:**
   - Check if `/pallet_markers` topic is being published
   - Verify RViz is subscribed to the topic
   - Ensure transform tree is correct

2. **Pallets not moving:**
   - Check if dispenser/ingestor requests are being received
   - Verify robot fleet states are being published
   - Check robot task assignments

3. **Wrong colors or positions:**
   - Verify pallet state transitions
   - Check pickup/dropoff coordinates
   - Ensure robot position data is accurate

### Debug Commands

```bash
# Check if pallet marker node is running
ros2 node list | grep pallet

# Check published topics
ros2 topic list | grep pallet

# Monitor pallet markers
ros2 topic echo /pallet_markers

# Check node info
ros2 node info /multi_pallet_marker_node_rgv

# Monitor dispenser requests
ros2 topic echo /dispenser_requests

# Monitor ingestor requests
ros2 topic echo /ingestor_requests
```

## Integration with RViz

To visualize the pallet markers in RViz:

1. **Add MarkerArray display:**
   - In RViz, go to "Add" → "By topic" → "/pallet_markers"
   - Or manually add "MarkerArray" display and set topic to `/pallet_markers`

2. **Configure marker appearance:**
   - Marker size: 0.6 x 0.5 x 0.25 meters
   - Colors: Dark blue (pickup/carried), Purple (dropped)
   - Frame: "map" for static markers, robot frame for carried markers

## Performance Considerations

- The pallet marker node runs at 20Hz (0.05s timer)
- Markers are published as MarkerArray for efficiency
- Robot position updates are throttled to prevent excessive processing
- Static dropoff markers are drawn once and reused

## Future Enhancements

Potential improvements for the pallet marker system:
- Add support for different pallet types/sizes
- Implement pallet collision detection
- Add pallet status indicators (damaged, priority, etc.)
- Support for dynamic pallet creation/removal
- Integration with warehouse management system 