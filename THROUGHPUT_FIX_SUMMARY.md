# RMF Robot Monitor Panel - Throughput Monitoring Fix

## Issue Description

The RMF Robot Monitor Panel was not properly recording simulation time for throughput calculations. The JSON log files showed:
- `total_sim_time_sec: 0.0`
- `total_sim_time_hms: "0s"`
- `throughput_rate_tasks_per_sec: 0.0`

This meant that the panel was only recording the number of completed tasks but not the time duration, making throughput calculations impossible.

## Root Cause

The issue was in the `simple_robot_panel.py` file:

1. **Missing initialization**: The `first_task_assignment_time` was not being properly initialized when the panel started
2. **Conditional logic**: The throughput calculation depended on `hasattr(self, 'first_task_assignment_time')` which could fail if the attribute wasn't set
3. **Timing dependency**: The first task assignment time was only set when the first dispatch message was received, which might not happen immediately

## Fixes Applied

### 1. Initialize `first_task_assignment_time` at Panel Startup

**File**: `src/demonstrations/rmf_demos/rmf_robot_monitor_panel/src/simple_robot_panel.py`

**Change**: Added initialization in the `__init__` method:
```python
# Initialize first_task_assignment_time to current simulation time
self.first_task_assignment_time = self.node.get_clock().now()
self.node.get_logger().info(f"Panel initialized at simulation time: {self.first_task_assignment_time.nanoseconds / 1e9:.2f}s")
```

### 2. Simplified Throughput Calculation Logic

**File**: `src/demonstrations/rmf_demos/rmf_robot_monitor_panel/src/simple_robot_panel.py`

**Change**: Removed conditional checks and always calculate time window:
```python
# Always calculate time window from panel initialization or first task assignment
current_sim_time = self.node.get_clock().now()
total_time_window = (current_sim_time.nanoseconds - self.first_task_assignment_time.nanoseconds) / 1e9
throughput_rate = num_pallets / total_time_window if total_time_window > 0 else 0
```

### 3. Updated Dispatch Request Callback

**File**: `src/demonstrations/rmf_demos/rmf_robot_monitor_panel/src/simple_robot_panel.py`

**Change**: Modified to update first task assignment time only for the first actual task:
```python
# Only update if this is the first actual task (not just initialization)
if len(self.completed_tasks) == 0 and not hasattr(self, '_first_actual_task_assigned'):
    self.first_task_assignment_time = self.node.get_clock().now()
    self._first_actual_task_assigned = True
    self.node.get_logger().info(f"First actual task assigned at simulation time: {self.first_task_assignment_time.nanoseconds / 1e9:.2f}s")
```

### 4. Consistent Throughput Display

**File**: `src/demonstrations/rmf_demos/rmf_robot_monitor_panel/src/simple_robot_panel.py`

**Change**: Updated `update_table` method to use the same calculation logic as the JSON logging.

## Expected Results

After applying these fixes:

1. **JSON Log Files**: Will now show proper simulation time and throughput rates:
   ```json
   {
     "summary": {
       "total_tasks_completed": 58,
       "total_sim_time_sec": 1200.5,
       "total_sim_time_hms": "20m 0s",
       "throughput_rate_tasks_per_sec": 0.048
     }
   }
   ```

2. **Panel Display**: The throughput label will show:
   ```
   Throughput: 58 tasks completed in 1200.5s (sim) | Rate: 0.048 tasks/sec
   ```

3. **Real-time Updates**: Both the JSON file and panel display will update in real-time as tasks complete.

## Testing

The fixes have been tested with a simulation that confirms:
- Time calculation works correctly
- Throughput rates are calculated properly
- JSON files contain valid time data
- Panel display shows accurate throughput information

## Build Instructions

To apply the fixes:

```bash
cd /home/ubuntu/rgv_layout
colcon build --packages-select rmf_robot_monitor_panel
source install/setup.bash
```

## Launch Instructions

The panel can be launched with:

```bash
ros2 launch rmf_robot_monitor_panel simple_robot_panel.launch.py
```

The launch file is already configured to use simulation time by default (`use_sim_time:=true`). 