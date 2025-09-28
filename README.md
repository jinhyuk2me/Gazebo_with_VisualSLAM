<img src="https://github.com/jinhyuk2me/Gazebo_with_VisualSLAM/blob/main/assets/rviz.gif?raw=true" width="100%"/>

# Gazebo_with_VisualSLAM

Visual SLAM robot simulation project using ROS2 Jazzy and Gazebo.

## Overview

This workspace provides Visual SLAM and keyboard control functionality for a mobile robot equipped with an RGB-D camera.
It implements Visual SLAM using the RTAB-Map library.

## Package Structure

- **`hello_gazebo_bringup`** - Integrated launch and simulation environment
- **`hello_gazebo_description`** - Robot URDF model definition (includes RGB-D camera)
- **`hello_gazebo_visual_slam`** - RTAB-Map based Visual SLAM mapping and visualization

## Quick Start

### 1. Environment Setup
```bash
cd ~/dev_ws/Gazebo_with_VisualSLAM/hello_gazebo
source install/setup.bash
```

### 2. Basic Simulation Launch
```bash
# Launch Gazebo simulation only (RViz separated)
ros2 launch hello_gazebo_bringup launch_sim.launch.xml

# Use different world environment
ros2 launch hello_gazebo_bringup launch_sim.launch.xml world_name:=simple_world.world

# Change robot initial position
ros2 launch hello_gazebo_bringup launch_sim.launch.xml x:=10.0 y:=-10.0 z:=0.3
```

### 3. RViz Visualization for Visual SLAM

```bash
# Launch RViz for Visual SLAM
cd ~/dev_ws/Gazebo_with_VisualSLAM/hello_gazebo && source install/setup.bash  
ros2 launch hello_gazebo_visual_slam rviz_visual_slam.launch.xml
```

### 4. Visual SLAM Mapping
```bash
# Step 1: Launch simulation
ros2 launch hello_gazebo_bringup launch_sim.launch.xml

# Step 2: Launch RViz for Visual SLAM (new terminal)
cd ~/dev_ws/Gazebo_with_VisualSLAM/hello_gazebo && source install/setup.bash
ros2 launch hello_gazebo_visual_slam rviz_visual_slam.launch.xml

# Step 3: Start Visual SLAM (new terminal)
cd ~/dev_ws/Gazebo_with_VisualSLAM/hello_gazebo && source install/setup.bash
ros2 launch hello_gazebo_visual_slam visual_slam.launch.xml

# Step 4: Keyboard control (new terminal)
cd ~/dev_ws/Gazebo_with_VisualSLAM/hello_gazebo && source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Step 5: Save map (using RTAB-Map built-in map saving feature)
# In RTAB-Map GUI: File -> Export map... or use command:
rtabmap-export --db ~/.ros/rtabmap.db --format pgm my_visual_map.pgm
```

### 📊 Visual SLAM Monitoring

#### Elements visible in Visual SLAM RViz:
- 🎥 **Camera View**: RGB camera feed
- 📊 **Depth Image**: Depth image data
- 🗺️ **Grid Map**: RTAB-Map generated grid map
- 🔗 **Point Cloud**: 3D point cloud
- 📌 **Graph Nodes**: SLAM graph nodes
- 🔄 **Loop Closures**: Loop closure connections

### ⚠️ Execution Order Important Notes

1. **Visual SLAM sequence**: Simulation → Visual SLAM RViz → Visual SLAM launch → Keyboard control
2. **Separate terminals**: Each launch file should run in different terminals
3. **Camera sensor required**: Ensure RGB-D camera is working properly

## RTAB-Map Special Features

### Database Management
```bash
# View RTAB-Map database
rtabmap-databaseViewer ~/.ros/rtabmap.db

# Delete database (when starting new mapping)
rm ~/.ros/rtabmap.db
```

### Map Export
```bash
# Export map from RTAB-Map to standard grid map format
rtabmap-export --db ~/.ros/rtabmap.db --format pgm my_visual_map.pgm
```

## Saved Map Files

- `my_visual_map.pgm` / `my_visual_map.yaml` - Visual SLAM mapping results from factory_L1 environment

## Key Controls (Keyboard Control Mode)

- **Movement**: `i`(forward), `k`(stop), `j`(turn left), `l`(turn right)
- **Speed**: `q`/`z`(increase/decrease linear speed), `w`/`x`(increase/decrease angular speed)
- **Exit**: `Ctrl+C`

## Troubleshooting

### Build Errors
```bash
rm -rf build install log
colcon build --symlink-install
source install/setup.bash
```

### Gazebo Launch Errors
```bash
export LIBGL_ALWAYS_SOFTWARE=1
ros2 launch hello_gazebo_bringup launch_sim.launch.xml
```

## Project Structure

```
hello_gazebo/                        # ROS2 workspace
├── src/
│   ├── hello_gazebo_bringup/        # Simulation launcher
│   ├── hello_gazebo_description/    # Robot model definition
│   └── hello_gazebo_visual_slam/    # Visual SLAM package
├── build/                           # Build files
├── install/                         # Installed packages
├── log/                             # Build logs
├── my_map.pgm                       # Saved map image
└── my_map.yaml                      # Map metadata
```

## Required Dependencies

To run this project, you need:
- ROS2 Jazzy
- Gazebo Garden/Harmonic
- RTAB-Map library
- ROS-Gazebo bridge packages
