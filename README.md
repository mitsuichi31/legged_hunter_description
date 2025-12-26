# legged_hunter_description

URDF Description package for the **legged_hunter** bipedal robot.

This package contains the robot model description including URDF/XACRO files, 3D meshes, and visualization launch files.

## Installation

This package is a standard ROS 2 package. You can build it using `colcon`:

```bash
cd ~/ros2_ws
colcon build --packages-select legged_hunter_description
source install/setup.bash
```

## Usage

### Visualization
To inspect the URDF model in RViz:

```bash
ros2 launch legged_hunter_description display.launch.py
```

### Simulation
For simulation with `ros2_control` and MuJoCo, use the separate `legged_hunter_controllers` package:

```bash
ros2 launch legged_hunter_controllers hunter_simulation.launch.py
```

## Package Structure

- **launch/**: Python launch files for visualization.
- **meshes/**: STL mesh files for the robot links.
- **urdf/**: XACRO/URDF robot descriptions with ros2_control tags.
- **rviz/**: RViz configuration files.

## Related Packages

- **legged_hunter_controllers**: Controller configurations and simulation launch files.

## License

MIT
