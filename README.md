# restaurant_sim

A production-quality, modular, two-story restaurant environment for Gazebo Classic 11 and ROS 2.

## Package Structure
```
restaurant_sim/
├── CMakeLists.txt
├── package.xml
├── README.md
├── hooks/
│   └── restaurant_sim.dsv.in
├── launch/
│   └── restaurant_sim.launch.py
├── models/
│   ├── _components/       # Modular walls, floors, doors, windows, pillars
│   ├── _fixtures/         # Staircase, lift shaft, lift cabin
│   ├── _furniture/        # Tables, chairs, counters, docks, plants
│   ├── _lighting/         # Ceiling light panels, pendant lights
│   ├── ground_floor/      # Complete ground floor layout assembly
│   └── first_floor/       # Complete first floor layout assembly
└── worlds/
    ├── restaurant.sdf               # Full two-story restaurant world
    └── restaurant_empty_test.sdf    # Architectural shell only
```

## Naming & Design System
- **Metric Coordinate System**: Centered on $(0, 0)$ with a building footprint of $24\,\text{m} \times 16\,\text{m}$.
- **Heights**: Ground floor ceiling height is $3.4\,\text{m}$, first floor level is at $z = 3.5\,\text{m}$.
- **Materials**: Standard, clean, performance-optimized visual materials (dark wood tables, brushed metal counters, concrete pillars).

## Build Instructions

In your ROS 2 workspace (e.g. `colcon_ws`):

```bash
# Build the package
colcon build --packages-select restaurant_sim

# Source the workspace
source install/setup.bash
```

## Launching the Simulation

To launch the full restaurant world:
```bash
ros2 launch restaurant_sim restaurant_sim.launch.py
```

To launch the empty test structural shell world:
```bash
ros2 launch restaurant_sim restaurant_sim.launch.py world_name:=restaurant_empty_test.sdf
```
