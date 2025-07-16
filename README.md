# Panda Base Simulation

A ROS package that emulates a Franka Emika Panda arm on a custom base. It provides URDF models, controller configurations and launch files for running the robot in Gazebo and RViz.

## Prerequisites
- Ubuntu 20.04 or newer
- ROS Noetic
- `franka_ros`
- `gazebo_ros`
- `interactive_markers`
- `franka_gazebo`
- catkin or colcon build tools

## Setup
1. Clone the repository into your workspace:
   ```bash
   cd ~/catkin_ws/src
   git clone https://github.com/yunusdanabas/panda_base_sim.git
   ```
2. Install dependencies:
   ```bash
   cd ~/catkin_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```
3. Build the workspace and source it:
   ```bash
   catkin_make    # or: colcon build
   source devel/setup.bash
   ```

## Running
Launch the full simulation with controllers and RViz:
```bash
roslaunch panda_base_sim panda_base_sim.launch
```
Spawn only the base and arm in Gazebo:
```bash
roslaunch panda_base_sim onlybase_gazebo.launch
```

## Directory Overview
- `config/` – controller parameters
- `launch/` – launch files
- `meshes/` – visual and collision meshes
- `robots/` – robot xacro files
- `scripts/` – auxiliary scripts
- `urdf/` – generated URDF files

## Technologies
- ROS and Gazebo
- Franka Gazebo plugins
- Standard ROS controllers and interactive markers

## Example
Start the simulation and interact with the robot in RViz:
```bash
roslaunch panda_base_sim panda_base_sim.launch
```

Design diagrams are available in the `pdf/` directory. Any demo video or report can be linked here if available.

