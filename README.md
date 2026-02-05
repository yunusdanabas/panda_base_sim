# panda_base_sim

Gazebo 11 + RViz digital twin of a Franka Emika Panda arm on a custom aluminium-profile base. ROS Noetic package for simulation-based controller bring-up and safe pre-hardware validation.

![Simulation](simulation.png)

## What it provides

- **One-command bring-up** — `panda_base_sim.launch` starts Gazebo, controllers, and RViz.
- **Modular robot description** — `base_panda.urdf.xacro` (base + arm + optional hand); `base_sensor_urdf.urdf` for base-only checks.
- **ros_control + Franka Gazebo** — `FrankaHWSim` with pre-wired controller configs (Cartesian impedance, joint position/velocity, trajectory).
- **Interactive marker tele-op** — 6-DoF equilibrium-pose marker in RViz when using the Cartesian impedance controller.

## Prerequisites

- Ubuntu 20.04 (or newer)
- ROS Noetic with `gazebo_ros`, `franka_ros`, `interactive_markers`
- `franka_gazebo` (Panda controllers and meshes)
- catkin or colcon

## Install

```bash
cd ~/catkin_ws/src
git clone https://github.com/yunusdanabas/panda_base_sim.git
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
```

## Run

**Full simulation (Gazebo + controllers + RViz):**

```bash
roslaunch panda_base_sim panda_base_sim.launch
```

**Base only (no arm):**

```bash
roslaunch panda_base_sim onlybase_gazebo.launch
```

**Useful launch args:**

```bash
roslaunch panda_base_sim panda_base_sim.launch headless:=true rviz:=false
roslaunch panda_base_sim panda_base_sim.launch use_gripper:=false
roslaunch panda_base_sim panda_base_sim.launch controller:=joint_position_example_controller
```

With the default Cartesian impedance controller, use the **Equilibrium Pose** marker in RViz to command the arm, or publish to `/equilibrium_pose` (e.g. `geometry_msgs/PoseStamped`).

## Layout

```
panda_base_sim/
├── config/   # Franka HW sim + controller YAMLs
├── launch/   # Gazebo / RViz launch files
├── robots/   # xacro macros + RViz config
├── meshes/   # STL/DAE (CAD exports)
├── scripts/  # interactive_marker.py
└── urdf/     # base_panda.urdf.xacro, base_sensor_urdf.urdf
```

## Gallery

| Simulation | Front | Isometric | Side |
|------------|-------|-----------|------|
| ![Simulation](simulation.png) | ![Front](front.png) | ![Iso](iso.png) | ![Side](side.png) |

## License

MIT. Developed during a Forschungspraxis internship at TUM-MIRMI; built on Franka ROS and Gazebo.
