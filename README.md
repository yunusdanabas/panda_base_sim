```markdown
# panda_base_sim

**ROS package for simulating a custom Panda‑arm base in Gazebo**

This package was developed during my Forschungspraxis internship at the Munich Institute of Robotics and Machine Intelligence (Technical University of Munich) for the project  
**“Investigating Force and Moment Scaling in Robotic Force‑Torque Sensors.”**  
It provides a digital twin of a Franka Emika Panda arm mounted on a bespoke aluminum‑profile base, complete with placeholder FT‑sensor, for pre‑hardware validation of high‑moment loading and controller tuning.

---

## 🚀 Features

- **Gazebo Simulation**  
  Full 3D simulation of a rigid base + Panda arm + FT‑sensor stub.
- **Modular URDF/Xacro**  
  Split into common, base, arm and sensor xacros for easy reuse and extension.
- **Integrated Controllers**  
  Preconfigured joint, trajectory and Cartesian‑impedance controllers tuned for the twin’s dynamics.
- **RViz Visualization**  
  Custom RViz preset with an interactive equilibrium‑pose marker for safe teleoperation.

---

## 🛠️ Prerequisites

- Ubuntu 20.04 or later  
- ROS Noetic (with `gazebo_ros`, `franka_ros`, `interactive_markers`)  
- Franka ROS driver (for controllers & robot_description)  
- `catkin_make` or `colcon` build tools

---

## 📦 Installation

1. Clone into your workspace:
   ```bash
   cd ~/catkin_ws/src
   git clone https://github.com/your‑username/panda_base_sim.git
   ```
2. Install dependencies:
   ```bash
   sudo apt update
   rosdep install --from-paths . --ignore-src -r -y
   ```
3. Build:
   ```bash
   cd ~/catkin_ws
   catkin_make  # or colcon build
   source devel/setup.bash
   ```

---

## 🎮 Usage

Launch the full simulation (Gazebo + controllers + RViz):
```bash
roslaunch panda_base_sim panda_base_sim.launch
```

If you only want to spawn the static base and arm in Gazebo:
```bash
roslaunch panda_base_sim onlybase_gazebo.launch
```

---

## 📸 Gallery

### In‑Simulation View  
![Gazebo + RViz](simulation.png)

### Design Photos  
| Front View | Isometric View | Side View |
|:----------:|:--------------:|:---------:|
| ![Front](front.png) | ![Iso](iso.png) | ![Side](side.png) |

---

## 📂 Package Structure

```
panda_base_sim/
├── config/                   # controller and hardware YAMLs
├── launch/                   # Gazebo & RViz launch files
├── meshes/                   # STL and Collada visual meshes
├── robots/
│   ├── common/               # shared xacros (arm, hand, utils)
│   └── panda/                # Panda-specific xacros & RViz config
├── scripts/
│   └── interactive_marker.py # equilibrium-pose marker node
├── urdf/                     # generated and source xacro URDFs
├── simulation.png            # example screenshot
├── front.png                 # design renders
├── iso.png
├── side.png
├── CMakeLists.txt
└── package.xml
```

---

## 🤝 Acknowledgements

– Research internship supported by the Munich Institute of Robotics and Machine Intelligence (TUM)  
– Based on existing Franka ROS drivers and Gazebo plugins  

---

## 📄 License

This project is released under the [MIT License](LICENSE).  
Feel free to use and adapt for your own digital‑twin and controller‑tuning needs!
```