# Husky A300 – Isaac Sim 5.1 + ROS 2 Bridge

Clearpath Husky A300 simulated in NVIDIA Isaac Sim 5.1 with a fully
functional ROS 2 Humble bridge. Everything is built programmatically
from a single Python script — no GUI interaction required.

## Prerequisites

| Component | Version |
|---|---|
| NVIDIA Isaac Sim | 5.1 |
| ROS 2 | Humble |
| GPU driver | ≥ 570 (RTX required) |
| Docker + NVIDIA Container Toolkit | For containerised run |

## Quick Start (Native)

```bash
# 1. Run the simulation (from Isaac Sim install directory)
./python.sh <path-to>/scripts/create_warehouse_scene.py

# 2. In another terminal, verify ROS 2 topics
ros2 topic list
ros2 topic hz /odom

# 3. Visualise in RViz2
ros2 launch scripts/view_in_rviz.launch.py
```

Keyboard teleop is also available in the Isaac Sim window:
`I` forward, `K` reverse, `J` left, `L` right, `X` stop.

## Quick Start (Docker)(in work)

```bash
# Pull the images
docker pull nvcr.io/nvidia/isaac-sim:5.1.0
docker pull osrf/ros:humble-desktop

# Start Isaac Sim + RViz2
docker compose up

# Or run only the simulation
docker compose up isaac-sim
```

## Published ROS 2 Topics

```
$ ros2 topic list
/clock
/cmd_vel
/joint_states
/odom
/tf
```

### Topic details

| Topic | Type | Hz | Notes |
|---|---|---|---|
| `/clock` | `rosgraph_msgs/Clock` | ~40 | Simulation time |
| `/tf` | `tf2_msgs/TFMessage` | ~40 | Full articulation tree (23 transforms) |
| `/joint_states` | `sensor_msgs/JointState` | ~40 | 4 wheel joints |
| `/odom` | `nav_msgs/Odometry` | ~40 | `frame_id: odom`, `child_frame_id: base_link` |
| `/cmd_vel` | `geometry_msgs/Twist` | — | Subscribed → differential drive controller |

## Project Structure

```
assignment/
├── Dockerfile                    # Isaac Sim container
├── docker-compose.yml            # Orchestrates Isaac Sim + RViz2
├── scripts/
│   ├── create_warehouse_scene.py # Main simulation script
│   ├── view_in_rviz.launch.py    # ROS 2 launch file for RViz2
│   └── husky.rviz                # RViz2 display config
├── simulation/
│   └── husky/husky_a300/         # Converted Husky USD assets
├── Kemabots-Robotics-Sim-Assignment/
│   ├── husky_a300.urdf           # Robot description
│   ├── meshes/                   # 3D mesh files
│   └── topics.md                 # Required topics spec
└── README.md
```

## Architecture

```
┌──────────────────────────────────────────────────┐
│              Isaac Sim 5.1                       │
│                                                  │
│  /World/Warehouse  ← NVIDIA Simple Warehouse USD │
│  /World/Husky      ← Husky A300 USD              │
│                                                  │
│  OmniGraph: /World/ROS2Publishers                │
│    OnPlaybackTick → PublishClock (/clock)         │
│                   → PublishTF (/tf)               │
│                   → PublishJointState (/joint_st) │
│                   → ComputeOdom → PublishOdom     │
│                                                  │
│  OmniGraph: /World/ROS2CmdVel                    │
│    SubscribeTwist → DiffController → ArticCtrl   │
│                                                  │
│  HuskyKeyboardTeleop (I/K/J/L/X)                 │
└──────────────┬───────────────────────────────────┘
               │ ROS 2 DDS (localhost)
               │
┌──────────────┴───────────────────────────────────┐
│  RViz2 + robot_state_publisher                   │
│    /robot_description (URDF)                     │
│    3D robot model + TF axes + Odometry arrows    │
└──────────────────────────────────────────────────┘
```
