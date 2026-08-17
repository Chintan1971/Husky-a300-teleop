import os
from pathlib import Path
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    project_root = Path(__file__).resolve().parents[1]
    
    # Path to URDF file and meshes folder
    urdf_path = project_root / "Kemabots-Robotics-Sim-Assignment" / "husky_a300.urdf"
    meshes_dir = project_root / "Kemabots-Robotics-Sim-Assignment" / "meshes"
    
    if not urdf_path.is_file():
        urdf_path = project_root / "simulation" / "husky" / "husky_a300.urdf"
        meshes_dir = project_root / "simulation" / "husky" / "meshes"

    with open(urdf_path, "r") as f:
        robot_desc = f.read()

    # Convert relative 'meshes/' paths in URDF to absolute file:// URIs for RViz2
    absolute_mesh_prefix = f"file://{meshes_dir.resolve()}/"
    robot_desc = robot_desc.replace('filename="meshes/', f'filename="{absolute_mesh_prefix}')

    return LaunchDescription([
        # Robot State Publisher -> publishes /robot_description with absolute mesh URIs
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "robot_description": robot_desc,
            }],
        ),
        # RViz2 -> displays robot model & TF
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            parameters=[{"use_sim_time": True}],
        ),
    ])
