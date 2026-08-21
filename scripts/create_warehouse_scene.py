#!/usr/bin/env python3
"""Create the Husky A300 warehouse scene with ROS 2 bridge.

Publishes:  /clock, /tf, /joint_states, /odom
Subscribes: /cmd_vel  (drives the robot via differential controller)

Keyboard teleop (I/K/J/L/X) remains available as a parallel control method.
"""

from __future__ import annotations

from pathlib import Path
from isaacsim import SimulationApp

# ── Project settings ────────────────────────────────────────────────────────
PROJECT_ROOT = Path(__file__).resolve().parents[1]
ROBOT_USD_PATH = PROJECT_ROOT / "simulation/husky/husky_a300/husky_a300.usd"
WAREHOUSE_USD_PATH = (
    "https://omniverse-content-production.s3-us-west-2.amazonaws.com/"
    "Assets/Isaac/5.1/Isaac/Environments/Simple_Warehouse/warehouse.usd"
)

if not ROBOT_USD_PATH.is_file():
    raise FileNotFoundError(f"Robot USD not found: {ROBOT_USD_PATH}")

simulation_app = SimulationApp({"headless": False})

# Isaac/Omniverse imports must happen only after SimulationApp is created.
import omni.timeline
import omni.usd
from isaacsim.core.utils.extensions import enable_extension
from pxr import Gf, UsdGeom, UsdPhysics

from ros2_bridge import create_ros2_bridge_graphs
from teleop import HuskyKeyboardTeleop

# ── Enable the ROS 2 bridge extension ──────────────────────────────────────
enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

# ── Build the stage ─────────────────────────────────────────────────────────
omni.usd.get_context().new_stage()
stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)

world = UsdGeom.Xform.Define(stage, "/World")
stage.SetDefaultPrim(world.GetPrim())

physics_scene = UsdPhysics.Scene.Define(stage, "/World/physicsScene")
physics_scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
physics_scene.CreateGravityMagnitudeAttr().Set(9.81)

warehouse = UsdGeom.Xform.Define(stage, "/World/Warehouse")
warehouse.GetPrim().GetReferences().AddReference(WAREHOUSE_USD_PATH)

husky = UsdGeom.Xform.Define(stage, "/World/Husky")
husky.GetPrim().GetReferences().AddReference(str(ROBOT_USD_PATH.resolve()))

translate = husky.GetPrim().GetAttribute("xformOp:translate")
if not translate:
    translate = husky.AddTranslateOp().GetAttr()
translate.Set(Gf.Vec3d(0.0, 0.0, 0.165))

# ── Setup ROS 2 Bridge ──────────────────────────────────────────────────────
create_ros2_bridge_graphs(stage)

# Let references and graphs compose.
for _ in range(30):
    simulation_app.update()

# ── Setup RTX Lidar  ──────────────────────────────────────────────────────
from isaacsim.sensors.rtx import LidarRtx
import numpy as np
import omni.replicator.core as rep

lidar_prim_path = "/World/Husky/lidar3d_0_link/lidar3d_0_sensor_link/lidar"

lidar = LidarRtx(
    prim_path=lidar_prim_path,
    translation=np.array([0.0, 0.0, 0.0]),
    orientation=np.array([1.0, 0.0, 0.0, 0.0]),
    config_file_name="Example_Rotary",
)

lidar.initialize()


hydra_texture = rep.create.render_product(
    lidar_prim_path,
    [1, 1],
    name="LidarRenderProduct"
)
# ── Publish point cloud data on ros2 topic   ──────────────────────────────────────────────────────

writer = rep.writers.get("RtxLidarROS2PublishPointCloud")

writer.initialize(
    topicName="/point_cloud",
    frameId="lidar3d_0_link"
)

writer.attach([hydra_texture])
# lidar.attach_annotator("IsaacExtractRTXSensorPointCloudNoAccumulator")

# ── Setup camera ─────────────────────────────────────────────────────────────────────
camera_prim_path = "/World/Husky/camera_0_camera_center/camera_0_left_camera_frame/camera_0_temp_left_link/camera"

from isaacsim.sensors.camera import Camera
import omni.syntheticdata._syntheticdata as sd

camera = Camera(
    prim_path=camera_prim_path,
    translation=np.array([0.0, 0.0, 0.0]),
    frequency=30,
    resolution=(640, 480),
)

camera.initialize()
render_product = camera._render_product_path

rv = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(
    sd.SensorType.Rgb.name)

writer = rep.writers.get(rv + "ROS2PublishImage")

writer.initialize(
    frameId="camera",
    nodeNamespace="",
    queueSize=1,
    topicName="/camera/rgb"
)

writer.attach([render_product])

rv = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(
    sd.SensorType.DistanceToImagePlane.name)

writer = rep.writers.get(rv + "ROS2PublishImage")

writer.initialize(
    frameId="camera",
    nodeNamespace="",
    queueSize=1,
    topicName="/camera/depth"
)

writer.attach([render_product])

# ────────Setup IMU sensor ───────────────

imu_prim_path = "/World/Husky/imu_0_link/imu"

from isaacsim.sensors.physics import IMUSensor

sensor = IMUSensor(
    prim_path=imu_prim_path,
    name="imu",
    frequency=60,
    translation=np.array([0, 0, 0]),
    orientation=np.array([1, 0, 0, 0]),
    linear_acceleration_filter_size = 10,
    angular_velocity_filter_size = 10,
    orientation_filter_size = 10,
)


# ── Run ─────────────────────────────────────────────────────────────────────
teleop = HuskyKeyboardTeleop(stage)
print("Teleop: I forward, K reverse, J left, L right, X stop.")
print("ROS 2:  ros2 topic pub /cmd_vel geometry_msgs/msg/Twist ...")
timeline = omni.timeline.get_timeline_interface()
timeline.play()



while simulation_app.is_running():
    teleop.update()
    simulation_app.update()
   

teleop.close()
timeline.stop()
simulation_app.close()
