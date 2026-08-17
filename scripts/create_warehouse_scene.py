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
