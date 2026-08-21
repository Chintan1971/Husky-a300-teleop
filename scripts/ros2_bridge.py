#!/usr/bin/env python3
"""ROS 2 Bridge OmniGraph setup module for Clearpath Husky A300 in Isaac Sim.

Creates Action Graphs for:
  Publishers:  /clock, /tf, /joint_states, /odom
  Subscriber:  /cmd_vel -> DifferentialController -> ArticulationController
"""

import omni.graph.core as og
from pxr import Sdf

ROBOT_PRIM_PATH = "/World/Husky"
CHASSIS_PRIM_PATH = "/World/Husky/base_link"
WHEEL_RADIUS = 0.1651          # metres
WHEEL_DISTANCE = 0.984         # effective track (0.562 m × 1.75 skid factor)
WHEEL_JOINT_NAMES = [
    "front_left_wheel_joint",
    "rear_left_wheel_joint",
    "front_right_wheel_joint",
    "rear_right_wheel_joint",
]

keys = og.Controller.Keys


def _set_prim_target(stage, prim_path: str, attr_name: str, targets: list[str]):
    """Set a USD relationship attribute on an OmniGraph node prim.

    targetPrim / targetPrims / chassisPrim are USD relationships, not regular
    OG attributes, so they cannot be set via og.Controller SET_VALUES.
    """
    prim = stage.GetPrimAtPath(Sdf.Path(prim_path))
    if not prim.IsValid():
        raise RuntimeError(f"Prim not found: {prim_path}")
    rel = prim.GetRelationship(attr_name)
    if not rel:
        rel = prim.CreateRelationship(attr_name, custom=False)
    rel.SetTargets([Sdf.Path(t) for t in targets])


def create_ros2_bridge_graphs(stage):
    """Build OmniGraph action graphs for the ROS 2 bridge.

    Graph 1 – Publishers:  /clock, /tf, /joint_states, /odom
    Graph 2 – Subscriber:  /cmd_vel → DifferentialController → ArticulationController
    """

    # ── Graph 1: Publishers ─────────────────────────────────────────────────
    og.Controller.edit(
        {"graph_path": "/World/ROS2Publishers", "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnPlaybackTick"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                # Clock
                ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
                # TF
                ("PublishTF", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
                # Joint States
                ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
                # Odometry
                ("ComputeOdom", "isaacsim.core.nodes.IsaacComputeOdometry"),
                ("PublishOdom", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
                # IMU
                ("ReadIMU", "isaacsim.sensors.physics.IsaacReadIMU"),
                ("PublishIMU", "isaacsim.ros2.bridge.ROS2PublishImu"),
            ],
            keys.SET_VALUES: [
                # ROS 2 context (domain 0)
                ("Context.inputs:domain_id", 0),
                # Clock
                ("PublishClock.inputs:topicName", "/clock"),
                # TF
                ("PublishTF.inputs:topicName", "/tf"),
                # Joint States
                ("PublishJointState.inputs:topicName", "/joint_states"),
                # Odometry
                ("PublishOdom.inputs:topicName", "/odom"),
                ("PublishOdom.inputs:odomFrameId", "odom"),
                ("PublishOdom.inputs:chassisFrameId", "base_link"),
                # IMU
                ("PublishIMU.inputs:topicName", "/imu"),
                ("PublishIMU.inputs:frameId", "imu"),
                ("PublishIMU.inputs:publishOrientation", True),
                ("PublishIMU.inputs:publishLinearAcceleration", True),
                ("PublishIMU.inputs:publishAngularVelocity", True),

            ],
            keys.CONNECT: [
                # Execution flow
                ("OnTick.outputs:tick", "PublishClock.inputs:execIn"),
                ("OnTick.outputs:tick", "PublishTF.inputs:execIn"),
                ("OnTick.outputs:tick", "PublishJointState.inputs:execIn"),
                ("OnTick.outputs:tick", "ComputeOdom.inputs:execIn"),
                ("OnTick.outputs:tick", "PublishOdom.inputs:execIn"),
                ("OnTick.outputs:tick", "ReadIMU.inputs:execIn"),
                ("OnTick.outputs:tick", "PublishIMU.inputs:execIn"),
                # ROS 2 context
                ("Context.outputs:context", "PublishClock.inputs:context"),
                ("Context.outputs:context", "PublishTF.inputs:context"),
                ("Context.outputs:context", "PublishJointState.inputs:context"),
                ("Context.outputs:context", "PublishOdom.inputs:context"),
                ("Context.outputs:context", "PublishIMU.inputs:context"),
                # Simulation time → clock, odom, joint_states, tf
                ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
                ("ReadSimTime.outputs:simulationTime", "PublishOdom.inputs:timeStamp"),
                ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
                ("ReadSimTime.outputs:simulationTime", "PublishTF.inputs:timeStamp"),
                ("ReadSimTime.outputs:simulationTime", "PublishIMU.inputs:timeStamp"),
                # Odometry data flow
                ("ComputeOdom.outputs:position", "PublishOdom.inputs:position"),
                ("ComputeOdom.outputs:orientation", "PublishOdom.inputs:orientation"),
                ("ComputeOdom.outputs:linearVelocity", "PublishOdom.inputs:linearVelocity"),
                ("ComputeOdom.outputs:angularVelocity", "PublishOdom.inputs:angularVelocity"),
                # IMU data flow
                ("ReadIMU.outputs:linAcc", "PublishIMU.inputs:linearAcceleration"),
                ("ReadIMU.outputs:angVel", "PublishIMU.inputs:angularVelocity"),
                ("ReadIMU.outputs:orientation", "PublishIMU.inputs:orientation"),
            ],
        },
    )

    # Set USD relationship targets (these are not regular OG attributes).
    _set_prim_target(
        stage, "/World/ROS2Publishers/PublishTF", "inputs:targetPrims", [CHASSIS_PRIM_PATH]
    )
    _set_prim_target(
        stage, "/World/ROS2Publishers/PublishJointState", "inputs:targetPrim", [CHASSIS_PRIM_PATH]
    )
    _set_prim_target(
        stage, "/World/ROS2Publishers/ComputeOdom", "inputs:chassisPrim", [CHASSIS_PRIM_PATH]
    )
    _set_prim_target(
        stage, "/World/ROS2Publishers/ReadIMU", "inputs:imuPrim",["/World/Husky/imu_0_link/imu"]
)
    print("[ROS2] Publisher graph created: /clock, /tf, /joint_states, /odom")

    # ── Graph 2: /cmd_vel subscriber → Differential Drive ───────────────────
    og.Controller.edit(
        {"graph_path": "/World/ROS2CmdVel", "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnPlaybackTick"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("SubscribeTwist", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
                ("BreakLinear", "omni.graph.nodes.BreakVector3"),
                ("BreakAngular", "omni.graph.nodes.BreakVector3"),
                ("DiffController", "isaacsim.robot.wheeled_robots.DifferentialController"),
                ("ArticulationCtrl", "isaacsim.core.nodes.IsaacArticulationController"),
            ],
            keys.SET_VALUES: [
                ("Context.inputs:domain_id", 0),
                ("SubscribeTwist.inputs:topicName", "/cmd_vel"),
                # Differential controller params
                ("DiffController.inputs:wheelRadius", WHEEL_RADIUS),
                ("DiffController.inputs:wheelDistance", WHEEL_DISTANCE),
                ("DiffController.inputs:maxWheelSpeed", 50.0),
                # Articulation controller – target the articulation root (base_link)
                ("ArticulationCtrl.inputs:robotPath", CHASSIS_PRIM_PATH),
                ("ArticulationCtrl.inputs:jointNames", WHEEL_JOINT_NAMES),
            ],
            keys.CONNECT: [
                # Execution flow
                ("OnTick.outputs:tick", "SubscribeTwist.inputs:execIn"),
                ("OnTick.outputs:tick", "DiffController.inputs:execIn"),
                ("OnTick.outputs:tick", "ArticulationCtrl.inputs:execIn"),
                # ROS 2 context
                ("Context.outputs:context", "SubscribeTwist.inputs:context"),
                # Extract linear X and angular Z from 3D vectors
                ("SubscribeTwist.outputs:linearVelocity", "BreakLinear.inputs:tuple"),
                ("SubscribeTwist.outputs:angularVelocity", "BreakAngular.inputs:tuple"),
                ("BreakLinear.outputs:x", "DiffController.inputs:linearVelocity"),
                ("BreakAngular.outputs:z", "DiffController.inputs:angularVelocity"),
                # Differential Controller → Articulation Controller
                ("DiffController.outputs:velocityCommand", "ArticulationCtrl.inputs:velocityCommand"),
            ],
        },
    )
    print("[ROS2] Subscriber graph created: /cmd_vel → differential drive")


    # ─── Graph 3: ─────────────────────────────────────────────────────────────
