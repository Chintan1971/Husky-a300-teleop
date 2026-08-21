#!/usr/bin/env python3
"""Keyboard Teleoperation module for Clearpath Husky A300 in Isaac Sim.

Maps keyboard input (I/K/J/L/X) to the four Husky wheel-drive angular targets.
"""

from math import degrees
import carb
import omni.appwindow
from pxr import Sdf, UsdPhysics


class HuskyKeyboardTeleop:
    """Map keyboard input to the four Husky wheel-drive targets."""

    WHEEL_RADIUS_M = 0.1651
    EFFECTIVE_TRACK_M = 0.984  # 0.562 m physical track × 1.75 skid-steer factor
    LINEAR_SPEED_MPS = 1.0
    TURN_RATE_RADPS = 1.0

    FORWARD_KEY = carb.input.KeyboardInput.K
    REVERSE_KEY = carb.input.KeyboardInput.I
    LEFT_KEY = carb.input.KeyboardInput.J
    RIGHT_KEY = carb.input.KeyboardInput.L
    STOP_KEY = carb.input.KeyboardInput.X

    def __init__(self, stage):
        wheel_names = (
            "front_left_wheel_joint",
            "rear_left_wheel_joint",
            "front_right_wheel_joint",
            "rear_right_wheel_joint",
        )
        self._drives = {}
        for joint_name in wheel_names:
            joint_path = f"/World/Husky/joints/{joint_name}"
            prim = stage.GetPrimAtPath(Sdf.Path(joint_path))
            if not prim.IsValid():
                continue
            drive = UsdPhysics.DriveAPI.Get(prim, UsdPhysics.Tokens.angular)
            if drive.GetTargetVelocityAttr().IsValid():
                self._drives[joint_name] = drive

        missing = set(wheel_names) - self._drives.keys()
        if missing:
            raise RuntimeError(f"Could not find angular drives for: {sorted(missing)}")

        self._pressed = set()
        self._input = carb.input.acquire_input_interface()
        keyboard = omni.appwindow.get_default_app_window().get_keyboard()
        self._keyboard_subscription = self._input.subscribe_to_keyboard_events(
            keyboard, self._on_keyboard_event
        )
        self.update()  # explicitly command zero on startup

    def _on_keyboard_event(self, event, *args, **kwargs):
        controls = {
            self.FORWARD_KEY,
            self.REVERSE_KEY,
            self.LEFT_KEY,
            self.RIGHT_KEY,
            self.STOP_KEY,
        }
        if event.input not in controls:
            return True

        if event.input == self.STOP_KEY and event.type == carb.input.KeyboardEventType.KEY_PRESS:
            self._pressed.clear()
            self._set_wheel_speeds(0.0, 0.0)
            return True

        if event.type in (
            carb.input.KeyboardEventType.KEY_PRESS,
            carb.input.KeyboardEventType.KEY_REPEAT,
        ):
            self._pressed.add(event.input)
        elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
            self._pressed.discard(event.input)
        return True

    def update(self):
        linear = self.LINEAR_SPEED_MPS * (
            (self.FORWARD_KEY in self._pressed) - (self.REVERSE_KEY in self._pressed)
        )
        turn = self.TURN_RATE_RADPS * (
            (self.LEFT_KEY in self._pressed) - (self.RIGHT_KEY in self._pressed)
        )
        left_speed = (linear - turn * self.EFFECTIVE_TRACK_M / 2.0) / self.WHEEL_RADIUS_M
        right_speed = (linear + turn * self.EFFECTIVE_TRACK_M / 2.0) / self.WHEEL_RADIUS_M
        self._set_wheel_speeds(left_speed, right_speed)

    def _set_wheel_speeds(self, left_radps, right_radps):
        # USD angular-drive target velocities are authored in degrees/second.
        left_degps = degrees(left_radps)
        right_degps = degrees(right_radps)
        self._drives["front_left_wheel_joint"].GetTargetVelocityAttr().Set(left_degps)
        self._drives["rear_left_wheel_joint"].GetTargetVelocityAttr().Set(left_degps)
        self._drives["front_right_wheel_joint"].GetTargetVelocityAttr().Set(right_degps)
        self._drives["rear_right_wheel_joint"].GetTargetVelocityAttr().Set(right_degps)

    def close(self):
        self._input.unsubscribe_to_keyboard_events(self._keyboard_subscription)
