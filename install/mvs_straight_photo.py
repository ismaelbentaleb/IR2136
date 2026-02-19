#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import csv
import time
import math
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from cv_bridge import CvBridge
import cv2

from pymavlink import mavutil


# =========================
# CONFIG
# =========================
CONNECTION = "udp:127.0.0.1:14550"

ALTITUDE_M = 10.0                 # takeoff altitude
FORWARD_SPEED_MPS = 2.0           # forward speed while cruising
PHOTO_EVERY_M = 10.0              # take photo each X meters traveled
SETPOINT_HZ = 20.0                # send setpoints at 20Hz
TAKEOFF_WAIT_S = 8.0              # time to stabilize after takeoff

# Camera topic (Gazebo -> ROS)
CAMERA_TOPIC = "/camera/image_raw"

# Output folder for photos
OUTPUT_DIR = os.path.expanduser("~/uav_photos")


# =========================
# MAVLink helpers
# =========================
def dist2d(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


class MVSPhotoMission(Node):
    """
    Single-file simplest functional approach:
    - pymavlink for flight
    - ROS subscription for camera images
    - save image every 10m traveled
    """

    def __init__(self):
        super().__init__("mvs_straight_photo")

        # --- output setup ---
        os.makedirs(OUTPUT_DIR, exist_ok=True)
        self.csv_path = os.path.join(OUTPUT_DIR, "log.csv")
        self.csv_file = open(self.csv_path, "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(["timestamp", "local_x", "local_y", "local_z", "photo_file"])

        # --- camera ---
        self.bridge = CvBridge()
        self.last_image_msg: Optional[Image] = None
        self.create_subscription(Image, CAMERA_TOPIC, self.image_cb, 10)

        # --- mavlink connect ---
        self.m = mavutil.mavlink_connection(CONNECTION)
        self.get_logger().info("Connecting to SITL (MAVLink)...")
        self.m.wait_heartbeat()
        self.get_logger().info("Heartbeat received ✅")

        # Ask for local position stream
        self.m.mav.request_data_stream_send(
            self.m.target_system,
            self.m.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_POSITION,
            10, 1
        )

        # Travel distance tracking
        self.last_xy: Optional[Tuple[float, float]] = None
        self.dist_accum = 0.0
        self.photo_idx = 0

        # Mission state machine
        self.phase = "INIT"

        # Timers
        self.setpoint_timer = self.create_timer(1.0 / SETPOINT_HZ, self.setpoint_loop)
        self.logic_timer = self.create_timer(0.2, self.logic_loop)

        # Internal target: keep moving forward in BODY frame after yaw
        self.cruise_enabled = False
        self.last_local: Optional[Tuple[float, float, float]] = None

        # Warmup: send a few setpoints before mode switch (helps some setups)
        self._warmup_sent = 0

    # =========================
    # ROS callbacks
    # =========================
    def image_cb(self, msg: Image):
        self.last_image_msg = msg

    # =========================
    # MAVLink basic actions
    # =========================
    def set_mode(self, mode: str):
        self.get_logger().info(f"Setting mode: {mode}...")
        self.m.set_mode(mode)
        while True:
            hb = self.m.recv_match(type="HEARTBEAT", blocking=True)
            if hb and mavutil.mode_string_v10(hb) == mode:
                self.get_logger().info(f"Mode {mode} ✅")
                return

    def arm(self):
        self.get_logger().info("Arming...")
        while True:
            self.m.mav.command_long_send(
                self.m.target_system,
                self.m.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                1, 0, 0, 0, 0, 0, 0
            )
            hb = self.m.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
            if hb and (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                self.get_logger().info("ARMED ✅")
                return

    def takeoff(self, alt_m: float):
        self.get_logger().info(f"Taking off to {alt_m:.1f} m...")
        self.m.mav.command_long_send(
            self.m.target_system,
            self.m.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0, 0, 0, 0,
            0, 0,
            alt_m
        )

    def yaw_plus_90(self):
        """
        Rotate +90° (CCW in the usual ENU math sense).
        MAV_CMD_CONDITION_YAW param3 is direction:
          -1 = CCW, 1 = CW
        If it turns the wrong way in your setup, flip -1 <-> 1.
        """
        self.get_logger().info("Rotating +90° (CCW)...")
        self.m.mav.command_long_send(
            self.m.target_system,
            self.m.target_component,
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
            0,
            90.0,   # angle
            30.0,   # yaw speed deg/s
            -1.0,   # direction: CCW
            1.0,    # relative
            0, 0, 0
        )

    # =========================
    # MAVLink telemetry
    # =========================
    def poll_local_position(self) -> Optional[Tuple[float, float, float]]:
        """
        LOCAL_POSITION_NED gives (x,y,z) in meters, z is DOWN (positive down).
        """
        msg = self.m.recv_match(type="LOCAL_POSITION_NED", blocking=False)
        if msg:
            self.last_local = (float(msg.x), float(msg.y), float(msg.z))
        return self.last_local

    # =========================
    # MAVLink setpoint: velocity in BODY frame
    # =========================
    def send_body_velocity_forward(self, vx: float, vy: float, vz: float):
        """
        Send velocity setpoint in BODY_NED frame:
          x = forward, y = right, z = down
        We'll ignore position and yaw here; we just stream velocities.
        """
        # type_mask: ignore position (bits 0,1,2), ignore accel (10,11,12), ignore yaw/yaw_rate (8,9)
        # keep velocity (bits 3,4,5) active -> do NOT set them
        type_mask = (0b111 << 0) | (0b111 << 10) | (0b11 << 8)

        now_ms = int(time.time() * 1000) & 0xFFFFFFFF
        self.m.mav.set_position_target_local_ned_send(
            now_ms,
            self.m.target_system,
            self.m.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            type_mask,
            0.0, 0.0, 0.0,      # x,y,z ignored
            vx, vy, vz,          # velocities
            0.0, 0.0, 0.0,       # accel ignored
            0.0, 0.0             # yaw ignored
        )

    # =========================
    # Photo saving
    # =========================
    def maybe_save_photo_every_distance(self):
        if self.last_local is None:
            return

        x, y, z = self.last_local
        curr_xy = (x, y)

        if self.last_xy is None:
            self.last_xy = curr_xy
            return

        step = dist2d(curr_xy, self.last_xy)

        # filter crazy jumps (telemetry glitches)
        if 0.0 <= step < 5.0:
            self.dist_accum += step

        self.last_xy = curr_xy

        while self.dist_accum >= PHOTO_EVERY_M:
            self.dist_accum -= PHOTO_EVERY_M
            self.save_photo(x, y, z)

    def save_photo(self, x: float, y: float, z: float):
        if self.last_image_msg is None:
            self.get_logger().warn("No camera image received yet, skipping photo.")
            return

        # Convert to OpenCV image
        cv_img = self.bridge.imgmsg_to_cv2(self.last_image_msg, desired_encoding="bgr8")

        self.photo_idx += 1
        ts = time.time()
        fname = f"photo_{self.photo_idx:04d}_{int(ts)}.png"
        fpath = os.path.join(OUTPUT_DIR, fname)

        cv2.imwrite(fpath, cv_img)
        self.csv_writer.writerow([f"{ts:.3f}", f"{x:.3f}", f"{y:.3f}", f"{z:.3f}", fname])
        self.csv_file.flush()

        self.get_logger().info(f"📸 Saved {fname} (every {PHOTO_EVERY_M}m)")

    # =========================
    # Timers
    # =========================
    def setpoint_loop(self):
        # Always poll telemetry
        self.poll_local_position()

        # Warmup setpoints before starting (some stacks like it)
        if self.phase in ("INIT", "WARMUP"):
            self.send_body_velocity_forward(0.0, 0.0, 0.0)
            return

        if self.cruise_enabled:
            # Move forward in body frame
            self.send_body_velocity_forward(FORWARD_SPEED_MPS, 0.0, 0.0)

    def logic_loop(self):
        # Keep position updated
        self.poll_local_position()

        if self.phase == "INIT":
            self.phase = "WARMUP"
            self.get_logger().info("Warmup: streaming neutral setpoints...")
            return

        if self.phase == "WARMUP":
            self._warmup_sent += 1
            if self._warmup_sent >= int(SETPOINT_HZ * 2.0):  # ~2 seconds
                # Start mission
                self.set_mode("GUIDED")
                self.arm()
                self.takeoff(ALTITUDE_M)
                self._takeoff_t0 = time.time()
                self.phase = "TAKEOFF_WAIT"
            return

        if self.phase == "TAKEOFF_WAIT":
            if time.time() - self._takeoff_t0 >= TAKEOFF_WAIT_S:
                self.yaw_plus_90()
                self._yaw_t0 = time.time()
                self.phase = "YAW_WAIT"
            return

        if self.phase == "YAW_WAIT":
            # simple wait (could be improved by reading attitude yaw)
            if time.time() - self._yaw_t0 >= 3.0:
                self.get_logger().info("Starting straight cruise + photo capture ✅")
                self.cruise_enabled = True
                self.phase = "CRUISE"
            return

        if self.phase == "CRUISE":
            # Save photos every distance while cruising
            self.maybe_save_photo_every_distance()
            return

    def destroy_node(self):
        try:
            self.csv_file.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = MVSPhotoMission()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
