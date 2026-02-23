#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
from typing import Optional, Tuple

from pymavlink import mavutil


class MavlinkFlight:
    """
    Thin helper around pymavlink.
    Keeps logic identical to your original methods, but moves them out of the ROS node.
    """

    def __init__(self, connection_str: str, logger):
        self.connection_str = connection_str
        self.logger = logger
        self.m = mavutil.mavlink_connection(connection_str)
        self.last_local: Optional[Tuple[float, float, float]] = None

    def connect_and_wait_heartbeat(self):
        self.logger.info("Connecting to SITL (MAVLink)...")
        self.m.wait_heartbeat()
        self.logger.info("Heartbeat received ✅")

        # Ask for GPS data stream (best effort)
        self.m.mav.request_data_stream_send(
            self.m.target_system,
            self.m.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS,
            10, 1
        )

    # =========================
    # MAVLink basic actions
    # =========================
    def set_mode(self, mode: str):
        self.logger.info(f"Setting mode: {mode}...")
        self.m.set_mode(mode)
        while True:
            hb = self.m.recv_match(type="HEARTBEAT", blocking=True)
            if hb and mavutil.mode_string_v10(hb) == mode:
                self.logger.info(f"Mode {mode} ✅")
                return

    def arm(self):
        self.logger.info("Arming...")
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
                self.logger.info("ARMED ✅")
                return

    def takeoff(self, alt_m: float):
        self.logger.info(f"Taking off to {alt_m:.1f} m...")
        self.m.mav.command_long_send(
            self.m.target_system,
            self.m.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0, 0, 0, 0,
            0, 0,
            alt_m
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

    def poll_gps_position(self) -> Optional[Tuple[float, float, float]]:
        """
        GPS_RAW_INT provides lat, lon in 1e7 degrees, alt in mm.
        """
        msg = self.m.recv_match(type="GLOBAL_POSITION_INT", blocking=False)
        if msg:
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            alt = msg.relative_alt / 1000.0
            return lat, lon, alt
        return None

    # =========================
    # MAVLink setpoint: velocity in BODY frame
    # =========================
    def send_body_velocity_forward(self, vx: float, vy: float, vz: float):
        """
        Velocity setpoint in BODY_NED:
          x forward, y right, z down.
        """
        type_mask = (0b111 << 0) | (0b111 << 10) | (0b11 << 8)
        now_ms = int(time.time() * 1000) & 0xFFFFFFFF
        self.m.mav.set_position_target_local_ned_send(
            now_ms,
            self.m.target_system,
            self.m.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            type_mask,
            0.0, 0.0, 0.0,
            vx, vy, vz,
            0.0, 0.0, 0.0,
            0.0, 0.0
        )
