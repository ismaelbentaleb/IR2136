#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import csv
import time
import math
import re
import json
from typing import Optional, Tuple

import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
from pymavlink import mavutil


# =========================
# CONFIG (Flight + Photos)
# =========================
CONNECTION = "udp:127.0.0.1:14550"

ALTITUDE_M = 10.0                 # takeoff altitude
FORWARD_SPEED_MPS = 2.0           # forward speed while cruising
PHOTO_EVERY_M = 10.0              # take photo each X meters traveled
SETPOINT_HZ = 20.0                # send setpoints at 20Hz
TAKEOFF_WAIT_S = 8.0              # time to stabilize after takeoff

# Camera topic (Gazebo -> ROS)
CAMERA_TOPIC = "/camera/image_raw"

# Output folder for raw photos (incoming)
OUTPUT_DIR = os.path.expanduser("~/Documentos/GitHub/IR2136/data/incoming")

# Baseline folder (clean road images)
BASELINE_DIR = os.path.expanduser("~/Documentos/GitHub/IR2136/src/uav_inspection/data/baseline")

# Results folder (annotated + incidents)
RESULTS_DIR = os.path.expanduser("~/Documentos/GitHub/IR2136/src/uav_inspection/data/results")
ANNOTATED_DIR = os.path.join(RESULTS_DIR, "annotated")
INCIDENTS_DIR = os.path.join(RESULTS_DIR, "incidents")

# Detector parameters (tune if needed)
DIFF_THR = 35          # pixel diff threshold [0..255]
SCORE_THR = 3.0        # % of changed pixels inside road mask to consider incident
MIN_GAP_M = 12.0       # do not spam incidents closer than this in local XY meters

# Road ROI polygon (relative coords) tuned for your baseline images
# (x_rel, y_rel), y grows downward
ROAD_POLY_REL = [
    (0.10, 0.98),  # bottom-left
    (0.90, 0.98),  # bottom-right
    (0.65, 0.08),  # top-right
    (0.35, 0.08),  # top-left
]


# =========================
# Helpers
# =========================
def dist2d(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def polygon_mask(h: int, w: int, poly_rel):
    pts = np.array([(int(x * w), int(y * h)) for x, y in poly_rel], dtype=np.int32)
    mask = np.zeros((h, w), dtype=np.uint8)
    cv2.fillPoly(mask, [pts], 255)
    return mask, pts


def ecc_align_translation(base_gray, curr_gray, mask_u8=None, max_iters=50, eps=1e-4):
    """
    Align base -> curr using ECC translation only.
    Returns aligned_base_gray.
    """
    warp = np.eye(2, 3, dtype=np.float32)  # translation
    criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, max_iters, eps)
    try:
        cv2.findTransformECC(
            curr_gray, base_gray, warp,
            motionType=cv2.MOTION_TRANSLATION,
            criteria=criteria,
            inputMask=mask_u8
        )
        aligned = cv2.warpAffine(
            base_gray, warp, (curr_gray.shape[1], curr_gray.shape[0]),
            flags=cv2.INTER_LINEAR + cv2.WARP_INVERSE_MAP,
            borderMode=cv2.BORDER_REPLICATE
        )
        return aligned
    except cv2.error:
        # If ECC fails, skip alignment
        return base_gray


def road_change_score(curr_bgr, base_bgr, diff_thr=35, score_thr=3.0):
    """
    Compute change score inside a road trapezoid ROI.
    Returns: (is_incident, score_percent, overlay_bgr)
    """
    h, w = curr_bgr.shape[:2]
    mask, pts = polygon_mask(h, w, ROAD_POLY_REL)

    curr_g = cv2.cvtColor(curr_bgr, cv2.COLOR_BGR2GRAY)
    base_g = cv2.cvtColor(base_bgr, cv2.COLOR_BGR2GRAY)

    curr_g = cv2.GaussianBlur(curr_g, (5, 5), 0)
    base_g = cv2.GaussianBlur(base_g, (5, 5), 0)

    # Align baseline to current (small shifts)
    base_aligned = ecc_align_translation(base_g, curr_g, mask_u8=mask)

    diff = cv2.absdiff(curr_g, base_aligned)
    _, changed = cv2.threshold(diff, diff_thr, 255, cv2.THRESH_BINARY)

    changed_road = cv2.bitwise_and(changed, changed, mask=mask)
    road_area = int((mask > 0).sum())
    changed_area = int((changed_road > 0).sum())

    score = (changed_area / max(1, road_area)) * 100.0
    is_incident = score >= score_thr

    # Build overlay
    overlay = curr_bgr.copy()
    cv2.polylines(overlay, [pts], True, (0, 255, 255), 2)

    # Paint changed pixels (only inside road) in red
    red = np.zeros_like(overlay)
    red[:, :, 2] = 255
    alpha = 0.35
    idx = (changed_road > 0)
    overlay[idx] = (alpha * red[idx] + (1 - alpha) * overlay[idx]).astype(np.uint8)

    cv2.putText(
        overlay,
        f"score={score:.2f}%  thr={score_thr:.2f}%  diff_thr={diff_thr}",
        (15, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255, 255, 255),
        2
    )

    return is_incident, float(score), overlay


# =========================
# Main Node
# =========================
class MVSPhotoMission(Node):
    """
    pymavlink for flight + ROS camera subscription.
    Saves photos every X meters and runs real-time anomaly detection vs baseline.
    Publishes:
      - /incidents/json (std_msgs/String with JSON payload)
      - /incidents/image (sensor_msgs/Image overlay for rqt_image_view)
    """

    def __init__(self):
        super().__init__("mvs_straight_photo")

        # --- folders ---
        os.makedirs(OUTPUT_DIR, exist_ok=True)
        os.makedirs(ANNOTATED_DIR, exist_ok=True)
        os.makedirs(INCIDENTS_DIR, exist_ok=True)

        # --- output CSV ---
        self.csv_path = os.path.join(OUTPUT_DIR, "log.csv")
        self.csv_file = open(self.csv_path, "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        # include local_x/y/z so you can visualize later if you want
        self.csv_writer.writerow([
            "timestamp", "latitude", "longitude", "altitude",
            "local_x", "local_y", "local_z",
            "photo_file"
        ])

        # --- ROS camera ---
        self.bridge = CvBridge()
        self.last_image_msg: Optional[Image] = None
        self.create_subscription(Image, CAMERA_TOPIC, self.image_cb, 10)

        # --- ROS pubs for incidents ---
        self.incident_pub = self.create_publisher(String, "/incidents/json", 10)
        self.incident_img_pub = self.create_publisher(Image, "/incidents/image", 10)

        # --- detector settings ---
        self.diff_thr = DIFF_THR
        self.score_thr = SCORE_THR
        self.min_gap_m = MIN_GAP_M
        self.last_incident_xy: Optional[Tuple[float, float]] = None

        # --- baseline map ---
        self.baseline_map = self._build_baseline_map(BASELINE_DIR)
        if not self.baseline_map:
            self.get_logger().warn(f"No baseline images found in: {BASELINE_DIR}")

        # --- mavlink connect ---
        self.m = mavutil.mavlink_connection(CONNECTION)
        self.get_logger().info("Connecting to SITL (MAVLink)...")
        self.m.wait_heartbeat()
        self.get_logger().info("Heartbeat received ✅")

        # Ask for GPS data stream (best effort)
        self.m.mav.request_data_stream_send(
            self.m.target_system,
            self.m.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS,
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

        # Control
        self.cruise_enabled = False
        self.last_local: Optional[Tuple[float, float, float]] = None

        # Warmup setpoints
        self._warmup_sent = 0

    # =========================
    # ROS callbacks
    # =========================
    def image_cb(self, msg: Image):
        self.last_image_msg = msg

    # =========================
    # Baseline helpers
    # =========================
    def _extract_idx(self, filename: str) -> int:
        m = re.search(r"photo_(\d+)_", filename)
        return int(m.group(1)) if m else -1

    def _build_baseline_map(self, folder: str):
        m = {}
        if not os.path.isdir(folder):
            return m
        for fn in os.listdir(folder):
            if fn.endswith(".png") and "photo_" in fn:
                idx = self._extract_idx(fn)
                if idx >= 0:
                    m[idx] = os.path.join(folder, fn)
        return m

    def _publish_incident(self, payload: dict):
        msg = String()
        msg.data = json.dumps(payload)
        self.incident_pub.publish(msg)

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
        msg = self.m.recv_match(type="GPS_RAW_INT", blocking=False)
        if msg:
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            alt = msg.alt / 1000.0
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
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            type_mask,
            0.0, 0.0, 0.0,
            vx, vy, vz,
            0.0, 0.0, 0.0,
            0.0, 0.0
        )

    # =========================
    # Photo saving + Real-time detection
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

        # filter crazy jumps
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

        cv_img = self.bridge.imgmsg_to_cv2(self.last_image_msg, desired_encoding="bgr8")

        self.photo_idx += 1
        ts = time.time()
        fname = f"photo_{self.photo_idx:04d}_{int(ts)}.png"
        fpath = os.path.join(OUTPUT_DIR, fname)

        # GPS
        gps = self.poll_gps_position()
        if gps:
            lat, lon, alt = gps
        else:
            lat, lon, alt = 0.0, 0.0, 0.0

        # Save raw photo
        cv2.imwrite(fpath, cv_img)

        # Log CSV (with local pos too)
        self.csv_writer.writerow([
            f"{ts:.3f}",
            f"{lat:.7f}", f"{lon:.7f}", f"{alt:.2f}",
            f"{x:.3f}", f"{y:.3f}", f"{z:.3f}",
            fname
        ])
        self.csv_file.flush()

        self.get_logger().info(f"📸 Saved {fname} (every {PHOTO_EVERY_M}m)")

        # --- REAL-TIME DETECTION ---
        base_path = self.baseline_map.get(self.photo_idx, None)
        if not base_path or not os.path.exists(base_path):
            return

        base_img = cv2.imread(base_path)
        if base_img is None:
            return

        # Resize baseline to match current (safety)
        if base_img.shape[:2] != cv_img.shape[:2]:
            base_img = cv2.resize(base_img, (cv_img.shape[1], cv_img.shape[0]))

        is_inc, score, overlay = road_change_score(
            cv_img, base_img,
            diff_thr=self.diff_thr,
            score_thr=self.score_thr
        )

        # Save annotated always (debug + demo)
        ann_path = os.path.join(ANNOTATED_DIR, fname)
        cv2.imwrite(ann_path, overlay)

        # Anti-spam by distance
        allow = True
        if is_inc and self.last_incident_xy is not None:
            d = dist2d((x, y), self.last_incident_xy)
            if d < self.min_gap_m:
                allow = False

        if is_inc and allow:
            self.last_incident_xy = (x, y)

            inc_path = os.path.join(INCIDENTS_DIR, fname)
            cv2.imwrite(inc_path, overlay)

            payload = {
                "timestamp": ts,
                "photo_file": fname,
                "baseline_file": os.path.basename(base_path),
                "score_percent": float(score),
                "gps": {"lat": float(lat), "lon": float(lon), "alt": float(alt)},
                "local": {"x": float(x), "y": float(y), "z": float(z)},
                "snapshot_path": inc_path
            }

            # Publish JSON
            self._publish_incident(payload)

            # Publish overlay image to ROS
            img_msg = self.bridge.cv2_to_imgmsg(overlay, encoding="bgr8")
            self.incident_img_pub.publish(img_msg)

            self.get_logger().warn(f"🚨 INCIDENT DETECTED score={score:.2f}% -> {inc_path}")

    # =========================
    # Timers / State machine
    # =========================
    def setpoint_loop(self):
        self.poll_local_position()

        if self.phase in ("INIT", "WARMUP"):
            self.send_body_velocity_forward(0.0, 0.0, 0.0)
            return

        if self.cruise_enabled:
            self.send_body_velocity_forward(FORWARD_SPEED_MPS, 0.0, 0.0)

    def logic_loop(self):
        self.poll_local_position()

        if self.phase == "INIT":
            self.phase = "WARMUP"
            self.get_logger().info("Warmup: streaming neutral setpoints...")
            return

        if self.phase == "WARMUP":
            self._warmup_sent += 1
            if self._warmup_sent >= int(SETPOINT_HZ * 2.0):  # ~2 seconds
                self.set_mode("GUIDED")
                self.arm()
                self.takeoff(ALTITUDE_M)
                self._takeoff_t0 = time.time()
                self.phase = "TAKEOFF_WAIT"
            return

        if self.phase == "TAKEOFF_WAIT":
            if time.time() - self._takeoff_t0 >= TAKEOFF_WAIT_S:
                self.get_logger().info("Starting straight cruise + photo capture + real-time detection ✅")
                self.cruise_enabled = True
                self.phase = "CRUISE"
            return

        if self.phase == "CRUISE":
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
