#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import csv
import time
import re
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2

from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy

from uav_inspection.vision_change_detector import road_change_score, polygon_mask, dist2d, ROAD_POLY_REL
from uav_inspection.mavlink_flight import MavlinkFlight
from uav_inspection.incidents_reporter import IncidentsReporter

# =========================
# CONFIG (Flight + Photos)
# =========================
CONNECTION = "udp:127.0.0.1:14550"

ALTITUDE_M = 10.0
FORWARD_SPEED_MPS = 2.0
PHOTO_EVERY_M = 40.0
SETPOINT_HZ = 20.0
TAKEOFF_WAIT_S = 8.0

CAMERA_TOPIC = "/camera/image_raw"

OUTPUT_DIR = os.path.expanduser("~/Documentos/GitHub/IR2136/data/incoming")
BASELINE_DIR = os.path.expanduser("~/Documentos/GitHub/IR2136/src/uav_inspection/data/baseline")
RESULTS_DIR = os.path.expanduser("~/Documentos/GitHub/IR2136/src/uav_inspection/data/results")
ANNOTATED_DIR = os.path.join(RESULTS_DIR, "annotated")
INCIDENTS_DIR = os.path.join(RESULTS_DIR, "incidents")

DIFF_THR = 35
SCORE_THR = 3.0
MIN_GAP_M = 12.0


class MVSPhotoMission(Node):
    """
    Same behavior as your original script:
    - pymavlink for flight + ROS camera subscription
    - Saves photos every X meters
    - Real-time anomaly detection vs baseline
    Publishes:
      - /incidents/json (std_msgs/String with JSON payload)
      - /incidents/image (sensor_msgs/Image overlay)
      - /annotated/image (overlay / annotated stream)
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
        self.csv_writer.writerow(["timestamp", "local_x", "local_y", "local_z", "photo_file"])

        # --- ROS camera ---
        self.bridge = CvBridge()
        self.last_image_msg: Optional[Image] = None
        self.create_subscription(Image, CAMERA_TOPIC, self.image_cb, 10)

        # --- QoS ---
        annotated_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        incident_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # --- pubs ---
        self.annotated_img_pub = self.create_publisher(Image, "/annotated/image", annotated_qos)
        self.incident_img_pub = self.create_publisher(Image, "/incidents/image", incident_qos)
        self.incident_pub = self.create_publisher(String, "/incidents/json", incident_qos)

        # --- reporter (same functionality, moved) ---
        self.reporter = IncidentsReporter(
            incidents_dir=INCIDENTS_DIR,
            incident_pub=self.incident_pub,
            logger=self.get_logger()
        )

        # --- detector settings ---
        self.diff_thr = DIFF_THR
        self.score_thr = SCORE_THR
        self.min_gap_m = MIN_GAP_M
        self.last_incident_xy: Optional[Tuple[float, float]] = None

        # --- baseline map ---
        self.baseline_map = self._build_baseline_map(BASELINE_DIR)
        if not self.baseline_map:
            self.get_logger().warn(f"No baseline images found in: {BASELINE_DIR}")

        # --- mavlink (same logic, moved) ---
        self.flight = MavlinkFlight(CONNECTION, logger=self.get_logger())
        self.flight.connect_and_wait_heartbeat()

        # Travel distance tracking
        self.last_xy: Optional[Tuple[float, float]] = None
        self.dist_accum = 0.0
        self.photo_idx = 0
        self.first_photo_taken = False

        # Mission state machine
        self.phase = "INIT"

        # Timers
        self.setpoint_timer = self.create_timer(1.0 / SETPOINT_HZ, self.setpoint_loop)
        self.logic_timer = self.create_timer(0.2, self.logic_loop)

        # Control
        self.cruise_enabled = False

        # Warmup setpoints
        self._warmup_sent = 0

        # landing
        self.landing_started = False

    # =========================
    # ROS callbacks
    # =========================
    def image_cb(self, msg: Image):
        self.last_image_msg = msg

    # =========================
    # Baseline helpers (same)
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

    # =========================
    # Photo saving + Real-time detection (same)
    # =========================
    def maybe_save_photo_every_distance(self):
        last_local = self.flight.last_local
        if last_local is None:
            return

        x, y, z = last_local
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

        # Save raw photo
        cv2.imwrite(fpath, cv_img)

        # Log CSV (with local pos too)
        self.csv_writer.writerow([
            f"{ts:.3f}",
            f"{x:.3f}", f"{y:.3f}", f"{z:.3f}",
            fname
        ])
        self.csv_file.flush()

        self.get_logger().info(f"📸 Saved {fname} (every {PHOTO_EVERY_M}m)")

        # ✅ Publicar annotated SIEMPRE (aunque no haya baseline) (same)
        overlay_for_pub = cv_img.copy()
        h, w = overlay_for_pub.shape[:2]
        _, pts = polygon_mask(h, w, ROAD_POLY_REL)
        cv2.polylines(overlay_for_pub, [pts], True, (0, 255, 255), 2)
        cv2.putText(
            overlay_for_pub,
            "ANNOTATED",
            (15, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2
        )

        ann_msg = self.bridge.cv2_to_imgmsg(overlay_for_pub, encoding="bgr8")
        if self.last_image_msg is not None:
            ann_msg.header = self.last_image_msg.header
        self.annotated_img_pub.publish(ann_msg)

        # 🚁 If 6th photo → initiate landing (same)
        if self.photo_idx >= 6 and not self.landing_started:
            self.get_logger().info("6th photo taken. Initiating landing...")
            self.cruise_enabled = False
            self.flight.set_mode("LAND")
            self.landing_started = True

        # --- REAL-TIME DETECTION (same) ---
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
            score_thr=self.score_thr,
            poly_rel=ROAD_POLY_REL
        )

        # Save annotated always (debug + demo)
        ann_path = os.path.join(ANNOTATED_DIR, fname)
        cv2.imwrite(ann_path, overlay)

        # ✅ Publicar SIEMPRE el overlay annotated
        ann_msg = self.bridge.cv2_to_imgmsg(overlay, encoding="bgr8")
        self.annotated_img_pub.publish(ann_msg)

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
                "local": {"x": float(x), "y": float(y), "z": float(z)},
                "snapshot_path": inc_path
            }

            self.reporter.append_incident_row(payload)
            self.reporter.publish_incident(payload)

            img_msg = self.bridge.cv2_to_imgmsg(overlay, encoding="bgr8")
            self.incident_img_pub.publish(img_msg)

            self.get_logger().warn(f"🚨 INCIDENT DETECTED score={score:.2f}% -> {inc_path}")

    # =========================
    # Timers / State machine (same)
    # =========================
    def setpoint_loop(self):
        self.flight.poll_local_position()

        if self.phase in ("INIT", "WARMUP"):
            self.flight.send_body_velocity_forward(0.0, 0.0, 0.0)
            return

        if self.cruise_enabled:
            self.flight.send_body_velocity_forward(0.0, -FORWARD_SPEED_MPS, 0.0)

    def logic_loop(self):
        self.flight.poll_local_position()

        if self.phase == "INIT":
            self.phase = "WARMUP"
            self.get_logger().info("Warmup: streaming neutral setpoints...")
            return

        if self.phase == "WARMUP":
            self._warmup_sent += 1
            if self._warmup_sent >= int(SETPOINT_HZ * 2.0):  # ~2 seconds
                self.flight.set_mode("GUIDED")
                self.flight.arm()
                self.flight.takeoff(ALTITUDE_M)
                self._takeoff_t0 = time.time()
                self.phase = "TAKEOFF_WAIT"
            return

        if self.phase == "TAKEOFF_WAIT":
            if time.time() - self._takeoff_t0 >= TAKEOFF_WAIT_S:
                self.get_logger().info("Starting straight cruise + photo capture + real-time detection ✅")
                self.cruise_enabled = True
                self.phase = "CRUISE"

                # 📸 Foto inicial (same)
                if not self.first_photo_taken and self.flight.last_local is not None:
                    x, y, z = self.flight.last_local
                    self.save_photo(x, y, z)
                    self.first_photo_taken = True

                    # Inicializar contador de distancia
                    self.last_xy = (x, y)
                    self.dist_accum = 0.0
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
