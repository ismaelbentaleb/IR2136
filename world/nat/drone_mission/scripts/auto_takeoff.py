#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from pymavlink import mavutil
import time
import numpy as np
from collections import deque
import math


FIXED_ALTITUDE = 10
STD_THRESHOLD = 0.02
WINDOW_SIZE = 30
STABLE_TIME_REQUIRED = 2.0
MIN_HEIGHT_TO_CHECK = 2.0
MAX_SPEED = 1.0

class DroneMission(Node):

    def __init__(self):
        super().__init__('drone_mission')

        # MAVLINK
        self.connection = mavutil.mavlink_connection('udp:127.0.0.1:14550')
        print("Conectando a SITL...")
        self.connection.wait_heartbeat()
        print("Heartbeat recibido")

        # Ventana deslizante
        self.window = deque(maxlen=WINDOW_SIZE)
        self.stable_start_time = None
        self.landing_triggered = False

        # Suscripciones
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Point, 'target_position', self.position_callback, 10)

        # Misión inicial
        self.set_guided()
        self.arm_vehicle()
        self.takeoff()
        time.sleep(8)

        print("🚀 Buscando zona plana...")

    # ===============================
    # MAVLINK FUNCIONES
    # ===============================

    def set_guided(self):
        print("Cambiando a GUIDED...")
        self.connection.set_mode('GUIDED')
        while True:
            hb = self.connection.recv_match(type='HEARTBEAT', blocking=True)
            if mavutil.mode_string_v10(hb) == "GUIDED":
                break

    def arm_vehicle(self):
        print("Armando...")
        while True:
            self.connection.mav.command_long_send(
                self.connection.target_system,
                self.connection.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 1, 0,0,0,0,0,0
            )
            msg = self.connection.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
            if msg and (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                print("ARMADO")
                break

    def takeoff(self):
        print("Despegando...")
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0,0,0,0, 0,0, FIXED_ALTITUDE
        )

    def land_vehicle(self):
        print("🟢 ATERRIZANDO")
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0, 0,0,0,0,0,0,0
        )

    MAX_SPEED = 1.0   # m/s  🔥 baja velocidad
    
    def go_to_position(self, x, y):
    	self.connection.mav.set_position_target_local_ned_send(
    		0,
		self.connection.target_system,
		self.connection.target_component,
		mavutil.mavlink.MAV_FRAME_LOCAL_NED,
		0b110111000111,   # usamos posición + velocidad
		x,
		y,
		-FIXED_ALTITUDE,
		MAX_SPEED,  # vx
		MAX_SPEED,  # vy
		0,
		0,0,0,
		0
	)



    # ===============================
    # CALLBACKS
    # ===============================

    def position_callback(self, msg):
        self.go_to_position(msg.x, msg.y)

    def scan_callback(self, msg):

        if self.landing_triggered:
            return

        distance = msg.ranges[0]

        if math.isinf(distance):
            return

        # Solo analizar si estamos suficientemente altos
        if distance < MIN_HEIGHT_TO_CHECK:
            return

        self.window.append(distance)

        if len(self.window) < WINDOW_SIZE:
            return

        std = np.std(self.window)
        mean_height = np.mean(self.window)

        print(f"Altura: {mean_height:.2f} m | STD: {std:.4f}")

        if std < STD_THRESHOLD:

            if self.stable_start_time is None:
                self.stable_start_time = time.time()

            elif time.time() - self.stable_start_time > STABLE_TIME_REQUIRED:
                print("✅ ZONA PLANA DETECTADA")
                self.land_vehicle()
                self.landing_triggered = True

        else:
            self.stable_start_time = None


def main():
    rclpy.init()
    node = DroneMission()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

