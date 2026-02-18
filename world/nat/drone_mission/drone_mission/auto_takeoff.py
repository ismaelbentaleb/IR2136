#!/usr/bin/env python3
import rclpy
from std_msgs.msg import Float32
from geometry_msgs.msg import Point
from pymavlink import mavutil
import time


connection = mavutil.mavlink_connection('udp:127.0.0.1:14550')
print("Conectando a SITL...")
connection.wait_heartbeat()
print(f"Heartbeat recibido de sistema {connection.target_system}")

# Altitud fija
FIXED_ALTITUDE = 10


def get_current_location():
    msg = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    if msg:
        lat = msg.lat / 1e7   # convertir de int a grados
        lon = msg.lon / 1e7
        alt = msg.relative_alt / 1000  # mm a metros
        return lat, lon, alt
    return None, None, None


def set_mode(mode):
    if mode not in connection.mode_mapping():
        print(f"Modo {mode} desconocido!")
        return
    connection.set_mode(mode)
    print(f"Modo set a {mode}")

def arm_vehicle():
    print('Intentando ARMAR (reintentando hasta éxito)...')

    while True:
        #Enviar orden de armado
        connection.mav.command_long_send(
            connection.target_system,
            connection.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,  # 1 = ARMAR
            0,0,0,0,0,0
        )

        #Esperar 1 segundo a ver si el dron responde con heartbeat
        msg = connection.recv_match(type='HEARTBEAT', blocking=True, timeout=1)

        # Comprobar si ya está armado
        if msg and (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            print('Dron ARMADO y listo.')
            break
        else:
            print('... esperando confirmación, reenviando orden ...')

def takeoff():
    print(f"Intentando despegar a {FIXED_ALTITUDE} m...")

    while True:
        connection.mav.command_long_send(
            connection.target_system,
            connection.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0, 0, 0, 0,
            0,0,
            10
        )

        ack = connection.recv_match(
            type='COMMAND_ACK',
            blocking=True,
            timeout=2
        )

        if ack and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            print("TAKEOFF ACEPTADO 🚀")
            return
        else:
            print("TAKEOFF rechazado, reintentando...")
            time.sleep(2)


def go_to_position(lat, lon):
    print(f"Moviéndose a: lat={lat}, lon={lon}, alt={FIXED_ALTITUDE}")
    connection.mav.set_position_target_global_int_send(
        0,
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        0b110111111000,  # solo posición
        int(lat * 1e7),
        int(lon * 1e7),
        FIXED_ALTITUDE,
        0,0,0,
        0,0,0,
        0,0
    )

def land_vehicle():
    print("Aterrizando vehículo...")
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0,
        0,0,0,0,0,0,0
    )
    ack = connection.recv_match(type='COMMAND_ACK', blocking=True)
    print(f"ACK land: {ack}")


def battery_callback(msg: Float32):
    print(f"Batería: {msg.data}%")
    if msg.data <= 20:
        print("BATERÍA BAJA → ATERRIZANDO")
        land_vehicle()

def position_callback(msg: Point):
    print(f"Nueva posición objetivo recibida: lat={msg.x}, lon={msg.y}")
    go_to_position(msg.x, msg.y)  # Altitud fija 10 m


def main():
    rclpy.init()
    node = rclpy.create_node('auto_takeoff')

    # Suscripciones
    node.create_subscription(Point, 'target_position', position_callback, 10)

    # -------- MISION --------
    connection.set_mode('GUIDED')
    time.sleep(5)  # esperar a que se cambie el modo

    arm_vehicle()


    takeoff()
    time.sleep(10)  # esperar ascenso

    print(f"Nodo mission listo para recibir posiciones en /target_position a altitud {FIXED_ALTITUDE} m")
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

