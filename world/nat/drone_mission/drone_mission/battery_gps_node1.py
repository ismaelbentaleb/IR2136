#!/usr/bin/env python3

import rclpy
from std_msgs.msg import Float32
from pymavlink import mavutil


connection = mavutil.mavlink_connection('udp:127.0.0.1:14550')
print("Connected to SITL")


def main():
    rclpy.init()
    node = rclpy.create_node('battery_gps_node')

    battery_pub = node.create_publisher(
        Float32,
        'battery_status',
        10
    )

    def publish_battery():
        msg = connection.recv_match(
            type='BATTERY_STATUS',
            blocking=False
        )

        if msg:
            battery_msg = Float32()
            battery_msg.data = float(msg.battery_remaining)
            battery_pub.publish(battery_msg)
            node.get_logger().info(
                f'Battery: {battery_msg.data}%'
            )

    node.create_timer(5.0, publish_battery)
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

