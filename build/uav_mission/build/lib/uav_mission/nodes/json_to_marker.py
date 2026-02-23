#!/usr/bin/env python3
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy

class JsonToMarker(Node):
    def __init__(self):
        super().__init__("json_to_marker")

        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.sub = self.create_subscription(String, "/incidents/json", self.cb, 10)
        self.pub = self.create_publisher(Marker, "/incidents/marker", latched_qos)

        self.marker = Marker()
        self.marker.header.frame_id = "map"
        self.marker.ns = "incidents"
        self.marker.id = 0
        self.marker.type = Marker.TEXT_VIEW_FACING
        self.marker.action = Marker.ADD
        self.marker.pose.position.x = 0.0
        self.marker.pose.position.y = 0.0
        self.marker.pose.position.z = 2.0
        self.marker.scale.z = 0.4
        self.marker.color.a = 1.0
        self.marker.color.r = 1.0
        self.marker.color.g = 1.0
        self.marker.color.b = 1.0

    def cb(self, msg: String):
        try:
            d = json.loads(msg.data)

            local = d.get("local", {})
            score = d.get("score_percent", 0.0)
            photo = d.get("photo_file", "")

            self.marker.header.stamp = self.get_clock().now().to_msg()

            self.marker.text = (
                f"INCIDENT\n"
                f"foto: {photo}\n"
                f"score: {score:.2f}%\n"
                f"x: {local.get('x', 0.0):.3f}\n"
                f"y: {local.get('y', 0.0):.3f}\n"
                f"z: {local.get('z', 0.0):.3f}"
            )

            self.pub.publish(self.marker)

        except Exception as e:
            self.get_logger().warn(f"JSON inválido: {e}")

def main():
    rclpy.init()
    n = JsonToMarker()
    rclpy.spin(n)
    n.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
