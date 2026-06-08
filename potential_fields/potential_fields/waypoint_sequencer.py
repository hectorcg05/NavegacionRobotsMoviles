#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from std_msgs.msg import Bool


class WaypointSequencer(Node):
    def __init__(self):
        super().__init__("waypoint_sequencer")
        self.get_logger().info("Secuenciador de waypoints iniciado")

        self.pub_point = self.create_publisher(Pose, "/next_point", 10)
        self.create_subscription(Bool, "/arrived", self.cb_arrived, 10)

        # Puntos a visitar (dentro del espacio 2x2)
        self.waypoints = [
            [2.0, 0.0],
            [0.0, 2.0]
        ]

        self.wp_index = 0
        self.pending_arrival = False
        self.mission_done = False

        self.create_timer(0.2, self.loop)

    def cb_arrived(self, msg):
        if msg.data and self.pending_arrival:
            self.get_logger().info("Waypoint alcanzado. Avanzando al siguiente.")
            self.wp_index += 1
            self.pending_arrival = False

    def loop(self):
        if self.mission_done:
            return

        if self.wp_index >= len(self.waypoints):
            self.get_logger().info("Misión completada")
            self.mission_done = True
            return

        x, y = self.waypoints[self.wp_index]

        msg = Pose()
        msg.x = x
        msg.y = y
        self.pub_point.publish(msg)
        self.pending_arrival = True


def main(args=None):
    rclpy.init(args=args)
    node = WaypointSequencer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Nodo detenido por el usuario")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
