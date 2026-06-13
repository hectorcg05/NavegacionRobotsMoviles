#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_msgs.msg import Bool


class APFController(Node):
    def __init__(self):
        super().__init__("vpf_controller")
        self.get_logger().info("Controlador APF activo")

        # Estado de la pose del robot
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.odom_ready = False

        # Estado del objetivo
        self.goal_x = None
        self.goal_y = None
        self.target_ready = False
        self.at_goal = False

        # Obstáculos en el espacio
        self.obstacles = [
            [0.8, 1.2],
            [1.5, 0.8]
        ]

        # Ganancias del campo potencial
        self.k_att = 0.8
        self.k_rep = 0.45
        self.rep_radius = 0.3

        # Ganancias del controlador
        self.Kv = 0.45
        self.Kw = 1.2

        # Límites de velocidad
        self.MAX_LIN_VEL = 0.16
        self.MAX_ANG_VEL = 1.1

        # Tolerancias
        self.goal_tolerance = 0.10
        self.angle_limit_to_move = 0.9

        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.pub_arrived = self.create_publisher(Bool, "/arrived", 10)
        self.create_subscription(Pose, "/odom", self.cb_odom, 10)
        self.create_subscription(Pose, "/next_point", self.cb_target, 10)
        self.create_timer(0.1, self.control_loop)

    def cb_odom(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.theta = msg.theta
        self.odom_ready = True

    def cb_target(self, msg):
        self.goal_x = msg.x
        self.goal_y = msg.y
        self.target_ready = True
        self.at_goal = False

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def clamp(self, value, min_value, max_value):
        return max(min(value, max_value), min_value)

    def stop_robot(self):
        msg = Twist()
        self.pub_cmd.publish(msg)

    def publish_arrived(self, value):
        msg = Bool()
        msg.data = value
        self.pub_arrived.publish(msg)

    def repulsive_force(self):
        fx_rep_total = 0.0
        fy_rep_total = 0.0

        for obs in self.obstacles:
            dx = self.x - obs[0]
            dy = self.y - obs[1]
            distance = math.sqrt(dx**2 + dy**2)

            if distance < 0.001:
                distance = 0.001

            if distance <= self.rep_radius:
                ux = dx / distance
                uy = dy / distance
                rep_magnitude = self.k_rep * (
                    (1.0 / distance) - (1.0 / self.rep_radius)
                ) * (1.0 / (distance**2))
                fx_rep_total += rep_magnitude * ux
                fy_rep_total += rep_magnitude * uy

        return fx_rep_total, fy_rep_total

    def attractive_force(self):
        fx_att = self.k_att * (self.goal_x - self.x)
        fy_att = self.k_att * (self.goal_y - self.y)
        return fx_att, fy_att

    def control_loop(self):
        if not self.odom_ready or not self.target_ready:
            return

        dx_goal = self.goal_x - self.x
        dy_goal = self.goal_y - self.y
        distance_to_goal = math.sqrt(dx_goal**2 + dy_goal**2)

        if distance_to_goal < self.goal_tolerance:
            self.stop_robot()
            if not self.at_goal:
                self.get_logger().info(
                    f"Objetivo alcanzado: ({self.goal_x:.2f}, {self.goal_y:.2f})"
                )
                self.publish_arrived(True)
                self.at_goal = True
            return
        else:
            self.publish_arrived(False)

        fx_att, fy_att = self.attractive_force()
        fx_rep, fy_rep = self.repulsive_force()

        fx_total = fx_att + fx_rep
        fy_total = fy_att + fy_rep

        desired_theta = math.atan2(fy_total, fx_total)
        angle_error = self.normalize_angle(desired_theta - self.theta)
        force_magnitude = math.sqrt(fx_total**2 + fy_total**2)

        cmd = Twist()
        cmd.angular.z = self.clamp(
            self.Kw * angle_error, -self.MAX_ANG_VEL, self.MAX_ANG_VEL
        )

        # Solo avanza si ya está suficientemente alineado
        if abs(angle_error) < self.angle_limit_to_move:
            cmd.linear.x = self.clamp(
                self.Kv * force_magnitude * math.cos(angle_error),
                0.0,
                self.MAX_LIN_VEL
            )

        self.pub_cmd.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = APFController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Nodo detenido")
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
