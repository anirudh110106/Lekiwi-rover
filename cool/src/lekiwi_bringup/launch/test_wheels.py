#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

WHEEL_TOPIC = "/base_velocity_controller/commands"

class WheelDemo(Node):

    def __init__(self):
        super().__init__("lekiwi_wheel_demo")

        self.pub = self.create_publisher(
            Float64MultiArray,
            WHEEL_TOPIC,
            10
        )

        # velocities (rad/s)
        self.commands = [
            [0.0, 0.0, 0.0],     # stop
            [6.0, 6.0, 6.0],     # forward
            # [-3.0, -3.0, -3.0],  # backward
            # [0.0, 0.0, 0.0],     # stop
        ]

        self.phase = 0

        self.get_logger().info(
            f"Publishing wheel velocities to {WHEEL_TOPIC}"
        )

        self.timer = self.create_timer(4.0, self.tick)

    def tick(self):

        msg = Float64MultiArray()
        msg.data = self.commands[self.phase]

        self.pub.publish(msg)

        self.get_logger().info(
            f"Phase {self.phase} → {self.commands[self.phase]}"
        )

        self.phase = (self.phase + 1) % len(self.commands)


def main():
    rclpy.init()
    node = WheelDemo()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()