import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist


class ControlNode(Node):

    def __init__(self):
        super().__init__('control_node')

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.start_time = self.get_clock().now()

        self.timer = self.create_timer(
            0.02,
            self.control_loop
        )

    def control_loop(self):

        elapsed = (
            self.get_clock().now() - self.start_time
        ).nanoseconds / 1e9

        cmd = Twist()

        # --------------------------------
        # State 1: Forward for 10 seconds
        # --------------------------------
        if elapsed < 10.0:

            cmd.linear.x = 0.0
            cmd.linear.y = 0.8

        # --------------------------------
        # State 2: Right for 10 second
        # --------------------------------
        elif elapsed < 20.0:

            cmd.linear.x = 0.8
            cmd.linear.y = 0.0

        # --------------------------------
        # State 3: Diagonal for 10 seconds
        # --------------------------------
        elif elapsed < 30.0:

            cmd.linear.x = -0.8
            cmd.linear.y = -0.8

        # --------------------------------
        # Stop
        # --------------------------------
        else:

            cmd.linear.x = 0.0
            cmd.linear.y = 0.0

        self.cmd_pub.publish(cmd)


def main(args=None):

    rclpy.init(args=args)

    node = ControlNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()