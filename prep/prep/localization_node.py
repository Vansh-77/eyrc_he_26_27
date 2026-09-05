import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose2D


class LocalizationNode(Node):

    def __init__(self):
        super().__init__('localization_node')

        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.pose_pub = self.create_publisher(
            Pose2D,
            '/robot_pose',
            10
        )

        # Arena coordinate system
        #
        # (0, 1.4) ---------------- (2, 1.4)
        #    ID 1                      ID 2
        #      |                        |
        #      |          ROBOT         |
        #      |                        |
        #    ID 4                      ID 3
        # (0, 0) ------------------ (2, 0)

        self.arena_points = {
            1: (0.0, 1.4),
            2: (2.0, 1.4),
            3: (2.0, 0.0),
            4: (0.0, 0.0)
        }

        # TODO:
        # Initialize ArUco dictionary/detector

    def image_callback(self, msg):

        # TODO:
        #
        # 1. Convert ROS Image -> OpenCV image
        #
        # 2. Detect ArUco markers
        #
        # 3. Detect arena markers:
        #       ID 1, 2, 3, 4
        #
        # 4. Detect robot marker:
        #       ID 0
        #
        # 5. Estimate robot position
        #
        # 6. Estimate robot orientation
        #
        # 7. Publish Pose2D on /robot_pose

        pass


def main(args=None):

    rclpy.init(args=args)

    node = LocalizationNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()