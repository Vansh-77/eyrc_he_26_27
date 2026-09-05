import os
import time

import mujoco

import rclpy
from rclpy.node import Node

import numpy as np

from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose


class CameraNode(Node):

    def __init__(self):
        super().__init__('camera_node')

        # -------------------------
        # MuJoCo
        # -------------------------

        xml_path = os.path.join(
            get_package_share_directory('prep'),
            'models',
            'holonomic_bot',
            'mujoco',
            'hb.xml'
        )

        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)

        self.renderer = mujoco.Renderer(
            self.model,
            height=480,
            width=640
        )

        # -------------------------
        # Robot pose
        # -------------------------

        self.robot_pose = None

        self.pose_subscription = self.create_subscription(
            Pose,
            '/robot_state',
            self.robot_state_callback,
            10
        )

        # -------------------------
        # Camera publisher
        # -------------------------

        self.image_pub = self.create_publisher(
            Image,
            '/camera/image_raw',
            10
        )

        # 30 Hz camera
        self.timer = self.create_timer(
            1.0 / 30.0,
            self.publish_frame
        )

        self.get_logger().info(
            'Camera node started'
        )

    def robot_state_callback(self, msg):

        self.robot_pose = msg

    def publish_frame(self):

        # Don't render until we have robot state
        if self.robot_pose is None:
            return

        # -------------------------
        # Update MuJoCo state
        # -------------------------

        pose = self.robot_pose

        self.data.qpos[0] = pose.position.x
        self.data.qpos[1] = pose.position.y
        self.data.qpos[2] = pose.position.z

        self.data.qpos[3] = pose.orientation.w
        self.data.qpos[4] = pose.orientation.x
        self.data.qpos[5] = pose.orientation.y
        self.data.qpos[6] = pose.orientation.z

        mujoco.mj_forward(
            self.model,
            self.data
        )

        # -------------------------
        # Render
        # -------------------------

        self.renderer.update_scene(
            self.data,
            camera='overhead_camera'
        )

        frame = self.renderer.render()

        frame = np.asarray(
            frame,
            dtype=np.uint8
        )

        frame = np.ascontiguousarray(frame)

        # -------------------------
        # ROS Image
        # -------------------------

        image_msg = Image()

        image_msg.header.stamp = (
            self.get_clock().now().to_msg()
        )

        image_msg.header.frame_id = 'overhead_camera'

        image_msg.height = frame.shape[0]
        image_msg.width = frame.shape[1]

        image_msg.encoding = 'rgb8'
        image_msg.is_bigendian = 0

        image_msg.step = frame.shape[1] * 3

        image_msg.data = frame.tobytes()

        self.image_pub.publish(image_msg)


def main(args=None):

    rclpy.init(args=args)

    node = CameraNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()