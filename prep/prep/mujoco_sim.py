import os
import threading

import mujoco
import mujoco.viewer

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import math
import numpy as np

from ament_index_python.packages import get_package_share_directory


def inverse_kinematics(vx, vy, wz):

    alpha1 = math.radians(30.0)
    alpha2 = math.radians(150.0)
    alpha3 = math.radians(270.0)

    M = np.array([
        [
            math.cos(alpha1 + math.pi / 2),
            math.cos(alpha2 + math.pi / 2),
            math.cos(alpha3 + math.pi / 2)
        ],
        [
            math.sin(alpha1 + math.pi / 2),
            math.sin(alpha2 + math.pi / 2),
            math.sin(alpha3 + math.pi / 2)
        ],
        [1.0, 1.0, 1.0]
    ])

    velocity = np.array([vx, vy, wz])

    return np.linalg.solve(M, velocity)

class CmdVelSubscriber(Node):

    def __init__(self):
        super().__init__('mujoco_cmd_vel')

        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

    def cmd_vel_callback(self, msg):
        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.wz = msg.angular.z


def main():

    rclpy.init()

    ros_node = CmdVelSubscriber()

    ros_thread = threading.Thread(
        target=rclpy.spin,
        args=(ros_node,),
        daemon=True
    )
    ros_thread.start()

    xml_path = os.path.join(
    get_package_share_directory('prep'),
    'models',
    'holonomic_bot',
    'mujoco',
    'hb.xml'
)

    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)
    left_actuator = mujoco.mj_name2id(
    model,
    mujoco.mjtObj.mjOBJ_ACTUATOR,
    "rim_left_vel"
    )

    back_actuator = mujoco.mj_name2id(
    model,
    mujoco.mjtObj.mjOBJ_ACTUATOR,
    "rim_back_vel"
    )

    right_actuator = mujoco.mj_name2id(
    model,
    mujoco.mjtObj.mjOBJ_ACTUATOR,
    "rim_right_vel"
    )
    with mujoco.viewer.launch_passive(model, data) as viewer:

        while viewer.is_running():

            # ROS command currently available as:
            vx = ros_node.vx
            vy = ros_node.vy
            wz = ros_node.wz
            s1, s2, s3 = inverse_kinematics(vx, vy, wz)
            data.ctrl[right_actuator] = s1
            data.ctrl[left_actuator] = s2
            data.ctrl[back_actuator] = s3
            

            mujoco.mj_step(model, data)

            viewer.sync()

    ros_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()