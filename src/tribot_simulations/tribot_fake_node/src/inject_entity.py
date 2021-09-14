#!/usr/bin/python3

import os
import sys

import rclpy
import xacro
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose


def inject(xml: str, initial_pose: Pose):
    rclpy.init()
    node = rclpy.create_node('inject_node')
    client = node.create_client(SpawnEntity, 'spawn_entity')

    if not client.service_is_ready():
        node.get_logger().info('waiting for spawn_entity service...')
        client.wait_for_service()

    request = SpawnEntity.Request()
    request.xml = xml
    request.initial_pose = initial_pose
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        node.get_logger().info('response: %r' % future.result())
    else:
        raise RuntimeError('exception while calling service %r' % future.exception())

    node.destroy_node()
    rclpy.shutdown()


if len(sys.argv) < 3:
    print('usage: ros2 run tribot_fake_node inject_entity.py -- initial_x initial_y initial_z')
    sys.exit(1)

p = Pose()
p.position.x = float(sys.argv[1])
p.position.y = float(sys.argv[2])
p.position.z = float(sys.argv[3])

urdf_file_name = 'tribot.urdf'
doc = xacro.parse(open(os.path.join(
    get_package_share_directory('tribot_description'), 'urdf', urdf_file_name)))
xacro.process_doc(doc)
urdf = doc.toxml()

inject(urdf, p)
