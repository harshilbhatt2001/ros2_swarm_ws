"""Script to spawn turtlebot3_waffle in a generic position."""

import argparse
import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import SpawnEntity
import rclpy


def parse_arguments():
    parser = argparse.ArgumentParser(description="Spawn Robot into gazebo with navigation2")
    parser.add_argument('-n', '--robot_name', type=str, default='robot',
                        help='Name of the robot to spawn')
    parser.add_argument('-ns', '--robot_namespace', type=str, default='robot',
                        help='ROS namespace to apply to the tf and plugins')
    parser.add_argument('-x', type=float, default=0,
                        help='the x component of the initial position [meters]')
    parser.add_argument('-y', type=float, default=0,
                        help='the y component of the initial position [meters]')
    parser.add_argument('-z', type=float, default=0,
                        help='the z component of the initial position [meters]')
    parser.add_argument('-k', '--timeout', type=float, default=10.0,
                        help="Seconds to wait. Block until the future is complete if negative. \
                              Don't wait if 0.")

    args, unknown = parser.parse_known_args()
    return args


def main():
    args = parse_arguments()

    # start node
    rclpy.init()
    node = rclpy.create_node('entity_spawner')

    node.get_logger().info('Creating Service client to connect to `/spawn_entity`')
    client = node.create_client(SpawnEntity, '/spawn_entity')

    node.get_logger().info('Connecting to `/spawn_entity` service...')
    if not client.service_is_ready():
        client.wait_for_service()
        node.get_logger().info('...connected')

    node.get_logger().info(
        f"spawning `{args.robot_name}` on namespace `{args.robot_namespace}` at \
            {args.x}, {args.y}, {args.z}")

    # TODO(harshil): Add correct path from #18
    sdf_file_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo',
                                    'models',
                                    'turtlebot3_waffle',
                                    'model.sdf')
    )

    # We need to remap the transform (/tf) topic so each robot has its own.
    # We do this by adding `ROS argument entries` to the sdf file for
    # each plugin broadcasting a transform. These argument entries provide the
    # remapping rule, i.e. /tf -> /<robot_id>/tf
    tree = ET.parse(sdf_file_path)
    root = tree.getroot()
    for plugin in root.iter('plugin'):
        # TODO(orduno) Handle case if an sdf file from non-turtlebot is provided
        if 'turtlebot3_diff_drive' in plugin.attrib.values():
            # The only plugin we care for now is 'diff_drive' which is
            # broadcasting a transform between`odom` and `base_footprint`
            break

    ros_params = plugin.find('ros')
    ros_tf_remap = ET.SubElement(ros_params, 'remapping')
    ros_tf_remap.text = '/tf:=/' + args.robot_namespace + '/tf'

    # Set data for request
    request = SpawnEntity.Request()
    request.name = args.robot_name
    request.xml = ET.tostring(root, encoding='unicode')
    request.robot_namespace = args.robot_namespace
    request.initial_pose.position.x = float(args.x)
    request.initial_pose.position.y = float(args.y)
    request.initial_pose.position.z = float(args.z)

    node.get_logger().info('Sending service request to `/spawn_entity`')
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, args.timeout)
    if future.result() is not None:
        print('response: %r' % future.result())
    else:
        raise RuntimeError(
            'exception while calling service: %r' % future.exception())

    node.get_logger().info('Done! Shutting down node.')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main
